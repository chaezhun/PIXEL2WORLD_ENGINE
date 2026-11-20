#!/usr/bin/env python3
"""NRV Delta-01 녹화 파일(.dvs, MGROUP 형식)을 (프레임, x, y, 극성) 이벤트 목록으로 푼다.

    python3 software/dvs_develop/dvs_read.py software/datasets/<파일>.dvs   # 요약 출력

라이브러리로 쓸 때는 load() 하나만 부르면 된다.

    from dvs_read import load
    d = load(path)            # d['frame'], d['x'], d['y'], d['p'], d['t_us'], ...

파일 구조 (32비트 big-endian 워드의 나열. 비트 번호는 워드의 MSB 가 31)
  머리       : 상위 바이트가 0x7f 인 워드 몇 개. [23:16] 이 태그, [15:0] 이 값(바이트 순서가 뒤집힌
               little-endian 16비트). 태그 0x10~0x14 로 타임스탬프 배율을 계산한다(_tick_us 참고).
  열 주소    : [31:24] = 0x04. [10:0] 열 번호 x(0~959), [18:11] 프레임 카운터(8비트, 한 바퀴 돌면 0),
               [21] 이 1 이면 프레임의 첫 주소. 뒤따르는 이벤트 워드는 모두 이 x 에 속한다.
  이벤트     : [31] = 1. 한 워드가 같은 열의 8행 묶음 두 개를 싣는다.
               [7:0]   아래 묶음 비트맵. 비트 k 가 1 이면 y = 8*G + k 에 이벤트, 극성 [16]
               [15:8]  위 묶음 비트맵.   비트 k 가 1 이면 y = 8*(G+A) + k 에 이벤트, 극성 [17]
               [25:18] G, 아래 묶음 번호(0~89)
               [30:26] A, 위 묶음이 아래 묶음보다 몇 묶음 위인가. A=0 이면 같은 묶음의 두 극성
  프레임 끝  : [31:24] = 0x0c. [18:11] 프레임 카운터. 같은 카운터로 두 번 나올 때가 있어(타임스탬프
               워드 사이에 끼어서) 카운터가 바뀔 때만 프레임 경계로 센다.
  타임스탬프 : [31:24] = 0x08. [23]=0 이면 상위(밀리초 단위 카운터), [23]=1 이면 하위(0~999).
               프레임 끝 뒤에 상위 · 하위 한 쌍이 붙고, 이 값이 다음 프레임의 시각이 된다.

극성 p 는 SDK 와 같게 0 = ON(밝아짐), 1 = OFF(어두워짐) 이다.

이 파서가 푼 이벤트는 check_sdk.py 로 NRV SDK(libDELTA_SDK.so)가 푼 결과(sdk_dump.cpp 출력)와
이벤트 하나하나 맞대어 볼 수 있다. 프레임 수, 프레임별 (x, y, 극성) 집합, 프레임 시각을 비교한다.
"""
import sys
import numpy as np

WIDTH, HEIGHT = 960, 720

# 워드 종류. 상위 바이트로 가른다. 이벤트 워드는 MSB 하나로 가르므로 여기 없다.
T_HEADER, T_ADDR, T_TS, T_END = 0x7f, 0x04, 0x08, 0x0c


def _header(w):
    """앞쪽 머리 워드 수와 {태그: 값} 을 돌려준다."""
    n = 0
    while n < len(w) and (w[n] >> 24) == T_HEADER:
        n += 1
    tags = {}
    for v in w[:n].tolist():
        tags[(v >> 16) & 0xff] = ((v & 0xff) << 8) | ((v >> 8) & 0xff)
    return n, tags


def _tick_us(tags):
    """타임스탬프 하위 1 칸과 상위 1 칸이 각각 몇 µs 인지.

    SDK 의 FileStream::CheckFile 과 같은 순서 · 같은 double 연산으로 계산한다. 24 MHz 기준 클럭을
    tag[0x11]/tag[0x10] 배 하고 2**tag[0x12] 로, 다시 10 으로 나눈 것이 카운터 클럭이고,
    하위 1 칸은 그 클럭 tag[0x13]+1 주기, 상위 1 칸은 하위 tag[0x14]+1 칸이다.
    Delta-01 녹화(3, 157, 0, 125, 999)에서는 하위 1.0031847 µs, 상위 1003.1847 µs 다.
    """
    f = 24e6 * tags[0x11] / tags[0x10] / (1 << tags[0x12])
    lo = 1e6 / (f / 10.0) * (tags[0x13] + 1)
    return lo, lo * (tags[0x14] + 1)


def load(path):
    """.dvs 파일 하나를 읽어 dict 로 돌려준다.

    frame, x, y, p : 이벤트별 배열. 프레임 번호는 0 부터, 파일 안 순서대로 매긴다.
    t_us           : 프레임별 시각(µs). 프레임 앞에 온 타임스탬프 워드로 만든다. 첫 프레임은 앞에
                     타임스탬프가 없어 0 이다(SDK 도 0).
    n_frames       : 끝 표지까지 온전히 들어온 프레임 수. 마지막 끝 표지 뒤에 남은 이벤트는 프레임이
                     끝나지 않았으므로 버린다(SDK 도 버린다). 버린 수는 n_dropped_tail.
    end_pos        : 프레임마다 그 프레임을 닫는 끝 표지의 워드 번호(파일 첫 워드가 0). 원시 워드를
                     프레임 단위로 잘라 RTL 테스트벤치에 넣을 때 쓴다.
    header         : 머리 태그 {태그: 값}
    first_x_missing: 첫 프레임이 열 주소 없이 이벤트로 시작했는지. 녹화가 프레임 중간에서 시작하면
                     그렇다. 이때 첫 열 주소 앞 이벤트는 x = 0 으로 둔다(SDK 와 같다).
    """
    w = np.fromfile(path, dtype='>u4').astype(np.uint32)
    nh, tags = _header(w)
    w = w[nh:]
    top = w >> 24
    is_ev = (w >> 31) == 1

    kinds = set(np.unique(top[~is_ev]).tolist())
    unknown = kinds - {T_ADDR, T_TS, T_END}
    if unknown:
        raise ValueError(f'{path}: 모르는 워드 종류 {sorted(hex(k) for k in unknown)}')

    # 프레임 경계: 끝 표지 중 프레임 카운터가 바로 앞 끝 표지와 다른 것
    end_pos = np.flatnonzero(top == T_END)
    fc = (w[end_pos] >> 11) & 0xff
    bound = end_pos[np.r_[True, fc[1:] != fc[:-1]]]

    # 프레임 시각: 타임스탬프 워드를 상위 · 하위 레지스터에 차례로 덮어쓰고, 프레임 경계마다 그때의
    # 레지스터 값을 그 프레임의 시각으로 읽는다. 경계 앞에서 마지막으로 쓰인 값을 찾는 것과 같다.
    ts_pos = np.flatnonzero(top == T_TS)
    ts_lo = ((w[ts_pos] >> 23) & 1) == 1
    lo_us, hi_us = _tick_us(tags)

    def last_before(pos, val):
        k = np.searchsorted(pos, bound) - 1
        return np.where(k >= 0, val[np.maximum(k, 0)], 0).astype(np.float64)

    hi = last_before(ts_pos[~ts_lo], w[ts_pos[~ts_lo]] & 0x7fffff)
    lo = last_before(ts_pos[ts_lo], w[ts_pos[ts_lo]] & 0x7fffff)
    t_us = np.floor(hi * hi_us + lo * lo_us + 0.5).astype(np.int64)

    # 이벤트 워드마다 프레임 번호와 x. x 는 바로 앞 열 주소에서 가져온다.
    ev_pos = np.flatnonzero(is_ev)
    ev_frame = np.searchsorted(bound, ev_pos).astype(np.int32)
    addr_pos = np.flatnonzero(top == T_ADDR)
    k = np.searchsorted(addr_pos, ev_pos) - 1
    ev_x = np.where(k >= 0, w[addr_pos[np.maximum(k, 0)]] & 0x7ff, 0).astype(np.uint16)

    # 비트맵 16비트를 펼쳐 1 인 자리마다 이벤트 하나를 만든다. 한 번에 다 펼치면 녹화 하나에 메모리가
    # 9 GB 까지 커져서 워드 2M 개씩 나눈다. 결과는 파일 순서 그대로다(워드 순, 한 워드 안에서는 비트 순).
    parts = []
    for s in range(0, len(ev_pos), 1 << 21):
        v = w[ev_pos[s:s + (1 << 21)]]
        bits = np.unpackbits((v & 0xffff).astype('<u2').view(np.uint8).reshape(-1, 2),
                             axis=1, bitorder='little')
        i, b = np.nonzero(bits)            # b: 비트맵의 비트 번호 0~15
        up = b >= 8                        # 위 묶음 [15:8]
        g = ((v >> 18) & 0xff).astype(np.int32)[i]
        a = ((v >> 26) & 0x1f).astype(np.int32)[i]
        parts.append((ev_frame[s:][i], ev_x[s:][i],
                      (8 * (g + np.where(up, a, 0)) + (b & 7)).astype(np.uint16),
                      ((v[i] >> np.where(up, 17, 16)) & 1).astype(np.uint8)))
    frame, x, y, p = (np.concatenate(c) for c in zip(*parts))
    # 프레임 번호는 파일 순서대로 늘어나므로 끝나지 않은 꼬리는 맨 뒤에 몰려 있다.
    n = int(np.searchsorted(frame, len(bound)))
    res = {
        'frame': frame[:n],
        'x': x[:n],
        'y': y[:n],
        'p': p[:n],
        't_us': t_us,
        'n_frames': len(bound),
        'end_pos': bound + nh,             # 프레임마다 끝 표지 워드의 파일 안 워드 번호(머리 포함)
        'n_dropped_tail': len(frame) - n,
        'header': tags,
        'first_x_missing': bool(len(ev_pos) and (not len(addr_pos) or ev_pos[0] < addr_pos[0])),
    }
    assert res['y'].max(initial=0) < HEIGHT and res['x'].max(initial=0) < WIDTH, '좌표가 센서 밖'
    return res


def main():
    for path in sys.argv[1:]:
        d = load(path)
        n = len(d['frame'])
        per = np.bincount(d['frame'], minlength=d['n_frames'])
        dt = np.diff(d['t_us'][1:])
        print(f'{path}')
        print(f'  프레임 {d["n_frames"]}, 이벤트 {n} (ON {int((d["p"] == 0).sum())}, OFF {int((d["p"] == 1).sum())}),'
              f' 끝나지 않은 꼬리 {d["n_dropped_tail"]} 개 버림')
        print(f'  프레임당 이벤트 최소 {per.min()} 중앙 {int(np.median(per))} 최대 {per.max()}')
        gap = np.flatnonzero(dt > 1.5 * np.median(dt)) + 2    # dt[j] 는 프레임 j+1 과 j+2 사이
        print(f'  프레임 간격 {dt.min()}~{dt.max()} µs, 길이 {(d["t_us"][-1] - d["t_us"][1]) / 1e6:.3f} s'
              f' (둘째 프레임부터), 간격이 중앙값의 1.5 배를 넘는 곳: 프레임 {gap.tolist()} 앞')
        print(f'  머리 {d["header"]}, 첫 프레임 x 누락 {d["first_x_missing"]}')


if __name__ == '__main__':
    main()
