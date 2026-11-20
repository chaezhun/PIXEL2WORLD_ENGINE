#!/usr/bin/env python3
"""녹화마다 RTL 이 실시간으로 처리해야 하는 워드 수 · 이벤트 수를 잰다.

    python3 software/dvs_develop/throughput.py software/datasets/*.dvs > <묶음>/throughput.txt

센서는 프레임 하나를 약 776 µs 마다 내보낸다. 한 프레임 안의 워드가 언제 도착하는지는 파일에 없으므로
(타임스탬프가 프레임마다 하나뿐) 요구량은 "프레임 하나를 그 프레임 간격 안에 다 처리한다" 로 잡는다.
입력 FIFO 가 프레임 몇 개를 담을 수 있으면 요구량은 여러 프레임 평균으로 내려가므로, 연속 1 · 4 · 16 ·
64 프레임 창의 최댓값도 같이 낸다.

세는 것
  워드     : 머리를 뺀 모든 워드. 입력 포트가 클럭마다 워드 몇 개를 받아야 하나를 정한다.
  이벤트   : 이벤트 워드 비트맵의 1 의 수. 메모리에 쓰는 횟수라 메모리 포트 수 · 뱅크 수를 정한다.
  열 주소  : x 가 바뀌는 횟수. dx 항(C·dx, S·dx)을 열 주소마다 한 번만 곱하면 이 수만큼만 곱한다.
  8행 묶음 : 이벤트 워드 하나에 묶음 두 개(같은 묶음이면 하나). dy 항을 묶음마다 한 번 곱할 때의 수.

프레임 f 는 끝 표지 f-1 다음 워드부터 끝 표지 f 까지다. 그 간격은 t_us[f+1] - t_us[f] 와
t_us[f] - t_us[f-1] 중 짧은 쪽을 쓴다. 파일의 프레임 시각이 프레임을 시작한 시각인지 끝낸 시각인지
파일만으로는 알 수 없으므로 둘 중 빡빡한 쪽을 택한다. 첫 프레임은 앞에 타임스탬프가 없어(t = 0) 빼고,
마지막 프레임은 뒤 간격이 없어 앞 간격만 쓴다.

열 주소가 중앙값의 1.1 배를 넘는 프레임은 열 훑기가 한 번보다 많이 들어 있으므로(예: 장면 1 프레임 513,
열 주소 1579 개. 바로 뒤에서 센서 프레임 3 개가 빠진다) 한 프레임 간격에 넣으면 요구량이 부풀려진다.
이런 프레임은 따로 적고, 창 최댓값에서는 그 프레임을 품은 창을 뺀다.
"""
import sys
import numpy as np
from dvs_read import load, _header, T_ADDR, T_END

# 이 클럭 주파수마다 클럭당 몇 개를 처리해야 하나를 표로 낸다(비교용 후보 값).
CLOCKS_MHZ = (100, 200, 400)
WINDOWS = (1, 4, 16, 64)


def per_frame(path):
    """프레임별 워드 · 이벤트 워드 · 이벤트 · 열 주소 · 8행 묶음 수와 간격(µs)을 돌려준다."""
    d = load(path)
    w = np.fromfile(path, dtype='>u4').astype(np.uint32)
    nh, _ = _header(w)
    nf = d['n_frames']
    end = d['end_pos']                       # 파일 안 워드 번호(머리 포함)
    start = np.r_[nh, end[:-1] + 1]          # 프레임 f 의 첫 워드
    words = end - start + 1

    # 종류별 위치를 프레임 경계로 세기: 위치 배열을 끝 표지 위치로 searchsorted 하면 누적 수가 나온다.
    body = w[:end[-1] + 1]
    is_ev = (body >> 31) == 1
    top = body >> 24

    def count(mask):
        pos = np.flatnonzero(mask)
        c = np.searchsorted(pos, end, side='right')
        return np.diff(np.r_[np.searchsorted(pos, nh), c])

    ev_words = count(is_ev)
    addr = count(~is_ev & (top == T_ADDR))
    # 8행 묶음: 위 묶음 비트맵이 비지 않고 A > 0 이면 묶음이 둘, 아니면 하나. 아래 묶음이 비고 위만 찬
    # 워드는 실제로 없지만 있어도 묶음 하나로 센다.
    v = body[is_ev]
    two = ((v >> 8) & 0xff != 0) & ((v & 0xff) != 0) & (((v >> 26) & 0x1f) != 0)
    grp_pos = np.flatnonzero(is_ev)
    grp_cum = np.r_[0, np.cumsum(1 + two.astype(np.int64))]
    k = np.searchsorted(grp_pos, end, side='right')
    groups = np.diff(np.r_[grp_cum[np.searchsorted(grp_pos, nh)], grp_cum[k]])
    events = np.bincount(d['frame'], minlength=nf)

    t = d['t_us'].astype(np.int64)
    back = np.r_[0, np.diff(t)]              # 프레임 f 와 f-1 사이
    fwd = np.r_[np.diff(t), 0]               # 프레임 f+1 과 f 사이
    dt = np.where(fwd > 0, np.minimum(back, fwd), back)    # 마지막 프레임은 fwd = 0 이라 앞 간격

    # 끝 표지 · 타임스탬프 워드는 이벤트도 열 주소도 아니다. 모든 워드가 셋 중 하나인지 확인.
    other = count(~is_ev & (top != T_ADDR))
    assert (ev_words + addr + other == words).all()
    assert events.sum() == len(d['frame'])
    return dict(name=path.rsplit('/', 1)[-1], words=words, ev_words=ev_words, addr=addr,
                groups=groups, events=events, dt=dt, t=t, n_events=len(d['frame']),
                popcount=np.bincount(_popcount16(v & 0xffff), minlength=17))


def _popcount16(x):
    x = x.astype(np.uint32)
    x = x - ((x >> 1) & 0x5555)
    x = (x & 0x3333) + ((x >> 2) & 0x3333)
    x = (x + (x >> 4)) & 0x0f0f
    return ((x + (x >> 8)) & 0x1f).astype(np.int64)


def window_peak(count, dt, n, ok):
    """연속 n 프레임 창에서 (개수 합 / 간격 합) 의 최댓값(/s)과 그 창의 첫 프레임. ok 가 거짓인 프레임을
    품은 창은 뺀다."""
    c = np.r_[0, np.cumsum(count)]
    s = np.r_[0, np.cumsum(dt)]
    bad = np.r_[0, np.cumsum(~ok)]
    rate = (c[n:] - c[:-n]) / (s[n:] - s[:-n]) * 1e6
    rate[(bad[n:] - bad[:-n]) > 0] = 0
    i = int(np.argmax(rate))
    return rate[i], i


def report(r):
    f = slice(1, None)                       # 첫 프레임(t = 0)은 뺀다
    dt = r['dt'][f].astype(np.float64)
    span_s = (r['t'][-1] - r['t'][1]) / 1e6
    addr = r['addr'][f]
    ok = addr <= 1.1 * np.median(addr)
    print(r['name'])
    print(f'  프레임 {len(r["words"])}, 길이 {span_s:.3f} s (둘째 프레임부터), '
          f'간격 중앙 {np.median(dt):.0f} µs, 최소 {dt.min():.0f}, 최대 {dt.max():.0f}')
    print(f'  워드 {r["words"].sum()}, 이벤트 워드 {r["ev_words"].sum()}, 열 주소 {r["addr"].sum()}, '
          f'8행 묶음 {r["groups"].sum()}, 이벤트 {r["n_events"]}')
    ew = r['ev_words'].sum()
    pc = r['popcount']
    print(f'  이벤트 워드당 이벤트 평균 {r["n_events"] / ew:.2f}, 16개 꽉 찬 워드 {pc[16] / ew * 100:.2f} %, '
          f'열 주소당 이벤트 {r["n_events"] / r["addr"].sum():.1f}, 8행 묶음당 이벤트 '
          f'{r["n_events"] / r["groups"].sum():.2f}')
    print('  이벤트 워드의 이벤트 수 분포(%) ' +
          ' '.join(f'{k}:{pc[k] / ew * 100:.1f}' for k in range(1, 17)))

    for j in np.flatnonzero(~ok):
        print(f'  열 훑기가 한 번보다 많은 프레임 {j + 1}: 열 주소 {addr[j]}, 워드 {r["words"][j + 1]}, '
              f'이벤트 {r["events"][j + 1]}, 앞 간격 {r["t"][j + 1] - r["t"][j]} µs, '
              f'뒤 간격 {r["t"][j + 2] - r["t"][j + 1] if j + 2 < len(r["t"]) else 0} µs')
    print(f'  {"항목":<8} {"녹화 평균 /s":>14} {"중앙 프레임":>10} {"최대 프레임":>10}  '
          + '  '.join(f'{n}프레임 창 최대 /s' for n in WINDOWS))
    peaks = {}
    for key, label in (('words', '워드'), ('events', '이벤트'), ('addr', '열 주소'), ('groups', '8행 묶음')):
        c = r[key][f]
        avg = c.sum() / span_s
        wp = [window_peak(c, dt, n, ok) for n in WINDOWS]
        peaks[key] = wp
        i = int(np.argmax(np.where(ok, c, 0)))
        print(f'  {label:<8} {avg / 1e6:12.2f} M {int(np.median(c)):10d} {int(c[i]):10d}  '
              + '  '.join(f'{p / 1e6:.2f} M(f{j + 1})' for p, j in wp))
    print('  클럭당 처리량 요구(1프레임 창 최댓값 기준) - '
          + ', '.join(f'{mhz} MHz: 워드 {peaks["words"][0][0] / (mhz * 1e6):.3f}, '
                      f'이벤트 {peaks["events"][0][0] / (mhz * 1e6):.3f}' for mhz in CLOCKS_MHZ))
    print()
    return peaks


def main():
    rs = [per_frame(p) for p in sys.argv[1:]]
    allp = [report(r) for r in rs]
    print('세 녹화 중 최악')
    for key, label in (('words', '워드'), ('events', '이벤트'), ('addr', '열 주소'), ('groups', '8행 묶음')):
        row = []
        for wi, n in enumerate(WINDOWS):
            j = int(np.argmax([p[key][wi][0] for p in allp]))
            row.append(f'{n}프레임 {allp[j][key][wi][0] / 1e6:.2f} M/s ({rs[j]["name"][:19]} f{allp[j][key][wi][1] + 1})')
        print(f'  {label}: ' + ', '.join(row))


if __name__ == '__main__':
    main()
