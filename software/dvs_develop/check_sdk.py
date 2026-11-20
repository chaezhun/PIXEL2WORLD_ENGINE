#!/usr/bin/env python3
"""dvs_read.load() 결과를 NRV SDK 가 푼 결과(sdk_dump 출력)와 이벤트 하나하나 맞대어 본다.

    python3 software/dvs_develop/check_sdk.py <파일>.dvs <sdk_dump 출력>.bin

sdk_dump 출력은 프레임마다 [int32 머리 5개, int32 이벤트 수] 뒤에 이벤트 12바이트
(uint16 x, uint16 y, uint8 극성, 3바이트 빈칸, uint32 시각 µs) 가 이어진 것이다(sdk_dump.cpp).

비교 항목
  프레임 수, 이벤트 수        : 같아야 한다
  프레임별 이벤트 집합         : (x, y, 극성) 을 정렬해 비트 단위로 같아야 한다. 순서는 보지 않는다
  프레임 시각                  : SDK 는 이벤트마다 시각을 매긴다. 프레임 안 이벤트 시각이 하나인
                                 프레임에서 그 값이 우리 t_us 와 같은지 센다. 다른 프레임은 목록으로 낸다
종료 코드 0 은 앞의 둘이 모두 같다는 뜻이다. 시각 차이는 종료 코드에 넣지 않는다.
"""
import sys
import numpy as np
from dvs_read import load

EV = np.dtype([('x', '<u2'), ('y', '<u2'), ('p', 'u1'), ('pad', 'u1', 3), ('t', '<u4')])


def read_sdk(path):
    """sdk_dump 출력을 (프레임별 이벤트 수, 이벤트 배열) 로 읽는다."""
    raw = np.fromfile(path, dtype=np.uint8)
    off, counts, chunks = 0, [], []
    while off < len(raw):
        n = int(raw[off + 20:off + 24].view('<i4')[0])
        off += 24
        chunks.append(raw[off:off + 12 * n].view(EV))
        counts.append(n)
        off += 12 * n
    return np.array(counts), np.concatenate(chunks)


def key(frame, x, y, p):
    """(프레임, x, y, 극성) 을 정수 하나로 묶는다. 정렬해서 집합 비교에 쓴다."""
    return (frame.astype(np.int64) << 22) | (x.astype(np.int64) << 11) | (y.astype(np.int64) << 1) | p


def main(dvs, sdk):
    d = load(dvs)
    counts, e = read_sdk(sdk)
    nf = len(counts)
    sdk_frame = np.repeat(np.arange(nf), counts)
    ok = True

    print(f'프레임 수   ours {d["n_frames"]:>10}  sdk {nf:>10}')
    print(f'이벤트 수   ours {len(d["x"]):>10}  sdk {len(e):>10}')
    ok &= d['n_frames'] == nf and len(d['x']) == len(e)

    if ok:
        a = np.sort(key(d['frame'], d['x'], d['y'], d['p']))
        b = np.sort(key(sdk_frame, e['x'], e['y'], e['p']))
        diff = int(np.count_nonzero(a != b))
        print(f'(프레임, x, y, 극성) 다른 이벤트 {diff}')
        ok &= diff == 0

    # 프레임 시각. SDK 이벤트 시각의 프레임별 최소 · 최대
    idx = np.r_[0, np.cumsum(counts)[:-1]]
    has = counts > 0
    tmin = np.full(nf, -1, np.int64)
    tmax = np.full(nf, -1, np.int64)
    tmin[has] = np.minimum.reduceat(e['t'].astype(np.int64), idx[has])
    tmax[has] = np.maximum.reduceat(e['t'].astype(np.int64), idx[has])
    n = min(nf, len(d['t_us']))
    single = has[:n] & (tmin[:n] == tmax[:n])
    same = single & (tmin[:n] == d['t_us'][:n])
    print(f'프레임 시각 SDK 와 같음 {int(same.sum())}/{n}, '
          f'SDK 가 한 프레임에 시각 두 개 {int((has[:n] & ~single).sum())}, '
          f'시각 하나인데 다름 {int((single & ~same).sum())}')
    for k in np.flatnonzero(has[:n] & ~same)[:30]:
        print(f'  프레임 {k}: ours {d["t_us"][k]}  sdk {tmin[k]}~{tmax[k]}  (차 {tmin[k] - d["t_us"][k]})')
    print('일치' if ok else '불일치')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main(*sys.argv[1:3]))
