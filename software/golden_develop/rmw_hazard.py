#!/usr/bin/env python3
"""파일 순서로 이어지는 이벤트가 같은 월드 칸을 얼마나 자주 치는지 잰다.

    python3 software/golden_develop/rmw_hazard.py <녹화.dvs> <골든 묶음> > <묶음>/hazard.txt

이벤트마다 월드 메모리를 읽고 1 을 더해 다시 쓰는 누적기에서, 앞 이벤트의 쓰기가 끝나기 전에 같은 칸을 다시
읽으면 갱신 전 값을 읽는다. 그래서 거리 k(k 번째 앞 이벤트)에서 같은 칸이 얼마나 나오는지가 쓰기 값 전달
(forwarding)이나 기다림이 몇 단까지 필요한지를 정한다. 같은 칸이 이어지는 묶음(run)의 수는 이웃 이벤트를 합쳐
쓰면 메모리 접근이 몇 번으로 줄어드는지다. 이벤트 순서는 파서 결과 그대로(워드 순, 워드 안에서는 아래 묶음 →
위 묶음, 행 번호 오름차순)이고 RTL 이 처리하는 순서와 같다.

칸 주소는 골든 묶음의 프레임별 cos · sin 과 회전 중심으로 golden.cells 가 낸 값이고, 메모리 범위 밖 이벤트도
빼지 않고 그대로 센다. 출력(표준 출력)은 거리 1 ~ 6 의 같은 칸 비율과 묶음 수 · 묶음당 이벤트 수다.
"""
import sys
import numpy as np
sys.path.insert(0, __file__.rsplit('/', 2)[0] + '/dvs_develop')
from dvs_read import load
import golden as g


def main(path, bundle):
    d = load(path)
    _, _, c, s, (cx, cy) = g.frame_trig(bundle)
    f = d['frame']
    wx, wy = g.cells(d['x'].astype(np.int64), d['y'].astype(np.int64), c[f], s[f], cx, cy)
    a = wy * g.N + wx
    print(f'{path.rsplit("/", 1)[-1]}, 이벤트 {len(a)}, 회전 중심 ({cx}, {cy}), 칸 {1 << g.CELL_SHIFT} px')
    for k in range(1, 7):
        print(f'  거리 {k}: 같은 칸 {(a[k:] == a[:-k]).mean() * 100:.2f} %')
    runs = int(np.count_nonzero(np.r_[True, a[1:] != a[:-1]]))
    print(f'  같은 칸이 이어지는 묶음 {runs}, 묶음당 이벤트 {len(a) / runs:.3f}')


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
