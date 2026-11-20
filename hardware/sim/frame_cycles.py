#!/usr/bin/env python3
"""tb_worldlock 의 +trace 출력에서 프레임마다 걸린 클럭 수를 뽑아 실시간 요구와 맞대 본다.

    obj_dir/sim +vec=<벡터> +trace > trace.txt          # run_sim.sh 로 한 번 빌드한 뒤
    python3 hardware/sim/frame_cycles.py trace.txt software/datasets/<녹화>.dvs

trace 의 "frame k at cycle c" 는 k 번째 프레임 경계가 디코더를 떠난 클럭이다. 입력을 매 클럭 내주므로
경계 k 와 k+1 사이의 클럭 수가 0 부터 센 프레임 k 의 처리 시간이다(파이프라인 지연은 앞뒤로 같아 빠진다).
그 프레임의 간격(throughput.py 와 같은 정의)으로 나누면 그 프레임을 제때 끝내는 데 필요한 클럭 주파수다.
"""
import re
import sys
import numpy as np
sys.path.insert(0, __file__.rsplit('/', 3)[0] + '/software/dvs_develop')
from throughput import per_frame


def main(trace, dvs):
    r = per_frame(dvs)
    fc, cyc = [], []
    for line in open(trace):
        m = re.match(r'frame (\d+) at cycle (\d+)', line)
        if m:
            fc.append(int(m[1]))
            cyc.append(int(m[2]))
    fr = np.array(fc[:-1])                      # 경계 k 뒤에 오는 프레임 = 0 부터 센 프레임 k
    c = np.diff(cyc)
    ev, dt, addr = r['events'][fr], r['dt'][fr], r['addr'][fr]
    ok = addr <= 1.1 * np.median(r['addr'])     # 열 훑기가 한 번보다 많은 프레임은 따로
    mhz = c / dt
    print(f'{r["name"]}: 프레임 {fr[0]} ~ {fr[-1]} ({len(fr)} 개), 클럭 {c.sum()}, 이벤트 {ev.sum()}, '
          f'이벤트당 클럭 {c.sum() / ev.sum():.4f}')
    i = int(np.argmax(np.where(ok, c, 0)))
    print(f'  가장 오래 걸린 프레임 {fr[i]}: {c[i]} 클럭, 이벤트 {ev[i]}, 간격 {dt[i]} µs')
    j = int(np.argmax(np.where(ok, mhz, 0)))
    print(f'  프레임을 간격 안에 끝내는 데 필요한 클럭 최대 {mhz[j]:.1f} MHz (프레임 {fr[j]})')
    for k in np.flatnonzero(~ok):
        print(f'  열 훑기가 한 번보다 많은 프레임 {fr[k]}: {c[k]} 클럭, 이벤트 {ev[k]}, '
              f'간격 {dt[k]} µs 로 나누면 {mhz[k]:.1f} MHz')


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
