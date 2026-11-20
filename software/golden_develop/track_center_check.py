#!/usr/bin/env python3
"""회전각을 어느 회전 중심으로 추적하느냐에 따라 막대 기울기가 구간마다 바뀌는지 본다.

    python3 software/golden_develop/track_center_check.py <녹화>.dvs <골든 묶음> <실험 묶음> <출력 폴더>

골든 묶음의 rotation.csv 는 track_rotation.py 가 회전각을 배경 중심(center.txt 의 bg_cx, bg_cy. 대비가 가장 큰
중심, 장면 1 에서 (500, 365))으로 추적한 것이고, 월드 메모리에는 막대 중심(center.txt 의 cx, cy. 장면 1 에서
(533, 418))을 쓴다. 회전각은 깊이와 상관없이 같아야 하지만, 추적 중심이 막대와 다르면 생기는 작은 각도 오차가
누적 구간에 따라 막대 여럿이 함께 기우는 모양으로 나타날 수 있다. 이 스크립트는 회전각을 막대 중심으로
다시 추적하고 위쪽 보정을 같은 방법으로 다시 구해 <실험 묶음>에 rotation.csv · center.txt 를 쓴다. 그 뒤 같은
폴더에 golden.py 와 verify_world.py 를 돌리면 골든 묶음과 똑같은 형식의 verify.txt 가 나와 구간별 기울기를
나란히 비교할 수 있다.

    python3 software/golden_develop/golden.py <녹화>.dvs <실험 묶음>
    python3 software/golden_develop/verify_world.py <녹화>.dvs <실험 묶음>

<출력 폴더>/track_center.txt 에는 두 회전각의 차이를 영상 회전 45° 마다 적는다.
"""
import os
import sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import track_rotation as tr
from golden import read_params
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'dvs_develop'))
from dvs_read import load


def main(path, bundle, exp, out):
    os.makedirs(exp, exist_ok=True)
    os.makedirs(out, exist_ok=True)
    d = load(path)
    p = read_params(os.path.join(bundle, 'center.txt'))
    c_bg, c = (p['bg_cx'], p['bg_cy']), (p['cx'], p['cy'])
    th_bg = np.radians(np.loadtxt(os.path.join(bundle, 'rotation.csv'), delimiter=',', skiprows=1)[:, 2])
    th = tr.track(d, c)

    # 위쪽: track_rotation.main 의 2) · 4) 와 같은 순서(REF 지도에서 막대 선, 전체 지도에서 기울기 평균)
    tr.set_events(d, th, *tr.REF, step=1)
    M = tr.world_map(c)
    lines = [tr.fit_line(M, np.add(a, tr.PAD), np.add(b, tr.PAD)) for a, b in tr.ROUGH.values()]
    lines = [(a - tr.PAD, b - tr.PAD) for a, b in lines]
    tr.set_events(d, th)
    M = tr.world_map(c)
    tilts = [tr.tilt_deg(tr.fit_line(M, np.add(a, tr.PAD), np.add(b, tr.PAD))) for a, b in lines]
    alpha = float(np.mean(tilts))

    rows = np.column_stack([np.arange(d['n_frames']), d['t_us'], np.degrees(th)])
    np.savetxt(os.path.join(exp, 'rotation.csv'), rows, delimiter=',', fmt=['%d', '%d', '%.4f'],
               header='frame,t_us,theta_deg', comments='')
    with open(os.path.join(exp, 'center.txt'), 'w') as fo:
        fo.write(f'# 회전각을 막대 중심으로 추적한 실험(track_center_check.py). 골든 묶음의 배경 중심은 {c_bg}\n'
                 f'cx = {c[0]:g}\ncy = {c[1]:g}\nalpha_deg = {alpha:.3f}\n')

    diff = np.degrees(th - th_bg)
    L = [f'# 회전각 추적 중심 비교. 골든: 배경 중심 {c_bg}, 실험: 막대 중심 {c}',
         f'누적 회전 골든 {np.degrees(th_bg[-1]):.2f}°, 실험 {np.degrees(th[-1]):.2f}°',
         f'실험의 위쪽 보정 alpha {alpha:.3f}° (막대 기울기 ' + ', '.join(f'{k} {v:.2f}' for k, v in zip(tr.ROUGH, tilts)) +
         f'), 골든 {p["alpha_deg"]:.3f}°',
         '영상 회전(골든)이 45° 넘어갈 때마다의 각도 차(실험 - 골든, 도)']
    deg = np.degrees(th_bg)
    for k in range(0, 9):
        f = int(np.flatnonzero(deg <= -45 * k)[0]) if (deg <= -45 * k).any() else d['n_frames'] - 1
        L.append(f'  프레임 {f:4d}  영상 회전 {deg[f]:8.1f}  차 {diff[f]:+.3f}')
    L.append(f'  전체에서 차의 범위 {diff[1:].min():+.3f} ~ {diff[1:].max():+.3f}')
    text = '\n'.join(L)
    print(text)
    with open(os.path.join(out, 'track_center.txt'), 'w') as fo:
        fo.write(text + '\n')


if __name__ == '__main__':
    main(*sys.argv[1:5])
