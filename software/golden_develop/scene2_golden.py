#!/usr/bin/env python3
"""장면 2 골든 모델 묶음: roll 칩에 그대로 넣을 장면 2 설정값(고정 회전 중심 (cx, cy) 와 위쪽 보정 α)을 고른다.

    python3 software/golden_develop/scene2_golden.py <장면 2.dvs> <장면 2 회전각 묶음> <출력 묶음>
    python3 software/golden_develop/golden.py <장면 2.dvs> <출력 묶음> 600 640

칩은 고정 회전 중심 (cx, cy) 하나와 프레임별 위상만 받는다. 위상은 α - θ_k(θ_k 는 프레임 k 의 영상 회전각)라서 α 는
위상에 섞여 들어간다. 이 스크립트는 장면 1 과 같은 방법으로 막대 4 개가 가장 날카로워지는 고정 중심과 α 를 고르고,
golden.py 가 읽는 형식(rotation.csv · center.txt)으로 써서 장면 2 도 같은 정수 모델 · 같은 회로로 돌릴 수 있게 한다.
<장면 2 회전각 묶음> 은 scene2_check.py 의 출력 폴더(2026-10-28_scene2)이고, 그 rotation.csv(중심 (478, 385) 로 추적한
프레임별 roll)를 회전각으로 쓴다.

  1) 막대 선: 녹화 앞쪽의 거의 안 돈 구간 REF2 를 배경 중심 C_BG 로 쌓은 1 px 지도에서, 눈으로 고른 막대 양 끝
     ROUGH2 를 fit_line 으로 다듬는다.
  2) 막대 중심: 회전각을 고정하고, 후보 중심마다 막대 4 개의 단면 날카로움을 재어 막대마다 최댓값으로 나눈 합이 가장 큰
     중심을 고른다(track_rotation.py 의 막대 중심 고르기와 같은 방법). 후보는 C_BG 둘레 ±120 px 를 12 px 간격, 다시
     그 최선 둘레 ±12 px 를 3 px 간격으로 훑는다. 넓은 격자에서는 막대마다 따로 가장 날카로운 중심도 기록한다.
  3) 위쪽 보정 α: 고른 중심으로 쌓은 전체 지도에서 잰 막대 4 개 기울기의 평균. 그 크기가 ALPHA_MIN 보다 작으면 0 으로 둔다.
  4) 막대 지표(verify_world 와 같은 함수, 골든 정수 모델의 4 px 칸 지도)를 배경 중심 C_BG 와 고른 막대 중심 각각으로 잰다.
  5) golden.py 가 읽는 rotation.csv · center.txt 와, 1) ~ 4) 의 수치를 담은 verify.txt 를 <출력 묶음> 에 쓴다.
"""
import os
import sys
import multiprocessing as mp
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import golden as g
import track_rotation as tr
import verify_world as vw
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'dvs_develop'))
from dvs_read import load

# 배경 중심: 장면 2 녹화 전체를 회전 중심 하나로 쌓을 때 1 px 지도의 대비 Σ M² / (Σ M)² 가 가장 큰 점.
# scene2_check.py 가 같은 회전각으로 찾은 값이다. 막대 중심 탐색의 출발점이자 막대 지표의 비교 기준이다.
C_BG = (484, 367)
REF2 = (1, 250)         # 프레임 1 ~ 249. 장면 2 녹화 앞쪽의 거의 안 돈 구간(영상 회전 0.01° 안)
# REF2 구간 지도에서 눈으로 고른 막대 양 끝 (x, y) px. 왼쪽부터 A ~ D. 장면 1 과 같은 막대이고 B 가 짧다.
ROUGH2 = {'A': ((152, 50), (150, 640)), 'B': ((298, 350), (300, 700)),
          'C': ((555, 5), (553, 700)), 'D': ((780, 135), (778, 700))}
ALPHA_MIN = 0.3         # REF2 에서 막대 4 개가 이 각도 안으로 서 있다. 0.2° 만 돌려도 짧은 막대 B 의 반치폭이 칸 하나 넘게 흔들린다
SEG_DEG = 45.0          # 막대 지표 [2] 에서 녹화를 끊는 회전 구간(도)


def bar_metrics(d, theta, cen, alpha):
    """중심 cen 과 위쪽 보정 alpha 로 정수 골든 모델(golden.run)의 지도를 만들고 verify_world 와 같은 막대 지표를 잰다.

    [1] 막대 기울기: REF2 구간 지도와 전체 지도에서 각각.
    [2] 녹화를 SEG_DEG 만큼 돌 때마다 끊은 구간 지도마다 막대가 전체 지도의 막대 선에서 가로로 어긋난 양(px)과,
        그 범위. 탐색 범위 ±2 칸(±8 px) 끝에 걸린 값의 개수도 센다.
    [3] 막대 가장자리 반치폭(px): REF2 구간 지도와 전체 지도에서 각각.
    그리고 전체 지도의 포화 칸 수. 돌려주는 값: (글 줄 목록, 막대별 전체 지도 반치폭 dict).
    """
    ph = g.to_phase(alpha - np.degrees(theta))
    c, s = g.cordic(ph)
    W = g.run(d, (ph, c, s), cen)[0].astype(np.float64)
    R_ = g.run(d, (ph, c, s), cen, *REF2)[0].astype(np.float64)
    fr = (REF2[0] + REF2[1]) // 2
    psi_ref = alpha - np.degrees(theta[fr])
    lines = {k: tr.fit_line(R_, vw.to_cell(p, psi_ref, cen), vw.to_cell(q, psi_ref, cen), band=3)
             for k, (p, q) in ROUGH2.items()}
    ref = {k: vw.best_line(R_, l) for k, l in lines.items()}
    full = {k: vw.best_line(W, l) for k, l in lines.items()}
    edges = [1]
    while True:
        nxt = np.flatnonzero(np.abs(theta - theta[edges[-1]]) >= np.radians(SEG_DEG))
        nxt = nxt[nxt > edges[-1]]
        if not len(nxt):
            break
        edges.append(int(nxt[0]))
    edges.append(d['n_frames'])
    shifts = {k: [] for k in ROUGH2}
    for f0, f1 in zip(edges[:-1], edges[1:]):
        S_ = g.run(d, (ph, c, s), cen, f0, f1)[0].astype(np.float64)
        for k in ROUGH2:
            x, _, _ = vw.best_line(S_, full[k][2], full[k][0], offs=np.arange(-2, 2.001, 0.25))
            shifts[k].append((x - full[k][0]) * vw.CELL)
    width = {k: vw.edge_width_px(tr.profile(W, full[k][2], vw.HALF)) for k in ROUGH2}
    at_end = sum(int(np.sum(np.abs(v) >= 8)) for v in shifts.values())
    out = ['    [1] 기울기(도) REF / 전체: ' + '  '.join(f'{k} {ref[k][1]:+.1f} / {full[k][1]:+.1f}' for k in ROUGH2),
           f'    [2] 45° 구간 {len(edges) - 1} 개, 구간끼리 어긋난 양의 범위(px): ' +
           '  '.join(f'{k} {np.ptp(v):.1f}' for k, v in shifts.items()) + f'  (±8 px 끝에 걸린 값 {at_end} 개)',
           '    [3] 가장자리 반치폭(px) REF / 전체: ' +
           '  '.join(f'{k} {vw.edge_width_px(tr.profile(R_, ref[k][2], vw.HALF)):.0f} / {width[k]:.0f}' for k in ROUGH2),
           f'    포화 칸 {int((W >= (1 << g.CELL_BITS) - 1).sum())}']
    return out, width


def main(path, rot_bundle, out):
    os.makedirs(out, exist_ok=True)
    ctx = mp.get_context('fork')
    d = load(path)
    nf = d['n_frames']
    rot = np.loadtxt(os.path.join(rot_bundle, 'rotation.csv'), delimiter=',', skiprows=1)
    theta = np.radians(rot[:, 2])
    assert len(theta) == nf

    # 1) 막대 선
    tr.set_events(d, theta, *REF2, step=1)
    M = tr.world_map(C_BG)
    lines = [tr.fit_line(M, np.add(p, tr.PAD), np.add(q, tr.PAD)) for p, q in ROUGH2.values()]
    lines = [(p - tr.PAD, q - tr.PAD) for p, q in lines]

    # 2) 막대 중심
    tr.set_events(d, theta)
    tr._ev['lines'] = lines
    c = C_BG
    per_bar = None
    with ctx.Pool(5) as pool:
        for span, step in ((120, 12), (12, 3)):
            cand, sc = tr.grid(tr._post_sharp, c, span, step, pool)
            sc = np.asarray(sc)
            if per_bar is None:         # 넓은 격자에서 막대마다 따로 가장 날카로운 중심(깊이마다 중심이 다른지 본다)
                per_bar = [cand[int(np.nanargmax(sc[:, i]))] for i in range(len(lines))]
            c = cand[int(np.argmax((sc / sc.max(0)).sum(1)))]
    c = (int(c[0]), int(c[1]))

    # 3) 위쪽 보정
    M = tr.world_map(c)
    tilts = [tr.tilt_deg(tr.fit_line(M, np.add(p, tr.PAD), np.add(q, tr.PAD))) for p, q in lines]
    alpha = float(np.mean(tilts))
    alpha = alpha if abs(alpha) >= ALPHA_MIN else 0.0
    tr._ev.clear()

    # 4) 막대 지표: 배경 중심 대 막대 중심
    txt = [f'장면 2 ({os.path.basename(path)}), 프레임 {nf}, 영상 회전 {np.degrees(theta[-1]):.1f}° '
           f'(회전각은 {os.path.basename(os.path.normpath(rot_bundle))}/rotation.csv, 추적 중심 (478, 385))',
           f'막대 선(1 px, REF 구간 {REF2[0]}~{REF2[1] - 1}): ' +
           '  '.join(f'{k} 기울기 {tr.tilt_deg(l):+.2f}°' for k, l in zip(ROUGH2, lines)),
           '막대마다 따로 가장 날카로운 중심(±120 px 를 12 px 간격): ' +
           '  '.join(f'{k} ({p[0]:.0f}, {p[1]:.0f})' for k, p in zip(ROUGH2, per_bar)),
           f'네 막대를 함께 본 막대 중심 {c}. 전체 지도 막대 기울기 ' +
           ', '.join(f'{k} {v:.2f}' for k, v in zip(ROUGH2, tilts)) + f', 평균 {np.mean(tilts):.3f}° → α = {alpha:g}°', '',
           '막대 지표(verify_world.py 와 같은 함수, 4 px 칸 정수 지도)']
    for name, cc in ((f'배경 중심 {C_BG}', C_BG), (f'막대 중심 {c} (고름)', c)):
        t, w = bar_metrics(d, theta, cc, alpha)
        txt += [f'  {name}'] + t + [f'    반치폭 합 {sum(w.values()):.0f} px']

    # 5) golden.py 가 읽는 파일
    np.savetxt(os.path.join(out, 'rotation.csv'), rot, delimiter=',', fmt=['%d', '%d', '%.4f'],
               header='frame,t_us,theta_deg', comments='')
    with open(os.path.join(out, 'center.txt'), 'w') as fo:
        fo.write(f'# 장면 2 설정값(scene2_golden.py). 회전각은 {os.path.basename(os.path.normpath(rot_bundle))}/rotation.csv 와 같다\n'
                 f'cx = {c[0]}\ncy = {c[1]}\nalpha_deg = {alpha:.3f}\n')
    with open(os.path.join(out, 'verify.txt'), 'w', encoding='utf-8') as fo:
        fo.write('\n'.join(txt) + '\n')
    print('\n'.join(txt))


if __name__ == '__main__':
    main(*sys.argv[1:4])
