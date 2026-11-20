#!/usr/bin/env python3
"""월드 칸 크기(1 · 2 · 4 · 8 px)에 따라 막대가 얼마나 날카롭게 쌓이는지와 메모리가 얼마나 드는지 나란히 잰다.

    python3 software/golden_develop/cell_size_check.py <녹화>.dvs <골든 묶음> <출력 폴더>

<골든 묶음> 의 rotation.csv · center.txt(track_rotation.py 출력)를 그대로 쓰고, 정수 골든 모델(golden.py)에서
칸 크기만 바꾼다. 메모리 한 변은 칸 크기와 상관없이 1408 px(4 px 칸 352 개와 같은 넓이)이고, 칸 카운터는
자르지 않게 24 비트로 센 뒤 필요한 비트 수(칸 최댓값을 담는 비트)를 따로 계산한다. 칸 수는 칸 크기의 제곱에
반비례하므로, 막대 측정과 메모리 크기를 한 표에 놓고 칸 크기를 고른다.

출력
  <폴더>/cell_size.txt : 칸 크기별 메모리 크기 · 칸 최댓값 · 필요한 비트 · 14 비트일 때 포화, 막대 4 개의
                         기울기 · 위치 · 가장자리 반치폭 · 봉우리/바닥
  <폴더>/cell_size.png : 칸 크기별 전체 지도와 막대 C 둘레 확대

시작할 때 4 px 칸에서 world_metrics 의 측정이 verify_world.py 와 같은 값을 내는지 확인한다.
"""
import os
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import golden as g
import verify_world as vw
import world_metrics as wm
from track_rotation import REF, profile
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'dvs_develop'))
from dvs_read import load

SHIFTS = (0, 1, 2, 3)      # 칸 1 · 2 · 4 · 8 px
DESIGN_BITS = 14           # 메모리 크기 · 포화 칸 · 잘리는 이벤트를 따로 셀 때 견주는 칸 비트 수


def self_check(W, R_, psi_ref, center):
    """4 px 칸에서 world_metrics 와 verify_world 가 같은 기울기 · 위치 · 반치폭을 내는지."""
    m = wm.post_metrics(W, R_, psi_ref, center)
    lines = {k: vw.fit_line(W, vw.to_cell(p, psi_ref, center), vw.to_cell(q, psi_ref, center), band=3)
             for k, (p, q) in vw.ROUGH.items()}
    for k, l in lines.items():
        xr, tr, lr = vw.best_line(R_, l)
        xf, tf, lf = vw.best_line(W, l)
        want = (tr, tf, (xf - xr) * vw.CELL, vw.edge_width_px(profile(R_, lr, vw.HALF)),
                vw.edge_width_px(profile(W, lf, vw.HALF)))
        got = (m[k]['tilt_ref'], m[k]['tilt_full'], m[k]['shift_px'], m[k]['width_ref'], m[k]['width_full'])
        assert np.allclose(want, got), (k, want, got)


def main(path, bundle, out):
    os.makedirs(out, exist_ok=True)
    d = load(path)
    psi, ph, c, s, center = g.frame_trig(bundle)
    psi_ref = psi[(REF[0] + REF[1]) // 2]
    rows, maps = [], {}
    for sh in SHIFTS:
        wm.set_cell(sh)
        W, n_out, n_ev = g.run(d, (ph, c, s), center)
        R_ = g.run(d, (ph, c, s), center, *REF)[0]
        W, R_ = W.astype(np.float64), R_.astype(np.float64)
        if sh == 2:
            self_check(W, R_, psi_ref, center)
        m = wm.post_metrics(W, R_, psi_ref, center)
        top = int(W.max())
        bits = int(np.ceil(np.log2(top + 1)))
        sat = W >= (1 << DESIGN_BITS) - 1
        rows.append(dict(cell=wm.CELL, n=g.N, top=top, bits=bits, n_out=n_out, used=int((W > 0).sum()),
                         sat_cells=int(sat.sum()), cut=float(np.maximum(W - ((1 << DESIGN_BITS) - 1), 0).sum() / W.sum()),
                         m=m))
        maps[wm.CELL] = (W, m)
        print(f'칸 {wm.CELL} px: 메모리 {g.N} x {g.M}, 칸 최댓값 {top}, 범위 밖 {n_out}', flush=True)

    keys = list(rows[0]['m'])
    lines = ['# 칸 크기별 월드 메모리. 장면 1 전체 누적, 메모리 한 변 1408 px, 회전 중심 '
             f'{center}. 막대 측정은 world_metrics.py(4 px 칸에서 verify_world.py 와 같음)',
             '', '[1] 메모리',
             '    칸(px)  칸 수(한 변)  칸 수(전체)  이벤트 든 칸  칸 최댓값  필요 비트  메모리(필요 비트)    '
             f'메모리({DESIGN_BITS}비트)   {DESIGN_BITS}비트 포화 칸  잘리는 이벤트']
    for r in rows:
        tot = r['n'] * r['n']
        lines.append(f'    {r["cell"]:6d}  {r["n"]:12d}  {tot:11d}  {r["used"]:12d}  {r["top"]:9d}  {r["bits"]:9d}  '
                     f'{tot * r["bits"]:12d} bit  {tot * DESIGN_BITS:12d} bit  {r["sat_cells"]:12d}  {r["cut"] * 100:10.3f} %')
    lines += ['', '[2] 막대 기울기(도): 짧은 구간 / 전체',
              '    칸(px)  ' + '  '.join(f'{k:>15}' for k in keys)]
    for r in rows:
        lines.append(f'    {r["cell"]:6d}  ' + '  '.join(f'{r["m"][k]["tilt_ref"]:+6.1f} / {r["m"][k]["tilt_full"]:+6.1f}'
                                                    for k in keys))
    lines += ['', '[3] 막대 가로 위치 차(전체 - 짧은 구간, px)',
              '    칸(px)  ' + '  '.join(f'{k:>8}' for k in keys)]
    for r in rows:
        lines.append(f'    {r["cell"]:6d}  ' + '  '.join(f'{r["m"][k]["shift_px"]:+8.1f}' for k in keys))
    lines += ['', '[4] 막대 가장자리 반치폭(px): 짧은 구간 / 전체',
              '    칸(px)  ' + '  '.join(f'{k:>11}' for k in keys)]
    for r in rows:
        lines.append(f'    {r["cell"]:6d}  ' + '  '.join(f'{r["m"][k]["width_ref"]:4.0f} / {r["m"][k]["width_full"]:4.0f}'
                                                    for k in keys))
    lines += ['', '[5] 막대 가장자리 봉우리 높이 / 바닥(바닥 = 단면 양 끝 16 px 중앙값): 짧은 구간 / 전체',
              '    칸(px)  ' + '  '.join(f'{k:>13}' for k in keys)]
    for r in rows:
        lines.append(f'    {r["cell"]:6d}  ' + '  '.join(f'{r["m"][k]["contrast_ref"]:5.2f} / {r["m"][k]["contrast_full"]:5.2f}'
                                                    for k in keys))
    text = '\n'.join(lines)
    print(text)
    with open(os.path.join(out, 'cell_size.txt'), 'w') as fo:
        fo.write(text + '\n')

    # 그림: 위는 전체 지도, 아래는 막대 C 가운데 둘레 160 px
    fig, axs = plt.subplots(2, len(SHIFTS), figsize=(4 * len(SHIFTS), 8.2))
    for j, sh in enumerate(SHIFTS):
        cell = 1 << sh
        W, m = maps[cell]
        L = np.log1p(W)
        vmax = np.percentile(L[W > 0], 99.5)
        axs[0, j].imshow(L, cmap='gray_r', vmax=vmax)
        axs[0, j].set_title(f'{cell} px cells, {W.shape[1]}x{W.shape[0]}', fontsize=10)
        p, q = m['C']['line']
        cx, cy = (p + q) / 2
        r = 80 / cell
        axs[1, j].imshow(L, cmap='gray_r', vmax=vmax, extent=(-0.5, W.shape[1] - 0.5, W.shape[0] - 0.5, -0.5))
        axs[1, j].set_xlim(cx - r, cx + r)
        axs[1, j].set_ylim(cy + r, cy - r)
        axs[1, j].set_title(f'post C, 160 px square', fontsize=10)
        for a in axs[:, j]:
            a.set_xticks([])
            a.set_yticks([])
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'cell_size.png'), dpi=80)


if __name__ == '__main__':
    main(*sys.argv[1:4])
