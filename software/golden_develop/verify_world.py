#!/usr/bin/env python3
"""골든 모델이 만든 장면 1 월드 메모리에서 수직 막대 4 개(A~D)가 곧게, 같은 자리에, 넓어지지 않고
쌓였는지와 고정소수점 · 칸 비트 수가 결과에 주는 영향을 숫자로 잰다.

    python3 software/golden_develop/verify_world.py <녹화>.dvs <묶음 폴더>

<묶음 폴더> 에 track_rotation.py 와 golden.py 를 먼저 돌려 둔다. 결과는 같은 폴더의
verify.txt 와 그림(world.png, segments.png, profiles.png, rotation.png)이다.

재는 것
  막대 선은 기울기(±5°, 0.1° 간격)와 가로 위치(±3 칸, 0.25 칸 간격)를 훑어, 가로 구조를 지운 지도에서 선을
  따라 읽은 값의 평균이 가장 큰 것으로 잡는다(best_line). 이벤트는 막대 가장자리에 몰리므로 이 선은 막대의
  한쪽 가장자리에 놓인다.
  비교 기준은 회전이 거의 없는 짧은 구간 REF(45 프레임, 영상 회전 2°)를 같은 골든 모델로 누적한 지도다.
  1. 막대가 곧은가: 짧은 구간과 전체 누적에서 막대 4 개가 수직에서 기운 각도.
  2. 막대가 같은 자리에 쌓이나: 녹화를 영상 회전 45° 마다 구간으로 나눠 구간마다 지도를 만들고, 전체 지도에서
     잡은 선 둘레 ±2 칸에서 다시 선을 잡아 가로 위치가 어긋난 양과 구간의 기울기를 잰다. 막대 반대쪽 가장자리로
     건너뛰지 않게 막대 폭(2~8 칸)보다 좁게 찾는다. 어긋난 양이 ±8 px 이면 찾는 범위 끝에 걸린 것이다.
  3. 막대가 넓어지지 않나: 짧은 구간과 전체 누적에서 선이 놓인 가장자리 봉우리의 반치폭을 잰다. 단면(선 방향
     평균)에서 바깥 띠(±6~10 칸)의 중앙값을 바닥으로 빼고, 봉우리에서 이어지며 봉우리 절반 이상인 칸 수다.
  4. 고정소수점: 이벤트마다 정수 모델이 낸 월드 좌표(시프트 전 u, v 를 px 로)와 실수 계산(양자화 안 한 ψ,
     실수 cos · sin)의 차이, 그리고 칸이 달라진 이벤트 비율. TRIG_F 를 12 · 13 · 14 로 바꿔 가며 잰다.
  5. 칸 비트 수: 전체 누적을 B 비트 포화 카운터에 담았을 때 포화하는 칸과 잘려 나가는 이벤트 비율.
"""
import os
import sys
from math import atan, pi, sqrt, prod
import numpy as np
from scipy import ndimage
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import golden as g
from track_rotation import ROUGH, REF, fit_line, tilt_deg, profile
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'dvs_develop'))
from dvs_read import load

CELL = 1 << g.CELL_SHIFT
HALF = 10                  # 단면 반폭(칸)
SEG_DEG = 45.0             # 구간 하나의 영상 회전량


def to_cell(p, psi_deg, center):
    """영상 좌표 p(px) 를 회전각 ψ 에서의 월드 칸 좌표(실수)로."""
    a = np.radians(psi_deg)
    dx, dy = p[0] - center[0], p[1] - center[1]
    return np.array([np.cos(a) * dx - np.sin(a) * dy + g.WC, np.sin(a) * dx + np.cos(a) * dy + g.WC]) / CELL


def excess(prof):
    """단면에서 바깥 띠의 중앙값을 바닥으로 뺀 값."""
    edge = np.r_[prof[:4], prof[-4:]]
    return prof - np.median(edge)


def edge_width_px(prof):
    """단면 가운데(선이 놓인 가장자리) ±1 칸에서 가장 높은 봉우리의 반치폭(px)."""
    e = excess(prof)
    i = HALF - 1 + int(np.argmax(e[HALF - 1:HALF + 2]))
    lo = hi = i
    while lo > 0 and e[lo - 1] >= e[i] / 2:
        lo -= 1
    while hi < len(e) - 1 and e[hi + 1] >= e[i] / 2:
        hi += 1
    return float((hi - lo + 1) * CELL)


def vertical_edges(M):
    """가로로 9 칸 이동 평균을 뺀 지도. 가로로 뻗은 구조(책상 · 모니터 윗변)는 지워지고 세로 가장자리만 남는다."""
    return M - ndimage.uniform_filter1d(M, 9, axis=1)


def best_line(M, line, x0=None, tilts=np.arange(-5, 5.001, 0.1), offs=np.arange(-3, 3.001, 0.25)):
    """막대 선을 (기울기, 가로 위치)로 훑어 선을 따라 읽은 vertical_edges(M) 의 평균이 가장 큰 선을 고른다.

    선은 line 의 가운데 높이 y_m 을 지나고 길이도 line 과 같다. 가로 위치는 y_m 에서 선의 x(칸)다. x0 를 주면
    그 둘레 offs 만, 안 주면 line 의 가운데 x 둘레 offs 를 본다. 막대의 두 가장자리는 나란하므로 어느 쪽을
    잡아도 기울기는 같다. 돌려주는 값: (가로 위치 칸, 기울기 도, 고른 선).
    """
    p, q = (np.asarray(v, float) for v in line)
    m = (p + q) / 2
    L = np.linalg.norm(q - p)
    xs = (m[0] if x0 is None else x0) + offs
    s = np.arange(-L / 2, L / 2, 0.5)
    t = np.radians(tilts)
    X = xs[None, :, None] + np.sin(t)[:, None, None] * s[None, None, :]
    Y = np.broadcast_to(m[1] + np.cos(t)[:, None, None] * s[None, None, :], X.shape)
    v = ndimage.map_coordinates(vertical_edges(M), [Y.ravel(), X.ravel()], order=1).reshape(X.shape).mean(2)
    i, j = np.unravel_index(int(np.argmax(v)), v.shape)
    u = np.array([np.sin(t[i]), np.cos(t[i])])
    c = np.array([xs[j], m[1]])
    return float(xs[j]), float(tilts[i]), (c - u * L / 2, c + u * L / 2)


def set_trig(F):
    """CORDIC 출력 소수 비트를 F 로 바꾼다(반복 횟수 F + 2, 보호 비트는 그대로). 고정소수점 비교에만 쓴다."""
    g.TRIG_F, g.CORDIC_IT = F, F + 2
    g._ATAN = [round(atan(2.0 ** -i) / (2 * pi) * (1 << (g.PHASE_BITS + g.GZ))) for i in range(g.CORDIC_IT)]
    g._X0 = round(prod(1 / sqrt(1 + 2.0 ** (-2 * i)) for i in range(g.CORDIC_IT)) * (1 << (F + g.GX)) / sqrt(2))


def fixed_vs_float(d, psi, center, step):
    """이벤트(step 개 중 하나)마다 정수 좌표와 실수 좌표의 최대 차이(px), 칸이 달라진 비율."""
    ph = g.to_phase(psi)
    c, s = g.cordic(ph)
    a = np.radians(psi)
    worst, diff, n = 0.0, 0, 0
    for lo in range(0, len(d['frame']), 1 << 23):
        sl = slice(lo, lo + (1 << 23), step)
        f = d['frame'][sl]
        dx = d['x'][sl].astype(np.int64) - center[0]
        dy = d['y'][sl].astype(np.int64) - center[1]
        u = (c[f] * dx - s[f] * dy) / (1 << g.TRIG_F)
        v = (s[f] * dx + c[f] * dy) / (1 << g.TRIG_F)
        uf = np.cos(a[f]) * dx - np.sin(a[f]) * dy
        vf = np.sin(a[f]) * dx + np.cos(a[f]) * dy
        worst = max(worst, float(np.abs(u - uf).max()), float(np.abs(v - vf).max()))
        wx, wy = g.cells(d['x'][sl], d['y'][sl], c[f], s[f], *center)
        diff += int(((wx != np.floor((uf + g.WC) / CELL)) | (wy != np.floor((vf + g.WC) / CELL))).sum())
        n += len(f)
    return worst, diff / n


def main(path, bundle):
    d = load(path)
    psi, ph, c, s, center = g.frame_trig(bundle)
    theta = np.loadtxt(os.path.join(bundle, 'rotation.csv'), delimiter=',', skiprows=1)[:, 2]
    W = np.load(os.path.join(bundle, 'world.npy')).astype(np.float64)
    out = []

    # 막대 선: REF 가운데 프레임의 ψ 로 ROUGH 를 칸 좌표로 옮긴 뒤 전체 지도에서 다듬는다
    psi_ref = psi[(REF[0] + REF[1]) // 2]
    lines = {k: fit_line(W, to_cell(p, psi_ref, center), to_cell(q, psi_ref, center), band=3)
             for k, (p, q) in ROUGH.items()}

    # 1. 짧은 구간과 전체 지도에서 막대 기울기
    R_ = g.run(d, (ph, c, s), center, *REF)[0].astype(np.float64)
    ref = {k: best_line(R_, l) for k, l in lines.items()}
    full = {k: best_line(W, l) for k, l in lines.items()}
    lines = {k: v[2] for k, v in full.items()}
    out.append(f'[1] 막대 기울기(도, 수직이 0, 아래로 갈수록 오른쪽이 +): 짧은 구간(프레임 {REF[0]}~{REF[1] - 1})'
               ' / 전체 누적')
    out.append('    ' + '  '.join(f'{k} {ref[k][1]:+.1f} / {full[k][1]:+.1f}' for k in ROUGH))

    # 2. 45° 구간마다 위치 · 기울기
    edges = [1]
    while True:
        nxt = np.flatnonzero(theta <= theta[edges[-1]] - SEG_DEG)
        nxt = nxt[nxt > edges[-1]]
        if not len(nxt):
            break
        edges.append(int(nxt[0]))
    edges.append(d['n_frames'])
    segs = []
    out.append(f'[2] 영상 회전 {SEG_DEG:g}° 구간마다 막대가 전체 지도에서 옆으로 어긋난 양(px, 오른쪽 +)과'
               ' 구간 기울기(도, 아래로 갈수록 오른쪽 +)')
    out.append('    구간 프레임        영상 회전(도)       ' + '  '.join(f'{k:>16}' for k in ROUGH))
    shifts = {k: [] for k in ROUGH}
    for f0, f1 in zip(edges[:-1], edges[1:]):
        S_, _, n = g.run(d, (ph, c, s), center, f0, f1)
        S_ = S_.astype(np.float64)
        segs.append((f0, f1, S_))
        cols = []
        for k, l in lines.items():
            x, tl, _ = best_line(S_, l, full[k][0], offs=np.arange(-2, 2.001, 0.25))
            sh = (x - full[k][0]) * CELL
            shifts[k].append(sh)
            cols.append(f'{sh:+5.1f} px {tl:+5.1f}°')
        out.append(f'    {f0:4d}~{f1 - 1:4d}  {theta[f0]:8.1f}~{theta[f1 - 1]:8.1f}  ' + '  '.join(cols))
    out.append('    구간끼리 어긋난 양의 범위(최대 - 최소, px): ' +
               '  '.join(f'{k} {np.ptp(v):.1f}' for k, v in shifts.items()))

    # 3. 짧은 구간과 전체의 막대 가장자리 폭
    out.append(f'[3] 막대 가장자리 반치폭(px): 짧은 구간 / 전체 누적. 두 지도의 가로 위치 차(px, 오른쪽 +)')
    out.append('    ' + '  '.join(f'{k} {edge_width_px(profile(R_, ref[k][2], HALF)):.0f} / '
                                  f'{edge_width_px(profile(W, full[k][2], HALF)):.0f}' for k in ROUGH))
    out.append('    ' + '  '.join(f'{k} {(full[k][0] - ref[k][0]) * CELL:+.0f}' for k in ROUGH))

    # 4. 고정소수점과 실수
    out.append('[4] 고정소수점 월드 좌표와 실수 좌표의 최대 차이(px), 칸이 달라진 이벤트 비율 (이벤트 8 개 중 하나)')
    keep = (g.TRIG_F, g.CORDIC_IT, g._ATAN, g._X0)
    for F in (12, 13, 14):
        set_trig(F)
        worst, frac = fixed_vs_float(d, psi, center, 8)
        out.append(f'    TRIG_F {F:2d} (반복 {g.CORDIC_IT}): 최대 {worst:.3f} px, 칸이 다른 이벤트 {frac * 100:.3f} %')
    g.TRIG_F, g.CORDIC_IT, g._ATAN, g._X0 = keep

    # 5. 칸 비트 수
    out.append(f'[5] 전체 누적({int(W.sum())} 이벤트, 이벤트가 든 칸 {int((W > 0).sum())})을 B 비트 포화 카운터에 담으면')
    for B in (8, 10, 12, 14, 16):
        top = (1 << B) - 1
        out.append(f'    B {B:2d}: 포화 칸 {int((W >= top).sum()):6d} ({(W >= top).sum() / (W > 0).sum() * 100:5.2f} %),'
                   f' 잘리는 이벤트 {np.maximum(W - top, 0).sum() / W.sum() * 100:5.2f} %')

    text = '\n'.join(out)
    print(text)
    with open(os.path.join(bundle, 'verify.txt'), 'w') as fo:
        fo.write(text + '\n')

    # 그림
    def show(ax, M, title):
        L = np.log1p(M)
        ax.imshow(L, cmap='gray_r', vmax=np.percentile(L[M > 0], 99.5) if (M > 0).any() else 1)
        ax.set_title(title, fontsize=9)
        ax.set_xticks([])
        ax.set_yticks([])

    fig, ax = plt.subplots(figsize=(9, 9))
    show(ax, W, f'world memory {g.N}x{g.M}, {CELL} px cells, all {d["n_frames"]} frames')
    for k, (p, q) in lines.items():
        ax.plot([p[0], q[0]], [p[1], q[1]], color='#eb6834', lw=0.8, alpha=0.8)
        ax.annotate(k, q, color='#eb6834', fontsize=11, xytext=(0, 8), textcoords='offset points', ha='center')
    fig.tight_layout()
    fig.savefig(os.path.join(bundle, 'world.png'), dpi=90)

    n = len(segs)
    cols = 5
    fig, axs = plt.subplots((n + cols - 1) // cols, cols, figsize=(3.2 * cols, 3.3 * ((n + cols - 1) // cols)))
    for ax, (f0, f1, S_) in zip(axs.ravel(), segs):
        show(ax, S_, f'frames {f0}-{f1 - 1}, image {theta[f0]:.0f}..{theta[f1 - 1]:.0f} deg')
        for p, q in lines.values():
            ax.plot([p[0], q[0]], [p[1], q[1]], color='#eb6834', lw=0.6, alpha=0.6)
    for ax in axs.ravel()[n:]:
        ax.axis('off')
    fig.tight_layout()
    fig.savefig(os.path.join(bundle, 'segments.png'), dpi=70)

    fig, axs = plt.subplots(1, 4, figsize=(15, 3.2))
    x = -np.arange(-HALF, HALF + 1) * CELL        # 단면 축이 왼쪽 법선이라 뒤집어 오른쪽을 + 로
    for ax, (k, l) in zip(axs, lines.items()):
        for M_, lab, col in ((R_, f'frames {REF[0]}-{REF[1] - 1}', '#2a78d6'), (W, 'all frames', '#eb6834')):
            pr = excess(profile(M_, l, HALF))
            ax.plot(x, pr / pr.max(), color=col, lw=2, label=lab)
        ax.set_title(f'post {k}', fontsize=10)
        ax.set_xlabel('across post (px)')
        ax.grid(alpha=0.3)
    axs[0].set_ylabel('profile minus floor, / max')
    axs[0].legend(frameon=False, fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(bundle, 'profiles.png'), dpi=80)

    t = np.loadtxt(os.path.join(bundle, 'rotation.csv'), delimiter=',', skiprows=1)[:, 1]
    tm = (t - t[1]) / 1e3
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 4))
    a1.plot(tm[1:], theta[1:], color='#2a78d6', lw=2)
    a1.set_xlabel('time from 2nd frame (ms)')
    a1.set_ylabel('image rotation theta (deg, clockwise +)')
    a1.grid(alpha=0.3)
    a2.plot(tm[2:], np.diff(theta[1:]) / np.diff(tm[1:]), color='#2a78d6', lw=0.8)
    a2.set_xlabel('time from 2nd frame (ms)')
    a2.set_ylabel('rotation rate (deg/ms)')
    a2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(bundle, 'rotation.png'), dpi=80)


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
