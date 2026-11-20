#!/usr/bin/env python3
"""장면 2(회전판 위에서 봉을 통째로 돌린 녹화)에 회전 하나의 모형이 어디까지 맞는지 잰다.

    python3 software/golden_develop/scene2_check.py <장면 2.dvs> <장면 1.dvs> <장면 1 골든 묶음> <출력 폴더>

장면 1 은 센서가 봉 둘레를 돌 뿐이라 영상의 회전 중심이 거의 한 곳이다. 장면 2 는 그 봉을 큰 회전판에 올려 통째로
돌렸으므로 카메라가 원을 그리며 옮겨 가고, 영상에는 회전에 이동이 섞인다. 고정 회전 중심 하나와 프레임별 회전각만으로
쌓는 모형(칩이 쓰는 roll 모형)은 이동을 담지 못한다. 얼마나 못 담는지를 이렇게 잰다.

  1) 회전각: track_rotation.track 으로 장면 2 의 프레임별 roll 을 추정한다. 중심은 장면 2 에서 프레임 쌍마다 잰 회전
     중심의 중앙값 C2 = (478, 385) 로 고정한다. <출력 폴더>/rotation.csv 가 이미 있으면 추적하지 않고 그것을 읽는다.
  2) 창별 회전 중심: 녹화를 WIN 프레임씩 끊고, 창마다 회전각을 고정한 채 1 px 월드 지도의 대비
     Σ M² / (Σ M)² 가 가장 큰 중심을 찾는다(±60 px 를 6 px 간격, 다시 ±6 px 를 1 px 간격). 이벤트는 4 개 중 하나만 쓴다.
     창마다 중심이 얼마나 움직이는지가 "회전 하나"로 맞지 않는 정도다. 중심이 δ 어긋나면 각도 Δθ 사이에서 같은 점이
     2 · δ · sin(Δθ / 2) 만큼 어긋난다(반 바퀴면 2δ).
  3) 같은 일을 장면 1 에도 해서 견준다. 장면 1 은 골든 묶음의 rotation.csv 회전각을 그대로 쓰고, 중심 탐색은 그 묶음
     center.txt 의 배경 중심 (bg_cx, bg_cy) 에서 시작한다.
  4) 창끼리의 어긋남: 녹화 전체에서 대비가 가장 큰 중심 하나로 창마다 월드 지도를 만들고, 이웃한 두 창의 지도를
     상호상관해 월드에서 몇 px 어긋났는지 찾는다(낮은 주파수를 빼고 ±MAXS px 안에서). 이것을 이어 더한 누적
     어긋남이 회전 하나로 담지 못한 이동이다. 창 안의 이동은 2) 처럼 중심을 조금 옮기는 것으로 흡수되지만, 창
     사이에 쌓이는 이동은 회전 하나로 표현되지 않는다.
  5) 시차: 회전각이 60 ~ 120° 떨어진 두 창의 지도를 96 px 타일마다 상호상관해 타일이 몇 px 어긋났는지 찾는다.
     모형이 맞으면 모든 타일이 0 px 근처이고, 카메라가 옮겨 가면 먼 배경과 가까운 물체(막대)가 서로 다르게 어긋난다.
  6) 녹화 전체를 그 중심 하나로 쌓은 지도(두 장면)와, 장면 2 짧은 창 하나의 지도를 그림으로 남긴다.

출력
  <폴더>/scene2.txt   : 장면 2 회전 범위, 창별 중심과 대비, 창끼리 어긋남, 타일별 어긋남, 장면 1 의 같은 값
  <폴더>/rotation.csv : 장면 2 프레임별 roll (frame, t_us, theta_deg)
  <폴더>/scene2.png   : 창별 중심 궤적(두 장면), 전체 지도(중심 하나, 두 장면), 장면 2 짧은 창 지도
"""
import os
import sys
import multiprocessing as mp
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import track_rotation as tr
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'dvs_develop'))
from dvs_read import load

C2 = (478.0, 385.0)    # 장면 2 에서 프레임 쌍마다 잰 회전 중심의 중앙값. 회전각 추적과 창별 중심 탐색의 출발점
WIN = 100              # 창 하나의 프레임 수(약 78 ms)
MIN_ROT = 3.0          # 창 안에서 이만큼(도) 넘게 돈 창만 중심을 찾는다. 거의 안 돌면 중심이 정해지지 않는다
MAXS = 100             # 창끼리 어긋남을 찾는 범위(px)


def best_center(theta, d, f0, f1, c0, pool):
    """프레임 f0 ~ f1-1 의 회전각을 고정하고 1 px 월드 지도의 대비가 가장 큰 회전 중심을 찾는다. (중심, 대비).

    c0 둘레 ±60 px 를 6 px 간격으로 훑고, 가장 좋은 점 둘레 ±6 px 를 다시 1 px 간격으로 훑는다.
    pool 의 작업 프로세스는 set_events 로 채운 표본(tr._ev)을 fork 때 물려받아야 하므로, 부르는 쪽이 같은 구간으로
    set_events 를 먼저 부른 뒤 pool 을 만든다.
    """
    tr.set_events(d, theta, f0, f1)
    c = c0
    for span, step in ((60, 6), (6, 1)):
        cand = [(c[0] + i, c[1] + j) for i in np.arange(-span, span + 1e-9, step)
                for j in np.arange(-span, span + 1e-9, step)]
        sc = np.array(pool.map(tr._contrast, cand))
        c = cand[int(np.argmax(sc))]
    return c, float(sc.max())


def windows(d, theta, c0, ctx):
    """창마다 (f0, f1, 돈 각도, 중심 x, 중심 y, 대비). 거의 안 돈 창은 중심을 NaN 으로."""
    out = []
    nf = d['n_frames']
    for f0 in range(1, nf - WIN + 1, WIN):
        f1 = f0 + WIN
        rot = float(np.degrees(np.ptp(theta[f0:f1])))
        if rot < MIN_ROT:
            out.append((f0, f1, rot, np.nan, np.nan, np.nan))
            continue
        tr.set_events(d, theta, f0, f1)
        with ctx.Pool(12) as pool:          # set_events 뒤에 fork 해야 작업 프로세스가 _ev 를 물려받는다
            c, s = best_center(theta, d, f0, f1, c0, pool)
        out.append((f0, f1, rot, c[0], c[1], s))
        print(f'  창 {f0}~{f1 - 1}: 회전 {rot:.1f}°, 중심 ({c[0]:.0f}, {c[1]:.0f})', file=sys.stderr, flush=True)
    return np.array(out)


def drift(d, theta, c, nf):
    """중심 c 로 만든 창별 월드 지도에서 이웃한 창끼리의 어긋남(dx, dy) px 와 상관 봉우리. [(f0, dx, dy, 봉우리)]."""
    from scipy import ndimage
    maps = []
    for f0 in range(1, nf - WIN + 1, WIN):
        tr.set_events(d, theta, f0, f0 + WIN, step=2)
        maps.append((f0, tr.world_map(c)))
    hp = [(f0, M - ndimage.gaussian_filter(M, 4.0)) for f0, M in maps]     # 낮은 주파수(밀도의 큰 변화)를 뺀다
    out = []
    for (f0, A), (f1, B) in zip(hp[:-1], hp[1:]):
        Fa, Fb = np.fft.rfft2(A), np.fft.rfft2(B)
        xc = np.fft.irfft2(Fa.conj() * Fb, s=A.shape)       # B 가 A 보다 (dy, dx) 옮겨 갔으면 xc[dy, dx] 가 봉우리
        xc = np.fft.fftshift(xc)
        cy, cx = np.array(xc.shape) // 2
        win = xc[cy - MAXS:cy + MAXS + 1, cx - MAXS:cx + MAXS + 1]
        iy, ix = np.unravel_index(int(np.argmax(win)), win.shape)
        peak = float(win.max() / np.sqrt((A * A).sum() * (B * B).sum()))
        out.append((f1, ix - MAXS, iy - MAXS, peak))
    return np.array(out), maps


def tile_shift(maps, theta, tile=96, smax=20):
    """회전각이 60 ~ 120° 떨어진 두 창의 월드 지도를 tile px 타일마다 상호상관해, 타일이 몇 px 어긋났는지 찾는다.

    모형이 맞으면(같은 물체가 같은 칸) 모든 타일이 0 px 근처다. 카메라가 옮겨 가면 먼 배경과 가까운 물체가
    서로 다르게 어긋나므로 타일마다 어긋남이 달라진다. 지도는 σ 1.5 px 로 흐리고 σ 6 px 성분을 뺀 대역으로
    견주고, 두 창이 모두 이벤트를 많이 가진 타일만, 상관 봉우리가 0.3 이상인 것만 남긴다.
    돌려주는 값: [(|어긋남| px, dx, dy, 봉우리)].
    """
    from scipy import ndimage
    from scipy.signal import fftconvolve
    mid = [theta[f0 + WIN // 2] for f0, _ in maps]
    bp = [ndimage.gaussian_filter(M, 1.5) - ndimage.gaussian_filter(M, 6.0) for _, M in maps]
    dens = [ndimage.uniform_filter(M, tile) for _, M in maps]
    out = []
    for i in range(len(maps)):
        for j in range(i + 1, len(maps)):
            if not 60 <= abs(np.degrees(mid[j] - mid[i])) <= 120:
                continue
            thr_i, thr_j = 0.5 * maps[i][1][maps[i][1] > 0].mean(), 0.5 * maps[j][1][maps[j][1] > 0].mean()
            for y0 in range(smax, bp[i].shape[0] - tile - smax, tile):
                for x0 in range(smax, bp[i].shape[1] - tile - smax, tile):
                    cy, cx = y0 + tile // 2, x0 + tile // 2
                    if dens[i][cy, cx] < thr_i or dens[j][cy, cx] < thr_j:
                        continue
                    A = bp[i][y0:y0 + tile, x0:x0 + tile]
                    B = bp[j][y0 - smax:y0 + tile + smax, x0 - smax:x0 + tile + smax]
                    na = np.sqrt((A * A).sum())
                    if na == 0:
                        continue
                    # 모든 (dx, dy) 의 Σ A·B 와 B 창 에너지를 FFT 합성곱으로 한 번에. 결과 [dy + smax, dx + smax]
                    num = fftconvolve(B, A[::-1, ::-1], mode='valid')
                    eb = fftconvolve(B * B, np.ones_like(A), mode='valid')
                    ncc = num / (na * np.sqrt(np.maximum(eb, 1e-12)))
                    iy, ix = np.unravel_index(int(np.argmax(ncc)), ncc.shape)
                    v = float(ncc[iy, ix])
                    if v >= 0.3:
                        out.append((float(np.hypot(ix - smax, iy - smax)), ix - smax, iy - smax, v))
    return np.array(out)


def summary(name, w):
    """windows() 결과의 요약 한 줄: 중심을 찾은 창 수, 창별 중심의 중앙값 · 범위, 중앙값에서 떨어진 거리."""
    ok = ~np.isnan(w[:, 3])
    cx, cy = w[ok, 3], w[ok, 4]
    mx, my = np.median(cx), np.median(cy)
    r = np.hypot(cx - mx, cy - my)
    return (f'{name}: 중심을 찾은 창 {ok.sum()} / {len(w)}, 중앙값 ({mx:.0f}, {my:.0f}), x {cx.min():.0f} ~ {cx.max():.0f}, '
            f'y {cy.min():.0f} ~ {cy.max():.0f}, 중앙값에서 떨어진 거리 중앙값 {np.median(r):.1f} px · 최대 {r.max():.1f} px')


def main(path2, path1, bundle1, out):
    os.makedirs(out, exist_ok=True)
    ctx = mp.get_context('fork')
    d2 = load(path2)
    rot_csv = os.path.join(out, 'rotation.csv')
    if os.path.exists(rot_csv):        # 추적은 몇 분 걸려서 있으면 다시 쓴다
        th2 = np.radians(np.loadtxt(rot_csv, delimiter=',', skiprows=1)[:, 2])
    else:
        th2 = tr.track(d2, C2)
    np.savetxt(os.path.join(out, 'rotation.csv'), np.column_stack([np.arange(d2['n_frames']), d2['t_us'],
               np.degrees(th2)]), delimiter=',', fmt=['%d', '%d', '%.4f'], header='frame,t_us,theta_deg', comments='')
    print(f'장면 2 회전 {np.degrees(th2[-1]):.1f}°', file=sys.stderr, flush=True)
    w2 = windows(d2, th2, C2, ctx)
    # 전체를 중심 하나로
    tr.set_events(d2, th2)
    with ctx.Pool(12) as pool:
        c_all, s_all = best_center(th2, d2, 0, d2['n_frames'], C2, pool)
    dr2, maps2 = drift(d2, th2, c_all, d2['n_frames'])
    ts2 = tile_shift(maps2, th2)
    del maps2
    tr.set_events(d2, th2)
    M_all = tr.world_map(c_all)
    ok = ~np.isnan(w2[:, 3])
    k = int(np.flatnonzero(ok)[np.argmax(w2[ok, 5])])
    tr.set_events(d2, th2, int(w2[k, 0]), int(w2[k, 1]), step=1)
    M_win = tr.world_map((w2[k, 3], w2[k, 4]))
    del d2

    d1 = load(path1)
    th1 = np.radians(np.loadtxt(os.path.join(bundle1, 'rotation.csv'), delimiter=',', skiprows=1)[:, 2])
    p1 = {}
    for line in open(os.path.join(bundle1, 'center.txt')):
        if '=' in line and not line.startswith('#'):
            a, b = line.split('=')
            p1[a.strip()] = float(b)
    c1 = (p1['bg_cx'], p1['bg_cy'])        # 장면 1 골든 묶음의 회전각은 배경 중심으로 추적한 것이라 그 점에서 찾기 시작한다
    w1 = windows(d1, th1, c1, ctx)
    tr.set_events(d1, th1)
    with ctx.Pool(12) as pool:
        c1_all, s1_all = best_center(th1, d1, 0, d1['n_frames'], c1, pool)
    dr1, maps1 = drift(d1, th1, c1_all, d1['n_frames'])
    ts1 = tile_shift(maps1, th1)
    del maps1
    tr.set_events(d1, th1)
    M1_all = tr.world_map(c1_all)
    del d1

    lines = [f'장면 2 ({os.path.basename(path2)}): 회전 {np.degrees(th2[-1]):.1f}°(추적 중심 {C2}), 프레임 {len(th2)}',
             f'창 {WIN} 프레임, 창 안 회전 {MIN_ROT:g}° 넘는 창만 중심을 찾음. 대비 = Σ M² / (Σ M)², 1 px 지도, 이벤트 4 개 중 하나', '',
             '[1] ' + summary('장면 2', w2), '    ' + summary('장면 1', w1),
             f'    녹화 전체를 중심 하나로: 장면 2 ({c_all[0]:.0f}, {c_all[1]:.0f}) 대비 {s_all:.3e}, '
             f'장면 1 ({c1_all[0]:.0f}, {c1_all[1]:.0f}) 대비 {s1_all:.3e}', '',
             f'[1b] 그 중심 하나로 만든 창별 월드 지도의 이웃 창끼리 어긋남(px). 누적 = 앞에서부터 더한 것']
    for name, dr in (('장면 2', dr2), ('장면 1', dr1)):
        cum = np.cumsum(dr[:, 1:3], axis=0)
        step = np.hypot(dr[:, 1], dr[:, 2])
        lines.append(f'    {name}: 이웃 창 어긋남 중앙값 {np.median(step):.1f} px · 최대 {step.max():.1f} px, '
                     f'누적 어긋남의 폭 x {np.ptp(np.r_[0, cum[:, 0]]):.0f} px · y {np.ptp(np.r_[0, cum[:, 1]]):.0f} px, '
                     f'상관 봉우리 중앙값 {np.median(dr[:, 3]):.3f}')
        lines.append('      ' + '  '.join(f'{int(f)}:({int(dx):+d},{int(dy):+d})' for f, dx, dy, _ in dr))
    lines.append('[1c] 회전각이 60 ~ 120° 떨어진 두 창의 월드 지도(같은 중심 하나)를 96 px 타일마다 상호상관한 어긋남. '
                 '두 창 모두 이벤트가 많고 상관 봉우리 0.3 이상인 타일만')
    for name, ts in (('장면 2', ts2), ('장면 1', ts1)):
        q = np.percentile(ts[:, 0], [25, 50, 75, 90])
        lines.append(f'    {name}: 타일 {len(ts)}개, |어긋남| 25/50/75/90 % = ' + ' / '.join(f'{v:.1f}' for v in q) +
                     f' px, 4 px(한 칸) 넘음 {np.mean(ts[:, 0] > 4) * 100:.0f} %, 8 px 넘음 {np.mean(ts[:, 0] > 8) * 100:.0f} %, '
                     f'봉우리 중앙값 {np.median(ts[:, 3]):.2f}')
    lines += ['',
             '[2] 장면 2 창별: 프레임, 창 안 회전(도), 누적 회전(도), 중심 (x, y), 대비']
    for f0, f1, rot, cx, cy, s in w2:
        lines.append(f'    {int(f0):4d}~{int(f1) - 1:4d}  {rot:6.1f}  {np.degrees(th2[int(f0)]):7.1f}  ' +
                     (f'({cx:4.0f}, {cy:4.0f})  {s:.3e}' if not np.isnan(cx) else '거의 안 돌아 생략'))
    lines += ['', '[3] 장면 1 창별: 프레임, 창 안 회전(도), 누적 회전(도), 중심 (x, y), 대비']
    for f0, f1, rot, cx, cy, s in w1:
        lines.append(f'    {int(f0):4d}~{int(f1) - 1:4d}  {rot:6.1f}  {np.degrees(th1[int(f0)]):7.1f}  ' +
                     (f'({cx:4.0f}, {cy:4.0f})  {s:.3e}' if not np.isnan(cx) else '거의 안 돌아 생략'))
    text = '\n'.join(lines)
    with open(os.path.join(out, 'scene2.txt'), 'w', encoding='utf-8') as fo:
        fo.write(text + '\n')
    print(text)

    fig = plt.figure(figsize=(16, 15))
    ax = fig.add_subplot(3, 2, 1)
    for name, w, col in (('scene 2', w2, 'C3'), ('scene 1', w1, 'C0')):
        okw = ~np.isnan(w[:, 3])
        ax.plot(w[okw, 3], w[okw, 4], 'o-', color=col, ms=4, lw=0.8, label=name)
    ax.invert_yaxis()
    ax.set_aspect('equal')
    ax.set_xlabel('x (px)')
    ax.set_ylabel('y (px)')
    ax.set_title(f'best rotation centre per {WIN}-frame window', fontsize=10)
    ax.legend()
    ax = fig.add_subplot(3, 2, 2)
    okw = ~np.isnan(w2[:, 3])
    ax.plot(np.degrees(th2[w2[okw, 0].astype(int)]), w2[okw, 3], 'o-', label='x')
    ax.plot(np.degrees(th2[w2[okw, 0].astype(int)]), w2[okw, 4], 's-', label='y')
    ax.set_xlabel('scene 2 accumulated rotation at window start (deg)')
    ax.set_ylabel('centre (px)')
    ax.legend()
    ax.set_title('scene 2 centre vs rotation', fontsize=10)
    for i, (M, t) in enumerate(((M_all, f'scene 2, all frames, one centre ({c_all[0]:.0f}, {c_all[1]:.0f})'),
                                (M_win, f'scene 2, frames {int(w2[k, 0])}-{int(w2[k, 1]) - 1} only, own centre'),
                                (M1_all, f'scene 1, all frames, one centre ({c1_all[0]:.0f}, {c1_all[1]:.0f}), same method'))):
        ax = fig.add_subplot(3, 2, 3 + i)
        L = np.log1p(M)
        ax.imshow(L, cmap='gray_r', vmax=np.percentile(L[M > 0], 99.5))
        ax.set_title(t, fontsize=10)
        ax.set_xticks([])
        ax.set_yticks([])
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'scene2.png'), dpi=70)


if __name__ == '__main__':
    main(*sys.argv[1:5])
