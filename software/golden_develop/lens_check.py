#!/usr/bin/env python3
"""렌즈 방사 왜곡을 보정하지 않아도 되는지 장면 1 데이터로 잰다.

    python3 software/golden_develop/lens_check.py <녹화>.dvs <골든 묶음> <출력 폴더>

세 가지를 본다.
  1. 막대 직선성으로 왜곡 계수 찾기
     회전이 거의 없는 구간 REF 의 1 px 지도에서 막대 4 개의 한쪽 가장자리를 행마다 소수 px 로 찾는다. 방사 왜곡
     한 항 모형 u = c0 + (p - c0)(1 + k1·|p - c0|²) 으로 가장자리 점을 펴고, 막대마다 직선을 맞춘 잔차가 가장
     작아지는 k1 을 격자로 찾는다. 막대 넷이 같은 k1 을 가리키지 않으면 이 데이터로는 왜곡이 드러나지 않는 것이다.
  2. 모의: 보정하지 않으면 월드에서 얼마나 번지나
     1 에서 찾은 k1 이 실제 왜곡이라고 두고, 회전 중심 c 둘레로 한 바퀴 도는 점을 왜곡해 관측한 뒤 왜곡을 모르는
     채 R(ψ) 로 되돌렸을 때 월드 좌표가 회전각에 따라 흩어지는 폭을 센서 위치별로 잰다.
  3. 실측: k1 로 펴면 막대 수치가 나아지나
     이벤트 좌표를 k1 로 편 뒤 같은 방법(막대 중심으로 회전각 추적, 위쪽 보정)으로 4 px 칸 월드를 만들어, 펴지
     않은 것과 막대 기울기 · 위치 · 가장자리 폭을 비교한다. 정수 모델이 아니라 실수로 센다.

왜곡 중심 c0 는 영상 가운데 (480, 360) 으로 둔다(이 스크립트는 주점을 따로 추정하지 않는다). 1 에서 막대들이
같은 k1 을 가리키지 않고 3 에서 펴도 막대 수치가 나아지지 않으면, 왜곡을 보정하지 않고 월드 변환을 해도 된다는
근거가 된다. 2 는 보정하지 않을 때 생길 번짐의 크기를 칸 크기(4 px)와 견줄 수 있게 한다.

출력
  <폴더>/lens.txt : 막대별 가장자리 점 수 · 직선 잔차, k1 격자 표와 최솟값, 모의 흩어짐, 실측 비교
"""
import os
import sys
import numpy as np
from scipy import ndimage
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import golden as g
import track_rotation as tr
import world_metrics as wm
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'dvs_develop'))
from dvs_read import load

C0 = np.array([480.0, 360.0])          # 왜곡 중심(영상 가운데)
POST_C = (533.0, 418.0)                # 막대 중심. 장면 1 골든 모델의 회전 중심 (cx, cy) 와 같은 값을 고정해 둔다
K1_GRID = np.arange(-40, 40.01, 2) * 1e-8


def edge_points(M, line, pad):
    """1 px 지도 M(좌표가 pad 만큼 밀림)에서 막대 선 둘레 ±10 px 안 가장 강한 가장자리를 행마다 소수 px 로 찾는다.

    가우시안(σ = 1 px)으로 흐린 가로 단면의 봉우리를 포물선 꼭짓점으로 다듬는다. 직선에서 2.5 σ 넘게 벗어난 점은
    세 번 걸러 버린다(다른 쪽 가장자리나 뒤 물체를 잡은 행). 돌려주는 값은 센서 좌표 (x, y) 배열.
    """
    (x0, y0), (x1, y1) = tr.fit_line(M, np.add(line[0], pad), np.add(line[1], pad))
    pts = []
    for y in range(int(min(y0, y1)) + 5, int(max(y0, y1)) - 5):
        xi = int(round(x0 + (y - y0) * (x1 - x0) / (y1 - y0)))
        seg = M[y, xi - 10:xi + 11].astype(float)
        if seg.sum() <= 0:
            continue
        s = ndimage.gaussian_filter1d(seg, 1.0)
        j = int(np.argmax(s))
        if 1 <= j <= 19:
            a, b, c = s[j - 1], s[j], s[j + 1]
            den = a - 2 * b + c
            pts.append((xi - 10 + j + (0.5 * (a - c) / den if den else 0.0) - pad, y - pad))
    v = np.array(pts)
    keep = np.ones(len(v), bool)
    for _ in range(3):
        cf = np.polyfit(v[keep, 1], v[keep, 0], 1)
        r = v[:, 0] - np.polyval(cf, v[:, 1])
        keep = np.abs(r) < 2.5 * np.std(r[keep])
    return v[keep]


def undistort(x, y, k1):
    """한 항 방사 모형으로 편 좌표."""
    dx, dy = x - C0[0], y - C0[1]
    f = 1 + k1 * (dx * dx + dy * dy)
    return C0[0] + dx * f, C0[1] + dy * f


def line_rms(v, k1):
    """가장자리 점을 k1 로 편 뒤 직선 x = a·y + b 에 맞춘 잔차 RMS(px)."""
    ux, uy = undistort(v[:, 0], v[:, 1], k1)
    cf = np.polyfit(uy, ux, 1)
    return float(np.sqrt(((ux - np.polyval(cf, uy)) ** 2).mean()))


def spread(k1, c, pts, n_angle=360):
    """센서 점 pts 를 c 둘레로 돌려 가며 왜곡해 관측하고 왜곡을 모르는 채 되돌렸을 때, 월드 좌표가 흩어지는 최대 폭(px).

    펴진(참) 좌표에서 회전 중심 c 둘레로 θ 만큼 돈 점을 k1 로 왜곡한 것이 관측이다. 왜곡은 펴기의 역이라
    1 차로 p = c0 + (u - c0)(1 - k1·r²) 를 쓴다(k1·r² 이 0.1 아래라 2 차 오차는 무시할 만하다).
    """
    out = []
    th = np.radians(np.arange(n_angle))
    for p in pts:
        d = np.asarray(p, float) - c
        ux = c[0] + np.cos(th) * d[0] - np.sin(th) * d[1]
        uy = c[1] + np.sin(th) * d[0] + np.cos(th) * d[1]
        ox, oy = undistort(ux, uy, -k1)
        wx = c[0] + np.cos(-th) * (ox - c[0]) - np.sin(-th) * (oy - c[1])
        wy = c[1] + np.sin(-th) * (ox - c[0]) + np.cos(-th) * (oy - c[1])
        out.append(float(np.hypot(wx - wx.mean(), wy - wy.mean()).max() * 2))
    return out


def float_world(x, y, frame, psi_deg, center, f0=0, f1=None, nf=None):
    """실수 좌표 이벤트를 4 px 칸 월드(352 x 352)에 센 지도. 정수 모델과 같은 회전 방향 · 중심 규칙."""
    lo, hi = np.searchsorted(frame, [f0, nf if f1 is None else f1])
    rad = np.radians(psi_deg)
    M = np.zeros(g.N * g.M)
    for s in range(lo, hi, 1 << 23):                 # 메모리를 아끼려고 나눠서 센다(golden.run 과 같다)
        e = min(s + (1 << 23), hi)
        a = rad[frame[s:e]]
        dx, dy = x[s:e] - center[0], y[s:e] - center[1]
        wx = np.floor((np.cos(a) * dx - np.sin(a) * dy + g.WC) / wm.CELL).astype(np.int64)
        wy = np.floor((np.sin(a) * dx + np.cos(a) * dy + g.WC) / wm.CELL).astype(np.int64)
        ok = (wx >= 0) & (wx < g.N) & (wy >= 0) & (wy < g.M)
        M += np.bincount(wy[ok] * g.N + wx[ok], minlength=g.N * g.M)
    return M.reshape(g.M, g.N)


def pipeline(d, x, y):
    """좌표 (x, y) 로 막대 중심 추적 → 위쪽 보정 → 월드 지도와 막대 수치(track_center_check.py 와 같은 순서)."""
    dd = dict(d, x=x, y=y)
    th = tr.track(dd, POST_C)
    tr.set_events(dd, th, *tr.REF, step=1)
    M = tr.world_map(POST_C)
    lines = [tr.fit_line(M, np.add(p, tr.PAD), np.add(q, tr.PAD)) for p, q in tr.ROUGH.values()]
    lines = [(p - tr.PAD, q - tr.PAD) for p, q in lines]
    tr.set_events(dd, th)
    M = tr.world_map(POST_C)
    alpha = float(np.mean([tr.tilt_deg(tr.fit_line(M, np.add(p, tr.PAD), np.add(q, tr.PAD))) for p, q in lines]))
    psi = alpha - np.degrees(th)
    W = float_world(x, y, d['frame'], psi, POST_C, nf=d['n_frames'])
    R_ = float_world(x, y, d['frame'], psi, POST_C, *tr.REF, nf=d['n_frames'])
    return np.degrees(th[-1]), alpha, wm.post_metrics(W, R_, psi[(tr.REF[0] + tr.REF[1]) // 2], POST_C)


def main(path, bundle, out):
    os.makedirs(out, exist_ok=True)
    d = load(path)
    th = np.radians(np.loadtxt(os.path.join(bundle, 'rotation.csv'), delimiter=',', skiprows=1)[:, 2])
    wm.set_cell(g.CELL_SHIFT, g.CELL_BITS)
    L = [f'# 렌즈 방사 왜곡. 장면 1, 왜곡 중심 {tuple(C0)}, 모형 u = c0 + (p - c0)(1 + k1 r²)', '']

    # 1. 막대 직선성
    tr.set_events(d, th, *tr.REF, step=1)
    M = tr.world_map(POST_C)
    E = {k: edge_points(M, line, tr.PAD) for k, line in tr.ROUGH.items()}
    L.append('[1] REF 구간 1 px 지도의 막대 가장자리')
    for k, v in E.items():
        L.append(f'    막대 {k}: 점 {len(v)}개, 센서 x {v[:, 0].min():.0f}~{v[:, 0].max():.0f}, y {v[:, 1].min():.0f}~{v[:, 1].max():.0f},'
                 f' 직선 잔차 RMS {line_rms(v, 0.0):.2f} px')
    tab = np.array([[line_rms(E[k], k1) for k in E] for k1 in K1_GRID])
    w = np.array([len(E[k]) for k in E])
    tot = np.sqrt((tab ** 2 * w).sum(1) / w.sum())
    j0 = int(np.argmin(np.abs(K1_GRID)))
    jb = int(np.argmin(tot))
    L.append('    k1 격자(1e-8 / px²)에서 직선 잔차 RMS(px). 반지름 450 px 점을 k1 이 옮기는 양도 적는다')
    L.append('      k1     전체     ' + '  '.join(f'{k:>5}' for k in E) + '   r=450 이동')
    for i in range(0, len(K1_GRID), 2):
        L.append(f'    {K1_GRID[i] * 1e8:+5.0f}  {tot[i]:6.3f}   ' + '  '.join(f'{x:5.2f}' for x in tab[i]) +
                 f'   {abs(K1_GRID[i]) * 450 ** 3:5.1f} px')
    L.append(f'    네 막대 함께 최소: k1 {K1_GRID[jb]:.1e}, 전체 RMS {tot[jb]:.3f} px (k1 = 0 이면 {tot[j0]:.3f} px, '
             f'{(1 - tot[jb] / tot[j0]) * 100:.1f} % 줄어듦)')
    L.append('    막대마다 따로 최소: ' + '  '.join(f'{k} {K1_GRID[int(np.argmin(tab[:, n]))]:.1e}' for n, k in enumerate(E)))
    k1 = float(K1_GRID[jb])

    # 2. 모의 흩어짐
    probe = [(480, 360), (533, 418), (160, 360), (800, 360), (480, 60), (480, 700), (100, 60), (900, 700)]
    L += ['', f'[2] 모의: k1 이 실제 왜곡일 때 보정 없이 회전 중심 {POST_C} 둘레로 한 바퀴 누적하면 생기는'
          ' 월드 좌표 흩어짐(최대 지름, px). 1 의 최솟값과, 3 에서 보정이 이득을 못 준 가장 작은 크기 5e-8']
    for kk in (k1, 5e-8):
        L.append(f'    k1 {kk:+.1e}: ' + ', '.join(f'{p} {s_:.1f}' for p, s_ in zip(probe, spread(kk, np.array(POST_C), probe))))
    L.append(f'    같은 k1 에서 회전 중심을 왜곡 중심 {tuple(C0)} 으로 두면: ' +
             ', '.join(f'{p} {s_:.2f}' for p, s_ in zip(probe[2:4], spread(k1, C0, probe[2:4]))) + ' px')

    # 3. 실측 비교
    L += ['', f'[3] 실측: 이벤트를 k1 로 편 것과 펴지 않은 것. 막대 중심 {POST_C} 로 추적, 4 px 칸 실수 누적.'
          ' 기울기는 전체 누적, 반치폭은 짧은 구간 / 전체']
    xs, ys = d['x'].astype(np.float32), d['y'].astype(np.float32)      # float32 로도 0.001 px 아래 정밀도
    summary = []
    for kk in sorted({k1, -1e-7, -5e-8, 0.0, 5e-8, 1e-7}):
        ux, uy = (v.astype(np.float32) for v in undistort(xs, ys, np.float32(kk)))
        rot, alpha, m = pipeline(d, ux, uy)
        L.append(f'    k1 {kk:+.1e}{" (펴지 않음)" if kk == 0 else ""}: 누적 회전 {rot:.2f}°, 위쪽 보정 {alpha:.3f}°')
        L.append('      ' + '  '.join(f'{k}: {m[k]["tilt_full"]:+.1f}° {m[k]["shift_px"]:+.0f}px '
                                      f'{m[k]["width_ref"]:.0f}/{m[k]["width_full"]:.0f}px' for k in m))
        summary.append((kk, sum(abs(m[k]['tilt_full']) for k in m), sum(m[k]['width_full'] for k in m),
                        float(np.mean([m[k]['contrast_full'] for k in m]))))
        print(L[-2], L[-1], flush=True)
    L.append('    요약: k1 별 전체 누적의 |기울기| 합(도) · 가장자리 반치폭 합(px) · 봉우리/바닥 평균')
    for kk, t_, w_, c_ in summary:
        L.append(f'      k1 {kk:+.1e}: {t_:5.1f}°  {w_:5.0f} px  {c_:.3f}')
    text = '\n'.join(L)
    print(text)
    with open(os.path.join(out, 'lens.txt'), 'w') as fo:
        fo.write(text + '\n')


if __name__ == '__main__':
    main(*sys.argv[1:4])
