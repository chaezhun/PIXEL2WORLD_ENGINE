#!/usr/bin/env python3
"""장면 1 녹화에서 프레임마다 영상 회전각(roll)을 추정하고, 월드 메모리에 쓸 회전 중심과 위쪽 방향을 고른다.

    python3 software/golden_develop/track_rotation.py software/datasets/2026-09-22-16-42-44-DELTA.dvs <출력 폴더>

출력
  <폴더>/rotation.csv : frame, t_us, theta_deg. 프레임 f 의 영상이 첫 프레임보다 화면에서 몇 도 돌았나
                        (y 가 아래로 자라는 화면 좌표에서 시계 방향이 +)
  <폴더>/center.txt   : key = value. 골든 모델(golden.py)이 읽는 cx, cy, alpha_deg 와, 값을 고를 때 잰
                        수치(bg_cx, bg_cy, bg_contrast, post_score). # 줄은 설명과 막대별 수치다

모형
  센서는 광축과 나란한 회전축 둘레를 돈다. 축이 센서에서 비켜 있어
  영상의 회전 중심은 물체 깊이마다 다르지만, 회전각은 모든 깊이에서 같다. 그래서
    1) 회전각은 이벤트 전체(대부분 배경)로 추정하고,
    2) 월드 메모리에 쓸 회전 중심은 막대 4 개가 가장 날카로워지는 점으로 따로 고른다.
  막대보다 멀거나 가까운 물체는 이 중심에서 조금 번진다. 회전 하나로는 깊이별 중심 차이를 못 담는다.

방법
  회전각 (track)
    2 px 칸의 추적 지도를 만들어 가며 프레임을 하나씩 맞춘다. 새 프레임 이벤트에서 6000 개를 뽑아 후보
    각도마다 되돌리고, 흐린 지도(가우시안 σ = 1 칸)에서 읽은 값의 합이 가장 큰 각도를 고른다. 후보는 앞 두
    프레임의 각속도와 시각 간격으로 예측한 값 ±0.8°(0.04° 간격), 다시 그 둘레 ±0.05°(0.004° 간격)다.
    고른 각도로 프레임의 이벤트를 모두 지도에 더한다. 지도는 녹화 첫 프레임부터 지우지 않고 쌓으므로, 한
    바퀴를 넘으면 앞서 쌓인 같은 장면과 다시 맞춰져 프레임마다의 오차가 누적되지 않는다.
  배경 중심 (c_bg)
    roll_check.py 가 잰 중심 C0 로 한 번 추적한 회전각을 고정하고, 후보 중심마다 1 px 월드 지도의 대비
    Σ M² / (Σ M)² 를 잰다. 가장 큰 중심을 격자를 좁혀 가며 찾고, 그 중심으로 다시 추적한 회전각을 쓴다.
  막대 중심 (cx, cy)
    회전이 거의 없는 구간 REF 의 지도에서 눈으로 고른 막대 선(ROUGH)을 둘레 12 px 띠의 가중 주성분으로
    다듬는다. 후보 중심마다 전체 지도에서 막대를 가로지르는 단면(선 방향으로 평균)의 정규화 분산을 막대별로
    재고, 막대마다 최댓값으로 나눈 합이 가장 큰 중심을 고른다.
  위쪽 (alpha_deg)
    막대 중심으로 만든 전체 지도에서 막대 4 개의 기울기를 재어 평균만큼 월드를 돌린다. 막대가 실제로 수직이므로
    이 방향을 월드의 위로 삼는다.
"""
import os
import sys
import multiprocessing as mp
import numpy as np
from scipy import ndimage
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'dvs_develop'))
from dvs_read import load

C0 = (504.0, 373.0)        # roll_check.py 가 16-42-44 녹화에서 잰 회전 중심 중앙값. 배경 중심 탐색의 출발점
REF = (170, 215)           # 회전이 거의 없는 구간(누적 -0.3 ~ -2.5°). 막대 선을 여기서 찾는다
# REF 구간 지도에서 눈으로 고른 막대 양 끝 (x, y) px. A~D 는 영상 왼쪽부터 차례로 붙인 이름이고, 녹화
# 셋업의 물체 1~4 와 같은 순서다(A = 물체 1, ..., D = 물체 4). 네 막대 가운데 B 만 짧다.
ROUGH = {'A': ((207, 5), (133, 600)), 'B': ((312, 345), (272, 700)),
         'C': ((603, 25), (528, 705)), 'D': ((812, 185), (745, 705))}
TR_CELL, TR_N, TR_SUB = 2.0, 720, 6000      # 추적 지도 칸(px), 한 변 칸 수, 프레임당 표본 수
PAD = 300                  # 평가 지도 여유(px). 월드 = R(-θ)(p - c) + c 를 [-PAD, 960+PAD) 범위에 담는다
EV_STEP = 4                # 평가 지도는 이벤트 4 개 중 하나만 쓴다(중심 고르기에는 충분하고 4 배 빠르다)
WW, WH = 960 + 2 * PAD, 720 + 2 * PAD

_ev = {}                   # 평가 지도용 표본. fork 한 작업 프로세스가 그대로 물려받는다


def track(d, c, seed=0):
    """중심 c 로 프레임마다 회전각(rad)을 추정한다."""
    cx, cy = c
    frame, X, Y, t = d['frame'], d['x'], d['y'], d['t_us']
    nf = d['n_frames']
    starts = np.searchsorted(frame, np.arange(nf + 1))
    rng = np.random.default_rng(seed)
    off = TR_N / 2

    def cell(xs, ys, th):
        co, s = np.cos(th), np.sin(th)
        dx, dy = xs - cx, ys - cy
        return (np.floor((-s * dx + co * dy) / TR_CELL + off).astype(np.int64) * TR_N
                + np.floor((co * dx + s * dy) / TR_CELL + off).astype(np.int64))

    M = np.zeros(TR_N * TR_N)
    Mb = M
    theta = np.zeros(nf)
    for f in range(1, nf):
        a, b = starts[f], starts[f + 1]
        xs, ys = X[a:b].astype(np.float64), Y[a:b].astype(np.float64)
        theta[f] = theta[f - 1]
        if f > 2 and b > a:
            w = (theta[f - 1] - theta[f - 2]) / (t[f - 1] - t[f - 2])
            best = theta[f - 1] + w * (t[f] - t[f - 1])      # 빠진 프레임(16-42-44 의 514 앞)도 시각으로 예측
            k = rng.choice(b - a, min(TR_SUB, b - a), replace=False)
            for span, step in ((0.8, 0.04), (0.05, 0.004)):
                cand = best + np.radians(np.arange(-span, span + 1e-9, step))
                score = [Mb[cell(xs[k], ys[k], th)].sum() for th in cand]
                best = cand[int(np.argmax(score))]
            theta[f] = best
        M += np.bincount(cell(xs, ys, theta[f]), minlength=TR_N * TR_N)
        if f % 4 == 0 or f < 20:
            Mb = ndimage.gaussian_filter(M.reshape(TR_N, TR_N), 1.0).ravel()
    return theta


def set_events(d, theta, f0=0, f1=None, step=EV_STEP):
    """평가 지도에 쓸 이벤트 표본과 프레임별 cos, sin 을 _ev 에 둔다."""
    lo, hi = np.searchsorted(d['frame'], [f0, d['n_frames'] if f1 is None else f1])
    f = d['frame'][lo:hi:step]
    _ev.update(x=d['x'][lo:hi:step].astype(np.float32), y=d['y'][lo:hi:step].astype(np.float32),
               co=np.cos(theta[f]).astype(np.float32), si=np.sin(theta[f]).astype(np.float32))


def world_map(c):
    """_ev 의 이벤트를 월드 = R(-θ)(p - c) + c 로 옮겨 1 px 칸에 센 지도. 좌표는 PAD 만큼 밀려 있다."""
    M = np.zeros(WW * WH)
    for a in range(0, len(_ev['x']), 1 << 22):
        s = slice(a, a + (1 << 22))
        dx, dy = _ev['x'][s] - c[0], _ev['y'][s] - c[1]
        co, si = _ev['co'][s], _ev['si'][s]
        ix = np.floor(co * dx + si * dy + c[0] + PAD).astype(np.int64)
        iy = np.floor(-si * dx + co * dy + c[1] + PAD).astype(np.int64)
        ok = (ix >= 0) & (ix < WW) & (iy >= 0) & (iy < WH)
        M += np.bincount(iy[ok] * WW + ix[ok], minlength=WW * WH)
    return M.reshape(WH, WW)


def fit_line(M, p, q, band=12):
    """선 p-q 둘레 band px 띠의 가중 주성분으로 다듬은 선(같은 길이, 아래로 향함)."""
    p, q = np.asarray(p, float), np.asarray(q, float)
    L = np.linalg.norm(q - p)
    u = (q - p) / L
    yy, xx = np.mgrid[0:M.shape[0], 0:M.shape[1]]
    s = (xx - p[0]) * u[0] + (yy - p[1]) * u[1]
    t = -(xx - p[0]) * u[1] + (yy - p[1]) * u[0]
    m = (s > 0) & (s < L) & (np.abs(t) < band)
    w, P = M[m], np.stack([xx[m], yy[m]], 1).astype(float)
    mu = (w[:, None] * P).sum(0) / w.sum()
    u2 = np.linalg.eigh(((P - mu).T * w) @ (P - mu))[1][:, 1]
    u2 = -u2 if u2[1] < 0 else u2
    return mu - u2 * L / 2, mu + u2 * L / 2


def tilt_deg(line):
    """선이 수직에서 기운 각도. 아래로 갈수록 오른쪽이면 +."""
    (x0, y0), (x1, y1) = line
    return float(np.degrees(np.arctan2(x1 - x0, y1 - y0)))


def profile(M, line, half=40, off=0.0):
    """선을 가로지르는 단면. 선 방향으로 1 px 마다 뽑아 평균한다. off 는 지도 좌표가 밀린 양."""
    p, q = (np.asarray(v, float) + off for v in line)
    L = np.linalg.norm(q - p)
    u = (q - p) / L
    n = np.array([-u[1], u[0]])
    P = p + np.arange(0, L)[:, None, None] * u + np.arange(-half, half + 1)[None, :, None] * n
    v = ndimage.map_coordinates(M, [P[..., 1].ravel(), P[..., 0].ravel()], order=1)
    return v.reshape(P.shape[:2]).mean(0)


def sharpness(prof):
    return float(((prof - prof.mean()) ** 2).mean() / prof.mean() ** 2)


def _contrast(c):
    M = world_map(c)
    return float((M ** 2).sum() / M.sum() ** 2)


def _post_sharp(c):
    M = world_map(c)
    return [sharpness(profile(M, l, off=PAD)) for l in _ev['lines']]


def grid(score, c, span, step, pool):
    """c 둘레 ±span 을 step 간격으로 훑어 (후보 목록, 점수 배열)."""
    cand = [(c[0] + i, c[1] + j) for i in np.arange(-span, span + 1e-9, step)
            for j in np.arange(-span, span + 1e-9, step)]
    return cand, np.array(pool.map(score, cand))


def main(path, out):
    d = load(path)
    os.makedirs(out, exist_ok=True)
    log = []
    pool_ctx = mp.get_context('fork')

    # 1) 배경 중심: C0 로 추적한 회전각을 고정하고 대비가 가장 큰 중심
    th0 = track(d, C0)
    set_events(d, th0)
    c_bg = C0
    with pool_ctx.Pool(6) as pool:
        for span, step in ((48, 8), (8, 2)):
            cand, sc = grid(_contrast, c_bg, span, step, pool)
            c_bg = cand[int(np.argmax(sc))]
    log.append(f'# 배경 중심: 회전각 고정, 대비 Σ M² / (Σ M)² 최대. C0 = {C0}')
    log.append(f'bg_cx = {c_bg[0]:g}\nbg_cy = {c_bg[1]:g}\nbg_contrast = {sc.max():.4e}')
    theta = track(d, c_bg)

    # 2) 막대 선: REF 구간 지도에서 다듬는다
    set_events(d, theta, *REF, step=1)
    M = world_map(c_bg)
    lines = [fit_line(M, np.add(p, PAD), np.add(q, PAD)) for p, q in ROUGH.values()]
    lines = [(p - PAD, q - PAD) for p, q in lines]

    # 3) 막대 중심: 막대별 단면 날카로움을 최댓값으로 나눈 합이 가장 큰 중심
    set_events(d, theta)
    _ev['lines'] = lines
    c = c_bg
    with pool_ctx.Pool(6) as pool:
        for span, step in ((60, 10), (15, 3)):
            cand, sc = grid(_post_sharp, c, span, step, pool)
            total = (sc / sc.max(0)).sum(1)
            c = cand[int(np.argmax(total))]
    log.append('# 막대 중심: 막대 단면의 정규화 분산을 막대별 최댓값으로 나눈 합 최대(최댓값 4)')
    log.append(f'cx = {c[0]:g}\ncy = {c[1]:g}\npost_score = {total.max():.3f}')
    for k, i in zip(ROUGH, range(4)):
        j = int(np.argmax(sc[:, i]))
        log.append(f'# 막대 {k} 혼자 가장 날카로운 중심 ({cand[j][0]:g}, {cand[j][1]:g})')

    # 4) 위쪽: 막대 중심으로 만든 전체 지도에서 막대 기울기 평균
    M = world_map(c)
    tilts = [tilt_deg(fit_line(M, np.add(p, PAD), np.add(q, PAD))) for p, q in lines]
    alpha = float(np.mean(tilts))
    log.append('# 위쪽: 전체 지도의 막대 기울기(아래로 갈수록 오른쪽이 +) ' +
               ', '.join(f'{k} {v:.2f}' for k, v in zip(ROUGH, tilts)))
    log.append(f'alpha_deg = {alpha:.3f}')

    rows = np.column_stack([np.arange(d['n_frames']), d['t_us'], np.degrees(theta)])
    np.savetxt(os.path.join(out, 'rotation.csv'), rows, delimiter=',', fmt=['%d', '%d', '%.4f'],
               header='frame,t_us,theta_deg', comments='')
    with open(os.path.join(out, 'center.txt'), 'w') as fo:
        fo.write('\n'.join(log) + '\n')
    print('\n'.join(log))
    print(f'누적 회전 {np.degrees(theta[-1]):.2f} 도')


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
