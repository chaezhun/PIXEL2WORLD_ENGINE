#!/usr/bin/env python3
"""9/15 녹화(수평 회전, yaw)를 원통면 월드에 쌓고, 고정 물체는 같은 칸에 모이고 움직이는 물체는 모이지 않음을
보인다.

    python3 software/golden_develop/yaw_check.py software/datasets/2026-09-15-14-27-41-DELTA.dvs <출력 폴더>

<출력 폴더>/lens.txt(lens_yaw.py 출력)의 초점거리 f_px 와 렌즈 왜곡 lam 을 읽는다.

모형
  픽셀 (x, y) 를 f 로 나눈 (xd, yd) 에서 division 모형으로 광선 (xn, yn) = (xd, yd) / (1 + lam · rd^2) 를 얻고,
  카메라가 세로축 둘레로 yaw φ_k 돌았다고 보고 반지름 f 인 원통에 투영한다.
      U = f · (φ_k + atan(xn)),   V = f · yn / sqrt(1 + xn^2)
  영상 가운데에서 1 px 이 월드 1 px 이다. 세상의 세로선은 원통면에서도 세로선이다. 장면 1 의 roll 과 같은 4 px
  칸으로 쌓는다.

yaw 추적 (track)
  2 px 칸 원통면 지도를 만들어 가며 프레임을 하나씩 맞춘다. 새 프레임 이벤트에서 6000 개를 뽑아 후보 yaw 마다
  월드로 옮기고, 흐린 지도(가우시안 σ = 1 칸)에서 읽은 값의 합이 가장 큰 yaw 를 고른다. 후보는 앞 두 프레임의
  각속도와 시각 간격으로 예측한 값 ±0.6°(0.02° 간격), 다시 그 둘레 ±0.025°(0.002° 간격)다. 고른 yaw 로 그 프레임의
  이벤트를 모두 지도에 더하고, 흐린 지도는 BLUR_EVERY 프레임마다 새로 만든다(track_rotation.py 의 roll 추적과 같은
  방식). 이벤트가 50 개 미만인 프레임은 예측값을 그대로 쓴다. 녹화 앞 STILL 프레임은 카메라가 돌기 전이라 그 이벤트를
  yaw 0 으로 쌓아 지도를 시작한다.

같은 칸에 모이는가
  카메라가 도는 구간(프레임 STILL 부터)의 이벤트를 영상 가운데(|x - PX0| < 160)에서 난 것과 가장자리
  (|x - PX0| >= 240)에서 난 것으로 나눠 월드 지도 두 장을 만든다. 카메라가 도는 동안 고정 물체는 영상 한쪽
  가장자리에서 가운데를 지나 반대쪽 가장자리로 지나가므로, 변환이 맞으면 두 지도의 같은 칸에 쌓인다.
  렌즈 왜곡이 크게 드러나는 곳이 가장자리라 렌즈 모형을 시험하기에 알맞다. 두 지도가 함께 덮는 칸에서 상관을
  재고, 렌즈 왜곡을 뺀 핀홀(lam = 0, 같은 f)로 다시 추적한 것과 견준다. WIN px 창마다 상관을 재어 이벤트는 많은데
  상관이 낮은 창(움직이는 물체나 모형이 안 맞는 곳의 후보)을 찾는다.

카메라가 서 있는 구간
  프레임 0 ~ STILL-1 에서 STILL_REGIONS 의 원마다 픽셀당 이벤트와, 그 원의 프레임별 이벤트 수가 나머지 영상의
  프레임별 수와 얼마나 함께 움직이는지(상관)를 잰다. 카메라가 서 있으므로 이벤트가 유독 많고 따로 움직이는 영역은
  스스로 움직이는 물체(예: 도는 선풍기 날개)다.

출력
  <폴더>/yaw.csv       : frame, t_us, yaw_deg
  <폴더>/yaw.txt       : 렌즈 값과 수평 시야, yaw 범위, 가운데 · 가장자리 지도의 상관(렌즈 왜곡 모형과 핀홀),
                         창별 상관 분포와 상관이 낮은 창, 서 있는 구간의 영역별 이벤트
  <폴더>/yaw_world.png : 원통면 월드 지도(전체), 가운데 · 가장자리 지도 겹침(렌즈 왜곡 모형, 핀홀), 창별 상관
  <폴더>/yaw_world.npy : 4 px 칸 원통면 월드 지도(전체 누적, 칸마다 이벤트 수)
"""
import os
import sys
import numpy as np
from scipy import ndimage
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'dvs_develop'))
from dvs_read import load, WIDTH, HEIGHT
import golden as g

PX0, PY0 = (WIDTH - 1) / 2, (HEIGHT - 1) / 2   # 주점: 영상 가운데
STILL = 200            # 녹화 앞 이 프레임 수 동안은 yaw 0 으로 둔다(9/15 녹화는 프레임 200 무렵부터 돈다)
TR_CELL = 2.0          # 추적 지도 칸(px)
WL_CELL = 1 << g.CELL_SHIFT   # 월드 메모리 칸(px), 장면 1 과 같은 4 px
TR_SUB = 6000          # 추적에서 프레임마다 뽑는 이벤트 수
U_SPAN = (-40.0, 140.0)       # 지도에 담는 방위각 범위(도)
COARSE = np.arange(-0.6, 0.6001, 0.02)         # 추적 1 단계 후보: 예측 yaw 둘레(도)
FINE = np.arange(-0.025, 0.02501, 0.002)       # 추적 2 단계 후보: 1 단계 최선 둘레(도)
BLUR_EVERY = 8         # 추적용 흐린 지도를 이 프레임 수마다 새로 만든다
WIN = 64               # 창별 상관의 창 크기(px)


def read_kv(path):
    """lens.txt 의 맨 앞에서 시작하는 key = value 줄 가운데 값이 수인 것만 dict 로. 들여쓴 줄과 '[' 줄은 건너뛴다."""
    out = {}
    for line in open(path, encoding='utf-8'):
        if '=' in line and not line.startswith(' ') and not line.startswith('['):
            k, v = line.split('=', 1)
            try:
                out[k.strip()] = float(v)
            except ValueError:
                pass
    return out


class Cyl:
    """초점거리 f · 왜곡 lam 에 대한 원통면 지도 칸과 픽셀 → (방위 성분, V) 표."""

    def __init__(self, f, lam, cell):
        self.f, self.cell = f, cell
        xd = (np.arange(WIDTH) - PX0)[None, :] / f
        yd = (np.arange(HEIGHT) - PY0)[:, None] / f
        s = 1 / (1 + lam * (xd * xd + yd * yd))
        xn, yn = xd * s, yd * s
        self.ax = np.arctan(xn)                                  # (y, x) → 방위 성분(rad)
        self.v = (f * yn / np.sqrt(1 + xn * xn)).astype(np.float32)
        self.u0 = np.radians(U_SPAN[0]) * f
        self.nu = int(np.ceil(np.radians(U_SPAN[1] - U_SPAN[0]) * f / cell)) + 1
        vmax = float(np.abs(self.v).max()) + 2 * cell
        self.v0 = -vmax
        self.nv = int(np.ceil(2 * vmax / cell)) + 1

    def index(self, x, y, phi):
        iu = np.floor((self.f * (phi + self.ax[y, x]) - self.u0) / self.cell).astype(np.int64)
        iv = np.floor((self.v[y, x] - self.v0) / self.cell).astype(np.int64)
        ok = (iu >= 0) & (iu < self.nu) & (iv >= 0) & (iv < self.nv)
        return np.where(ok, iv * self.nu + iu, -1)


def track(d, b, f, lam, seed=0):
    """프레임마다 yaw(rad). 방법은 모듈 설명의 yaw 추적. b 는 프레임 k 의 이벤트가 b[k] ~ b[k+1]-1 인 경계 배열."""
    cyl = Cyl(f, lam, TR_CELL)
    rng = np.random.default_rng(seed)
    nf = d['n_frames']
    x_all, y_all = d['x'], d['y']
    t = d['t_us'].astype(np.float64)
    M = np.zeros(cyl.nv * cyl.nu)
    phi = np.zeros(nf)
    idx = cyl.index(x_all[:b[STILL]], y_all[:b[STILL]], 0.0)
    M += np.bincount(idx[idx >= 0], minlength=M.size)
    B = ndimage.gaussian_filter(M.reshape(cyl.nv, cyl.nu), 1.0).ravel()
    for k in range(STILL, nf):
        lo, hi = b[k], b[k + 1]
        if k >= STILL + 2 and t[k - 1] > t[k - 2]:
            pred = phi[k - 1] + (phi[k - 1] - phi[k - 2]) / (t[k - 1] - t[k - 2]) * (t[k] - t[k - 1])
        else:
            pred = phi[k - 1]
        if hi - lo >= 50:
            sel = rng.choice(hi - lo, min(TR_SUB, hi - lo), replace=False) + lo
            xs, ys = x_all[sel][None, :], y_all[sel][None, :]
            best = pred
            for grid in (COARSE, FINE):
                cand = best + np.radians(grid)
                ii = cyl.index(xs, ys, cand[:, None])
                best = cand[int(np.argmax(np.where(ii >= 0, B[np.maximum(ii, 0)], 0).sum(1)))]
            phi[k] = best
        else:
            phi[k] = pred
        idx = cyl.index(x_all[lo:hi], y_all[lo:hi], phi[k])
        M += np.bincount(idx[idx >= 0], minlength=M.size)
        if (k - STILL) % BLUR_EVERY == BLUR_EVERY - 1:
            B = ndimage.gaussian_filter(M.reshape(cyl.nv, cyl.nu), 1.0).ravel()
    return phi


def world(d, b, f, lam, phi, f0, f1, cell=WL_CELL):
    """프레임 f0 ~ f1-1 을 원통면 지도(칸 cell)에 쌓는다."""
    cyl = Cyl(f, lam, cell)
    M = np.zeros(cyl.nv * cyl.nu)
    for k0 in range(f0, f1, 256):
        k1 = min(k0 + 256, f1)
        lo, hi = b[k0], b[k1]
        ph = phi[d['frame'][lo:hi]]
        idx = cyl.index(d['x'][lo:hi], d['y'][lo:hi], ph)
        M += np.bincount(idx[idx >= 0], minlength=M.size)
    return M.reshape(cyl.nv, cyl.nu)


def corr(a, b, mask):
    """mask 칸에서 두 지도의 피어슨 상관."""
    a, b = a[mask], b[mask]
    a, b = a - a.mean(), b - b.mean()
    return float((a * b).sum() / np.sqrt((a * a).sum() * (b * b).sum()))


def world_split(d, b, f, lam, phi, f0, f1, cell=WL_CELL):
    """프레임 f0 ~ f1-1 을 영상 가운데(|x - PX0| < 160)와 가장자리(>= 240) 이벤트로 나눠 쌓은 두 지도."""
    cyl = Cyl(f, lam, cell)
    C = np.zeros(cyl.nv * cyl.nu)
    E = np.zeros(cyl.nv * cyl.nu)
    for k0 in range(f0, f1, 256):
        lo, hi = b[k0], b[min(k0 + 256, f1)]
        x = d['x'][lo:hi]
        idx = cyl.index(x, d['y'][lo:hi], phi[d['frame'][lo:hi]])
        ax = np.abs(x.astype(np.float64) - PX0)
        for M, sel in ((C, ax < 160), (E, ax >= 240)):
            ii = idx[sel & (idx >= 0)]
            M += np.bincount(ii, minlength=M.size)
    return C.reshape(cyl.nv, cyl.nu), E.reshape(cyl.nv, cyl.nu)


def halves(d, b, f, lam, phi):
    """카메라가 도는 구간의 가운데 · 가장자리 지도, 둘 다 덮는 칸, 전체 상관."""
    A, Bm = world_split(d, b, f, lam, phi, STILL, d['n_frames'])
    # 두 지도가 모두 본 곳: 두 지도 각각에서 5 x 5 칸 평균 이벤트 수가 0.5 를 넘는 칸(평균으로 흐려 작은 구멍을 메움).
    # 상관은 그 칸에서, 두 지도를 σ 0.7 칸 가우시안으로 흐린 뒤 잰다
    cov = (ndimage.uniform_filter(A, 5) > 0.5) & (ndimage.uniform_filter(Bm, 5) > 0.5)
    sa, sb = ndimage.gaussian_filter(A, 0.7), ndimage.gaussian_filter(Bm, 0.7)
    return A, Bm, cov, corr(sa, sb, cov)


def window_corr(A, Bm, cov, cell=WL_CELL):
    """WIN px 창마다 가운데 · 가장자리 두 지도의 상관과 이벤트 수. 창 안 칸의 절반 이상을 둘 다 덮은 창만.

    돌려주는 값: [(창 왼쪽 위 행 칸, 열 칸, 상관, 두 지도 이벤트 합)].
    """
    w = WIN // cell
    out = []
    sa, sb = ndimage.gaussian_filter(A, 0.7), ndimage.gaussian_filter(Bm, 0.7)
    for i in range(0, A.shape[0] - w + 1, w):
        for j in range(0, A.shape[1] - w + 1, w):
            m = cov[i:i + w, j:j + w]
            if m.mean() < 0.5:
                continue
            a, bb = sa[i:i + w, j:j + w], sb[i:i + w, j:j + w]
            if a[m].std() == 0 or bb[m].std() == 0:
                continue
            out.append((i, j, corr(a, bb, m), float(A[i:i + w, j:j + w].sum() + Bm[i:i + w, j:j + w].sum())))
    return np.array(out)


# 정지 구간(프레임 0 ~ STILL-1)에서 견줄 영역: 영상 좌표 (x, y, 반지름 px)
STILL_REGIONS = {'선풍기 그릴': (268, 330, 50), '점 무늬 판': (750, 350, 50), '왼쪽 패널': (70, 150, 50),
                 '위 가운데': (517, 112, 50)}


def still_check(d, b):
    """카메라가 서 있는 동안 영역별 이벤트. 도는 날개가 있으면 그 영역만 이벤트가 많고 시간에 따라 따로 움직인다.

    돌려주는 값: ([(이름, 픽셀당 이벤트, 프레임당 평균, 원 밖 나머지 영상의 프레임별 수와의 상관)],
                  영상 전체의 프레임당 이벤트 수)
    """
    lo, hi = b[0], b[STILL]
    x, y, fr = d['x'][lo:hi].astype(np.int64), d['y'][lo:hi].astype(np.int64), d['frame'][lo:hi]
    total = np.bincount(fr, minlength=STILL).astype(float)
    yy, xx = np.mgrid[0:HEIGHT, 0:WIDTH]
    out = []
    for name, (cx, cy, r) in STILL_REGIONS.items():
        m = (xx - cx) ** 2 + (yy - cy) ** 2 < r * r
        sel = m[y, x]
        per = np.bincount(fr[sel], minlength=STILL).astype(float)
        out.append((name, sel.sum() / m.sum(), per.mean(), float(np.corrcoef(per, total - per)[0, 1])))
    return out, len(fr) / STILL


def main(path, out):
    kv = read_kv(os.path.join(out, 'lens.txt'))
    f, lam = kv['f_px'], kv['lam']
    d = load(path)
    b = np.searchsorted(d['frame'], np.arange(d['n_frames'] + 1))
    nf = d['n_frames']
    phi = track(d, b, f, lam)
    np.savetxt(os.path.join(out, 'yaw.csv'), np.column_stack([np.arange(nf), d['t_us'], np.degrees(phi)]),
               delimiter=',', fmt=['%d', '%d', '%.5f'], header='frame,t_us,yaw_deg', comments='')
    phi0 = track(d, b, f, 0.0)             # 비교: 렌즈 왜곡을 뺀 핀홀
    W = world(d, b, f, lam, phi, 0, nf)
    np.save(os.path.join(out, 'yaw_world.npy'), W.astype(np.uint32))
    A, Bm, cov, c_lens = halves(d, b, f, lam, phi)
    A0, B0, cov0, c_pin = halves(d, b, f, 0.0, phi0)
    wc = window_corr(A, Bm, cov)
    wc0 = window_corr(A0, B0, cov0)
    fov = np.degrees(np.arctan((PX0 / f) / (1 + lam * (PX0 / f) ** 2)))
    lines = [f'{os.path.basename(path)}: 프레임 {nf}, 이벤트 {len(d["frame"]):,}. 주점 ({PX0}, {PY0}), '
             f'yaw 0 구간 프레임 0 ~ {STILL - 1}',
             f'렌즈(lens.txt): f = {f:.1f} px, lam = {lam:+.4f}. 수평 시야 ±{fov:.1f}°(핀홀이면 '
             f'±{np.degrees(np.arctan(PX0 / f)):.1f}°)', '',
             f'[1] yaw: 끝(프레임 {nf - 1}) {np.degrees(phi[-1]):.2f}°, 프레임당 최대 {np.degrees(np.diff(phi)).max():.3f}°, '
             f'역방향 최대 {np.degrees(np.diff(phi)).min():.3f}°. 핀홀로 추적하면 끝 {np.degrees(phi0[-1]):.2f}°',
             f'    원통면에 쓰인 방위 {np.degrees(phi.min()) - fov:.1f}° ~ {np.degrees(phi.max()) + fov:.1f}°, '
             f'4 px 칸 월드 지도 {W.shape[1]} x {W.shape[0]} 중 이벤트 든 칸 {int((W > 0).sum()):,}',
             '', f'[2] 영상 가운데(|x - {PX0}| < 160) · 가장자리(>= 240) 이벤트로 나눈 두 지도(프레임 {STILL} ~ {nf - 1})의 상관, '
             '둘 다 덮는 칸에서(σ 0.7 칸으로 흐린 뒤)',
             f'    렌즈 왜곡 모형 {c_lens:.3f}, 핀홀(같은 f) {c_pin:.3f}. 둘 다 덮는 칸 {int(cov.sum()):,} / {int(cov0.sum()):,}',
             f'[3] {WIN} px 창별 상관(둘 다 덮는 창 {len(wc)}개 / 핀홀 {len(wc0)}개)']
    for name, w in (('렌즈 왜곡 모형', wc), ('핀홀', wc0)):
        q = np.percentile(w[:, 2], [10, 25, 50, 75, 90])
        lines.append(f'    {name}: 상관 10/25/50/75/90 % = ' + ' / '.join(f'{v:.2f}' for v in q))
    # 이벤트는 많은데 상관이 낮은 창: 이벤트 수 상위 절반 중 상관 하위 10 개
    busy = wc[wc[:, 3] >= np.median(wc[:, 3])]
    low = busy[np.argsort(busy[:, 2])[:10]]
    lines.append('    이벤트가 많은(위 절반) 창 중 상관이 가장 낮은 10 개: (U, V) px 왼쪽 위, 상관, 이벤트')
    cyl = Cyl(f, lam, WL_CELL)
    for i, j, c, n in low:
        lines.append(f'      U {cyl.u0 + j * WL_CELL:7.0f}  V {cyl.v0 + i * WL_CELL:6.0f}  상관 {c:5.2f}  이벤트 {int(n):,}')
    sc, per_frame = still_check(d, b)
    lines += ['', f'[4] 카메라가 서 있는 프레임 0 ~ {STILL - 1}(프레임당 이벤트 {per_frame:.0f}개)의 영역별 이벤트. 반지름 50 px 원',
              '    도는 날개가 있으면 그 영역만 픽셀당 이벤트가 많고, 프레임별 수가 나머지 영상과 따로 움직인다']
    for name, pp, pf, cc in sc:
        lines.append(f'    {name:<8} 픽셀당 {pp:5.2f}, 프레임당 {pf:6.1f}, 나머지 영상의 프레임별 수와 상관 {cc:+.2f}')
    text = '\n'.join(lines)
    with open(os.path.join(out, 'yaw.txt'), 'w', encoding='utf-8') as fo:
        fo.write(text + '\n')
    print(text)

    used = np.flatnonzero(W.sum(0) > 0)
    c0, c1 = used.min(), used.max() + 1
    fig, axs = plt.subplots(4, 1, figsize=(16, 15))

    def show(ax, M, title):
        L = np.log1p(M[:, c0:c1])
        ax.imshow(L, cmap='gray_r', vmax=np.percentile(L[L > 0], 99.5), aspect='equal')
        ax.set_title(title, fontsize=10)
        ax.set_xticks([])
        ax.set_yticks([])

    show(axs[0], W, f'cylinder world, all frames, f = {f:.0f} px, lens model lam = {lam:+.3f}')
    rgb = np.zeros(A.shape + (3,))
    na, nb = np.log1p(A) / np.log1p(A).max(), np.log1p(Bm) / np.log1p(Bm).max()
    rgb[..., 0], rgb[..., 1], rgb[..., 2] = 1 - nb, 1 - 0.5 * (na + nb), 1 - na
    axs[1].imshow(np.clip(rgb[:, c0:c1] * 1.0, 0, 1))
    axs[1].set_title(f'events seen at image centre (orange) vs image edges (blue), overlap is dark: corr {c_lens:.3f}',
                     fontsize=10)
    axs[1].set_xticks([])
    axs[1].set_yticks([])
    rgb0 = np.zeros(A0.shape + (3,))
    na0, nb0 = np.log1p(A0) / np.log1p(A0).max(), np.log1p(B0) / np.log1p(B0).max()
    rgb0[..., 0], rgb0[..., 1], rgb0[..., 2] = 1 - nb0, 1 - 0.5 * (na0 + nb0), 1 - na0
    u_used = np.flatnonzero(A0.sum(0) + B0.sum(0) > 0)
    axs[2].imshow(np.clip(rgb0[:, u_used.min():u_used.max() + 1], 0, 1))
    axs[2].set_title(f'pinhole (lam = 0, same f), centre vs edges: corr {c_pin:.3f}', fontsize=10)
    axs[2].set_xticks([])
    axs[2].set_yticks([])
    cm = np.full(A.shape, np.nan)
    w = WIN // WL_CELL
    for i, j, c, n in wc:
        cm[int(i):int(i) + w, int(j):int(j) + w] = c
    im = axs[3].imshow(cm[:, c0:c1], cmap='RdYlGn', vmin=-0.2, vmax=1)
    axs[3].set_title(f'{WIN} px window correlation, centre vs edges (lens model)', fontsize=10)
    axs[3].set_xticks([])
    axs[3].set_yticks([])
    fig.colorbar(im, ax=axs[3], fraction=0.02)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'yaw_world.png'), dpi=70)


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
