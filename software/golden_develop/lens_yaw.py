#!/usr/bin/env python3
"""yaw 녹화에서 초점거리(px)와 렌즈 방사 왜곡을 함께 추정한다.

    python3 software/golden_develop/lens_yaw.py <yaw 녹화.dvs> <출력 폴더>

왜 필요한가
  핀홀 렌즈라면 순수 yaw 에서 영상의 가로 속도는 깊이와 상관없이 가운데에서 가장 느리고 가장자리로 갈수록
  1 + ((x - PX0) / f)^2 배 빨라진다. 9/15 녹화에서는 반대로 가장자리가 가운데보다 몇 % 느리다. 렌즈의 배럴
  왜곡이 가장자리를 누르기 때문이다. 그래서 핀홀 모형으로 f 를 고르면 맞지 않는다.

모형
  주점은 영상 가운데 (PX0, PY0). 픽셀을 f 로 나눈 좌표 (xd, yd) = ((x - PX0)/f, (y - PY0)/f), 반지름 rd 에서
  광선 방향 (xn, yn) = (광선 x/z, y/z) 은 division 모형
      (xn, yn) = (xd, yd) / (1 + L · rd^2)
  으로 얻는다. L < 0 이면 가장자리가 눌린 배럴 왜곡이다. 픽셀 → 광선이 식 하나로 바로 나와서 모서리까지 풀 수
  있다. 다항 모형 rd = rn (1 + K1 rn^2) 은 K1 < 0 이면 rd 가 rn = 1 / sqrt(-3 K1) 에서 최댓값
  (2/3) / sqrt(-3 K1) 을 가지고 그 너머는 식이 접혀 거꾸로 풀 수 없다. 같은 대응에 다항 모형을 맞추면
  K1 = -0.39 안팎이 나오는데, 이때 한계가 rd = 0.614 라서 영상 모서리(rd = 0.616)를 풀 수 없다. 그래서 division
  모형을 쓰고, lens.txt 에는 다항 모형으로 맞춘 값(f = 974.4 px, K1 = -0.3930)을 비교용 고정 문장으로 적는다.
  카메라가 세로축 둘레로 Δφ 돌면, 세로 위치가 영상 가운데 근처인 점의 방위각 atan(xn) 이 Δφ 만큼 바뀌고
  원통면 높이 h = yn · cos(방위각) 는 그대로다. 그래서
    가로  : 픽셀 열 X 에 있던 것이 X' 로 간다.  atan(xn(X')) = atan(xn(X)) - Δφ
    세로  : 같은 물체의 세로 크기가 m(X') / m(X) 배가 된다.  m(x) = sqrt(1 + xn^2) · (1 + L · xd^2)
  작은 이동에서는 가로 속도 비의 2 차 항 하나만 정해지지만, 이동이 크면(150 ~ 330 px) 고차 항이 f 와 L 을 가른다.
  세로 배율은 효과가 1 % 안팎이라 보탬이 작다.

측정
  카메라가 도는 구간에서 시점 t 와 t + Δ(가운데가 150 px 넘게 움직이도록 고름) 의 이벤트 영상(6 프레임 누적,
  σ = 1 px 로 흐림)을 만든다. 영상 오른쪽 띠(가로 90 px, 세로 가운데 320 행)마다 가로 이동을 상관으로 찾아
  (부화소는 봉우리 둘레 포물선) X → X' 대응을 얻고, 같은 띠(세로 660 행)를 X' 에서 세로로 σ 배 늘인 것과의
  상관이 가장 큰 σ 를 찾는다. 세로 상관이 0.5 넘는 대응만 남겨, 쌍마다 Δφ 를 두고 (f, L, Δφ...) 를 가로 대응과
  세로 배율로 함께 맞춘다. 흔들림은 시점 쌍을 반으로 나눈 맞춤, 세로 배율 무게를 바꾼 맞춤, 시점 쌍 bootstrap
  40 번으로 본다.

출력
  <폴더>/lens.txt : 측정한 대응 수, 맞춘 f · L, 잔차, 비교용 핀홀(L = 0) · 가로만 맞춤, 흔들림, 끝에 채택한 값
                    f_px · lam 을 key = value 로
  <폴더>/lens_pairs.csv : 대응 원자료 (t, Δ, X, X', σ, 가로 상관, 세로 상관)
"""
import os
import sys
import numpy as np
from scipy import ndimage
from scipy.optimize import least_squares
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'dvs_develop'))
from dvs_read import load, WIDTH, HEIGHT

PX0, PY0 = (WIDTH - 1) / 2, (HEIGHT - 1) / 2
NACC = 6                      # 이벤트 영상 하나에 겹치는 프레임 수
HB = 45                       # 가로 이동을 재는 띠의 반폭(px)
HROWS = slice(200, 520)       # 가로 이동을 재는 행(세로 이동이 작은 가운데)
VROWS = np.arange(30, 690)    # 세로 배율을 재는 행
VB = 30                       # 세로 배율을 재는 띠의 반폭
SIGMAS = np.arange(0.95, 1.0501, 0.0025)


def xn_of(x, f, lam):
    """세로 가운데 행의 픽셀 열 x → 광선 xn (division 모형)."""
    xd = (np.asarray(x, float) - PX0) / f
    return xd / (1 + lam * xd * xd)


def x_of(xn, f, lam):
    """xn → 픽셀 열. xd / (1 + lam xd^2) = xn 의 작은 근."""
    xn = np.asarray(xn, float)
    if lam == 0:
        return PX0 + f * xn
    small = np.abs(xn) < 1e-12
    q = np.where(small, 1.0, xn)
    xd = (1 - np.sqrt(np.maximum(1 - 4 * lam * q * q, 0))) / (2 * lam * q)
    return PX0 + f * np.where(small, 0.0, xd)


def m_of(x, f, lam):
    """세로 가운데 행의 픽셀 열 x 에서 세로 배율 m(x) = sqrt(1 + xn^2) · (1 + lam · xd^2). 두 열의 비가 세로 크기 비다."""
    xd = (np.asarray(x, float) - PX0) / f
    xn = xd / (1 + lam * xd * xd)
    return np.sqrt(1 + xn * xn) * (1 + lam * xd * xd)


def poly_xn(x, f, k1):
    """비교용 다항 모형 rd = rn (1 + k1 rn^2) 의 역(뉴턴 30 번). 이 파일의 맞춤에서는 부르지 않는다."""
    rd = (np.asarray(x, float) - PX0) / f
    rn = rd.copy()
    for _ in range(30):
        rn = rn - (rn * (1 + k1 * rn * rn) - rd) / (1 + 3 * k1 * rn * rn)
    return rn


class Frames:
    """프레임 경계를 한 번 구해 두고, 시작 프레임에서 NACC 프레임을 겹친 이벤트 영상을 만든다."""

    def __init__(self, d):
        self.d = d
        self.b = np.searchsorted(d['frame'], np.arange(d['n_frames'] + 1))

    def img(self, f0):
        """프레임 f0 ~ f0 + NACC - 1 의 이벤트 수 영상(극성 무시)을 σ = 1 px 가우시안으로 흐린 것."""
        lo, hi = self.b[f0], self.b[f0 + NACC]
        m = np.zeros((HEIGHT, WIDTH), np.float32)
        np.add.at(m, (self.d['y'][lo:hi], self.d['x'][lo:hi]), 1)
        return ndimage.gaussian_filter(m, 1.0)


def ncc(a, b):
    """두 배열의 정규화 상호상관(평균을 빼고 크기로 나눈 내적, -1 ~ 1)."""
    a = a - a.mean()
    b = b - b.mean()
    return float((a * b).sum() / np.sqrt((a * a).sum() * (b * b).sum() + 1e-12))


def hshift(a, b, X, smin=-330, smax=10):
    """띠 X 의 내용이 b 에서 몇 px 옮겨 갔나(부화소). (이동, 상관)."""
    A = a[HROWS, X - HB:X + HB]
    ss, vs = [], []
    for s in range(smin, smax + 1):
        lo = X - HB + s
        if lo < 0 or lo + 2 * HB > WIDTH:
            continue
        ss.append(s)
        vs.append(ncc(A, b[HROWS, lo:lo + 2 * HB]))
    if not vs:
        return None, -1
    i = int(np.argmax(vs))
    s = float(ss[i])
    if 0 < i < len(vs) - 1:
        y0, y1, y2 = vs[i - 1], vs[i], vs[i + 1]
        den = y0 - 2 * y1 + y2
        if den < 0:
            s += 0.5 * (y0 - y2) / den
    return s, vs[i]


def vscale(a, b, X, X2):
    """띠 X(시점 t)와 X2(시점 t+Δ)의 세로 배율 σ. (σ, 상관)."""
    A = a[VROWS, X - VB:X + VB]
    best = (-2.0, 1.0)
    for sg in SIGMAS:
        yy = PY0 + (VROWS - PY0) * sg
        for dx in (-1.0, -0.5, 0.0, 0.5, 1.0):
            cols = np.arange(-VB, VB) + X2 + dx
            if cols[0] < 0 or cols[-1] > WIDTH - 1:
                continue
            B = ndimage.map_coordinates(b, [np.repeat(yy, len(cols)), np.tile(cols, len(yy))],
                                        order=1).reshape(len(yy), len(cols))
            v = ncc(A, B)
            if v > best[0]:
                best = (v, sg)
    return best[1], best[0]


def measure(d):
    """X → X' 대응과 세로 배율을 모은다. 행마다 (t0, Δ, X, X', σ, 가로 상관, 세로 상관).

    시작 프레임 t0 를 640 부터 30 프레임 간격으로 두고, Δ 를 100 · 140 · 180 · 220 · 250 프레임 순으로 늘려 영상
    가운데 띠가 150 px 넘게(왼쪽으로) 움직이고 상관이 0.3 넘는 첫 Δ 를 쓴다. 그런 Δ 가 없으면 그 t0 는 건너뛴다.
    그 쌍에서 X = 500, 550, ..., 850 띠마다 가로 이동과 세로 배율을 잰다. 가로 상관이 0.3 미만이거나 옮겨 간
    띠가 영상 왼쪽 끝에 걸리면 버린다.
    """
    fr = Frames(d)
    rows = []
    for t0 in range(640, d['n_frames'] - 260, 30):
        a = fr.img(t0)
        ok = False
        for dl in (100, 140, 180, 220, 250):
            if t0 + dl + NACC >= d['n_frames']:
                break
            b = fr.img(t0 + dl)
            s, c = hshift(a, b, int(PX0))
            if s is not None and s < -150 and c > 0.3:
                ok = True
                break
        if not ok:
            continue
        for X in range(500, 900, 50):
            s, c = hshift(a, b, X)
            if s is None or c < 0.3:
                continue
            X2 = X + s
            if X2 < VB + 2:
                continue
            sg, cv = vscale(a, b, X, X2)
            rows.append((t0, dl, X, X2, sg, c, cv))
            print(t0, dl, X, round(X2, 1), round(sg, 4), round(c, 2), round(cv, 2), flush=True)
    return np.array(rows)


def fit(R, k1_free=True, use_v=True, wv=300.0):
    """(f, L, 쌍마다 Δφ) 를 가로 대응(px)과 세로 배율(log, 가중)로 맞춘다."""
    keys = sorted(set(zip(R[:, 0].astype(int), R[:, 1].astype(int))))
    pid = np.array([keys.index((int(t), int(dl))) for t, dl in R[:, :2]])
    X, X2, sg = R[:, 2], R[:, 3], R[:, 4]
    WV = wv        # 기본 300: 세로 배율 1 % 를 가로 3 px 와 같은 무게로

    def resid(p):
        f, k1 = p[0], (p[1] if k1_free else 0.0)
        dphi = p[2:]
        pred = x_of(np.tan(np.arctan(xn_of(X, f, k1)) - dphi[pid]), f, k1)
        r = [pred - X2]
        if use_v:
            r.append(WV * (np.log(m_of(X2, f, k1) / m_of(X, f, k1)) - np.log(sg)))
        return np.concatenate(r)

    p0 = np.r_[1000.0, -0.3, np.zeros(len(keys))]
    for i, (t, dl) in enumerate(keys):
        sel = pid == i
        p0[2 + i] = np.median((X[sel] - X2[sel])) / 1000.0
    lb = np.r_[200.0, -1.5, -np.inf * np.ones(len(keys))]
    ub = np.r_[5000.0, 1.0, np.inf * np.ones(len(keys))]
    sol = least_squares(resid, p0, bounds=(lb, ub), x_scale='jac')
    n = len(X)
    rh = sol.fun[:n]
    rv = sol.fun[n:] / WV if use_v else np.array([0.0])
    return sol, rh, rv


def main(path, out):
    os.makedirs(out, exist_ok=True)
    pairs = os.path.join(out, 'lens_pairs.csv')
    if os.path.exists(pairs):          # 측정은 오래 걸리므로(10 분 안팎) lens_pairs.csv 가 이미 있으면 측정을 건너뛰고 읽는다
        R = np.loadtxt(pairs, delimiter=',', skiprows=1)
    else:
        R = measure(load(path))
        np.savetxt(pairs, R, delimiter=',', fmt='%.4f', header='t0,delta,X,X2,sigma,ncc_h,ncc_v', comments='')
    good = R[R[:, 6] > 0.5]
    lines = [f'{os.path.basename(path)}: 대응 {len(R)} 개, 세로 상관 0.5 넘는 것 {len(good)} 개 '
             f'(시점 쌍 {len(set(map(tuple, good[:, :2])))} 개). 주점 ({PX0}, {PY0})', '']
    for name, kf, uv in (('방사 왜곡 + 세로 배율 (채택)', True, True), ('핀홀, 가로만', False, False),
                         ('핀홀, 가로 + 세로', False, True), ('방사 왜곡, 가로만', True, False)):
        sol, rh, rv = fit(good, kf, uv)
        f, k1 = sol.x[0], (sol.x[1] if kf else 0.0)
        lines.append(f'[{name}] f = {f:.1f} px, L = {k1:+.4f}, 가로 잔차 RMS {np.sqrt((rh ** 2).mean()):.2f} px, '
                     f'세로 배율 잔차 RMS {np.sqrt((rv ** 2).mean()) * 100:.3f} %')
        if name.endswith('(채택)'):
            best = (f, k1)
            xs = np.array([PX0 + 100, PX0 + 200, PX0 + 300, PX0 + 400, PX0 + 480])
            xn = xn_of(xs, f, k1)
            xd = (xs - PX0) / f
            sp = (1 + xn * xn) * (1 + k1 * xd * xd) ** 2 / (1 - k1 * xd * xd)    # dx/dφ 를 가운데로 나눈 값
            lines.append('    가로 속도 비(가운데 = 1) x - PX0 = 100/200/300/400/480 px: ' +
                         ' / '.join(f'{v:.3f}' for v in sp))
            corner = np.hypot(PX0, PY0) / f
            lines.append(f'    모서리(rd = {corner:.3f})의 광선 반지름 {corner / (1 + k1 * corner ** 2):.3f}, '
                         f'1 + L rd^2 = {1 + k1 * corner ** 2:.3f} (> 0 이라 모서리까지 풀림)')
            lines.append('    세로 배율 m(x) 같은 자리: ' + ' / '.join(f'{v:.4f}' for v in m_of(xs, f, k1)))
            lines.append(f'    6 mm 렌즈라면 픽셀 크기 {6000 / f:.2f} µm')
    lines.append('    비교: rd = rn (1 + K1 rn^2) 모형으로 같은 대응을 맞추면 f = 974.4 px, K1 = -0.3930 이었다(10-28 첫 실행). '
                 '이 K1 에서는 rd > 0.614 를 풀 수 없어 모서리 픽셀(rd = 0.616)을 쓸 수 없다')
    # 쌍을 반으로 나눠 다시 맞춰 흔들림을 본다
    keys = sorted(set(map(tuple, good[:, :2])))
    half = [set(keys[0::2]), set(keys[1::2])]
    for i, hk in enumerate(half):
        sub = good[[tuple(r[:2]) in hk for r in good]]
        sol, _, _ = fit(sub)
        lines.append(f'    시점 쌍 절반 {i + 1}: f = {sol.x[0]:.1f} px, L = {sol.x[1]:+.4f}')
    # 세로 배율의 무게를 바꾸면
    lines.append('    세로 배율 무게(기본 300)를 바꾸면: ' + ', '.join(
        f'{wv:g} → f {fit(good, wv=wv)[0].x[0]:.0f} px' for wv in (100.0, 1000.0, 3000.0)))
    # 시점 쌍을 복원 추출(bootstrap)해 40 번 다시 맞춘다
    rng = np.random.default_rng(0)
    fs = []
    for _ in range(40):
        parts = []
        for n, j in enumerate(rng.integers(0, len(keys), len(keys))):
            sub = good[[tuple(r[:2]) == keys[j] for r in good]].copy()
            sub[:, 0] += n * 100000          # 같은 쌍이 두 번 뽑혀도 따로 Δφ 를 두게
            parts.append(sub)
        fs.append(fit(np.concatenate(parts))[0].x[0])
    lo, hi = np.percentile(fs, [16, 84])
    lines.append(f'    시점 쌍 bootstrap 40 번: f 중앙값 {np.median(fs):.0f} px, 16 ~ 84 % {lo:.0f} ~ {hi:.0f} px')
    text = '\n'.join(lines)
    with open(os.path.join(out, 'lens.txt'), 'w', encoding='utf-8') as fo:
        fo.write(text + '\n')
        fo.write(f'f_px = {best[0]:.2f}\nlam = {best[1]:.5f}\n')
    print(text)


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
