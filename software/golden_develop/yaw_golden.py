#!/usr/bin/env python3
"""yaw(수평 회전) 녹화를 원통면 월드 메모리에 쌓는 정수 골든 모델과 RTL 테스트벤치 벡터.

    python3 software/golden_develop/yaw_golden.py <9/15 녹화.dvs> <yaw 묶음> <출력 폴더> [<f0> <f1>]

<yaw 묶음>(software/golden_data/2026-10-28_yaw)의 lens.txt(lens_yaw.py 가 맞춘 초점거리 f_px, 렌즈 왜곡 lam)와
yaw.csv(yaw_check.py 가 추적한 프레임별 yaw)를 읽는다. roll 골든 모델(golden.py)과는 별개의 모델이고, 칸 크기와 칸
비트만 golden.py 의 CELL_SHIFT · CELL_BITS 를 그대로 쓴다.

왜 이 구조인가
  원통면 월드(반지름 f)에서 카메라 yaw φ 는 가로 좌표 U 에 f · φ 를 더할 뿐이다. 그래서 roll 처럼 프레임마다
  cos · sin 을 구하고 이벤트마다 회전할 필요가 없다. 비선형인 부분(렌즈 왜곡과 원통 투영)은 픽셀 (x, y) 에만 걸린다.
  렌즈 왜곡의 배율 1 / (1 + lam · (xd^2 + yd^2)) 에 yd 가 들어가므로 방위 성분 atan(xn) 이 x 뿐 아니라 y 에도 걸린다.
  그래도 열 x 를 고정하면 U_cam, V 는 y 에 대해 매끄럽게 변하므로 y 에 대한 낮은 차수 다항식(U 는 2 차, V 는 3 차)으로
  열마다 맞추면 충분하다. 맞춘 오차와 양자화 오차는 make_lut 가 재어 yaw_golden.txt [1] 에 적는다.

정수 사양 (RTL 이 그대로 따를 것)
  e = 2y - 719                                   홀수 정수, -719 ~ 719 (y 를 주점 359.5 기준으로 2 배 한 것)
  프레임마다   PHI_k = round(f · φ_k · 16)          가로 이동, 1/16 px 단위 정수(부호 PHI_BITS 비트)
  열 x 마다    A0[x], A2[x], C1[x], C3[x]           열 표(LUT). 아래 다항식의 계수를 고정소수점으로.
                                                  A0 은 소수 A0F 비트, C1 은 소수 C1F 비트만 두고 시프트해 넣는다
  이벤트마다
      U = (PHI_k << (UF - 4)) + (A0[x] << (UF - A0F)) + A2[x] · e^2          단위 2^-UF px
      V = (C1[x] << (VF - C1F)) · e + C3[x] · e^3                             단위 2^-VF px
      wu = (U + (U_OFF << UF)) >> (UF + CELL_SHIFT)         (산술 시프트 = 내림)
      wv = (V + (V_OFF << VF)) >> (VF + CELL_SHIFT)
      0 <= wu < NU 이고 0 <= wv < NV 이면 mem[wv · NU + wu] 를 1 올리되 2^CELL_BITS - 1 에서 멈춘다. 밖이면 버린다.
  중간에 자르거나 반올림하는 곳이 없는 정수 다항식이라, 8 행 묶음(e = e0 + 2k, k = 0 ~ 7)에서는 묶음마다 곱셈 몇 번으로
  k = 0 의 값과 차분을 구한 뒤 이벤트마다 덧셈만으로 같은 값을 낸다(group_eval). U 는 2 차라 차분 두 개(ΔU, Δ²U = 8 · A2),
  V 는 3 차라 차분 세 개(ΔV, Δ²V, Δ³V = 48 · C3)다. e0² 와 e0³ 는 묶음 번호(0 ~ 89)만으로 정해져 표로 둘 수 있다.
  _self_check 가 모든 열 · 모든 묶음 · 모든 k 에서 두 계산이 비트 단위로 같은지 확인한다.

계수 만들기
  렌즈 모형(division 모형)으로 픽셀 → 광선 (xn, yn) 을 구하고 원통면 좌표를 계산한다(exact_uv).
      xd = (x - 479.5) / f,   yd = (y - 359.5) / f,   (xn, yn) = (xd, yd) / (1 + lam · (xd^2 + yd^2))
      U_cam = f · atan(xn),   V = f · yn / sqrt(1 + xn^2)
  를 열마다 e 에 대해 최소제곱으로 맞춘다: U_cam ≈ a0 + a2 · e^2, V ≈ c1 · e + c3 · e^3. 렌즈와 원통이 좌우 · 위아래로
  대칭이라 U_cam 은 e 의 짝함수, V 는 홀함수다. 맞춘 실수를 2^A0F, 2^UF, 2^C1F, 2^VF 배해 반올림한다. 열 x 와 959 - x 는
  주점(479.5)에 대해 대칭이라 왼쪽 반만 맞추고 오른쪽으로 옮긴다. U 는 x 에 대해 홀함수라 A0 · A2 는 부호를
  바꾸고, V 는 짝함수라 C1 · C3 는 그대로다. 그래서 표는 반(480 열)만 두면 된다.

출력
  <폴더>/yaw_golden.txt : 계수 근사 · 양자화 오차, 열 표 비트 수, 실수 모델과 칸이 달라진 이벤트 비율, 월드 메모리
                          크기 · 칸 최댓값 · 포화, 영상 가운데 · 가장자리 지도의 상관
  <폴더>/yaw_world.npy · yaw_world.png : 녹화 전체 누적(정수 모델)
  <폴더>/vectors/      : 프레임 f0 ~ f1-1 (기본 1000 ~ 1040) 의 RTL 테스트벤치 입력과 기댓값. write_vectors 참고
"""
import os
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'dvs_develop'))
from dvs_read import load, T_ADDR, WIDTH, HEIGHT
import golden as g

PX0, PY0 = (WIDTH - 1) / 2, (HEIGHT - 1) / 2
UF = 24            # U 의 소수 비트
VF = 32            # V 의 소수 비트. 3 차 계수가 아주 작아(약 1e-8 px) 이만큼 있어야 양자화 오차가 0.05 px 안이다
PHI_FRAC = 4       # PHI 의 소수 비트(1/16 px)
A0F = 8            # A0 를 표에 둘 때의 소수 비트(1/256 px). U 에는 << (UF - A0F) 해서 넣는다
C1F = 16           # C1 를 표에 둘 때의 소수 비트. e 가 최대 719 라 C1 양자화 오차가 0.006 px 안이다
PHI_BITS = 18      # PHI 부호 비트 수. ±8192 px 미만(1/16 단위)
U_OFF = 544        # 월드 U 원점(px). yaw 0 에서 영상 왼쪽 모서리(렌즈 왜곡까지 넣으면 U_cam = -521.6 px)가 0 보다 오른쪽에 오게
V_OFF = 384        # 월드 V 원점(px). V 는 -383 ~ 383
NU = 1024          # 월드 메모리 열 수(4 px 칸이면 4096 px). 9/15 녹화의 f(약 968 px)에서 yaw 약 179° 까지 담는다
NV = 192           # 행 수(768 px)
E_MAX = 2 * (HEIGHT - 1) - (HEIGHT - 1)    # 719


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


def exact_uv(x, y, f, lam):
    """렌즈 모형과 원통 투영의 실수 값 (U_cam, V) px. x, y 는 배열(브로드캐스트)."""
    xd = (np.asarray(x, float) - PX0) / f
    yd = (np.asarray(y, float) - PY0) / f
    s = 1 / (1 + lam * (xd * xd + yd * yd))
    xn, yn = xd * s, yd * s
    return f * np.arctan(xn), f * yn / np.sqrt(1 + xn * xn)


def make_lut(f, lam):
    """열마다 정수 계수 (A0, A2, C1, C3) 와 근사 · 양자화 오차(px)."""
    e = 2 * np.arange(HEIGHT) - E_MAX                       # y = 0 ~ 719 → e = -719 ~ 719
    U, V = exact_uv(np.arange(WIDTH)[None, :], np.arange(HEIGHT)[:, None], f, lam)
    Me = np.stack([np.ones_like(e, float), e.astype(float) ** 2], 1)
    Mo = np.stack([e.astype(float), e.astype(float) ** 3], 1)
    lut = np.zeros((WIDTH, 4), np.int64)
    fit_err = np.zeros((WIDTH, 2))
    for xi in range(WIDTH // 2):                            # 왼쪽 반을 맞추고 오른쪽은 대칭으로
        a = np.linalg.lstsq(Me, U[:, xi], rcond=None)[0]
        c = np.linalg.lstsq(Mo, V[:, xi], rcond=None)[0]
        fit_err[xi] = fit_err[WIDTH - 1 - xi] = np.abs(Me @ a - U[:, xi]).max(), np.abs(Mo @ c - V[:, xi]).max()
        lut[xi] = [round(a[0] * 2 ** A0F), round(a[1] * 2 ** UF), round(c[0] * 2 ** C1F), round(c[1] * 2 ** VF)]
        lut[WIDTH - 1 - xi] = [-lut[xi, 0], -lut[xi, 1], lut[xi, 2], lut[xi, 3]]   # U 는 x 에 대해 홀, V 는 짝
    # 정수 계수로 다시 계산한 값과 실수 값의 차(양자화까지 넣은 전체 오차)
    ee = e.astype(np.int64)[:, None]
    Ui = ((lut[None, :, 0] << (UF - A0F)) + lut[None, :, 1] * ee * ee) / 2.0 ** UF
    Vi = ((lut[None, :, 2] << (VF - C1F)) * ee + lut[None, :, 3] * ee ** 3) / 2.0 ** VF
    tot = (np.abs(Ui - U).max(), np.abs(Vi - V).max())
    return lut, fit_err, tot


def phi_int(yaw_deg, f):
    """프레임별 yaw(도)를 PHI_k = round(f · φ_k · 2^PHI_FRAC) 정수로. PHI_BITS 비트 부호 범위를 넘으면 멈춘다."""
    p = np.round(np.radians(yaw_deg) * f * (1 << PHI_FRAC)).astype(np.int64)
    assert np.all(np.abs(p) < 1 << (PHI_BITS - 1)), 'PHI 가 PHI_BITS 를 넘는다'
    return p


def cells(x, y, phi, lut):
    """정수 사양 그대로 이벤트마다 (wu, wv). phi 는 이벤트마다의 PHI_k."""
    x = np.asarray(x, np.int64)
    e = 2 * np.asarray(y, np.int64) - E_MAX
    a0, a2, c1, c3 = (lut[x, i] for i in range(4))
    U = (np.asarray(phi, np.int64) << (UF - PHI_FRAC)) + (a0 << (UF - A0F)) + a2 * e * e
    V = (c1 << (VF - C1F)) * e + c3 * e * e * e
    return (U + (U_OFF << UF)) >> (UF + g.CELL_SHIFT), (V + (V_OFF << VF)) >> (VF + g.CELL_SHIFT)


def group_eval(phi, coef, grp):
    """8 행 묶음 grp(0 ~ 89) 의 행 k = 0 ~ 7 에서 (U, V) 를 묶음마다 곱셈, 이벤트마다 덧셈으로 낸다.

    RTL 이 쓸 구조를 그대로 적은 것이다. 묶음마다
        e0 = 16 · grp - 719, 표에서 e0², e0³
        U0 = (phi << (UF - 4)) + (A0 << (UF - A0F)) + A2 · e0²,  ΔU0 = A2 · (4 · e0 + 4),  Δ²U = 8 · A2
        C = C1 << (VF - C1F)
        V0 = C · e0 + C3 · e0³,  ΔV0 = 2 · C + C3 · (6 · e0² + 12 · e0 + 8),  Δ²V0 = C3 · (24 · e0 + 48),  Δ³V = 48 · C3
    를 구하고, 행마다 값을 내보낸 뒤 차분을 더한다. 곱셈은 A2 · e0², A2 · e0, C1 · e0, C3 · e0³, C3 · e0², C3 · e0 의 6 번이고
    상수배(4, 8, 12, 24, 48)는 시프트와 덧셈이다.
    """
    a0, a2, c1, c3 = coef
    a0, c1 = a0 << (UF - A0F), c1 << (VF - C1F)
    e0 = 16 * grp - E_MAX
    U = (phi << (UF - PHI_FRAC)) + a0 + a2 * e0 * e0
    dU = a2 * (4 * e0 + 4)
    d2U = 8 * a2
    V = c1 * e0 + c3 * e0 ** 3
    dV = 2 * c1 + c3 * (6 * e0 * e0 + 12 * e0 + 8)
    d2V = c3 * (24 * e0 + 48)
    d3V = 48 * c3
    out = []
    for _ in range(8):
        out.append((U, V))
        U, dU = U + dU, dU + d2U
        V, dV, d2V = V + dV, dV + d2V, d2V + d3V
    return out


def _self_check(lut):
    """모든 열 · 묶음 · 행에서 group_eval 과 직접 계산(cells 의 U, V)이 같은지."""
    phi = 12345                                             # 아무 값. 더하기만 하므로 결과에 영향 없음
    for xi in range(WIDTH):
        coef = [int(v) for v in lut[xi]]
        for grp in range(HEIGHT // 8):
            for k, (U, V) in enumerate(group_eval(phi, coef, grp)):
                e = 2 * (8 * grp + k) - E_MAX
                assert U == (phi << (UF - PHI_FRAC)) + (coef[0] << (UF - A0F)) + coef[1] * e * e
                assert V == (coef[2] << (VF - C1F)) * e + coef[3] * e ** 3


def accumulate(wu, wv):
    """칸 (wu, wv) 들을 빈 메모리에 포화 누적한 (NV x NU 메모리, 메모리 밖이라 버린 수)."""
    ok =(wu >= 0) & (wu < NU) & (wv >= 0) & (wv < NV)
    mem = np.bincount(wv[ok] * NU + wu[ok], minlength=NU * NV)
    return np.minimum(mem, (1 << g.CELL_BITS) - 1).reshape(NV, NU), int((~ok).sum())


def run(d, phi_f, lut, f0=0, f1=None, xsel=None):
    """프레임 f0 ~ f1-1 을 누적한 (메모리, 버린 수, 이벤트 수). xsel 은 영상 열 x 로 이벤트를 고르는 함수(없으면 전부)."""
    lo, hi = np.searchsorted(d['frame'], [f0, d['n_frames'] if f1 is None else f1])
    mem, n_out, n = 0, 0, 0
    for a in range(lo, hi, 1 << 23):
        b = min(a + (1 << 23), hi)
        x, y, fr = d['x'][a:b], d['y'][a:b], d['frame'][a:b]
        if xsel is not None:
            m = xsel(x)
            x, y, fr = x[m], y[m], fr[m]
        wu, wv = cells(x, y, phi_f[fr], lut)
        ok = (wu >= 0) & (wu < NU) & (wv >= 0) & (wv < NV)
        mem = mem + np.bincount(wv[ok] * NU + wu[ok], minlength=NU * NV)
        n_out += int((~ok).sum())
        n += len(x)
    return np.minimum(mem, (1 << g.CELL_BITS) - 1).reshape(NV, NU), n_out, n


def write_vectors(path, d, phi_f, lut, f0, f1, out):
    """프레임 f0 ~ f1-1 의 테스트벤치 벡터.

    words.hex    : 원시 32 비트 워드. 프레임 f0-1 을 닫는 끝 표지부터 프레임 f1-1 을 닫는 끝 표지까지(golden.py 와 같은 규칙)
    phase.hex    : 프레임 f0 ~ f1-1 의 PHI_k (PHI_BITS 비트 2 의 보수, 16 진 5 자리)
    lut.hex      : 열 x = 0 ~ 959 마다 "A0 A2 C1 C3" (각각 2 의 보수, 폭은 params.txt 의 *_BITS)
    expected.hex : 빈 메모리에서 시작해 이 프레임들을 누적한 메모리. 주소 wv · NU + wu 순, 16 진
    params.txt   : 사양 상수와 프레임 · 워드 · 이벤트 수
    """
    os.makedirs(out, exist_ok=True)
    w = np.fromfile(path, dtype='>u4')
    a, b = (d['end_pos'][f0 - 1] if f0 else 0), d['end_pos'][f1 - 1] + 1
    np.savetxt(os.path.join(out, 'words.hex'), w[a:b], fmt='%08x')
    np.savetxt(os.path.join(out, 'phase.hex'), phi_f[f0:f1] & ((1 << PHI_BITS) - 1), fmt='%05x')
    bits = [int(np.abs(lut[:, i]).max()).bit_length() + 1 for i in range(4)]
    with open(os.path.join(out, 'lut.hex'), 'w') as fo:
        for row in lut:
            fo.write(' '.join(f'{int(v) & ((1 << bb) - 1):0{(bb + 3) // 4}x}' for v, bb in zip(row, bits)) + '\n')
    mem, n_out, n_ev = run(d, phi_f, lut, f0, f1)
    np.savetxt(os.path.join(out, 'expected.hex'), mem.ravel(), fmt=f'%0{(g.CELL_BITS + 3) // 4}x')
    n_addr = int(((w[a:b] >> 24) == T_ADDR).sum())
    with open(os.path.join(out, 'params.txt'), 'w') as fo:
        for k, v in [('file', os.path.basename(path)), ('f0', f0), ('f1', f1), ('n_frames', f1 - f0),
                     ('n_words', b - a), ('n_addr_words', n_addr), ('n_events', n_ev), ('n_dropped', n_out),
                     ('NU', NU), ('NV', NV), ('CELL_SHIFT', g.CELL_SHIFT), ('CELL_BITS', g.CELL_BITS),
                     ('UF', UF), ('VF', VF), ('A0F', A0F), ('C1F', C1F), ('PHI_FRAC', PHI_FRAC), ('PHI_BITS', PHI_BITS),
                     ('U_OFF', U_OFF), ('V_OFF', V_OFF), ('A0_BITS', bits[0]), ('A2_BITS', bits[1]),
                     ('C1_BITS', bits[2]), ('C3_BITS', bits[3]), ('mem_sum', int(mem.sum())),
                     ('mem_max', int(mem.max())), ('n_saturated', int((mem == (1 << g.CELL_BITS) - 1).sum()))]:
            fo.write(f'{k} = {v}\n')


def check_vectors(vdir):
    """벡터 폴더만으로 RTL 이 할 일을 흉내 내어 expected.hex 와 같은지 본다. 같으면 (이벤트 수, 프레임 수).

    words.hex 를 워드 단위로 푼다. DVS 워드 형식(dvs_read.py 와 같은 해석)은
      열 주소 워드  [31:24] = 0x04, [10:0] 이 열 x
      이벤트 워드   [31] = 1. 아래 묶음 비트맵 [7:0] 의 묶음 번호 G = [25:18], 위 묶음 비트맵 [15:8] 의 묶음 번호 G + A,
                    A = [30:26]. 비트맵의 비트 k 가 켜져 있으면 그 묶음의 행 k 에 이벤트가 있다
      끝 표지 워드  [31:24] = 0x0c, [18:11] 이 프레임 카운터. 카운터가 바로 앞 끝 표지와 다를 때만 프레임 경계로 본다
    이고, 첫 끝 표지는 카운터만 기억한다. 8 행 묶음마다 group_eval 로 행 0 ~ 7 의 (U, V) 를 낸 뒤 켜진
    행만 칸으로 바꿔 포화 누적한다. cells()(이벤트마다 직접 계산)와 다른 길로 같은 메모리가 나오는지 보는 것이다.
    """
    words = np.loadtxt(os.path.join(vdir, 'words.hex'), dtype=str)
    words = [int(w, 16) for w in words]
    phases = [int(p, 16) for p in np.loadtxt(os.path.join(vdir, 'phase.hex'), dtype=str, ndmin=1)]
    phases = [p - (1 << PHI_BITS) if p >> (PHI_BITS - 1) else p for p in phases]
    prm = {}
    for line in open(os.path.join(vdir, 'params.txt')):
        k, v = line.split('=', 1)
        prm[k.strip()] = v.strip()
    widths = [int(prm[k]) for k in ('A0_BITS', 'A2_BITS', 'C1_BITS', 'C3_BITS')]
    lut = []
    for line in open(os.path.join(vdir, 'lut.hex')):
        row = []
        for h, bb in zip(line.split(), widths):
            v = int(h, 16)
            row.append(v - (1 << bb) if v >> (bb - 1) else v)
        lut.append(row)
    top = (1 << g.CELL_BITS) - 1
    mem = np.zeros(NU * NV, np.int64)
    x, frame, last_fc, n_ev = 0, -1, None, 0
    shu, shv = UF + g.CELL_SHIFT, VF + g.CELL_SHIFT
    offu, offv = U_OFF << UF, V_OFF << VF
    for w in words:
        t = w >> 24
        if w >> 31:
            if frame < 0:
                continue                                      # 벡터는 끝 표지로 시작하므로 여기 오지 않는다
            G, A = (w >> 18) & 0xff, (w >> 26) & 0x1f
            for bitmap, grp in ((w & 0xff, G), ((w >> 8) & 0xff, G + A)):
                if not bitmap:
                    continue
                for k, (U, V) in enumerate(group_eval(phases[frame], lut[x], grp)):
                    if bitmap >> k & 1:
                        wu, wv = (U + offu) >> shu, (V + offv) >> shv
                        n_ev += 1
                        if 0 <= wu < NU and 0 <= wv < NV:
                            a = wv * NU + wu
                            if mem[a] < top:
                                mem[a] += 1
        elif t == T_ADDR:
            x = w & 0x7ff
        elif t == 0x0c:
            fc = (w >> 11) & 0xff
            if last_fc is None:
                frame = 0
            elif fc != last_fc:
                frame += 1
            last_fc = fc
    expected = np.array([int(h, 16) for h in np.loadtxt(os.path.join(vdir, 'expected.hex'), dtype=str)])
    assert np.array_equal(mem, expected), f'벡터 재현 불일치 {int((mem != expected).sum())} 칸'
    return n_ev, frame


def float_cells(x, y, phi_rad, f, lam):
    """실수 모델의 칸(비교용). 같은 렌즈 · 원통 · yaw 를 실수로."""
    U, V = exact_uv(x, y, f, lam)
    U = U + f * phi_rad
    return (np.floor((U + U_OFF) / (1 << g.CELL_SHIFT)).astype(np.int64),
            np.floor((V + V_OFF) / (1 << g.CELL_SHIFT)).astype(np.int64))


def corr(a, b, mask):
    """mask 칸에서 두 지도의 피어슨 상관."""
    a, b = a[mask], b[mask]
    a, b = a - a.mean(), b - b.mean()
    return float((a * b).sum() / np.sqrt((a * a).sum() * (b * b).sum()))


def main(path, ybundle, out, f0=1000, f1=1040):
    from scipy import ndimage
    os.makedirs(out, exist_ok=True)
    kv = read_kv(os.path.join(ybundle, 'lens.txt'))
    f, lam = kv['f_px'], kv['lam']
    yaw_deg = np.loadtxt(os.path.join(ybundle, 'yaw.csv'), delimiter=',', skiprows=1)[:, 2]
    lut, fit_err, tot = make_lut(f, lam)
    _self_check(lut)
    d = load(path)
    phi_f = phi_int(yaw_deg, f)
    assert len(phi_f) == d['n_frames']

    # 실수 모델과 칸이 달라진 이벤트(8 개 중 하나)
    sl = slice(0, len(d['frame']), 8)
    x, y, fr = d['x'][sl], d['y'][sl], d['frame'][sl]
    wu, wv = cells(x, y, phi_f[fr], lut)
    fu, fv = float_cells(x, y, np.radians(yaw_deg)[fr], f, lam)
    diff = float(np.mean((wu != fu) | (wv != fv)))
    W, n_out, n_ev = run(d, phi_f, lut)
    np.save(os.path.join(out, 'yaw_world.npy'), W.astype(np.uint16))
    # 영상 가운데 · 가장자리 이벤트로 나눈 지도의 상관. yaw_check.py 의 halves 와 같은 시험(같은 x 경계 160 · 240,
    # 같은 덮는 칸 판정과 σ 0.7 칸 흐림)을 정수 모델 메모리로 한다. 카메라가 도는 프레임 200 부터
    C, _, _ = run(d, phi_f, lut, 200, None, lambda xx: np.abs(xx.astype(float) - PX0) < 160)
    E, _, _ = run(d, phi_f, lut, 200, None, lambda xx: np.abs(xx.astype(float) - PX0) >= 240)
    C, E = C.astype(float), E.astype(float)
    cov = (ndimage.uniform_filter(C, 5) > 0.5) & (ndimage.uniform_filter(E, 5) > 0.5)
    c_ce = corr(ndimage.gaussian_filter(C, 0.7), ndimage.gaussian_filter(E, 0.7), cov)
    write_vectors(path, d, phi_f, lut, f0, f1, os.path.join(out, 'vectors'))
    vec_ev, vec_fr = check_vectors(os.path.join(out, 'vectors'))

    bits = [int(np.abs(lut[:, i]).max()).bit_length() + 1 for i in range(4)]
    sym = (all(np.array_equal(lut[:, i], -lut[::-1, i]) for i in (0, 1)) and
           all(np.array_equal(lut[:, i], lut[::-1, i]) for i in (2, 3)))
    lines = [f'yaw 정수 골든 모델. {os.path.basename(path)}, 렌즈 f = {f:.2f} px, lam = {lam:+.5f} (yaw 묶음 lens.txt)',
             f'월드 메모리 {NU} x {NV} = {NU * NV:,} 칸({1 << g.CELL_SHIFT} px 칸, {g.CELL_BITS}비트 포화), '
             f'U_OFF {U_OFF} px, V_OFF {V_OFF} px. 담는 yaw 범위 약 {np.degrees((NU * (1 << g.CELL_SHIFT) - U_OFF - 522) / f):.0f}°',
             '', '[1] 열 표(LUT)',
             f'    열마다 다항식으로 맞춘 오차(실수 계수): U 최대 {fit_err[:, 0].max():.3f} px, V 최대 {fit_err[:, 1].max():.3f} px',
             f'    정수 계수까지 넣은 전체 오차: U 최대 {tot[0]:.3f} px, V 최대 {tot[1]:.3f} px',
             f'    계수 비트(부호 포함): A0 {bits[0]}, A2 {bits[1]}, C1 {bits[2]}, C3 {bits[3]}. 한 열 {sum(bits)}비트 x {WIDTH}열 = '
             f'{sum(bits) * WIDTH:,}비트' + (f'. 좌우 대칭(A0 · A2 는 부호만 다르고 C1 · C3 는 같음)이라 반만 두면 {sum(bits) * WIDTH // 2:,}비트' if sym else ''),
             f'    8 행 묶음 차분 계산(group_eval)이 모든 열 {WIDTH} · 묶음 {HEIGHT // 8} · 행 8 에서 직접 계산과 같음',
             '', '[2] 녹화 전체',
             f'    이벤트 {n_ev:,}, 메모리 밖 {n_out:,}, 칸 최댓값 {int(W.max())}, 포화 칸 {int((W == (1 << g.CELL_BITS) - 1).sum())}, '
             f'이벤트 든 칸 {int((W > 0).sum()):,}',
             f'    실수 모델(같은 렌즈 · 원통 · yaw)과 칸이 다른 이벤트(8 개 중 하나): {diff * 100:.3f} %',
             f'    영상 가운데(|x - {PX0}| < 160) · 가장자리(>= 240) 이벤트로 나눈 두 지도의 상관 {c_ce:.3f} '
             '(yaw_check.py 의 실수 모델 값과 견줌)',
             '', '[3] 테스트벤치 벡터',
             f'    check_vectors: words.hex 를 워드 단위로 풀고 8 행 묶음 차분 계산으로 누적한 메모리가 expected.hex 와 '
             f'칸 단위로 같음(이벤트 {vec_ev:,}, 프레임 경계 {vec_fr}개)',
             open(os.path.join(out, 'vectors', 'params.txt')).read().rstrip()]
    text = '\n'.join(lines)
    with open(os.path.join(out, 'yaw_golden.txt'), 'w', encoding='utf-8') as fo:
        fo.write(text + '\n')
    print(text)
    used = np.flatnonzero(W.sum(0) > 0)
    L = np.log1p(W[:, used.min():used.max() + 1].astype(float))
    fig, ax = plt.subplots(figsize=(16, 4.5))
    ax.imshow(L, cmap='gray_r', vmax=np.percentile(L[L > 0], 99.5))
    ax.set_title(f'yaw integer golden model, cylinder world {NU} x {NV} cells (used columns shown), whole recording', fontsize=10)
    ax.set_xticks([])
    ax.set_yticks([])
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'yaw_world.png'), dpi=70)


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], sys.argv[3], *map(int, sys.argv[4:6]))
