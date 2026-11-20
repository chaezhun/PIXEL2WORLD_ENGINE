#!/usr/bin/env python3
"""녹화 안에서 영상이 얼마나, 어느 점을 중심으로 회전하는지 짧은 구간끼리 맞대어 잰다.

    python3 software/dvs_develop/roll_check.py <출력 폴더> <파일>.dvs [<파일>.dvs ...]

장면 판별(장면 1 · 2)과 광축 회전(roll) 확인용 거친 측정이다. SW 골든 모델의 회전 추정이 아니다.

방법
  1. 이벤트를 4x4 로 묶어(240x180) 프레임 W 장씩 쌓은 개수 영상을 만든다. 극성은 보지 않는다.
  2. S 프레임 떨어진 두 영상 A, B 에 대해, 각도 th 마다 A 를 영상 중심 o 둘레로 th 만큼 돌린 뒤
     B 와 위상 상관(phase correlation)으로 남은 평행이동 t 를 찾는다. 상관 봉우리가 가장 높은 th 를 고른다.
  3. o 둘레 회전 + 평행이동 t 는 한 점 c 둘레의 회전과 같다. (I - R)(c - o) = t 를 풀어 c 를 얻는다.
     c 가 영상의 회전 중심이다. 센서가 회전축에서 떨어져 돌수록 c 는 영상 중심에서 멀어지고 흔들린다.
  4. 겹치지 않게 이어 붙인 쌍의 th 를 더하면 녹화 전체의 회전량이 된다.
각도 부호: 양수는 화면(y 아래 방향)에서 시계 방향.

출력: <폴더>/roll_<파일 앞 19자>.csv (쌍마다 프레임, 시각, 각도, 회전 중심, 상관 봉우리), roll.png
"""
import os
import sys
import numpy as np
from scipy import ndimage
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from dvs_read import load, WIDTH, HEIGHT

B = 4                                    # 묶음 크기
W, S = 6, 40                             # 쌓을 프레임 수, 쌍 사이 간격(프레임)
THETAS = np.arange(-20, 20.001, 0.25)    # 찾을 각도(도). 40 프레임(31 ms)에 한 바퀴/1.6 s 면 약 7도
H2, W2 = HEIGHT // B, WIDTH // B
O = np.array([(W2 - 1) / 2, (H2 - 1) / 2])            # 영상 중심 (x, y), 묶은 좌표
HANN = np.outer(np.hanning(H2), np.hanning(W2))


def counts(d, f0):
    lo, hi = np.searchsorted(d['frame'], [f0, f0 + W])
    idx = (d['y'][lo:hi] // B).astype(np.int64) * W2 + d['x'][lo:hi] // B
    return np.bincount(idx, minlength=H2 * W2).reshape(H2, W2).astype(np.float64)


def rotate(img, th_deg):
    """영상을 중심 O 둘레로 th 만큼 돌린다(화면에서 시계 방향이 양수)."""
    c, s = np.cos(np.radians(th_deg)), np.sin(np.radians(th_deg))
    m = np.array([[c, -s], [s, c]])                     # (행, 열) 좌표에서 R 의 역행렬
    o = O[::-1]
    return ndimage.affine_transform(img, m, offset=o - m @ o, order=1)


def shift(a, b):
    """b 가 a 를 얼마나 옮긴 것인지 (x, y) 와 상관 봉우리 높이. 봉우리 둘레 포물선으로 소수점까지."""
    fa, fb = np.fft.fft2(a * HANN), np.fft.fft2(b * HANN)
    r = fb * np.conj(fa)
    corr = np.fft.ifft2(r / (np.abs(r) + 1e-12)).real
    py, px = np.unravel_index(np.argmax(corr), corr.shape)

    def sub(m1, m0, p1):
        den = m1 - 2 * m0 + p1
        return 0.0 if den == 0 else 0.5 * (m1 - p1) / den

    dy = py + sub(corr[py - 1, px], corr[py, px], corr[(py + 1) % H2, px])
    dx = px + sub(corr[py, px - 1], corr[py, px], corr[py, (px + 1) % W2])
    dy = dy - H2 if dy > H2 / 2 else dy
    dx = dx - W2 if dx > W2 / 2 else dx
    return np.array([dx, dy]), corr[py, px]


def estimate(a, b):
    """a 를 b 로 옮기는 회전 각도(도)와 회전 중심(묶은 좌표 x, y), 상관 봉우리."""
    best = max(((th,) + shift(rotate(a, th), b) for th in THETAS), key=lambda r: r[2])
    th, t, peak = best
    if th == 0:
        return 0.0, np.array([np.nan, np.nan]), peak
    c, s = np.cos(np.radians(th)), np.sin(np.radians(th))
    center = O + np.linalg.solve(np.eye(2) - np.array([[c, -s], [s, c]]), t)
    return th, center, peak


def self_check():
    """알고 있는 회전으로 만든 영상에서 각도와 중심을 되찾는지 본다."""
    rng = np.random.default_rng(0)
    a = ndimage.gaussian_filter(rng.random((H2, W2)) ** 8, 1.0)
    th, c_true = 6.0, np.array([150.0, 70.0])
    cs, sn = np.cos(np.radians(th)), np.sin(np.radians(th))
    m = np.array([[cs, -sn], [sn, cs]])                 # 행, 열 좌표의 역회전
    cr = c_true[::-1]
    b = ndimage.affine_transform(a, m, offset=cr - m @ cr, order=1)
    th_e, c_e, _ = estimate(a, b)
    assert abs(th_e - th) <= 0.25 and np.linalg.norm(c_e - c_true) < 3, (th_e, c_e)


def run(path):
    d = load(path)
    rows = []
    for f0 in range(1, d['n_frames'] - S - W, S):
        th, c, peak = estimate(counts(d, f0), counts(d, f0 + S))
        rows.append((f0, (d['t_us'][f0] - d['t_us'][1]) / 1e3, th, c[0] * B, c[1] * B, peak))
    return np.array(rows)


def main(out, paths):
    self_check()
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 4.8))
    colors = ['#2a78d6', '#eb6834']
    for path, col in zip(paths, colors):
        r = run(path)
        name = os.path.basename(path)[:19]
        np.savetxt(os.path.join(out, f'roll_{name}.csv'), r, delimiter=',', fmt='%.4f',
                   header='frame,t_ms,theta_deg,center_x_px,center_y_px,peak', comments='')
        cum = np.cumsum(r[:, 2])
        good = r[:, 5] > np.median(r[:, 5]) * 0.5
        cx, cy = r[good, 3], r[good, 4]
        print(f'{name}: 쌍 {len(r)}, 누적 회전 {cum[-1]:.1f} 도, 쌍당 각도 중앙값 {np.median(r[:, 2]):.2f} 도'
              f' ({S} 프레임), 회전 중심 중앙값 ({np.nanmedian(cx):.0f}, {np.nanmedian(cy):.0f}) px,'
              f' 사분위 범위 x {np.nanpercentile(cx, 25):.0f}~{np.nanpercentile(cx, 75):.0f}'
              f' y {np.nanpercentile(cy, 25):.0f}~{np.nanpercentile(cy, 75):.0f}')
        ax1.plot(r[:, 1] + r[0, 1] * 0, cum, color=col, lw=2, label=name)
        ax1.annotate(name[11:], (r[-1, 1], cum[-1]), color='#333333', fontsize=9,
                     xytext=(4, 0), textcoords='offset points', va='center')
        ax2.plot(cx, cy, 'o', ms=4, color=col, alpha=0.6, label=name)
    ax1.set_xlabel('time from 2nd frame (ms)')
    ax1.set_ylabel('cumulative image rotation (deg, clockwise +)')
    ax1.grid(alpha=0.3)
    ax1.legend(frameon=False, fontsize=9)
    ax2.set_xlim(0, WIDTH)
    ax2.set_ylim(HEIGHT, 0)
    ax2.set_aspect('equal')
    ax2.plot([WIDTH / 2], [HEIGHT / 2], '+', color='#333333', ms=12)
    ax2.set_title('estimated rotation center per pair (+ = image center)', fontsize=10)
    ax2.legend(frameon=False, fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'roll.png'), dpi=80)


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2:])
