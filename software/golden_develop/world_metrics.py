#!/usr/bin/env python3
"""칸 크기를 바꿔도 같은 기준으로 막대를 잴 수 있게 verify_world.py 의 측정을 px 단위로 옮긴 모음.

verify_world.py 는 4 px 칸에 맞춘 상수(단면 반폭 10 칸, 바닥 띠 4 칸, 가로 이동 평균 9 칸, 선 찾기 ±3 칸)를
쓴다. 여기서는 같은 길이를 px 로 두고 칸 크기 cell 로 나눠 쓴다. cell = 4 이면 verify_world.py 와 같은 값이
나와야 하고, cell_size_check.py 가 시작할 때 이것을 확인한다.

    from world_metrics import set_cell, post_metrics
    set_cell(2)                               # 골든 모델 g 의 칸 크기 · 메모리 크기를 2 px 칸으로
    m = post_metrics(W, R_, psi_ref, center)  # 막대 4 개의 기울기 · 위치 · 가장자리 폭
"""
import os
import sys
import numpy as np
from scipy import ndimage
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import golden as g
from track_rotation import ROUGH, fit_line, profile

# 4 px 칸에서 verify_world.py 가 쓰는 길이를 px 로 적은 것
HALF_PX = 40          # 단면 반폭 (10 칸)
FLOOR_PX = 16         # 단면 양 끝의 바닥 띠 (4 칸)
PEAK_PX = 4           # 가운데에서 봉우리를 찾는 범위 (±1 칸)
VE_PX = 36            # 가로 구조를 지우는 이동 평균 폭 (9 칸)
OFF_PX = 12           # 선의 가로 위치를 훑는 범위 (±3 칸)
BAND_PX = 12          # fit_line 의 띠 반폭 (3 칸)
EXTENT_PX = 1408      # 월드 메모리 한 변(px). 4 px 칸 352 개와 같은 넓이를 칸 크기와 상관없이 담는다

CELL = 4


def set_cell(shift, bits=24):
    """골든 모델의 칸 크기를 2^shift px 로, 메모리 한 변을 EXTENT_PX 로 맞춘다. bits 는 포화 카운터 비트.

    기본 24 비트는 칸 최댓값을 자르지 않고 그대로 세려는 것이다. 월드 중심 WC 는 칸 크기와 상관없이 704 px 다.
    """
    global CELL
    CELL = 1 << shift
    g.CELL_SHIFT = shift
    g.N = g.M = EXTENT_PX >> shift
    g.WC = g.N << shift >> 1
    g.CELL_BITS = bits


def n_cells(px):
    """px 길이를 지금 칸 크기에서 몇 칸인지(적어도 1)."""
    return max(1, int(round(px / CELL)))


def to_cell(p, psi_deg, center):
    """영상 좌표 p(px) 를 회전각 ψ 에서의 월드 칸 좌표(실수)로. verify_world.to_cell 과 같다."""
    a = np.radians(psi_deg)
    dx, dy = p[0] - center[0], p[1] - center[1]
    return np.array([np.cos(a) * dx - np.sin(a) * dy + g.WC, np.sin(a) * dx + np.cos(a) * dy + g.WC]) / CELL


def vertical_edges(M):
    """가로 이동 평균을 뺀 지도. 폭은 VE_PX 를 칸으로 바꾼 홀수."""
    w = n_cells(VE_PX) | 1
    return M - ndimage.uniform_filter1d(M, w, axis=1)


def best_line(M, line, x0=None, off_px=OFF_PX, tilts=np.arange(-5, 5.001, 0.1)):
    """verify_world.best_line 과 같은 선 훑기. 가로 위치는 ±off_px 를 1 px 와 1/4 칸 중 촘촘한 간격으로 본다."""
    p, q = (np.asarray(v, float) for v in line)
    m = (p + q) / 2
    L = np.linalg.norm(q - p)
    step = min(0.25, 1.0 / CELL)
    offs = np.arange(-off_px / CELL, off_px / CELL + 1e-9, step)
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


def excess(prof):
    """단면에서 양 끝 바닥 띠의 중앙값을 뺀 값."""
    k = n_cells(FLOOR_PX)
    return prof - np.median(np.r_[prof[:k], prof[-k:]])


def edge(prof):
    """선이 놓인 가장자리 봉우리의 (반치폭 px, 봉우리 높이 / 바닥). verify_world.edge_width_px 와 같은 규칙."""
    half = (len(prof) - 1) // 2
    k = n_cells(FLOOR_PX)
    floor = float(np.median(np.r_[prof[:k], prof[-k:]]))
    e = prof - floor
    r = n_cells(PEAK_PX)
    i = half - r + int(np.argmax(e[half - r:half + r + 1]))
    lo = hi = i
    while lo > 0 and e[lo - 1] >= e[i] / 2:
        lo -= 1
    while hi < len(e) - 1 and e[hi + 1] >= e[i] / 2:
        hi += 1
    return float((hi - lo + 1) * CELL), float(e[i] / floor) if floor > 0 else float('inf')


def post_lines(W, psi_ref, center):
    """ROUGH 막대 선을 REF 가운데 회전각으로 칸 좌표에 옮겨 전체 지도에서 다듬는다(verify_world 와 같은 순서)."""
    return {k: fit_line(W, to_cell(p, psi_ref, center), to_cell(q, psi_ref, center), band=n_cells(BAND_PX))
            for k, (p, q) in ROUGH.items()}


def post_metrics(W, R_, psi_ref, center):
    """막대 4 개마다 {기울기 짧은/전체, 가로 위치 차 px, 가장자리 반치폭 짧은/전체 px, 봉우리/바닥 짧은/전체}."""
    lines = post_lines(W, psi_ref, center)
    half = n_cells(HALF_PX)
    out = {}
    for k, l in lines.items():
        xr, tr, lr = best_line(R_, l)
        xf, tf, lf = best_line(W, l)
        wr, cr = edge(profile(R_, lr, half))
        wf, cf = edge(profile(W, lf, half))
        out[k] = dict(tilt_ref=tr, tilt_full=tf, shift_px=(xf - xr) * CELL,
                      width_ref=wr, width_full=wf, contrast_ref=cr, contrast_full=cf, line=lf)
    return out
