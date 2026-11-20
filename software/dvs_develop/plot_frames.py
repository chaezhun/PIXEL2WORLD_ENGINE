#!/usr/bin/env python3
"""녹화 하나에서 고르게 떨어진 시점 8곳의 이벤트를 그림 한 장(2x4)으로 그린다.

    python3 software/dvs_develop/plot_frames.py <파일>.dvs <출력>.png [묶을 프레임 수=4]

각 칸은 연속한 프레임 몇 개(기본 4 프레임, 약 3 ms)의 이벤트를 겹친 것이다. ON 은 빨강, OFF 는 파랑,
이벤트가 없는 곳은 흰색이다(DVS Viewer 의 RGB 기본값과 같다). 한 픽셀에 ON · OFF 가 다 오면 나중
것이 남는다. 장면이 무엇인지, 물체가 영상 안에서 어떻게 움직이는지 눈으로 보려는 그림이다.
"""
import os
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from dvs_read import load, WIDTH, HEIGHT

ON, OFF = np.array([214, 39, 40], np.uint8), np.array([31, 119, 180], np.uint8)


def render(d, f0, n):
    """프레임 f0 부터 n 장을 겹친 RGB 영상(높이 x 너비 x 3)."""
    lo, hi = np.searchsorted(d['frame'], [f0, f0 + n])
    img = np.full((HEIGHT, WIDTH, 3), 255, np.uint8)
    y, x, p = d['y'][lo:hi], d['x'][lo:hi], d['p'][lo:hi]
    img[y, x] = np.where(p[:, None] == 0, ON, OFF)
    return img


def main(path, out, n=4):
    d = load(path)
    starts = np.linspace(1, d['n_frames'] - n, 8).astype(int)   # 첫 프레임은 앞부분이 잘려 뺀다
    fig, axes = plt.subplots(2, 4, figsize=(16, 6.4))
    for ax, f0 in zip(axes.ravel(), starts):
        ax.imshow(render(d, f0, n), interpolation='nearest')
        t = (d['t_us'][f0] - d['t_us'][1]) / 1e3
        ax.set_title(f'frame {f0}-{f0 + n - 1}  (t = {t:.0f} ms)', fontsize=10)
        ax.set_xticks([0, 480, 959])
        ax.set_yticks([0, 360, 719])
        ax.tick_params(labelsize=8)
    fig.suptitle(f'{os.path.basename(path)}   ON red / OFF blue, {n} frames per panel, x right, y down',
                 fontsize=11)
    fig.tight_layout()
    fig.savefig(out, dpi=80)
    print(out)


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], *(int(a) for a in sys.argv[3:4]))
