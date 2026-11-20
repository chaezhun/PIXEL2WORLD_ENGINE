#!/usr/bin/env python3
"""센서 노이즈와 핫픽셀이 월드 메모리 누적과 막대 검증 수치에 주는 영향을 잰다.

    python3 software/golden_develop/noise_check.py <녹화>.dvs <골든 묶음> <출력 폴더>

주최 측 녹화는 노이즈를 거르지 않은 원본이다. 센서가 돌기 전 구간 REST(장면이 움직이지 않아 이벤트가 거의
노이즈뿐)에서 노이즈의 양을 재고, 두 가지 거르기를 골든 모델 앞에 넣어 막대 수치가 달라지는지 본다.

  핫픽셀 가리기 : REST 프레임의 절반 이상에서 이벤트를 낸 픽셀(HOT_FRAC)의 이벤트를 버린다. RTL 이라면
                  픽셀 좌표 목록 하나와 비교기면 된다.
  고립 이벤트 거르기(background activity filter) : 같은 프레임이나 바로 앞 프레임에 둘레 8 픽셀 중 하나라도
                  이벤트가 없으면 버린다. RTL 이라면 두 프레임의 픽셀 비트맵(960 x 720 x 2 비트)이 든다.

핫픽셀은 센서에 붙어 있으므로 센서가 돌면 월드에서는 회전 중심 둘레의 원호로 번진다. 이 원호가 막대 둘레의
바닥을 얼마나 올리는지가 영향의 크기다. 거르기를 넣어도 막대 수치가 거의 그대로이면, RTL 에 거르기 회로
(특히 비트맵이 드는 고립 이벤트 거르기)를 넣지 않아도 된다는 근거가 된다.

출력
  <폴더>/noise.txt  : 노이즈 양, 핫픽셀 수, 거르기마다 버린 이벤트와 막대 4 개의 기울기 · 위치 · 가장자리 폭 ·
                      봉우리/바닥
  <폴더>/noise.png  : REST 의 픽셀별 이벤트 수(센서 좌표), 핫픽셀 이벤트만 누적한 월드 지도, 막대 단면 비교
"""
import os
import sys
import numpy as np
from scipy import ndimage
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import golden as g
import world_metrics as wm
from track_rotation import REF, profile
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'dvs_develop'))
from dvs_read import load, WIDTH, HEIGHT

REST = (1, 150)            # 센서가 돌기 전(프레임당 회전 중앙값 0.002°). 첫 프레임은 녹화가 중간에서 시작할 수 있어 뺀다
HOT_FRAC = 0.5             # REST 프레임 중 이 비율 이상에서 이벤트를 내면 핫픽셀


def isolated(d):
    """이벤트마다 '둘레 8 픽셀에 같은 프레임이나 앞 프레임 이벤트가 하나도 없음' 여부(bool 배열)."""
    nf = d['n_frames']
    starts = np.searchsorted(d['frame'], np.arange(nf + 1))
    k = np.ones((3, 3), np.uint8)
    prev = np.zeros((HEIGHT, WIDTH), np.uint8)
    out = np.zeros(len(d['frame']), bool)
    for f in range(nf):
        a, b = starts[f], starts[f + 1]
        cur = np.zeros((HEIGHT, WIDTH), np.uint8)
        cur[d['y'][a:b], d['x'][a:b]] = 1
        near = ndimage.convolve(cur | prev, k, mode='constant') - cur      # 자기 자신은 뺀다
        out[a:b] = near[d['y'][a:b], d['x'][a:b]] == 0
        prev = cur
    return out


def subset(d, keep):
    """keep 이 참인 이벤트만 남긴 사본. 프레임 순서는 그대로라 golden.run 에 그대로 넣을 수 있다."""
    return dict(d, frame=d['frame'][keep], x=d['x'][keep], y=d['y'][keep], p=d['p'][keep])


def main(path, bundle, out):
    os.makedirs(out, exist_ok=True)
    d = load(path)
    psi, ph, c, s, center = g.frame_trig(bundle)
    psi_ref = psi[(REF[0] + REF[1]) // 2]
    wm.set_cell(g.CELL_SHIFT, g.CELL_BITS)       # golden.py 의 칸 크기 · 칸 비트 그대로
    nf = d['n_frames']
    n = len(d['frame'])
    per = np.bincount(d['frame'], minlength=nf)
    pix = d['y'].astype(np.int64) * WIDTH + d['x']
    lo, hi = np.searchsorted(d['frame'], REST)
    nrest = REST[1] - REST[0]
    # REST 에서 픽셀마다 이벤트를 낸 프레임 수(한 프레임에 ON · OFF 둘 다 내도 한 번)
    fr_pix = np.unique(d['frame'][lo:hi].astype(np.int64) * (WIDTH * HEIGHT) + pix[lo:hi]) % (WIDTH * HEIGHT)
    rest_frames = np.bincount(fr_pix, minlength=WIDTH * HEIGHT)
    hot = rest_frames >= HOT_FRAC * nrest
    hot_ev = hot[pix]
    iso = isolated(d)
    lines = [f'# 장면 1 노이즈. REST = 프레임 {REST[0]}~{REST[1] - 1}(센서가 돌기 전), 핫픽셀 = REST 프레임의 '
             f'{HOT_FRAC:.0%} 이상에서 이벤트, 고립 = 둘레 8 픽셀에 같은 · 앞 프레임 이벤트 없음', '']
    rest_rate = per[REST[0]:REST[1]].mean()
    t = d['t_us']
    dt = np.median(np.diff(t[1:])) * 1e-6
    lines.append('[1] 노이즈의 양')
    lines.append(f'    REST 프레임당 이벤트 평균 {rest_rate:.0f}, 픽셀당 {rest_rate / (WIDTH * HEIGHT) / dt:.2f} 개/s. '
                 f'회전 중(프레임 300~1299) 평균 {per[300:1300].mean():.0f} 의 {rest_rate / per[300:1300].mean() * 100:.1f} %')
    lines.append(f'    REST 에서 이벤트를 한 번이라도 낸 픽셀 {(rest_frames > 0).sum()} ({(rest_frames > 0).mean() * 100:.1f} %), '
                 f'REST 이벤트 중 고립 이벤트 {iso[lo:hi].mean() * 100:.1f} %')
    lines.append(f'    핫픽셀 {int(hot.sum())} 개, 이들의 REST 이벤트 비율 {hot_ev[lo:hi].mean() * 100:.1f} %, '
                 f'녹화 전체 이벤트 비율 {hot_ev.mean() * 100:.3f} %')
    lines.append(f'    녹화 전체에서 고립 이벤트 {iso.mean() * 100:.2f} % (회전 중 프레임 300~1299: '
                 f'{iso[np.searchsorted(d["frame"], 300):np.searchsorted(d["frame"], 1300)].mean() * 100:.2f} %)')

    variants = [('거르지 않음', np.ones(n, bool)), ('핫픽셀 가리기', ~hot_ev), ('고립 이벤트 거르기', ~iso),
                ('둘 다', ~hot_ev & ~iso)]
    res = {}
    for name, keep in variants:
        dd = subset(d, keep)
        W = g.run(dd, (ph, c, s), center)[0].astype(np.float64)
        R_ = g.run(dd, (ph, c, s), center, *REF)[0].astype(np.float64)
        res[name] = (int((~keep).sum()), W, wm.post_metrics(W, R_, psi_ref, center))
        print(f'{name}: 버린 이벤트 {(~keep).sum()}', flush=True)
    keys = list(res['거르지 않음'][2])
    lines += ['', '[2] 거르기마다 버린 이벤트와 막대 수치(4 px 칸, 14 비트). 기울기 · 위치 차 · 반치폭은 전체 누적,'
              ' 봉우리/바닥은 짧은 구간 / 전체']
    for name, (drop, W, m) in res.items():
        lines.append(f'    {name}: 버린 이벤트 {drop} ({drop / n * 100:.2f} %), 메모리 합 {int(W.sum())}, 칸 최댓값 {int(W.max())}')
        lines.append('      ' + '  '.join(f'{k}: {m[k]["tilt_full"]:+.1f}° {m[k]["shift_px"]:+.0f}px {m[k]["width_full"]:.0f}px '
                                          f'{m[k]["contrast_ref"]:.2f}/{m[k]["contrast_full"]:.2f}' for k in keys))
    text = '\n'.join(lines)
    print(text)
    with open(os.path.join(out, 'noise.txt'), 'w') as fo:
        fo.write(text + '\n')

    # 그림
    Wh = g.run(subset(d, hot_ev), (ph, c, s), center)[0].astype(np.float64)
    fig, axs = plt.subplots(1, 3, figsize=(16, 5))
    axs[0].imshow(rest_frames.reshape(HEIGHT, WIDTH), cmap='gray_r', vmax=max(4, np.percentile(rest_frames, 99.9)))
    axs[0].set_title(f'frames with events per pixel, frames {REST[0]}-{REST[1] - 1} (sensor)', fontsize=9)
    axs[1].imshow(np.log1p(Wh), cmap='gray_r')
    axs[1].set_title(f'world memory from {int(hot.sum())} hot pixels only', fontsize=9)
    x = np.arange(-wm.n_cells(wm.HALF_PX), wm.n_cells(wm.HALF_PX) + 1) * wm.CELL
    for name, col in (('거르지 않음', '#eb6834'), ('둘 다', '#2a78d6')):
        W, m = res[name][1], res[name][2]
        pr = profile(W, m['B']['line'], wm.n_cells(wm.HALF_PX))
        axs[2].plot(-x, pr, color=col, lw=2, label='no filter' if name == '거르지 않음' else 'hot pixels + isolated removed')
    axs[2].set_title('post B profile, all frames', fontsize=9)
    axs[2].set_xlabel('across post (px)')
    axs[2].legend(frameon=False, fontsize=8)
    axs[2].grid(alpha=0.3)
    for a in axs[:2]:
        a.set_xticks([])
        a.set_yticks([])
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'noise.png'), dpi=80)


if __name__ == '__main__':
    main(*sys.argv[1:4])
