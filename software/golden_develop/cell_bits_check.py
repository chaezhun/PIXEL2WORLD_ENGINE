#!/usr/bin/env python3
"""월드 칸 하나의 비트 수를 줄이는 방식마다 막대 측정이 포화 없이 센 기준 지도와 같은지 잰다.

    python3 software/golden_develop/cell_bits_check.py <장면 1.dvs> <골든 묶음> <출력 폴더>

월드 메모리 비트 = 칸 수 × 칸 비트라서 칸 비트가 SRAM 면적 · 누설 전력을 정한다. 이 스크립트는 칸 비트를 몇
까지 줄여도 막대 측정이 그대로인지를 보아 칸 비트를 고르는 근거를 낸다. 기준은 칸마다 포화 없이 센 개수이고
출력에서는 "14비트" 로 적는다(칸 최댓값이 2^14 - 1 이하이면 14 비트 포화 카운터와 같다). 다음 방식을 비트
수별로 만들어, world_metrics.post_metrics(4 px 칸에서 verify_world.py 와 같은 값)의 막대 기울기 · 가로 위치 ·
가장자리 반치폭 · 봉우리/바닥이 기준과 같은지 본다.

  포화 k        min(개수, 2^k - 1). 순서와 상관없으므로 기준 지도를 자르면 된다
  로그 k/j      k 비트 부호 c 를 지수 e = c >> j 와 가수 r = c & (2^j - 1) 로 본다. 이벤트마다 확률 2^-e 로
                c 를 1 올리고 2^k - 1 에서 멈춘다. e = 0 인 동안(c < 2^j)은 확률 1 이라 정확히 센다. 읽을 때
                    n̂(c) = 2^j · (2^e - 1) + r · 2^e
                로 푼다. c 에 이르기까지 드는 이벤트 수의 기댓값이라 n̂ 은 n 의 불편 추정이다.
  decay T/k     k 비트 포화 카운터. T 프레임이 끝날 때마다 모든 칸을 반으로(>> 1). 오래된 이벤트일수록 덜 센다
  반감 k        k 비트 포화 카운터. 프레임이 끝날 때 칸 하나라도 2^(k-1) 이상이면 모든 칸을 반으로 줄이고
                공통 지수를 1 올린다. 지도 전체가 같은 배율이라 막대 측정에는 배율이 상관없어 지수는 되돌리지 않는다

판정
  막대 측정의 간격은 기울기 0.1°, 가로 위치 1 px(1/4 칸), 반치폭 4 px(1 칸)이다.
  엄격       기준과의 차가 측정 간격 안(기울기 0.1° 이하, 위치 1 px 이하, 반치폭 같음).
  기준선 안  기준 지도를 이벤트 절반만 무작위로 골라(2 배 해서) 만들 때 생기는 흔들림 이하. 로그 카운터는 칸마다
             개수에 잡음을 얹으므로, 데이터 자체의 표본 잡음과 구분되지 않는 정도인지를 본다.
  난수를 쓰는 방식(로그 카운터, 절반 표본)은 시작값 8 개로 돌려 가장 나쁜 값으로 판정한다.

로그 카운터의 난수 (RTL 로 옮긴다면 그대로 따를 수 있게 정한 사양)
  16 비트 Galois LFSR, 다항식 x^16 + x^14 + x^13 + x^11 + 1(LFSR_POLY = 0xB400), 주기 65535.
  한 번 밀기: lsb = s & 1;  s >>= 1;  lsb 면 s ^= 0xB400.
  메모리를 지울 때 s = LFSR_SEED 로 두고, 누적하는 이벤트마다 지금 s 로 판정한 뒤 16 번 민다(LFSR_LEAP).
  판정: s 의 아래 e 비트가 모두 0 이면 올린다. s 는 0 이 아니므로 확률은 (2^(16-e) - 1) / 65535 이고, 2^-e 와의
  비는 (65536 - 2^e) / 65535 라서 e <= 8 에서 0.4 % 안으로 같다. 쓰인 지수의 최댓값은 출력에 적는다
  (로그 10/7 은 장면 1 에서 6 까지 쓰여 0.1 %).
  누적 순서는 파일 순서(golden.run 과 같고 RTL 처리 순서와 같음)이고, 메모리 밖 이벤트는 LFSR 을 밀지 않는다.

시작할 때 기준 지도를 golden.CELL_BITS 비트로 포화시킨 것이 골든 묶음의 world.npy 와 같은지 확인한다.

출력
  <출력 폴더>/cell_bits.txt : 방식별 메모리 비트, 엄격 · 기준선 판정, 기준과의 최대 차, 막대 4 개의 측정값,
                              로그 카운터의 추정 오차(개수 64 이상 칸의 |n̂ - n| / n)
"""
import os
import sys
import time
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import golden as g
import world_metrics as wm
from track_rotation import REF
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'dvs_develop'))
from dvs_read import load

LFSR_POLY, LFSR_SEED, LFSR_LEAP, LFSR_PERIOD = 0xB400, 0xACE1, 16, 65535
OTHER_SEEDS = (0x1234, 0xBEEF, 0x0001, 0x8000, 0x5A5A, 0x0F0F, 0x7777)   # 난수에 따라 판정이 바뀌는지 보려고 더 돌리는 LFSR 시작값
SEEDS = tuple(range(8))                  # 절반 표본 기준선의 난수 시작값
CHUNK = 1 << 23

SAT_BITS = (4, 6, 8, 10, 11, 12, 13)
LOG_CFG = ((4, 0), (5, 1), (6, 2), (7, 3), (8, 4), (8, 5), (9, 5), (9, 6), (10, 6), (10, 7), (11, 7), (11, 8),
           (12, 9), (12, 10))
DECAY_CFG = ((64, 8), (256, 10), (1024, 12))
HALVE_BITS = (6, 8, 10)


def lfsr_table(seed=LFSR_SEED):
    """S_0 = seed 에서 한 번씩 민 상태 S_0 ~ S_65534. 이벤트 i 는 S_(16 i mod 65535) 를 쓴다."""
    t = np.empty(LFSR_PERIOD, np.uint16)
    s = seed
    for i in range(LFSR_PERIOD):
        t[i] = s
        lsb = s & 1
        s >>= 1
        if lsb:
            s ^= LFSR_POLY
    assert s == seed           # 주기가 정확히 65535 인지
    return t


def addresses(d, c, s, center, f0=0, f1=None):
    """프레임 f0 ~ f1-1 이벤트의 칸 주소(누적 순서, 메모리 안만)와 프레임 번호."""
    lo, hi = np.searchsorted(d['frame'], [f0, d['n_frames'] if f1 is None else f1])
    a_out, f_out = [], []
    for a in range(lo, hi, CHUNK):
        b = min(a + CHUNK, hi)
        f = d['frame'][a:b]
        wx, wy = g.cells(d['x'][a:b], d['y'][a:b], c[f], s[f], *center)
        ok = (wx >= 0) & (wx < g.N) & (wy >= 0) & (wy < g.M)
        a_out.append((wy * g.N + wx)[ok].astype(np.int32))
        f_out.append(f[ok])
    return np.concatenate(a_out), np.concatenate(f_out)


class LogSim:
    """칸 주소 열 하나에 대해 로그 카운터를 여러 설정으로 돌리는 준비물(정렬 · 난수)을 한 번만 만든다.

    칸끼리는 서로 영향이 없으므로, 이벤트를 칸별로(칸 안에서는 누적 순서대로) 모은 뒤 "칸마다 t 번째 이벤트"를
    모든 칸에서 한꺼번에 처리한다. 이벤트 수가 많은 칸부터 늘어놓아 t 번째 이벤트가 있는 칸이 앞쪽에 모이게 했다.
    """

    def __init__(self, addr, table):
        n_cells = g.N * g.M
        order = np.argsort(addr, kind='stable')
        self.rnd = table[(order * LFSR_LEAP) % LFSR_PERIOD]      # 이벤트 번호 = 누적 순서
        del order
        cnt = np.bincount(addr, minlength=n_cells)
        start = np.r_[0, np.cumsum(cnt)[:-1]]
        self.by = np.argsort(-cnt, kind='stable')
        self.cs = cnt[self.by]
        self.st = start[self.by]
        self.cnt = cnt

    def run(self, k, j):
        """돌려주는 값: (칸별 부호 c, 칸별 추정 n̂, 쓰인 지수 최댓값)."""
        cmax = (1 << k) - 1
        c = np.zeros(len(self.cs), np.int64)
        neg = -self.cs
        for t in range(int(self.cs[0])):
            na = int(np.searchsorted(neg, -t, 'left'))           # t 번째 이벤트가 있는 칸 수
            cc = c[:na]
            e = cc >> j
            hit = ((self.rnd[self.st[:na] + t].astype(np.int64) & ((1 << e) - 1)) == 0) & (cc < cmax)
            c[:na] = cc + hit
        code = np.empty_like(c)
        code[self.by] = c
        e, r = code >> j, code & ((1 << j) - 1)
        est = (1 << j) * ((1 << e) - 1) + r * (1 << e)
        return code, est, int((c >> j).max())


def frame_counts(addr, frm, n_frames):
    """프레임마다 (그 프레임 이벤트의 칸 주소) 조각의 경계."""
    return np.searchsorted(frm, np.arange(n_frames + 1))


def decay_map(addr, bounds, k, period=None):
    """k 비트 포화 카운터. period 가 있으면 period 프레임마다 >> 1, 없으면 '반감' 규칙. (지도, 지수, 잘린 이벤트)."""
    n_cells = g.N * g.M
    cmax = (1 << k) - 1
    mem = np.zeros(n_cells, np.int64)
    ex, cut = 0, 0
    for f in range(len(bounds) - 1):
        lo, hi = bounds[f], bounds[f + 1]
        if hi > lo:
            m2 = mem + np.bincount(addr[lo:hi], minlength=n_cells)
            cut += int(np.maximum(m2 - cmax, 0).sum())
            mem = np.minimum(m2, cmax)
        if period is not None:
            if (f + 1) % period == 0:
                mem >>= 1
        elif mem.max() >= 1 << (k - 1):
            mem >>= 1
            ex += 1
    return mem, ex, cut


def metrics(W, R_, psi_ref, center):
    """막대마다 (기울기, 가로 위치 차, 반치폭, 봉우리/바닥). 막대 선 둘레가 비어 선을 못 맞추면 None.

    decay 처럼 오래된 이벤트를 지우는 방식은 녹화 끝의 짧은 구간만 남아 막대 선 둘레가 비기도 한다.
    """
    try:
        m = wm.post_metrics(W.reshape(g.M, g.N).astype(np.float64), R_.reshape(g.M, g.N).astype(np.float64),
                            psi_ref, center)
    except ValueError:
        return None
    return {k: (v['tilt_full'], v['shift_px'], v['width_full'], v['contrast_full']) for k, v in m.items()}


def main(path, bundle, outdir):
    os.makedirs(outdir, exist_ok=True)
    t0 = time.time()
    d = load(path)
    psi, ph, c, s, center = g.frame_trig(bundle)
    psi_ref = psi[(REF[0] + REF[1]) // 2]
    nf = d['n_frames']
    addr, frm = addresses(d, c, s, center)
    raddr, rfrm = addresses(d, c, s, center, *REF)
    del d
    n_cells = g.N * g.M
    W14 = np.bincount(addr, minlength=n_cells)
    R14 = np.bincount(raddr, minlength=n_cells)
    gold = np.load(os.path.join(bundle, 'world.npy')).ravel()
    assert np.array_equal(np.minimum(W14, (1 << g.CELL_BITS) - 1), gold), '골든 world.npy 와 다르다'
    print(f'주소 {len(addr)} 개, 골든과 같음 ({time.time() - t0:.0f} s)', file=sys.stderr, flush=True)

    # 방식마다 {이름, 칸 비트, 측정(난수를 쓰는 방식은 시작값마다 하나, 첫째가 대표), 메모}. 첫 줄은 기준(포화 없는 개수)
    rows = []
    base = metrics(W14, R14, psi_ref, center)
    rows.append(dict(name='14비트(기준)', bits=14, ms=[base], note=''))

    # 기준선: 기준 지도를 이벤트 절반만 무작위로 골라 만들면(2 배 해서) 측정이 얼마나 흔들리는가.
    # 로그 카운터는 칸마다 개수에 잡음을 얹는 방식이라, 이 정도 흔들림은 데이터 자체의 표본 잡음과 구분이 안 된다
    ms = []
    for sd in SEEDS:
        rng = np.random.default_rng(sd)
        keep, rkeep = rng.integers(0, 2, len(addr), np.uint8) == 1, rng.integers(0, 2, len(raddr), np.uint8) == 1
        ms.append(metrics(2 * np.bincount(addr[keep], minlength=n_cells),
                          2 * np.bincount(raddr[rkeep], minlength=n_cells), psi_ref, center))
    rows.append(dict(name='14비트 절반 표본', bits=14, ms=ms, note=f'기준선: 이벤트를 1/2 확률로 남긴 지도 x 2, 난수 시작값 {len(SEEDS)}개'))

    for k in SAT_BITS:
        cm = (1 << k) - 1
        rows.append(dict(name=f'포화 {k}', bits=k, ms=[metrics(np.minimum(W14, cm), np.minimum(R14, cm), psi_ref, center)],
                         note=f'포화 칸 {int((W14 >= cm).sum()):,}, 잘린 이벤트 '
                              f'{np.maximum(W14 - cm, 0).sum() / W14.sum() * 100:.2f} %'))

    big = W14 >= 64
    sims = [(LogSim(addr, lfsr_table(sd)), LogSim(raddr, lfsr_table(sd))) for sd in (LFSR_SEED,) + OTHER_SEEDS]
    for k, j in LOG_CFG:
        t1 = time.time()
        ms, rels, emax, nsat = [], [], 0, 0
        for a, b in sims:
            code, est, e = a.run(k, j)
            ms.append(metrics(est, b.run(k, j)[1], psi_ref, center))
            rels.append(np.abs(est[big] - W14[big]) / W14[big])
            emax, nsat = max(emax, e), max(nsat, int((code == (1 << k) - 1).sum()))
        top = (1 << j) * ((1 << ((1 << (k - j)) - 1)) - 1) + ((1 << j) - 1) * (1 << ((1 << (k - j)) - 1))
        rel = rels[0]
        rows.append(dict(name=f'로그 {k}/{j}', bits=k, ms=ms,
                         note=f'n̂ 최댓값 {top:,}, 쓰인 지수 최대 {emax}, 포화 칸 최대 {nsat:,}. 개수 64 이상 칸 '
                              f'{int(big.sum()):,}개의 |n̂ - n|/n 중앙값 {np.median(rel) * 100:.1f} %, '
                              f'95 % {np.percentile(rel, 95) * 100:.1f} % (시작값 0x{LFSR_SEED:04X})'))
        print(f'로그 {k}/{j}: {time.time() - t1:.0f} s', file=sys.stderr, flush=True)
    del sims

    bounds, rbounds = frame_counts(addr, frm, nf), frame_counts(raddr, rfrm, nf)
    for T, k in DECAY_CFG:
        W, _, cut = decay_map(addr, bounds, k, T)
        R_, _, _ = decay_map(raddr, rbounds, k, T)
        rows.append(dict(name=f'decay {T}/{k}', bits=k, ms=[metrics(W, R_, psi_ref, center)],
                         note=f'잘린 이벤트 {cut / len(addr) * 100:.2f} %'))
    for k in HALVE_BITS:
        # 지도 전체가 같은 배율 2^지수 라서 막대 측정(기울기 · 위치 · 반치폭 · 봉우리/바닥)에는 배율이 상관없다.
        # 지수가 수백까지 커져 정수로 되돌리면 넘치므로 되돌리지 않고 잰다
        W, ex, cut = decay_map(addr, bounds, k)
        R_, _, _ = decay_map(raddr, rbounds, k)
        rows.append(dict(name=f'반감 {k}', bits=k, ms=[metrics(W, R_, psi_ref, center)],
                         note=f'반감 {ex}번, 잘린 이벤트 {cut / len(addr) * 100:.2f} %'))

    posts = list(base)

    def dev(m):
        """기준과의 최대 차 (기울기 도, 가로 위치 px, 반치폭 px). 네 막대 중 가장 큰 차. 측정 불가면 None."""
        if m is None:
            return None
        return tuple(max(abs(m[p][i] - base[p][i]) for p in posts) for i in range(3))

    def worst(ms):
        ds = [dev(m) for m in ms]
        return None if any(x is None for x in ds) else tuple(max(x[i] for x in ds) for i in range(3))

    strict = (0.1 + 1e-9, 1 + 1e-9, 0)
    tol = worst(rows[1]['ms'])          # 절반 표본 기준선의 최대 흔들림
    out = [f'칸 내용 비트 줄이기. 장면 1 전체 {len(addr):,} 이벤트, 칸 {1 << g.CELL_SHIFT} px, '
           f'{g.N} x {g.M} 칸, 회전 중심 {center}',
           '막대 측정: world_metrics.post_metrics. 막대마다 기울기(도) · 가로 위치(전체 - 짧은 구간, px) · 반치폭(px) · 봉우리/바닥.',
           f'최대 차 = 네 막대 · (난수를 쓰는 방식은) 시작값 {len(SEEDS)}개 중 14비트와 가장 많이 다른 값. 기울기 / 위치 / 반치폭.',
           '엄격 = 14비트와의 차가 측정 간격 안(기울기 0.1°, 위치 1 px, 반치폭 0 px). 시작값 여럿 중 몇 개가 엄격히 같은지 적는다.',
           f'기준선 안 = 최대 차가 "14비트 절반 표본" 의 최대 차(기울기 {tol[0]:.1f}°, 위치 {tol[1]:.0f} px, 반치폭 {tol[2]:.0f} px) 이하.',
           f'LFSR: 다항식 0x{LFSR_POLY:04X}, 대표 시작값 0x{LFSR_SEED:04X}, 다른 시작값 '
           + ', '.join(f'0x{x:04X}' for x in OTHER_SEEDS) + f', 이벤트마다 {LFSR_LEAP} 번 밀기', '']
    out.append(f'{"방식":<16} {"칸 비트":>6} {"메모리(bit)":>12}  {"엄격":>5}  {"최대 차":>16}  기준선 안  '
               + '  '.join(f'{p + " 기울기/위치/반치폭/대비":>30}' for p in posts) + '  (대표 측정)')
    for r in rows:
        w = worst(r['ms'])
        n_strict = sum(d is not None and all(d[i] <= strict[i] for i in range(3)) for d in map(dev, r['ms']))
        m = r['ms'][0]
        if m is None or w is None:
            cols, wtxt, inside = '막대 선 둘레가 비어 측정 불가', '-', '아니오'
        else:
            cols = '  '.join(f'{m[p][0]:+6.1f} {m[p][1]:+5.0f} {m[p][2]:5.0f} {m[p][3]:6.2f}'.rjust(30) for p in posts)
            wtxt = f'{w[0]:.1f}° {w[1]:.0f} {w[2]:.0f}'
            inside = '예' if all(w[i] <= tol[i] + 1e-9 for i in range(3)) else '아니오'
        out.append(f'{r["name"]:<16} {r["bits"]:>6} {r["bits"] * n_cells:>12,}  {n_strict:>2}/{len(r["ms"])}  '
                   f'{wtxt:>16}  {inside:<8}  {cols}')
        if r['note']:
            out.append(f'{"":<16} {r["note"]}')
    text = '\n'.join(out)
    open(os.path.join(outdir, 'cell_bits.txt'), 'w', encoding='utf-8').write(text + '\n')
    print(text)
    print(f'전체 {time.time() - t0:.0f} s', file=sys.stderr)


if __name__ == '__main__':
    main(*sys.argv[1:4])
