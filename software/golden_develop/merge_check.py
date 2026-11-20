#!/usr/bin/env python3
"""같은 칸 업데이트를 합치면 월드 메모리 읽기-더하기-쓰기(RMW)가 몇 번으로 줄어드는지 센다.

    python3 software/golden_develop/merge_check.py <골든 묶음> <출력 폴더> <장면 1.dvs> [<다른 녹화.dvs> ...]

합치지 않으면 이벤트마다 칸 하나를 읽고 1 을 더해 쓴다. 같은 칸으로 가는 이벤트 n 개를 모아 +n 한 번으로
쓰면 RMW 가 준다. 포화 덧셈은 min(값 + n, 최댓값) 이 +1 을 n 번 한 것과 같아 결과 메모리는 바뀌지 않는다.
얼마나 주는지는 어디까지를 한데 모으느냐(합치는 범위)에 달려 있어 다음을 나란히 센다. 범위가 넓을수록 RMW 는
줄지만 모아 둘 버퍼가 커지므로, 이 표로 RTL 에 넣을 합치기 범위와 SRAM 접근 횟수를 정한다.

  없음      이벤트마다 RMW. 기준 횟수
  이웃      처리 순서로 바로 앞 이벤트와 같은 칸이면 합친다(쓰기 대기 레지스터 하나)
  묶음      8 행 묶음(워드의 반쪽, 이벤트 최대 8 개) 안에서 칸별로 센다
  워드      이벤트 워드 하나(8 행 묶음 두 개, 최대 16 개) 안에서 칸별로 센다
  워드+이웃 워드 안에서 칸별로 센 뒤, 그 결과를 칸이 맨 먼저 나온 순서로 내보내며 앞 워드의 마지막 칸과 같으면
            또 합친다
  열        열 주소 하나 뒤의 이벤트 전부(같은 프레임, 같은 x) 안에서 칸별로. 버퍼가 커서 참고용 상한
  프레임    한 프레임 안에서 칸별로. 프레임 크기 버퍼가 필요한 이론 상한

이벤트가 어느 워드 · 어느 묶음에서 왔는지는 dvs_read.load() 가 돌려주지 않으므로 원시 워드에서 다시 붙인다.
load() 는 이벤트 워드를 파일 순서로, 한 워드 안에서는 비트 0 ~ 15 순서(아래 묶음 [7:0] 다음 위 묶음 [15:8])로
펼치므로, 워드마다 아래 · 위 비트맵의 1 의 개수만큼 묶음 번호를 되풀이하면 이벤트 순서와 정확히 맞는다.
붙인 뒤에는 묶음 안의 x 와 y >> 3 이 하나인지 확인한다.

한 묶음 안의 이벤트는 같은 열의 행을 오름차순으로 지나므로 월드에서는 직선 위를 한 방향으로 간다. u, v 가 행
번호 k 의 일차식이고 내림(>>)은 단조라서 wx, wy 가 k 에 대해 단조이고, 한 번 떠난 칸으로 돌아오지 않는다.
그래서 "묶음 안에서 칸별로 세기" 는 "묶음 안에서 이웃끼리 합치기" 와 횟수가 같고, 묶음 경계를 넘어서도 합치는
"이웃" 보다 많을 수 없다. 이것도 실제로 센 값으로 확인한다(assert).

녹화의 회전각
  장면 1 은 <골든 묶음> 의 프레임별 cos · sin 과 회전 중심(golden.frame_trig)을 그대로 쓴다. 골든 모델과 같은 칸이다.
  다른 녹화는 이 스크립트가 회전각을 받지 않으므로 모든 프레임을 고정 각도 0° · 22.5° · 45° 로 돌려 센다.
  장면 1 도 같은 고정 각도로 세어 녹화끼리 견줄 수 있게 한다. 고정 각도의 회전 중심은 장면 1 의 것을 쓴다.

출력
  <출력 폴더>/merge.txt : 경우마다 RMW 횟수와 감소율, 묶음당 칸 수 분포, RTL 크기를 정할 최댓값,
                          장면 1 은 회전각 구간별 감소율
"""
import os
import sys
import time
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'dvs_develop'))
from dvs_read import load
import golden as g

ADDR_BITS = 17                 # 칸 주소 wy·N + wx < 2^17 (N = M = 352)
CHUNK = 1 << 23                # 한 번에 다루는 이벤트 수(대략). 프레임 경계에서 자른다
FIXED_DEG = (0.0, 22.5, 45.0)  # 모든 녹화에 쓰는 고정 회전각(장면 1 밖의 녹화는 이것만 쓴다)
SCOPES = ('없음', '이웃', '묶음', '워드', '워드+이웃', '열', '프레임')


def group_ids(path, n):
    """이벤트마다 8 행 묶음 번호(파일 안 이벤트 워드 순번 × 2 + 위 묶음이면 1). 앞 n 개만 돌려준다."""
    w = np.fromfile(path, dtype='>u4').astype(np.uint32)
    v = w[(w >> 31) == 1]
    cnt = np.stack([np.bitwise_count(v & 0xff), np.bitwise_count((v >> 8) & 0xff)], axis=1).ravel()
    gid = np.repeat(np.arange(len(cnt), dtype=np.int32), cnt)
    assert len(gid) >= n, (len(gid), n)
    return gid[:n]


def n_distinct(key):
    """정렬하지 않은 정수 배열의 서로 다른 값 수."""
    return len(np.unique(key))


def count_chunk(gid, frame, x, a, ok):
    """이벤트 한 조각(프레임 경계에서 자른 것)의 범위별 RMW 횟수와 분포.

    gid, frame, x : 이벤트별 묶음 번호 · 프레임 · 열. a : 칸 주소, ok : 월드 메모리 안인가.
    범위 밖 이벤트는 RTL 이 버리므로 먼저 빼고 센다. 남은 이벤트는 처리 순서 그대로다.
    """
    gid, frame, x, a = gid[ok], frame[ok], x[ok], a[ok]
    n = len(a)
    r = {'없음': n}
    if n == 0:
        return r, {}
    # 이웃: 바로 앞 이벤트와 칸이 다를 때마다 새 RMW
    new_run = np.r_[True, a[1:] != a[:-1]]
    r['이웃'] = int(new_run.sum())
    # 묶음 · 워드 · 열 · 프레임: (범위 번호, 칸) 쌍의 수
    kg = gid.astype(np.int64) << ADDR_BITS | a
    ug, first_g, cnt_g = np.unique(kg, return_index=True, return_counts=True)
    r['묶음'] = len(ug)
    # 묶음 안에서 칸이 단조로 바뀐다는 성질 확인: 묶음 안 이웃 합치기 횟수 == 묶음 안 칸 수
    in_group_runs = int((np.r_[True, (gid[1:] != gid[:-1]) | (a[1:] != a[:-1])]).sum())
    assert in_group_runs == r['묶음'], (in_group_runs, r['묶음'])
    kw = (gid >> 1).astype(np.int64) << ADDR_BITS | a
    uw, first_w, cnt_w = np.unique(kw, return_index=True, return_counts=True)
    r['워드'] = len(uw)
    # 워드+이웃: 워드 안 칸별 합을 칸이 맨 먼저 나온 순서로 늘어놓고, 이어지는 같은 칸을 다시 합친다
    seq = (uw & ((1 << ADDR_BITS) - 1))[np.argsort(first_w, kind='stable')]
    r['워드+이웃'] = int(1 + np.count_nonzero(seq[1:] != seq[:-1]))
    col = np.cumsum(np.r_[True, (frame[1:] != frame[:-1]) | (x[1:] != x[:-1])])
    r['열'] = n_distinct(col.astype(np.int64) << ADDR_BITS | a)
    r['프레임'] = n_distinct(frame.astype(np.int64) << ADDR_BITS | a)
    # RTL 크기를 정할 값: 묶음 하나의 칸 수 분포, 칸 하나에 모이는 이벤트 수의 최댓값
    per_group = np.bincount(np.unique(ug >> ADDR_BITS, return_counts=True)[1], minlength=9)
    run_start = np.flatnonzero(new_run)
    st = {'cells_per_group': per_group, 'max_group_n': int(cnt_g.max()), 'max_word_n': int(cnt_w.max()),
          'max_run_n': int(np.diff(np.r_[run_start, n]).max()), 'first': int(a[0]), 'last': int(a[-1])}
    # 장면 1 회전각 구간별 감소율을 내려고 프레임별 횟수도 남긴다(이웃 · 워드+이웃 · 없음)
    st['frame'] = frame
    st['run_frame'] = frame[new_run]
    keep = np.r_[True, seq[1:] != seq[:-1]]
    st['wr_frame'] = frame[np.sort(first_w)][keep]
    return r, st


def count(path, d, gid, cos_f, sin_f, center):
    """녹화 하나를 한 가지 각도 설정으로 센다. cos_f, sin_f 는 프레임별 정수 배열."""
    tot = dict.fromkeys(SCOPES, 0)
    hist = np.zeros(9, np.int64)
    mx = {'max_group_n': 0, 'max_word_n': 0, 'max_run_n': 0}
    nf = d['n_frames']
    ev_f = np.zeros(nf, np.int64)
    run_f = np.zeros(nf, np.int64)
    wr_f = np.zeros(nf, np.int64)
    n_out = 0
    prev_last = -1
    lo, n = 0, len(d['frame'])
    while lo < n:
        hi = n if lo + CHUNK >= n else int(np.searchsorted(d['frame'], d['frame'][lo + CHUNK], 'left'))
        if hi <= lo:      # 프레임 하나가 CHUNK 보다 크면 그 프레임 끝까지
            hi = int(np.searchsorted(d['frame'], d['frame'][lo], 'right'))
        f = d['frame'][lo:hi]
        wx, wy = g.cells(d['x'][lo:hi].astype(np.int64), d['y'][lo:hi].astype(np.int64), cos_f[f], sin_f[f], *center)
        ok = (wx >= 0) & (wx < g.N) & (wy >= 0) & (wy < g.M)
        n_out += int((~ok).sum())
        r, st = count_chunk(gid[lo:hi], f, d['x'][lo:hi], wy * g.N + wx, ok)
        for k in SCOPES:
            tot[k] += r.get(k, 0)
        if st:
            # 조각은 프레임 경계에서 잘랐다. 앞 조각 마지막 칸과 이 조각 첫 칸이 같으면 이웃 · 워드+이웃 은
            # 이어서 합쳐지므로 하나씩 뺀다(프레임 경계를 넘는 합치기. 묶음 · 워드 · 열 · 프레임은 영향 없음)
            if st['first'] == prev_last:
                tot['이웃'] -= 1
                tot['워드+이웃'] -= 1
            prev_last = st['last']
            hist += st['cells_per_group']
            for k in mx:
                mx[k] = max(mx[k], st[k])
            ev_f += np.bincount(st['frame'], minlength=nf)
            run_f += np.bincount(st['run_frame'], minlength=nf)
            wr_f += np.bincount(st['wr_frame'], minlength=nf)
        lo = hi
    return tot, hist, mx, n_out, (ev_f, run_f, wr_f)


def check_groups(d, gid):
    """붙인 묶음 번호가 맞는지: 한 묶음 안의 프레임 · x · y >> 3 이 하나이고, 행이 오름차순이다."""
    for lo in range(0, len(gid), CHUNK):
        sl = slice(lo, min(lo + CHUNK + 1, len(gid)))
        gg, f, x, y = gid[sl], d['frame'][sl], d['x'][sl].astype(np.int32), d['y'][sl].astype(np.int32)
        same = gg[1:] == gg[:-1]
        assert np.all(f[1:][same] == f[:-1][same])
        assert np.all(x[1:][same] == x[:-1][same])
        assert np.all((y[1:] >> 3)[same] == (y[:-1] >> 3)[same])
        assert np.all(y[1:][same] > y[:-1][same])


def report(out, name, label, tot, hist, mx, n_out):
    base = tot['없음']
    out.append(f'{name}  {label}')
    out.append(f'  이벤트(메모리 안) {base:,}, 범위 밖 {n_out:,}')
    out.append(f'  {"범위":<8} {"RMW 횟수":>14} {"이벤트당":>8} {"감소율":>8}')
    for k in SCOPES:
        out.append(f'  {k:<8} {tot[k]:>14,} {tot[k] / base:>8.4f} {(1 - tot[k] / base) * 100:>7.2f} %')
    nz = hist.sum()
    dist = ', '.join(f'{c}칸 {hist[c] / nz * 100:.2f} %' for c in range(1, 9) if hist[c])
    out.append(f'  8행 묶음 하나가 닿는 칸 수: {dist} (묶음 {nz:,}개, 평균 {(np.arange(9) * hist).sum() / nz:.3f})')
    out.append(f'  칸 하나에 모이는 이벤트 최댓값: 묶음 안 {mx["max_group_n"]}, 워드 안 {mx["max_word_n"]}, '
               f'이웃 합치기 한 번 {mx["max_run_n"]}')
    out.append('')


def main(bundle, outdir, paths):
    os.makedirs(outdir, exist_ok=True)
    _, _, c_real, s_real, center = g.frame_trig(bundle)
    psi_real = np.degrees(np.arctan2(s_real, c_real))
    out = [f'같은 칸 업데이트 합치기. 칸 {1 << g.CELL_SHIFT} px, 메모리 {g.N} x {g.M}, '
           f'회전 중심 {center}', '감소율 = 1 - (RMW 횟수 / 합치지 않을 때 RMW 횟수). RMW 한 번 = SRAM 읽기 1 + 쓰기 1', '']
    for i, path in enumerate(paths):
        t0 = time.time()
        name = os.path.basename(path)
        d = load(path)
        gid = group_ids(path, len(d['frame']))
        check_groups(d, gid)
        nf = d['n_frames']
        cases = []
        if i == 0:
            assert len(c_real) == nf, (len(c_real), nf)
            cases.append(('골든 모델 회전각(프레임별)', c_real, s_real))
        for deg in FIXED_DEG:
            c, s = g.cordic(g.to_phase(np.full(nf, deg)))
            cases.append((f'고정 {deg:g}°', c, s))
        for label, c, s in cases:
            tot, hist, mx, n_out, per_f = count(path, d, gid, c, s, center)
            report(out, name, label, tot, hist, mx, n_out)
            # 프레임별 이벤트당 RMW 의 범위. SRAM 포트 수 · 뱅크를 정할 때는 평균이 아니라 무거운 프레임이 중요하다.
            # 이벤트가 아주 적은 프레임은 비율이 흔들리므로 1,000 개 이상인 프레임만 본다
            ev_f, run_f, wr_f = per_f
            big = ev_f >= 1000
            out.pop()
            out.append(f'  프레임별 이벤트당 RMW(이벤트 1,000개 이상 프레임 {big.sum():,}개): 이웃 '
                       f'{(run_f[big] / ev_f[big]).min():.4f} ~ {(run_f[big] / ev_f[big]).max():.4f}, 워드+이웃 '
                       f'{(wr_f[big] / ev_f[big]).min():.4f} ~ {(wr_f[big] / ev_f[big]).max():.4f}. '
                       f'가장 무거운 프레임 {int(ev_f.argmax())}(이벤트 {int(ev_f.max()):,}) 이웃 '
                       f'{run_f[ev_f.argmax()] / ev_f.max():.4f}')
            out.append('')
            if label.startswith('골든'):
                # 회전각을 0 ~ 45° 로 접은 구간별 감소율. 칸 격자에 대한 묶음 직선의 기울기만 중요하므로
                # 90° 주기로 접고, 45° 를 넘으면 90° 에서 뺀다
                fold = np.abs(((psi_real + 45) % 90) - 45)
                ev_f, run_f, wr_f = per_f
                out.append('  회전각 구간(0 ~ 45°로 접음)별 감소율: 이웃 / 워드+이웃, 이벤트 비중')
                for a0, a1 in [(0, 5), (5, 15), (15, 25), (25, 35), (35, 45.01)]:
                    m = (fold >= a0) & (fold < a1)
                    e = ev_f[m].sum()
                    if e:
                        out.append(f'    {a0:>4g} ~ {min(a1, 45):>4g}°  {(1 - run_f[m].sum() / e) * 100:6.2f} % / '
                                   f'{(1 - wr_f[m].sum() / e) * 100:6.2f} %, 이벤트 {e / ev_f.sum() * 100:5.1f} %')
                out.append('')
        print(f'{name}: {time.time() - t0:.0f} s', file=sys.stderr)
        del d, gid
    text = '\n'.join(out)
    open(os.path.join(outdir, 'merge.txt'), 'w', encoding='utf-8').write(text + '\n')
    print(text)


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], sys.argv[3:])
