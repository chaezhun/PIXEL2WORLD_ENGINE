#!/usr/bin/env python3
"""월드 메모리 누적의 정수 골든 모델. RTL 은 이 파일과 비트 단위로 같아야 한다.

    python3 software/golden_develop/golden.py <녹화>.dvs <묶음 폴더> [<f0> <f1>]

<묶음 폴더> 의 rotation.csv · center.txt(track_rotation.py 출력)를 읽어
  phase.csv      : 프레임마다 월드 회전각 ψ, 위상, CORDIC 이 낸 cos · sin
  world.npy      : 녹화 전체를 누적한 월드 메모리 (M 행 N 열, uint16)
  vectors/       : 프레임 f0 ~ f1-1 (기본 500 ~ 540) 의 RTL 테스트벤치 입력과 기댓값. 형식은 write_vectors 참고
을 쓴다.

정수 연산 (RTL 이 그대로 따를 사양)
  프레임마다 한 번
    위상 φ (PHASE_BITS = 16 비트, 2^16 = 360°) → cordic() → cos · sin (부호 16 비트, 소수 TRIG_F = 14 비트)
  이벤트마다
    dx = x - CX,  dy = y - CY                                   (부호 11 비트)
    u  = C·dx - S·dy + (WC << TRIG_F)                           (WC = 월드 중심 px = N · 2^CELL_SHIFT / 2)
    v  = S·dx + C·dy + (WC << TRIG_F)
    wx = u >> (TRIG_F + CELL_SHIFT),  wy = v >> (TRIG_F + CELL_SHIFT)     (산술 시프트 = 내림)
    0 <= wx < N 이고 0 <= wy < M 이고 저장 영역 표 안이면 mem[wy·N + wx] 를 1 올리되 2^CELL_BITS - 1 에서 멈춘다.
    밖이면 버린다. 저장 영역 표(stored_mask)는 월드 중심에서 반지름 REGION_R 인 원에 걸치는 칸을 뱅크 16 개의
    뱅크 안 좌표 (wy >> 2, wx >> 2) 로 모은 합집합이다. RTL 의 wl_region_map 도 같은 표를 쓴다.
  센 값은 더하는 순서와 상관없다(포화 덧셈도 min(합, 최댓값) 이라 같다). 그래서 RTL 이 이벤트를 어떤 순서로
  처리하든 마지막 메모리는 이 모델과 같아야 한다.

회전 방향
  월드 좌표 w = R(ψ)(p - c) + WC,  R(ψ) = [[cos, -sin], [sin, cos]] (x 오른쪽, y 아래). ψ_f = α - θ_f.
  θ_f 는 영상이 돈 각도(track_rotation.py), α 는 막대가 수직이 되도록 월드를 돌리는 각도다.
"""
import os
import sys
from math import atan, pi, sqrt, prod
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'dvs_develop'))
from dvs_read import load, T_ADDR, WIDTH, HEIGHT

PHASE_BITS = 16      # 위상 비트. 1 LSB = 0.0055°
TRIG_F = 14          # cos · sin 소수 비트
CORDIC_IT = 16       # CORDIC 반복 횟수
GZ, GX = 3, 3        # CORDIC 각도 누산기와 x · y 누산기의 보호 비트
CELL_SHIFT = 2       # 월드 칸 = 2^2 = 4 px
N = M = 352          # 월드 메모리 열 · 행 수. 4 px 칸이라 1408 px. 장면 1 회전 중심에서 센서 모서리까지 677 px 를 담는다
# 칸 하나의 비트 수(포화 카운터, 2^13 - 1 = 8191 에서 멈춘다). 장면 1 에서 포화하는 칸이 조금 있지만, 막대
# 측정(기울기 · 가로 위치 · 가장자리 반치폭, cell_bits_check.py 가 잰다)이 포화 없이 센 지도와 같게 나오는
# 가장 작은 비트 수다.
CELL_BITS = 13
# 저장 영역 원의 반지름(px). 뱅크마다 저장 칸 수(stored_mask 의 True 수)가 4,096 + 1,024 x 2 = 6,144 word
# 안에 드는 가장 큰 반지름이다(696 px 에서 뱅크당 6,132 칸). 장면 1 회전 중심에서 가장 먼 센서 모서리
# (677 px)보다 크므로 장면 1 의 이벤트는 원 밖으로 나가지 않는다.
REGION_R = 696
WC = N << CELL_SHIFT >> 1   # 월드 중심(px). 회전 중심 c 가 여기로 간다

_ATAN = [round(atan(2.0 ** -i) / (2 * pi) * (1 << (PHASE_BITS + GZ))) for i in range(CORDIC_IT)]
_X0 = round(prod(1 / sqrt(1 + 2.0 ** (-2 * i)) for i in range(CORDIC_IT)) * (1 << (TRIG_F + GX)) / sqrt(2))


def cordic(phase):
    """위상(정수 배열) → (cos, sin) 정수 배열, 소수 TRIG_F 비트.

    위상의 위 2 비트로 사분면 q 를 고르고, 사분면 안 각도 r 을 45° 를 0 으로 옮겨 [-45°, 45°) 에서 회전 모드
    CORDIC 을 돈다. 시작 벡터는 45° 방향 (X0, X0) 이고 X0 에 CORDIC 이득의 역수가 들어 있다. 반복마다
      d = (z >= 0 ? +1 : -1),  x' = x - d·(y >>> i),  y' = y + d·(x >>> i),  z' = z - d·ATAN[i]
    마지막에 x · y 를 GX 비트 반올림해 내리고(+2^(GX-1) 뒤 >>> GX) 사분면만큼 90° 씩 돌린다.
    전체 위상 2^16 개에서 실수 cos · sin 과의 차이는 최대 1.92 LSB(2^-14 단위)다.
    """
    ph = np.asarray(phase, np.int64) & ((1 << PHASE_BITS) - 1)
    q = ph >> (PHASE_BITS - 2)
    z = ((ph & ((1 << (PHASE_BITS - 2)) - 1)) << GZ) - (1 << (PHASE_BITS - 3 + GZ))
    x = np.full_like(z, _X0)
    y = x.copy()
    for i in range(CORDIC_IT):
        d = np.where(z >= 0, 1, -1)
        x, y = x - d * (y >> i), y + d * (x >> i)
        z = z - d * _ATAN[i]
    x = (x + (1 << (GX - 1))) >> GX
    y = (y + (1 << (GX - 1))) >> GX
    c = np.choose(q, [x, -y, -x, y])
    s = np.choose(q, [y, x, -y, -x])
    return c, s


def to_phase(psi_deg):
    """월드 회전각(도) → 위상. 가장 가까운 정수로 반올림하고 한 바퀴로 접는다."""
    return np.round(np.asarray(psi_deg) / 360.0 * (1 << PHASE_BITS)).astype(np.int64) & ((1 << PHASE_BITS) - 1)


def cells(x, y, c, s, cx, cy):
    """이벤트 좌표와 (이벤트마다의) cos · sin 정수 → 월드 칸 (wx, wy). 범위 검사는 하지 않는다."""
    dx = np.asarray(x, np.int64) - cx
    dy = np.asarray(y, np.int64) - cy
    off = WC << TRIG_F
    sh = TRIG_F + CELL_SHIFT
    return (c * dx - s * dy + off) >> sh, (s * dx + c * dy + off) >> sh


_STORED = None


def stored_mask():
    """뱅크 안 좌표 (wy >> 2, wx >> 2) 마다 저장하는지. 88 x 88 bool. 표를 만드는 식은 region_check.py 에 있다."""
    global _STORED
    if _STORED is None:
        import region_check                     # region_check 가 이 파일을 import 하므로 여기서 늦게 부른다
        _STORED = region_check.bank_union(region_check.touch_mask(REGION_R))
    return _STORED


def accumulate(wx, wy):
    """월드 칸 목록을 포화 카운터 메모리(M x N)에 센다. 돌려주는 값: (메모리, 범위 밖이라 버린 수)."""
    ok = (wx >= 0) & (wx < N) & (wy >= 0) & (wy < M)
    ok[ok] = stored_mask()[wy[ok] >> 2, wx[ok] >> 2]
    mem = np.bincount(wy[ok] * N + wx[ok], minlength=N * M)
    return np.minimum(mem, (1 << CELL_BITS) - 1).reshape(M, N), int((~ok).sum())


def read_params(path):
    """center.txt 같은 key = value 파일을 dict 로. # 줄은 주석."""
    out = {}
    for line in open(path, encoding='utf-8'):
        if '=' in line and not line.startswith('#'):
            k, v = line.split('=', 1)
            out[k.strip()] = float(v)
    return out


def frame_trig(bundle):
    """묶음 폴더에서 프레임별 (ψ 도, 위상, cos, sin) 과 회전 중심 (cx, cy)."""
    rot = np.loadtxt(os.path.join(bundle, 'rotation.csv'), delimiter=',', skiprows=1)
    p = read_params(os.path.join(bundle, 'center.txt'))
    psi = p['alpha_deg'] - rot[:, 2]
    ph = to_phase(psi)
    c, s = cordic(ph)
    return psi, ph, c, s, (int(round(p['cx'])), int(round(p['cy'])))


def run(d, ph_c_s, center, f0=0, f1=None):
    """프레임 f0 ~ f1-1 의 이벤트를 누적한 메모리와 버린 수, 이벤트 수."""
    _, c, s = ph_c_s
    lo, hi = np.searchsorted(d['frame'], [f0, d['n_frames'] if f1 is None else f1])
    mem, n_out = 0, 0
    for a in range(lo, hi, 1 << 23):                  # 메모리를 아끼려고 나눠서 센다
        b = min(a + (1 << 23), hi)
        f = d['frame'][a:b]
        m, k = accumulate(*cells(d['x'][a:b], d['y'][a:b], c[f], s[f], *center))
        mem, n_out = mem + m, n_out + k               # 조각끼리 더한 뒤 다시 포화시킨다
    return np.minimum(mem, (1 << CELL_BITS) - 1), n_out, hi - lo


def write_vectors(path, d, ph_c_s, center, f0, f1, out):
    """프레임 f0 ~ f1-1 의 테스트벤치 벡터.

    words.hex    : 원시 32 비트 워드(16 진 8 자리, 한 줄에 하나). 프레임 f0-1 을 닫는 끝 표지부터 프레임
                   f1-1 을 닫는 끝 표지까지. 첫 워드가 끝 표지라서 RTL 은 그 프레임 카운터를 알고 시작한다.
                   카운터가 바로 앞 끝 표지와 다른 끝 표지만 프레임 경계다(첫 끝 표지는 카운터만 기억한다).
                   f0 = 0 이면 파일 첫 워드(머리 포함)부터다. 녹화는 프레임 중간에서 시작하므로 이때는 첫
                   끝 표지 앞에 이벤트가 있고, RTL 은 그 끝 표지를 프레임 0 의 끝으로 센다.
    phase.hex    : 프레임 f0 ~ f1-1 의 위상(16 진 4 자리). k 번째 프레임의 이벤트는 k 번째 위상을 쓴다.
    trig.hex     : 같은 프레임의 CORDIC 출력 "cos sin" (각각 16 비트 2 의 보수, 16 진 4 자리). 디버그용
    cordic_all.hex : 위상 0 ~ 65535 전부의 CORDIC 출력 "cos sin". CORDIC 단독 전수 시험(tb_cordic)용
    expected.hex : 빈 메모리에서 시작해 이 프레임들을 누적한 메모리. 주소 wy·N + wx 순, 16 진
    params.txt   : 사양 상수와 프레임 · 워드 · 이벤트 수
    """
    os.makedirs(out, exist_ok=True)
    w = np.fromfile(path, dtype='>u4')
    a, b = (d['end_pos'][f0 - 1] if f0 else 0), d['end_pos'][f1 - 1] + 1
    np.savetxt(os.path.join(out, 'words.hex'), w[a:b], fmt='%08x')
    ph, c, s = (v[f0:f1] for v in ph_c_s)
    np.savetxt(os.path.join(out, 'phase.hex'), ph, fmt='%04x')
    np.savetxt(os.path.join(out, 'trig.hex'), np.column_stack([c & 0xffff, s & 0xffff]), fmt='%04x')
    ca, sa = cordic(np.arange(1 << PHASE_BITS))
    np.savetxt(os.path.join(out, 'cordic_all.hex'), np.column_stack([ca & 0xffff, sa & 0xffff]), fmt='%04x')
    mem, n_out, n_ev = run(d, ph_c_s, center, f0, f1)
    hexw = (CELL_BITS + 3) // 4
    np.savetxt(os.path.join(out, 'expected.hex'), mem.ravel(), fmt=f'%0{hexw}x')
    n_addr = int(((w[a:b] >> 24) == T_ADDR).sum())
    with open(os.path.join(out, 'params.txt'), 'w') as fo:
        for k, v in [('file', os.path.basename(path)), ('f0', f0), ('f1', f1), ('n_frames', f1 - f0),
                     ('n_words', b - a), ('n_addr_words', n_addr), ('n_events', n_ev), ('n_dropped', n_out),
                     ('cx', center[0]), ('cy', center[1]), ('N', N), ('M', M), ('CELL_SHIFT', CELL_SHIFT),
                     ('CELL_BITS', CELL_BITS), ('WC', WC), ('PHASE_BITS', PHASE_BITS), ('TRIG_F', TRIG_F),
                     ('CORDIC_IT', CORDIC_IT), ('GZ', GZ), ('GX', GX), ('X0', _X0),
                     ('ATAN', ' '.join(map(str, _ATAN))), ('mem_sum', int(mem.sum())), ('mem_max', int(mem.max())),
                     ('n_saturated', int((mem == (1 << CELL_BITS) - 1).sum()))]:
            fo.write(f'{k} = {v}\n')


def _self_check():
    """CORDIC 이 실수 cos · sin 과 2 LSB 안에서 같은지, 변환이 알려진 점을 제자리에 보내는지 본다."""
    ph = np.arange(1 << PHASE_BITS)
    c, s = cordic(ph)
    a = ph / (1 << PHASE_BITS) * 2 * pi
    err = np.maximum(np.abs(c - np.cos(a) * (1 << TRIG_F)), np.abs(s - np.sin(a) * (1 << TRIG_F)))
    assert err.max() < 2.0, err.max()
    # 회전 중심은 어느 각도에서나 월드 중심 칸으로 간다. 90° 에서 (dx, dy) = (102, 2) 는 (-2, 102) 로 간다.
    # 칸 경계에서 떨어진 점을 골라 cos · sin 의 1 LSB 오차가 칸을 바꾸지 않게 했다.
    c90, s90 = cordic(np.array([1 << (PHASE_BITS - 2)]))
    wx, wy = cells([500], [400], c90, s90, 500, 400)
    assert (wx[0], wy[0]) == (WC >> CELL_SHIFT, WC >> CELL_SHIFT), (wx, wy)
    wx, wy = cells([602], [402], c90, s90, 500, 400)
    assert (wx[0], wy[0]) == ((WC - 2) >> CELL_SHIFT, (WC + 102) >> CELL_SHIFT), (wx, wy)


def main(path, bundle, f0=500, f1=540):
    _self_check()
    d = load(path)
    psi, ph, c, s, center = frame_trig(bundle)
    np.savetxt(os.path.join(bundle, 'phase.csv'), np.column_stack([np.arange(len(ph)), psi, ph, c, s]),
               delimiter=',', fmt=['%d', '%.4f', '%d', '%d', '%d'], header='frame,psi_deg,phase,cos,sin',
               comments='')
    mem, n_out, n_ev = run(d, (ph, c, s), center)
    np.save(os.path.join(bundle, 'world.npy'), mem.astype(np.uint16))
    print(f'전체 {n_ev} 이벤트, 범위 밖 {n_out}, 칸 최댓값 {mem.max()}, 포화 칸 {(mem == (1 << CELL_BITS) - 1).sum()}')
    write_vectors(path, d, (ph, c, s), center, f0, f1, os.path.join(bundle, 'vectors'))
    print(open(os.path.join(bundle, 'vectors', 'params.txt')).read())


def write_events(path, d, ph, f0, f1, out):
    """기준 설계 wl_base_top(이벤트마다 x, y, 위상을 받는 비교용 RTL)의 입력.
    프레임 f0 ~ f1-1 의 이벤트를 파일 순서대로 한 줄에 하나, {위상 16, y 10, x 10} 36 비트를 16진수 9 자리로
    쓴다(events.hex). 위상은 그 이벤트가 든 프레임의 위상이다.
    기댓값은 같은 프레임의 vectors/expected.hex 와 같다."""
    lo, hi = np.searchsorted(d['frame'], [f0, f1])
    x, y, f = (np.asarray(d[k][lo:hi], np.int64) for k in ('x', 'y', 'frame'))
    w = (np.asarray(ph, np.int64)[f] << 20) | (y << 10) | x
    os.makedirs(out, exist_ok=True)
    np.savetxt(os.path.join(out, 'events.hex'), w, fmt='%09x')
    return hi - lo


def empty_frames(path, frames, out):
    """녹화에서 고른 프레임의 이벤트 워드와 열 주소 워드를 지워 빈 프레임을 만든 .dvs 를 쓴다(빈 프레임 시험용).
    머리 · 타임스탬프 · 끝 표지는 남기므로 프레임 수와 번호, 위상 순서는 그대로다. 지운 프레임 다음 프레임이
    열 주소 없이 시작하면 그 앞 마지막 열 주소를 이어 쓴다(파서와 RTL 이 같은 규칙). 돌려주는 값: 지운 워드 수."""
    w = np.fromfile(path, dtype='>u4')
    end = load(path)['end_pos']
    drop = np.zeros(len(w), bool)
    for f in frames:
        a, b = (end[f - 1] + 1 if f else 0), end[f]
        drop[a:b] = ((w[a:b] >> 31) == 1) | ((w[a:b] >> 24) == T_ADDR)
    w[~drop].astype('>u4').tofile(out)
    return int(drop.sum())


if __name__ == '__main__':
    if sys.argv[1] == 'empty':
        # 빈 프레임 시험 녹화: golden.py empty <녹화> <출력 .dvs> <프레임> [<프레임> ...]
        print(empty_frames(sys.argv[2], list(map(int, sys.argv[4:])), sys.argv[3]), 'words removed')
    elif sys.argv[1] == 'events':
        # 기준 설계 입력만 따로: golden.py events <녹화> <묶음> <f0> <f1> <출력 폴더>
        _, _, path, bundle, f0, f1, out = sys.argv[:7]
        _, ph, _, _, _ = frame_trig(bundle)
        print(write_events(path, load(path), ph, int(f0), int(f1), out), 'events')
    elif sys.argv[1] == 'vectors':
        # 벡터만 따로: golden.py vectors <녹화> <묶음|-> <f0> <f1|all> <출력 폴더> [cx cy]
        # 장면 1 전체(f0 = 0, f1 = all)처럼 커서 저장소에 두지 않을 벡터를 빌드 폴더에 만든다.
        # 묶음 자리에 - 를 주면 모든 프레임을 위상 0(회전 없음), 중심 (480, 360)으로 둔다. 회전각을 아직
        # 추정하지 않은 녹화로 디코더만 시험할 때 쓴다(장면 2 는 첫 열 주소 앞에 이벤트가 있다).
        _, _, path, bundle, f0, f1, out = sys.argv[:7]
        d = load(path)
        if bundle == '-':
            ph = np.zeros(d['n_frames'], np.int64)
            c, s = cordic(ph)
            center = (WIDTH // 2, HEIGHT // 2)
        else:
            _, ph, c, s, center = frame_trig(bundle)
        if len(sys.argv) > 7:
            center = (int(sys.argv[7]), int(sys.argv[8]))
        write_vectors(path, d, (ph, c, s), center, int(f0), d['n_frames'] if f1 == 'all' else int(f1), out)
        print(open(os.path.join(out, 'params.txt')).read())
    else:
        main(sys.argv[1], sys.argv[2], *map(int, sys.argv[3:5]))
