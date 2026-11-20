#!/usr/bin/env python3
"""roll 이 쓸고 지나가는 원 안의 칸만 저장하면 월드 메모리 칸이 얼마나 주는지, 주소 매핑 방식별로 센다.

    python3 software/golden_develop/region_check.py <골든 묶음> <출력 폴더>

roll 에서는 영상의 모든 픽셀이 회전 중심 둘레를 돌므로, 월드 메모리에 닿는 곳은 월드 중심(회전 중심이 가는
곳)에서 반지름 R 인 원 안뿐이다. R 은 회전 중심에서 가장 먼 센서 모서리까지의 거리다. 정사각형 메모리
(352 x 352 칸)의 네 귀퉁이는 쓰이지 않는다. 그 귀퉁이를 빼는 주소 매핑을 세 가지로 견준다.

  행 표       행 wy 마다 (시작 열 s, 칸 수 L, 앞 행까지의 칸 수 합 base) 를 표로 두고
              주소 = base[wy] + (wx - s[wy]). 원에 꼭 맞춘다. 표는 위아래 대칭이라 반만 두어도 된다
  띠 B 개     행을 같은 높이의 띠 B 개로 나누고 띠마다 가장 넓은 행의 폭을 쓴다. 표가 B 줄로 준다
  타일        메모리를 SRAM macro 하나 크기의 직사각형 타일로 나누고, 원에 걸치지 않는 타일은 두지 않는다.
              주소 = (타일 번호 → macro 고르기, 타일 안 위치). 표는 타일 수만큼의 macro 번호뿐이다

원에 걸치는 칸: 칸(4 px 정사각형)의 원 중심에 가장 가까운 점이 원 안이면 넣는다. 반지름은
  센서 모서리   회전 중심에서 가장 먼 센서 모서리 픽셀 중심까지(장면 1 에서 677.4 px)
  실측          센서 모서리 거리를 정수로 올린 값에서 시작해, 장면 1 골든 지도에서 이벤트가 든 칸이 모두
                들어갈 때까지 1 px 씩 늘린 값(장면 1 에서는 올린 값 678 그대로)
  내접원        704 px (메모리 한 변의 절반). 정사각형 메모리가 어느 방향으로나 담는 가장 큰 원
을 나란히 본다.

뱅크로 나눈 메모리 (wl_accum)
  wl_accum 은 칸을 좌표 아래 2 비트로 뱅크 16 개({wy[1:0], wx[1:0]})에 나누고, 뱅크 안에서 칸은 88 x 88 격자의
  (wy >> 2, wx >> 2) 에 놓인다. 원 영역만 두면 뱅크마다 88 x 88 격자 안의 원 모양이 남는다. 뱅크 16 개가 표 하나를
  같이 쓰도록 16 개 원 모양의 합집합을 행 표로 만들고(bank_union), 뱅크 하나가 담아야 할 word 수를 OpenRAM 1RW
  macro(1,024 · 4,096 word, 면적은 MACRO_CSV)로 가장 작은 면적이 되게 채운다. golden.py 의 stored_mask 와
  RTL 의 wl_region_map 이 이 표를 쓴다.

    python3 software/golden_develop/region_check.py <골든 묶음> <출력 폴더> [<RTL 파일>]

  <RTL 파일> 을 주면 실측 반지름과 macro 조합 면적이 같은 가장 큰 반지름의 표로 Verilog 파일 하나를 쓴다. 그 안에
  설계가 쓰는 주소 모듈 wl_region_map 과, 로직 비용을 합성으로 견주는 비교용 모듈 wl_region(같은 표, 레지스터와
  macro 나눔 포함) · wl_region_sq(정사각형 주소 (wy >> 2) · 88 + (wx >> 2))가 들어간다. hardware/src/wl_region.v 가
  이렇게 쓴 파일이다.

출력
  <출력 폴더>/region.txt : 반지름별 칸 수, 방식별 칸 수 · 표 크기, 장면 1 지도에서 원 밖에 떨어진 이벤트,
                          뱅크 16 개에서 뱅크당 word 수와 macro 조합 · 면적(OpenRAM 가정치)
"""
import csv
import os
import sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import golden as g
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'dvs_develop'))
from dvs_read import WIDTH, HEIGHT

BANDS = (1, 2, 4, 8, 16, 32, 88, 176)
# 타일(행 x 열). OpenRAM macro 의 word 수 1,024 와 4,096 에 맞춘 크기
TILES = ((32, 32), (32, 64), (64, 64), (32, 128))
CELL_BITS_CASES = (14, 13, 10)   # 메모리 비트 수를 셀 때 쓰는 칸 비트 수. 13 은 RTL 의 포화 카운터 폭(BITS)
BK = 2                            # 뱅크를 고르는 좌표 비트 수(wl_accum 의 BK 와 같다). 뱅크 4^BK 개
MACRO_CSV = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'hardware', 'sram', 'results',
                         '2026-10-28_openram', 'summary.csv')


def touch_mask(R):
    """원(반지름 R px, 월드 중심)에 걸치는 칸. M x N bool."""
    c = 1 << g.CELL_SHIFT
    lo = np.arange(g.N) * c - g.WC                 # 칸 왼쪽(위쪽) 경계, 월드 중심 기준 px
    hi = lo + c
    near = np.where((lo <= 0) & (hi >= 0), 0, np.minimum(np.abs(lo), np.abs(hi)))
    return near[:, None] ** 2 + near[None, :] ** 2 < R * R


def row_table(m):
    """행마다 (시작 열, 칸 수). 빈 행은 (0, 0). 원은 볼록이라 한 행에서 쓰는 칸은 이어져 있다."""
    L = m.sum(1)
    s = np.where(L > 0, m.argmax(1), 0)
    assert all(m[i, s[i]:s[i] + L[i]].all() for i in range(len(L)))
    return s, L


def bands(m, B):
    """행을 B 개 띠로 나눠 띠마다 그 띠의 가장 넓은 행 폭을 쓸 때의 칸 수. 폭은 가운데 맞춤이라 대칭이다."""
    L = m.sum(1)
    edges = np.linspace(0, g.M, B + 1).round().astype(int)
    return int(sum(L[a:b].max() * (b - a) for a, b in zip(edges[:-1], edges[1:])))


def tiles(m, th, tw):
    """th x tw 칸 타일로 나눴을 때 (원에 걸치는 타일의 칸 수, 그 타일 수, 전체 타일 수).

    352 가 th · tw 의 배수가 아니면 오른쪽 · 아래 가장자리 타일은 잘린 크기(더 작은 macro)로 둔다고 보고
    잘린 칸 수로 센다. 원에 걸치지 않는 타일만 뺀다.
    """
    ty, tx = -(-g.M // th), -(-g.N // tw)
    pad = np.zeros((ty * th, tx * tw), bool)
    pad[:g.M, :g.N] = m
    hit = pad.reshape(ty, th, tx, tw).any(axis=(1, 3))
    size = np.outer(np.minimum(th, g.M - np.arange(ty) * th), np.minimum(tw, g.N - np.arange(tx) * tw))
    return int(size[hit].sum()), int(hit.sum()), ty * tx


def bank_union(m):
    """뱅크 4^BK 개의 원 모양(뱅크 안 좌표)의 합집합. 표 하나를 모든 뱅크가 같이 쓸 때 뱅크마다 두는 칸."""
    b = 1 << BK
    u = np.zeros((g.M >> BK, g.N >> BK), bool)
    for by in range(b):
        for bx in range(b):
            u |= m[by::b, bx::b]
    return u


def macros():
    """OpenRAM 1RW macro {word 수: 면적 µm²}. 칸 14 비트로 만든 것이다."""
    return {int(r['words']): float(r['area_um2']) for r in csv.DictReader(open(MACRO_CSV)) if r['ports'] == '1rw' and r['word_bits'] == '14'}


def packing(n, area):
    """word n 개를 담는 macro 조합 중 면적이 가장 작은 것. (면적, {word 수: 개수})."""
    big, small = max(area), min(area)
    best = None
    for a in range(0, -(-n // big) + 1):
        b = max(0, -(-(n - a * big) // small))
        cand = (a * area[big] + b * area[small], {big: a, small: b})
        if best is None or cand[0] < best[0]:
            best = cand
    return best


def write_rtl(path, R, u):
    """표 u(뱅크 안 88 x 88) 로 설계가 쓰는 wl_region_map 과 비교용 wl_region · wl_region_sq 를 path 에 쓴다."""
    s_, L = row_table(u)
    base = np.r_[0, np.cumsum(L)[:-1]]
    rows = u.shape[0]
    words = int(L.sum())
    assert words <= 6144 and rows == 88
    lines = []
    for r in range(rows):
        if L[r]:
            off = (int(base[r]) - int(s_[r])) & 0x3fff
            lines.append(f"            7'd{r}: rom = {{14'h{off:04x}, 7'd{int(s_[r])}, 7'd{int(s_[r] + L[r])}}};")
    table = '\n'.join(lines)
    text = f"""// wl_region : camera roll 이 쓸고 지나가는 원만 저장할 때 월드 칸의 bank, macro,
// macro 주소. 출력은 레지스터로 한 클록 늦춰 낸다.
//
// 이 파일에는 모듈이 셋 있다. wl_region 과 wl_region_sq 는 비교용 모듈이다:
// 둘을 나란히 합성해 원 매핑의 로직 비용을 단순 정사각형 매핑과 견준다.
// 설계 자체는 wl_region_map(이 파일 맨 끝)만 인스턴스한다. 이는 wl_region 과
// 같은 원 표에서 레지스터와 macro 나눔을 뺀 것이다.
//
// 월드 메모리는 wl_accum 처럼 bank 16 개로 나뉜다: bank =
// {{wy[1:0], wx[1:0]}} 이고, bank 안에서 칸은 88 x 88 격자의 bank 행 r = wy >> 2,
// bank 열 c = wx >> 2 에 놓인다. 격자 전체를 저장하면 주소가
// r * 88 + c 이고 bank 당 7,744 word 가 든다(wl_region_sq).
//
// roll 에서는 모든 픽셀이 회전 중심 둘레를 돌고 회전 중심은 월드 중심에 놓이므로,
// 쓰기는 월드 중심 둘레의 원 안에서만 일어난다. 여기서 bank 는 반지름 {R} px
// 원의 칸만 둔다: bank 행 r 은 열 START[r] .. END[r] - 1 을 담고 행을 차례로
// 이어 붙이므로, 주소는 BASE[r] + c - START[r] 이다. BASE[r] 은 행
// 0 .. r - 1 의 칸 수다. 표 하나를 bank 16 개가 같이 쓰고(16 개 원 칸의 합집합)
// bank 당 {words} word 가 필요하다. 이는 4,096-word macro 하나와 1,024-word
// macro 두 개(6,144 word)에 들어간다. 반지름은 스크립트가 시도한 것 가운데 표가 이
// macro 조합에 아직 들어가는 가장 큰 값이다. 28 비트 표 항목은 각각
// {{OFF, START, END}} 이다: OFF = BASE - START(14 비트 2의 보수)는 [27:14],
// START 는 [13:7], END 는 [6:0] 에 있다. 표 밖의 칸은 저장하지 않는다.
//
// software/golden_develop/region_check.py 가 자기 표로 이 파일을 쓴다. 이 파일이 아니라
// 스크립트를 고친다.
module wl_region (
    input  wire        clk,
    input  wire        in_valid,
    input  wire [8:0]  wx,          // 월드 칸 열, 0 .. 351
    input  wire [8:0]  wy,          // 월드 칸 행, 0 .. 351
    output reg         out_valid,   // 한 클록 뒤의 in_valid, 원 밖이면 0
    output reg  [3:0]  bank,
    output reg  [1:0]  macro,       // 0: 4,096-word macro, 1 과 2: 1,024-word macro
    output reg  [11:0] maddr        // 그 macro 안의 word
);
    function [27:0] rom(input [6:0] r);
        case (r)
{table}
            default: rom = 28'd0;   // 빈 행: START = END = 0, 안에 든 칸이 없다
        endcase
    endfunction

    wire [6:0]  r = wy[8:2];
    wire [6:0]  c = wx[8:2];
    wire [27:0] t = rom(r);
    wire        in_row = (c >= t[13:7]) && (c < t[6:0]);
    wire [13:0] a = t[27:14] + {{7'd0, c}};   // bank 안 word, in_row 이면 0 .. {words - 1}

    // word 0 .. 4,095 는 macro 0, 4,096 .. 5,119 는 macro 1,
    // 5,120 .. 6,143 은 macro 2 로 간다(a[10] 이 작은 macro 둘 중 하나를 고른다).
    always @(posedge clk) begin
        out_valid <= in_valid & in_row;
        bank      <= {{wy[1:0], wx[1:0]}};
        macro     <= a[12] ? (a[10] ? 2'd2 : 2'd1) : 2'd0;
        maddr     <= a[12] ? {{2'd0, a[9:0]}} : a[11:0];
    end
endmodule

// wl_region_sq : 비교용 모듈. 표 없는 단순 정사각형 매핑이다:
// 주소 r * 88 + c(0 .. 7,743)를 bank 마다 4,096-word macro 두 개에 나눠 담으므로
// 버리는 칸이 없다(out_valid = in_valid). wl_region 과 포트가 같아 둘을
// 나란히 합성할 수 있다.
module wl_region_sq (
    input  wire        clk,
    input  wire        in_valid,
    input  wire [8:0]  wx,
    input  wire [8:0]  wy,
    output reg         out_valid,
    output reg  [3:0]  bank,
    output reg  [1:0]  macro,
    output reg  [11:0] maddr
);
    wire [12:0] a = {{6'd0, wy[8:2]}} * 13'd88 + {{6'd0, wx[8:2]}};   // 0 .. 7,743

    always @(posedge clk) begin
        out_valid <= in_valid;
        bank      <= {{wy[1:0], wx[1:0]}};
        macro     <= {{1'b0, a[12]}};
        maddr     <= a[11:0];
    end
endmodule

// wl_region_map : wl_region 의 표를 조합 논리로 둔 것으로, 레지스터와 macro 나눔이
// 없다. 설계가 쓰는 매핑이 이것이다: wl_xform 은 칸이 표 밖인 이벤트를 버리고
// (hit 0), wl_accum 은 addr 를 bank {{wy[1:0], wx[1:0]}} 안의 word 로 삼아 갱신과
// 읽기에 쓴다. bank 하나는 {words} word 를 담고, 이를 macro 로 어떻게 나눌지는
// 메모리 쪽에 맡긴다.
// 좌표 352 .. 511 은 r 이나 c 가 88 .. 127 이 되어 모든 표 행 밖이므로
// (행 88 .. 127 은 비어 있고 END <= 88) 거기서는 hit 가 0 이다.
module wl_region_map (
    input  wire [8:0]  wx,
    input  wire [8:0]  wy,
    output wire        hit,         // 이 칸을 저장한다
    output wire [12:0] addr         // 그 bank 안의 word, hit 이면 0 .. {words - 1}
);
    function [27:0] rom(input [6:0] r);
        case (r)
{table}
            default: rom = 28'd0;
        endcase
    endfunction

    wire [6:0]  r = wy[8:2];
    wire [6:0]  c = wx[8:2];
    wire [27:0] t = rom(r);
    wire [13:0] a = t[27:14] + {{7'd0, c}};

    assign hit  = (c >= t[13:7]) && (c < t[6:0]);
    assign addr = a[12:0];
endmodule
"""
    with open(path, 'w') as fo:
        fo.write(text)


def main(bundle, outdir, rtl=None):
    os.makedirs(outdir, exist_ok=True)
    p = g.read_params(os.path.join(bundle, 'center.txt'))
    cx, cy = int(round(p['cx'])), int(round(p['cy']))
    W = np.load(os.path.join(bundle, 'world.npy'))
    used = W > 0
    corners = [(0, 0), (WIDTH - 1, 0), (0, HEIGHT - 1), (WIDTH - 1, HEIGHT - 1)]
    r_corner = max(np.hypot(x - cx, y - cy) for x, y in corners)
    r_meas = int(np.ceil(r_corner))
    while (used & ~touch_mask(r_meas)).any():
        r_meas += 1
    full = g.N * g.M
    out = [f'쓰는 영역만 저장. 월드 메모리 {g.N} x {g.M} = {full:,} 칸, 칸 {1 << g.CELL_SHIFT} px, '
           f'월드 중심 {g.WC} px, 장면 1 회전 중심 ({cx}, {cy})', '']
    out.append('[1] 반지름별로 원에 걸치는 칸')
    out.append(f'    {"반지름":<22} {"px":>7} {"칸 수":>9} {"정사각형 대비":>12}  장면 1 이벤트 든 칸 중 원 밖')
    radii = [('센서 모서리', r_corner), ('실측(이벤트 든 칸 모두)', r_meas), ('내접원(중심 무관)', g.WC)]
    masks = {}
    for name, R in radii:
        m = touch_mask(R)
        masks[name] = (R, m)
        out.append(f'    {name:<22} {R:7.1f} {int(m.sum()):9,} {m.sum() / full * 100:11.2f} %  '
                   f'{int((used & ~m).sum())} 칸, 이벤트 {int(W[~m].sum())}')
    out.append(f'    장면 1 지도에서 이벤트가 든 칸 {int(used.sum()):,} ({used.sum() / full * 100:.2f} %)')
    out.append('')

    for name in ('실측(이벤트 든 칸 모두)', '내접원(중심 무관)'):
        R, m = masks[name]
        s, L = row_table(m)
        n_row = int(L.sum())
        out.append(f'[2] {name}, 반지름 {R:.0f} px: 방식별 칸 수와 표')
        out.append(f'    {"방식":<26} {"칸 수":>9} {"줄어든 칸":>9} {"감소":>7}  표')
        # 행 표: 위아래 대칭이면 반만 두고, 시작 열은 가운데에서 폭의 절반으로 바로 나온다
        sym = np.array_equal(L, L[::-1]) and np.array_equal(s, g.N - s - L)
        half_bits = int(np.ceil(np.log2(g.N // 2 + 1)))
        base_bits = int(np.ceil(np.log2(n_row + 1)))
        col_bits = int(np.ceil(np.log2(g.N + 1)))
        if sym:     # 위아래 · 좌우 대칭: 반만 두고 시작 열은 가운데에서 반폭으로 바로 나온다
            rows, entry, what = g.M // 2, half_bits + base_bits, f'반폭 {half_bits} + base {base_bits}'
        else:
            rows, entry, what = g.M, 2 * col_bits + base_bits, f'시작 열 {col_bits} + 길이 {col_bits} + base {base_bits}'
        out.append(f'    {"행 표":<26} {n_row:9,} {full - n_row:9,} {(1 - n_row / full) * 100:6.2f} %  '
                   f'{rows} 줄 x ({what}) 비트 = {rows * entry:,} 비트{" (위아래 대칭이라 반만)" if sym else ""}')
        for B in BANDS:
            n = bands(m, B)
            out.append(f'    {"띠 " + str(B) + "개":<26} {n:9,} {full - n:9,} {(1 - n / full) * 100:6.2f} %  {B} 줄')
        for th, tw in TILES:
            n, hit, tot = tiles(m, th, tw)
            out.append(f'    {"타일 " + f"{th}x{tw} ({th * tw:,} word)":<26} {n:9,} {full - n:9,} {(1 - n / full) * 100:6.2f} %  '
                       f'타일 {hit} / {tot} 개의 macro 번호(가장자리 타일은 잘린 크기)')
        out.append('')

    out.append('[3] 월드 메모리 비트(행 표 기준)')
    for name in ('실측(이벤트 든 칸 모두)', '내접원(중심 무관)'):
        n = int(masks[name][1].sum())
        out.append(f'    {name:<22} ' + ', '.join(f'칸 {b}비트: {full * b:,} → {n * b:,} 비트' for b in CELL_BITS_CASES))
    area = macros()
    b = 1 << BK
    nb = b * b
    full_bank = (g.M >> BK) * (g.N >> BK)
    sq_area, sq_pack = packing(full_bank, area)
    out.append('')
    out.append(f'[4] 뱅크 {nb}개(wl_accum, 뱅크 = {{wy[{BK - 1}:0], wx[{BK - 1}:0]}}), 뱅크 안 {g.M >> BK} x {g.N >> BK} 칸. '
               f'macro 는 OpenRAM 1RW 14비트 ' + ', '.join(f'{w:,} word {a:,.0f} µm²' for w, a in sorted(area.items())))
    out.append(f'    정사각형(전체 저장): 뱅크당 {full_bank:,} word, macro ' +
               ' + '.join(f'{k:,} x {v}' for k, v in sorted(sq_pack.items(), reverse=True) if v) +
               f', 뱅크 {nb}개 {sq_area * nb / 1e6:.3f} mm²')
    out.append(f'    {"반지름 px":>9} {"뱅크당 원 칸":>12} {"공유 표 word":>12}  macro 조합(뱅크당)           SRAM 면적   감소')
    fit = None
    for R in sorted(set([int(np.ceil(r_corner)), r_meas] + list(range(680, g.WC + 1, 4)) + [g.WC])):
        m = touch_mask(R)
        per = [int(m[by::b, bx::b].sum()) for by in range(b) for bx in range(b)]
        u = bank_union(m)
        a_, pk = packing(int(u.sum()), area)
        if a_ <= packing(int(bank_union(touch_mask(r_meas)).sum()), area)[0]:
            fit = (R, u)
        out.append(f'    {R:9d} {min(per):5,} ~ {max(per):5,} {int(u.sum()):12,}  ' +
                   f'{" + ".join(f"{k:,} x {v}" for k, v in sorted(pk.items(), reverse=True) if v):<28} '
                   f'{a_ * nb / 1e6:7.3f} mm²  {(1 - a_ / sq_area) * 100:5.1f} %')
    R_fit, u_fit = fit
    s_, L = row_table(u_fit)
    out.append(f'    실측 반지름과 macro 조합이 같은 가장 큰 반지름: {R_fit} px. 공유 표 {int((L > 0).sum())} 줄 x '
               f'(OFF 14 + START 7 + END 7) 비트 = {int((L > 0).sum()) * 28:,} 비트')
    if rtl:
        write_rtl(rtl, R_fit, u_fit)
        out.append(f'    RTL: {os.path.relpath(rtl)} (wl_region: 반지름 {R_fit} px 표, wl_region_sq: 비교용 정사각형)')

    text = '\n'.join(out)
    open(os.path.join(outdir, 'region.txt'), 'w', encoding='utf-8').write(text + '\n')
    print(text)


if __name__ == '__main__':
    main(*sys.argv[1:4])
