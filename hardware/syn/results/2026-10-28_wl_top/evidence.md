# 2026-10-28 wl_top · 기준 설계 합성 근거

이 묶음은 처음 판이다. 활동도 VCD와 SRAM 타이밍 모델의 slew 규약을 고쳐 다시 돌린 최종 판은
[2026-10-29_pnr](../2026-10-29_pnr/evidence.md)에 있다.

대회 서버(`snu.polaris.04`)에서 Genus로 설계(`wl_top`)와 기준 설계(`wl_base_top`)를 합성한 리포트다. 해석은
[설계 근거 9절](../../../../docs/reports/01_설계_근거.md#9-합성--배치--배선과-기준-설계-비교)에 있다. RTL은 2026-10-28 판 `hardware/src/`(칸 13비트,
같은 칸 합치기, 1RW 뱅크 16개, 원 영역 저장)이다.

| 항목 | 값 |
|---|---|
| 툴 | Genus 23.14-s090_1, VCD 읽기는 Genus 안의 Joules 파서 |
| 라이브러리 | `gsclib045_all_v4.7/gsclib045/timing/slow_vdd1v0_basicCells.lib` ([libs.txt](libs.txt)) |
| 코너 | `PVT_0P9V_125C` (0.9 V, 125 °C, process 1). [wl_top/corner.txt](wl_top/corner.txt) |
| 목표 주파수 | 200 MHz (주기 5 ns). [wl_top.sdc](../../../constraints/wl_top.sdc), [wl_base_top.sdc](../../../constraints/wl_base_top.sdc) |
| SRAM | 뱅크 16개는 합성하지 않는다. [wl_sram.lib](../../wl_sram.lib)의 타이밍 전용 셀(면적 · 전력 0, 읽기 2.0 ns, setup 0.3 ns 가정)로 둔다 |
| 계층 | `auto_ungroup none`. 블록별 면적 · 전력이 나온다 |

| 폴더 | 명령(서버 `~/worldlock`) | 무엇 |
|---|---|---|
| `wl_top/` | `env VCD=vcd/wl_top_20k.vcd hardware/syn/run_genus.sh wl_top` | 설계, clock gating 넣음 |
| `wl_top_nocg/` | `env CG=0 VCD=vcd/wl_top_20k.vcd hardware/syn/run_genus.sh wl_top - nocg_vcd` | 설계, clock gating 뺌(clock gating 비교용) |
| `wl_base_top/` | `env VCD=vcd/base_20k.vcd VCD_SCOPE=TOP/tb_base/dut hardware/syn/run_genus.sh wl_base_top` | 기준 설계(이벤트마다 입력), clock gating 넣음 |

`wl_top_nocg/`는 서버에서 `wl_top_nocg_vcd`라는 이름으로 돌린 결과를 옮긴 것이다.

## 결과

| | `wl_top` | `wl_top_nocg` | `wl_base_top` |
|---|---:|---:|---:|
| 셀 면적 (µm²) | 11,081.142 | 12,340.044 | 21,204.684 |
| 순차 셀 (SRAM 뱅크 16 · clock gating 셀 포함) | 633 | 610 | 1,527 |
| 최악 slack (ps) | +5.9 | +3.5 | +483.0 |
| 전력, vectorless (mW) | 0.633 | 1.120 | 2.059 |
| 전력, VCD 구간 (mW) | 0.424 | 0.717 | 1.050 |
| VCD 구간의 이벤트 | 19,614 | 19,614 | 19,717 |
| 이벤트당 로직 에너지 (pJ) | 2.16 | 3.65 | 5.33 |

리포트: 폴더마다 `qor.rpt` · `area.rpt` · `area_hier.rpt` · `gates.rpt` · `timing.rpt` · `power.rpt`(vectorless) ·
`power_vcd.rpt` · `power_vcd_hier.rpt` · `clock_gating.rpt` · `check_design.rpt`, 그리고 SRAM 뱅크로 들어가는 경로
`timing_sram_addr.rpt` · `timing_sram_wd.rpt`.

- `check_design.rpt`: 미해결 참조 0개. 뱅크 16개가 `wl_sram` 셀로 묶였다(`gates.rpt`의 `wl_sram 16`).
- `wl_top` 최악 경로는 좌표 변환 X 단의 덧셈(`u_xf_xv_reg` → `u_xf_bv_reg`)이다. SRAM 읽기 → 더하기 → 쓰기 경로는
  데이터 경로 4,217 ps에 slack +483 ps라, 가정한 접근 시간 2.0 ns가 2.48 ns까지 늘어도 맞는다.
- `wl_base_top`의 최악 경로는 그 SRAM 경로다. 나머지는 단마다 레지스터를 둔 파이프라인이라 여유가 있다.

## LEC

`wl_top/lec.rpt`는 Conformal LEC 24.10으로 RTL(`hardware/src`, 뱅크는 같은 블랙박스 셀)과 `wl_top` 넷리스트를 견준
결과다. 비교한 667점(출력 57, 플립플롭 594, SRAM 뱅크 16)이 모두 등가이고 "Compare Results: PASS"다. 곱셈기 출력 26점은
그냥 비교하면 판정이 나지 않아서(abort) `analyze datapath -merge`와 `analyze abort -compare`를 넣었다.

    hardware/syn/run_lec.sh wl_top      # 서버 ~/worldlock, run_genus.sh 뒤

## 게이트 레벨 시뮬레이션

`wl_top/gls.txt`는 `wl_top` 넷리스트를 Xcelium 23.09로 지연 없이(zero delay) 시뮬레이션한 결과다. 테스트벤치는 RTL 시험과
같은 `tb_worldlock.v`(GLS 정의), 벡터는 장면 1 프레임 500 ~ 539, 표준 셀은 `slow_vdd1v0_basicCells.v`, SRAM 뱅크는 RTL
동작 모델(`wl_sram.v`)이다. 123,904칸이 모두 골든과 같고, 클럭 수도 RTL(xsim)과 같은 2,888,025다. 벡터 세 파일은 사용자
허용(2026-10-28)으로 서버 `~/worldlock/vec500/`에 올렸다.

    hardware/sim/run_gls.sh vec500      # 서버 ~/worldlock, run_genus.sh 뒤

## VCD

활동도는 Verilator 테스트벤치에서 뽑았다. 기본 벡터(장면 1 프레임 500 ~ 539)에서 입력을 내기 시작한 지 1,400,000클럭
뒤부터 20,000클럭(200 MHz로 100 µs) 구간이다. 이벤트가 거의 매 클럭 들어오는 구간이라 활동 중 전력이다.
Verilator는 `$dumpoff`를 따르지 않아 끝까지 덤프하므로, 구간 끝 시각(시작 + 100,000,000 ps) 뒤를 잘라 썼다.

    VFLAGS="--trace --trace-max-array 64" hardware/sim/run_sim.sh verilator tb_worldlock "" \
        +vcd=wl_top_full.vcd +vcd_from=1400000 +vcd_len=20000
    awk -v end=<첫 시각 + 100000000> '/^#/{t=substr($0,2)+0; if (t>end) exit} {print}' wl_top_full.vcd > wl_top_20k.vcd
    # 기준 설계: tb_base, 벡터 ~/wl_build/vec_base, 같은 구간

Genus의 주석 리포트에서 입출력 포트와 플립플롭 활동도가 모두(100 %) 붙었다. 이벤트당 로직 에너지는 VCD 구간 전력 ×
100 µs ÷ 구간의 이벤트 수다. SRAM 에너지는 들어 있지 않다.

면적 · 전력은 리포트 단위 그대로다(면적 µm², 전력 W). VCD 파일(17 · 21 MB)은 다시 만들 수 있어 저장소에 두지 않았다.
