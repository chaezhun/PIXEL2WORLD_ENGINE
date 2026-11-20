# 2026-10-29 최종 합성 · 배치 · 배선 근거

대회 서버(`snu.polaris.04`)에서 설계 `wl_top`과 기준 설계 `wl_base_top`을 다시 합성하고, `wl_top`을 Innovus로 배치 · 배선한
결과다. 그 뒤 두 넷리스트를 등가 확인(LEC)과 게이트 레벨 시뮬레이션으로 확인했다. 2026-10-28 23:48부터 10-29 00:10까지
돌렸다. 해석은 [설계 근거 9절](../../../../docs/reports/01_설계_근거.md#9-합성--배치--배선과-기준-설계-비교)과 [설계 근거 9절](../../../../docs/reports/01_설계_근거.md#9-합성--배치--배선과-기준-설계-비교)에
있다.

앞 묶음 [2026-10-28_wl_top](../2026-10-28_wl_top/evidence.md)과 RTL · 합성 스크립트가 같다. 다른 점은 셋이다.

- 활동도 VCD를 막대 중심 골든 벡터([2026-10-28_golden_bar_center](../../../../software/golden_data/2026-10-28_golden_bar_center/2026-10-28_golden_bar_center.md))로
  다시 뽑았다.
- SRAM 뱅크 타이밍 모델 [wl_sram.lib](../../wl_sram.lib)의 `slew_derate_from_library`를 1.0에서 0.5로 바꿨다. 표준 셀
  라이브러리와 같은 규약으로 맞춘 것이다. 예전 값에서는 뱅크 출력 slew 0.2 ns가 표준 셀 표 기준 0.4 ns로 읽혀,
  배치 · 배선에서 max transition 위반 190개가 났다.
- 게이트 레벨 시뮬레이션과 넷리스트 등가 확인을 배치 · 배선 뒤 넷리스트에도 했다.

| 항목 | 값 |
|---|---|
| 툴 | Genus 23.14-s090_1, Innovus 23.14-s088_1(Common UI), Conformal LEC 24.10, Xcelium 23.09 |
| 라이브러리 | `gsclib045` `slow_vdd1v0_basicCells.lib`(셋업 · 합성), `fast_vdd1v0_basicCells.lib`(hold). [libs.txt](libs.txt) |
| 코너 | 셋업 `PVT_0P9V_125C`(0.9 V, 125 °C), hold `PVT_1P1V_0C`(1.1 V, 0 °C). 배선 RC는 `gpdk045.tch`(QRC 기술 파일)로 뽑고 온도만 코너에 맞췄다 |
| 목표 주파수 | 200 MHz (주기 5 ns). [wl_top.sdc](../../../constraints/wl_top.sdc), [wl_base_top.sdc](../../../constraints/wl_base_top.sdc) |
| SRAM | 합성 · 배치 · 배선 모두 블랙박스 뱅크 16개. 타이밍은 [wl_sram.lib](../../wl_sram.lib), 배치용 외곽은 [wl_sram.lef](../../wl_sram.lef)(320.00 × 422.37 µm, 신호 핀만 있고 전원 핀은 없음) |

## 명령(서버 `~/worldlock`)

| 폴더 | 명령 |
|---|---|
| `wl_top/` | `env VCD=vcd/wl_top_20k.vcd hardware/syn/run_genus.sh wl_top`, 이어서 `hardware/syn/run_lec.sh wl_top`, `hardware/sim/run_gls.sh vec500` |
| `wl_top_nocg/` | `env CG=0 VCD=vcd/wl_top_20k.vcd hardware/syn/run_genus.sh wl_top - nocg` |
| `wl_base_top/` | `env VCD=vcd/base_20k.vcd VCD_SCOPE=TOP/tb_base/dut hardware/syn/run_genus.sh wl_base_top` |
| `wl_top_pnr/` | `env VCD=vcd/wl_top_20k.vcd hardware/syn/run_innovus.sh wl_top`, `hardware/syn/run_lec.sh wl_top pnr`, `hardware/sim/run_gls.sh vec500 hardware/syn/out/wl_top_pnr/netlist.v` |
| `wl_top_pnr/power_gls*.rpt` | `hardware/sim/run_gls.sh vec500 hardware/syn/out/wl_top_pnr/netlist.v +vcd=vcd/wl_top_pnr_20k.vcd +vcd_from=1400000`, 이어서 `env PWR=1 VCD=vcd/wl_top_pnr_20k.vcd VCD_SCOPE=tb_worldlock/dut hardware/syn/run_innovus.sh wl_top` |

`lec.rpt`는 `run_lec.sh`가 쓴 `lec.log`이고, `gls.txt`는 `run_gls.sh`가 화면에 낸 줄이다. `layout.png`는 개발 서버에서
`python3 hardware/sim/plot_layout.py <wl_top.def> layout.png`로 그렸다. DEF(16 MB) · SDF · 넷리스트 · Innovus 데이터베이스는
다시 만들 수 있어 저장소에 두지 않았다.

## 합성

| | `wl_top` | `wl_top_nocg` | `wl_base_top` |
|---|---:|---:|---:|
| 셀 면적 (µm²) | 11,081.142 | 12,340.044 | 21,204.684 |
| 셀 수 / 순차 셀(SRAM 뱅크 16 포함) | 4,412 / 633 | 4,778 / 610 | 6,727 / 1,527 |
| 최악 slack (ps) | +5.9 | +3.5 | +505.3 |
| SRAM 읽기 → 더하기 → 쓰기 경로 slack (ps) | +607 (데이터 경로 4,093) | +607 | +607 |
| 전력, vectorless (mW) | 0.633 | 1.120 | 2.059 |
| 전력, VCD 구간 (mW) | 0.432 | 0.725 | 1.054 |
| VCD 구간의 이벤트 | 19,639 | 19,639 | 19,771 |
| 이벤트당 로직 에너지 (pJ) | 2.20 | 3.69 | 5.33 |

면적 · 셀 수 · 최악 slack은 앞 묶음과 같다. VCD 전력은 벡터가 바뀌어 조금 달라졌고(0.424 → 0.432 mW), SRAM 경로 slack은
뱅크 출력 slew를 고쳐 +483에서 +607 ps로 늘었다. 기준 설계는 최악 경로가 SRAM 경로라(앞 묶음은 쓰기 데이터, 이번은 뱅크 주소)
그 slack이 +483에서 +505 ps가 됐다.
VCD 구간은 앞 묶음과 같은 방법(입력 시작 뒤 1,400,000클럭부터 20,000클럭)이다. 이벤트당 로직 에너지는 VCD 구간 전력 ×
100 µs ÷ 구간의 이벤트 수다. Genus의 주석 리포트에서 입출력 포트와 플립플롭의 활동도가 모두 붙었다.

## 배치 · 배선 (`wl_top_pnr/`)

| 항목 | 값 | 파일 |
|---|---|---|
| 다이 / 코어 | 1,440.00 × 1,850.79 µm (2.665 mm²) / 1,400.00 × 1,810.89 µm | `floorplan.txt` |
| SRAM 뱅크 | 16개 고정 배치, 4 × 4, 가운데 폭 60 µm 십자 통로, 그 밖의 틈 20 µm | `floorplan.txt`, `layout.png` |
| 표준 셀 | 5,323개, 13,070.6 µm² (합성 11,081 µm²에 클럭 버퍼 · DRV · hold 버퍼 · tie 셀이 더해짐) | `floorplan.txt`, `area.rpt` |
| 셋업 (slow 코너) | WNS +0.065 ns, 위반 0. 가장 나쁜 경로는 뱅크 읽기 → 읽기 포트 `rd_data`(출력 지연 1 ns). 레지스터 사이는 +0.164 ns | `summary_setup.rpt`, `timing_setup.rpt` |
| SRAM 경로 | 읽기 → 더하기 → 쓰기 +0.279 ns, 뱅크 주소 +0.162 ns | `timing_sram_wd.rpt`, `timing_sram_addr.rpt` |
| hold (fast 코너) | WNS +0.031 ns, 위반 0 | `summary_hold.rpt`, `timing_hold.rpt` |
| 전기 규칙 | max transition · max capacitance · max fanout 위반 0 | `summary_setup.rpt` |
| 클럭 트리 | 버퍼 15, clock gating 셀 23, 싱크 610. CTS 직후 삽입 지연 0.336 ~ 0.439 ns, skew 0.103 ns | `clock_tree.rpt`, Innovus 로그 |
| DRC · 연결 | 위반 0, 연결 문제 0 | `drc.rpt`, `connectivity.rpt` |
| 전력, vectorless | 1.412 mW | `power.rpt` |
| 전력, 활동 중 | 0.594 mW (이벤트당 3.02 pJ). 배선 뒤 넷리스트를 게이트 레벨로 시뮬레이션한 VCD, 활동도 100 % 주석 | `power_gls.rpt`, `power_gls_hier.rpt` |

활동 중 전력 0.594 mW의 몫은 누적 블록 `u_acc` 0.284 mW, 좌표 변환 `u_xf` 0.228 mW, 디코더 `u_dec` 0.044 mW다. 클럭망은
0.149 mW(25 %)다. 합성의 0.432 mW보다 큰 것은 대부분 배선 용량에서 온다. 스위칭 전력이 합성 0.116 mW에서 0.303 mW로 늘었고,
그중 `u_acc`가 0.200 mW다. 다이 전체에 흩어진 뱅크 16개로 주소 · 쓰기 데이터 · 읽기 데이터를 보내는 배선이다.

배치 · 배선 뒤 `power_vcd.rpt`(RTL VCD로 낸 것)는 활동도가 16 %만 붙어 쓰지 않는다. 넷 이름이 RTL과 넷리스트에서 달라서다.
`power_gls.rpt`의 머리에는 저장한 데이터베이스에 남아 있던 RTL VCD와 게이트 레벨 VCD가 함께 적혀 있다. 나중에 읽은
게이트 레벨 VCD가 5,928개 넷 모두를 덮었다(coverage 5928/5928).

## 등가 확인과 게이트 레벨 시뮬레이션

| 넷리스트 | LEC (RTL 대비) | 게이트 레벨 시뮬레이션 (장면 1 프레임 500 ~ 539, zero delay) |
|---|---|---|
| 합성 `wl_top/` | 667점(출력 57, 플립플롭 594, SRAM 뱅크 16) 모두 등가, PASS | 123,904칸 모두 골든과 같음, 2,888,258클럭(RTL xsim과 같음) |
| 배치 · 배선 뒤 `wl_top_pnr/` | 667점 모두 등가, PASS | 같음 |

시험 벡터는 막대 중심 골든 묶음의 `vectors/`를 서버 `~/worldlock/vec500/`에 다시 올린 것이다(사용자 허용 2026-10-28).
