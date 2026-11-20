# 2026-10-28 원 영역 주소의 로직 비용

월드 메모리에 roll이 쓰는 원 영역만 두는 주소 모듈(`wl_region`)과 지금의 정사각형 주소 모듈(`wl_region_sq`)을 같은
조건으로 합성한 리포트다. 해석은
[설계 근거 6절](../../../../docs/reports/01_설계_근거.md#6-저장-영역)에 있다.

    hardware/syn/run_genus.sh wl_region
    hardware/syn/run_genus.sh wl_region_sq

두 모듈은 `hardware/src/wl_region.v` 한 파일에 있고, 원 영역 표는 `software/golden_develop/region_check.py`가 쓴다.
아직 `wl_top`에 들어가지 않은 평가용 블록이다. main이 같은 서버 폴더(`~/worldlock`)에서 다른 합성을 돌릴 수 있어서,
개발 서버에서 `wl_region.v` · 제약 두 개 · `run_genus.sh` · `genus.tcl`만 서버의 `~/worldlock_eval/`에 올려
돌렸다. 표준 셀은 `run_genus.sh`가 쓰는 `~/worldlock/lib/`(이미 풀어 둔 것)를 그대로 읽는다.

| 항목 | 값 |
|---|---|
| 툴 | Genus 23.14-s090_1 |
| 라이브러리 | `gsclib045_all_v4.7/gsclib045/timing/slow_vdd1v0_basicCells.lib` |
| 코너 | `PVT_0P9V_125C` (nom_voltage 0.9, nom_temperature 125, nom_process 1). [corner.txt](wl_region/corner.txt) |
| 목표 주파수 | 125 MHz (주기 8 ns), [wl_region.sdc](../../../constraints/wl_region.sdc) · [wl_region_sq.sdc](../../../constraints/wl_region_sq.sdc) |

| 모듈 | 셀 수 | 면적 | 그중 플립플롭 | 최악 경로 | 데이터 경로 | slack | 전력(vectorless) |
|---|---:|---:|---:|---|---:|---:|---:|
| `wl_region` (원, 반지름 696 px) | 380 | 636.804 | 19개, 103.968 | `wy[2]` → `maddr_reg[11]` | 2871 ps | +5006 ps | 15.03 µW |
| `wl_region_sq` (정사각형, 지금) | 43 | 195.624 | 18개, 98.496 | `wy[2]` → `macro_reg[0]` | 1741 ps | +6121 ps | 8.08 µW |

리포트: [wl_region/area.rpt](wl_region/area.rpt) · [timing.rpt](wl_region/timing.rpt) · [power.rpt](wl_region/power.rpt) ·
[gates.rpt](wl_region/gates.rpt) · [qor.rpt](wl_region/qor.rpt),
[wl_region_sq/area.rpt](wl_region_sq/area.rpt) · [timing.rpt](wl_region_sq/timing.rpt) · [power.rpt](wl_region_sq/power.rpt) ·
[gates.rpt](wl_region_sq/gates.rpt) · [qor.rpt](wl_region_sq/qor.rpt). [libs.txt](wl_region/libs.txt)는 풀어 둔 `.lib` 목록이다.

면적은 리포트의 라이브러리 단위 그대로이고 wireload 없이(배치 전) 잰 값이다. 원 영역 주소가 더하는 로직은 두 모듈의
차 441.180이다. 전력은 입력 활동을 주지 않은 vectorless 값이라 비교용으로만 본다.

합성 전에 개발 서버에서 Verilator lint를 통과시키고, iverilog로 352 × 352칸 전부를 넣어 Python 표와 맞대었다.
반지름 696 px 원에 걸치는 칸은 모두 남고, 뱅크 안 주소가 서로 겹치지 않으며 최대 6,132(< 6,144)이고, 장면 1에서
이벤트가 든 칸은 하나도 버려지지 않는다. `wl_region_sq`는 `wl_accum`의 주소식과 같다.
