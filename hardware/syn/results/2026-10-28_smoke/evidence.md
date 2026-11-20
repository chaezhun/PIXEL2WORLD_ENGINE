# 2026-10-28 대회 서버 합성 흐름 점검

대회 서버(`snu.polaris.04`)에서 Genus로 `hardware/src/smoke_top.v`를 합성한 리포트다. 설계 본체가 아니라
라이브러리 · 제약 · 스크립트가 맞물리는지 보는 점검이다.

    hardware/syn/run_genus.sh smoke_top

| 항목 | 값 |
|---|---|
| 툴 | Genus 23.14-s090_1 |
| 라이브러리 | `gsclib045_all_v4.7/gsclib045/timing/slow_vdd1v0_basicCells.lib` |
| 코너 | `PVT_0P9V_125C` (nom_voltage 0.9, nom_temperature 125, nom_process 1). [corner.txt](corner.txt) |
| 목표 주파수 | 100 MHz (주기 10 ns), [smoke_top.sdc](../../../constraints/smoke_top.sdc) |
| 면적 | 셀 34개, 193.230 ([area.rpt](area.rpt)) |
| 타이밍 | 최악 경로 `b_q_reg[0]` → `sum_reg[7]`, 데이터 경로 1732 ps, slack +8142 ps ([timing.rpt](timing.rpt)) |
| 전력 | 9.18 µW, vectorless ([power.rpt](power.rpt)) |

[libs.txt](libs.txt)는 압축을 푼 라이브러리의 `.lib` 목록이다. 표준 셀은 `gsclib045`(기본), `_hvt`, `_lvt`,
`_backbias`가 있고 각각 slow · fast와 1.0 V · 1.2 V 코너가 있다.

면적은 리포트의 라이브러리 단위 그대로다. 이 모듈은 흐름 점검용이라 설계 수치로 인용하지 않는다.
