# 2026-10-29 칸 내용 비트 (막대 중심 골든에서 다시)

2026-10-28_cell_bits와 같은 측정을, 막대 기준 중심으로 바꾼 지금 골든
모델(막대 중심 회전 추적, [2026-10-28_golden_bar_center](../2026-10-28_golden_bar_center/2026-10-28_golden_bar_center.md))의
회전각으로 다시 한 원자료다. 해석은 [설계 근거 5절](../../../docs/reports/01_설계_근거.md#5-칸-비트)과
[설계 근거 6절](../../../docs/reports/01_설계_근거.md#6-저장-영역)에 있다.

    python3 software/golden_develop/cell_bits_check.py software/datasets/2026-09-22-16-42-44-DELTA.dvs software/golden_data/2026-10-28_golden_bar_center software/golden_data/2026-10-29_cell_bits_bar_center

스크립트는 바꾸지 않았다. 출력의 "14비트(지금)" 줄은 자르지 않은 개수(14비트와 같음)로 만든 기준이다. 지금 RTL은 포화
13비트이고, 표의 "포화 13" 줄이 그것이다. 스크립트가 시작할 때 자기가 센 지도를 13비트로 자른 것이 골든 `world.npy`와
같은지 확인한다(통과).

| 파일 | 내용 |
|---|---|
| [cell_bits.txt](cell_bits.txt) | 방식별 칸 비트 · 메모리 비트, 엄격 판정(시작값 몇 개가 같은지), 14비트와의 최대 차, 기준선 안 여부, 막대 4개의 측정값(대표 시작값), 로그 카운터 추정 오차 |

개발 서버에서 309 s, 최대 메모리 4.2 GB.
