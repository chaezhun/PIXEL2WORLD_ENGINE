# 2026-10-28 쓰는 영역만 저장

roll이 쓸고 지나가는 원 안의 칸만 월드 메모리에 둘 때 칸 수 · macro 조합 · SRAM 면적이 얼마나 주는지 센
원자료다. 해석은 [설계 근거 6절](../../../docs/reports/01_설계_근거.md#6-저장-영역)에 있다.

    python3 software/golden_develop/region_check.py software/golden_data/2026-10-28_golden_model software/golden_data/2026-10-28_write_region hardware/src/wl_region.v

회전 중심과 칸에 이벤트가 들었는지는 [2026-10-28_golden_model](../2026-10-28_golden_model/2026-10-28_golden_model.md)의
`center.txt` · `world.npy`에서 읽는다. macro 면적은 main이 OpenRAM으로 만든 1RW 14비트 macro
([summary.csv](../../../hardware/sram/results/2026-10-28_openram/summary.csv), FreePDK45)다. 다른 공정의 값이라 SRAM
assumptions로만 쓴다.

| 파일 | 내용 |
|---|---|
| [region.txt](region.txt) | [1] 반지름별로 원에 걸치는 칸, [2] 주소 매핑 방식(행 표 · 띠 · macro 타일)별 칸 수와 표 크기, [3] 칸 비트별 메모리 비트, [4] 지금 RTL의 뱅크 16개에서 뱅크당 word 수 · macro 조합 · SRAM 면적과 표 크기 |

세 번째 인자를 주면 [4]의 가장 큰 반지름(696 px) 표로 `hardware/src/wl_region.v`(원 영역 주소와 비교용 정사각형
주소)를 쓴다. 그 합성 리포트는 [`hardware/syn/results/2026-10-28_region/`](../../../hardware/syn/results/2026-10-28_region/evidence.md)에 있다.

개발 서버에서 몇 초 걸린다.
