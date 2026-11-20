# 2026-10-28 칸 크기와 노이즈

장면 1(`2026-09-22-16-42-44`)에서 월드 칸 크기(1 · 2 · 4 · 8 px)와 노이즈 거르기가 막대 누적과 메모리 크기에
주는 영향을 잰 원자료다. 해석은 [설계 근거 4절](../../../docs/reports/01_설계_근거.md#4-월드-칸-크기와-노이즈-거르기)에 있다.

    python3 software/golden_develop/cell_size_check.py software/datasets/2026-09-22-16-42-44-DELTA.dvs software/golden_data/2026-10-28_golden_model software/golden_data/2026-10-28_cell_noise
    python3 software/golden_develop/noise_check.py software/datasets/2026-09-22-16-42-44-DELTA.dvs software/golden_data/2026-10-28_golden_model software/golden_data/2026-10-28_cell_noise

회전각 · 회전 중심은 [2026-10-28_golden_model](../2026-10-28_golden_model/2026-10-28_golden_model.md)의 것을 그대로 쓴다.
막대 측정은 `software/golden_develop/world_metrics.py`이고, `cell_size_check.py`가 시작할 때 4 px 칸에서
`verify_world.py`와 같은 값을 내는지 확인한다(통과).

| 파일 | 내용 |
|---|---|
| [cell_size.txt](cell_size.txt) | 칸 크기별 메모리 크기 · 칸 최댓값 · 필요한 비트 · 14비트 포화, 막대 기울기 · 위치 · 가장자리 반치폭 · 봉우리/바닥 |
| [cell_size.png](cell_size.png) | 칸 크기별 전체 지도와 막대 C 둘레 160 px 확대 |
| [noise.txt](noise.txt) | 정지 구간의 노이즈 양, 핫픽셀 수, 거르기 네 가지의 버린 이벤트와 막대 수치 |
| [noise.png](noise.png) | 정지 구간 픽셀별 이벤트 프레임 수(센서 좌표), 핫픽셀만 누적한 월드 지도, 막대 B 단면 |

개발 서버에서 돌렸다. `cell_size_check.py` 43 s, `noise_check.py` 114 s, 최대 메모리는 각각 3.3 GB · 3.7 GB.
