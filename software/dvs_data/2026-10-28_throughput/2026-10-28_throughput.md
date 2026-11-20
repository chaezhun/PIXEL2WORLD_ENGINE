# 2026-10-28 실시간 처리율 요구량 근거

[설계 근거 7절](../../../docs/reports/01_설계_근거.md#7-좌표-변환-구조와-처리율) 2 · 3절의 원자료다. 모두 개발 서버에서
2026-10-28에 돌렸다.

| 파일 | 만든 도구 | 내용 |
|---|---|---|
| [throughput.txt](throughput.txt) | `software/dvs_develop/throughput.py` | 녹화 세 개의 워드 · 이벤트 · 열 주소 · 8행 묶음 수, 녹화 평균과 1 · 4 · 16 · 64프레임 창의 초당 최댓값, 이벤트 워드당 이벤트 수 분포 |
| [hazard.txt](hazard.txt) | `software/golden_develop/rmw_hazard.py` | 장면 1에서 k번째 앞 이벤트와 같은 월드 칸에 떨어지는 비율(k = 1 ~ 6), 같은 칸이 이어지는 묶음 수 |

재현 명령

    python3 software/dvs_develop/throughput.py software/datasets/*.dvs > throughput.txt
    python3 software/golden_develop/rmw_hazard.py software/datasets/2026-09-22-16-42-44-DELTA.dvs \
        software/golden_data/2026-10-28_golden_model > hazard.txt
