# 2026-10-29 장면 2 골든 모델 (지금 칩 그대로)

장면 2(`2026-09-22-16-42-32`)를 설계 대상 칩 `hardware/`의 `wl_top`에 그대로 넣기 위한 설정값과 골든 모델 묶음이다.
칩은 고정 회전 중심 (cx, cy)와 프레임별 위상만 받으므로, 장면 2에 맞는 중심과 위쪽 보정만 고르면 된다. 해석은
[설계 근거 10.1절](../../../docs/reports/01_설계_근거.md#101-장면-2-회전판)에 있다. 개발 서버에서
2026-10-29에 만들었다.

    DVS=software/datasets/2026-09-22-16-42-32-DELTA.dvs
    E=software/golden_data/2026-10-29_scene2_golden
    python3 software/golden_develop/scene2_golden.py $DVS software/golden_data/2026-10-28_scene2 $E   # 약 5분
    python3 software/golden_develop/golden.py $DVS $E 600 640                                     # 약 15초

회전각은 [2026-10-28_scene2](../2026-10-28_scene2/2026-10-28_scene2.md)의 `rotation.csv`(추적 중심 (478, 385))를 그대로 쓴다.
`scene2_golden.py`는 장면 1 골든 모델과 같은 방법으로 막대 4개가 가장 날카로워지는 중심을 고르고, 배경 기준 중심과 같은 막대
지표로 견준다.

| 회전 중심 | 반치폭 A / B / C / D (px) | 합 | 포화 칸 |
|---|---|---|---|
| 배경 기준 (484, 367) | 12 / 48 / 16 / 28 | 104 | 167 |
| 막대 기준 (460, 439) (고름) | 8 / 24 / 16 / 8 | 56 | 0 |

위쪽 보정 α는 0이다. 전체 지도의 막대 기울기 평균이 -0.20°로, 처음 구간에서 막대가 서 있는 범위(0.3°) 안이라 보정하지 않았다.

| 파일 | 만든 도구 | 내용 |
|---|---|---|
| [verify.txt](verify.txt) | `scene2_golden.py` | 막대 선, 막대마다 따로 고른 중심, 두 중심의 막대 기울기 · 45° 구간 어긋남 · 반치폭 · 포화 칸 |
| `rotation.csv` | `scene2_golden.py` | 회전각 (frame, t_us, theta_deg). `golden.py`가 묶음 안에서 읽는 형식이라 둔다. 2026-10-28_scene2의 것과 값이 같다 |
| [center.txt](center.txt) | `scene2_golden.py` | cx = 460, cy = 439, alpha_deg = 0 |
| `phase.csv` | `golden.py` | 프레임마다 ψ, 위상, cos, sin |
| `world.npy` | `golden.py` | 장면 2 전체 2,056프레임을 누적한 월드 메모리(352 × 352, 13비트) |
| `vectors/` | `golden.py` | 프레임 600 ~ 639의 테스트벤치 벡터. 수는 [params.txt](vectors/params.txt) |

확인한 것

- 장면 2 전체 75,329,889개 이벤트 가운데 범위 밖으로 버린 것이 없고, 칸 최댓값 7,811로 포화한 칸이 없다.
- 막대마다 따로 가장 날카로운 중심은 A (496, 379), B (448, 463), C (484, 463), D (364, 283)이다(D는 탐색 격자 끝).
- `hardware/` RTL이 이 묶음과 칸 단위로 같다([근거](../../../hardware/sim/results/2026-10-29_scene2/evidence.md)).
