# 2026-10-29 yaw 정수 골든 모델

9/15 녹화(수평 회전, yaw)를 원통면 월드 메모리에 쌓는 정수 골든 모델의 결과와 RTL 테스트벤치 벡터다. 해석은
[설계 근거 10.2절](../../../docs/reports/01_설계_근거.md#102-915-녹화-수평-회전)에 있다.

    python3 software/golden_develop/yaw_golden.py software/datasets/2026-09-15-14-27-41-DELTA.dvs software/golden_data/2026-10-28_yaw software/golden_data/2026-10-29_yaw_golden

렌즈 모형(`lens.txt`의 `f_px` · `lam`)과 프레임별 yaw(`yaw.csv`)는
[2026-10-28_yaw](../2026-10-28_yaw/2026-10-28_yaw.md)의 것을 읽는다. 칸 크기(4 px)와 칸 비트(13비트 포화)는 roll 골든
모델 `golden.py`의 `CELL_SHIFT` · `CELL_BITS`를 그대로 쓰고, `golden.py`는 고치지 않았다. 정수 사양은 `yaw_golden.py` 머리
주석에 있다.

| 파일 | 내용 |
|---|---|
| [yaw_golden.txt](yaw_golden.txt) | 열 표의 근사 · 양자화 오차와 비트 수, 8행 묶음 차분 계산 확인, 녹화 전체 누적(메모리 밖 · 칸 최댓값 · 포화), 실수 모델과 칸이 달라진 이벤트 비율, 영상 가운데 · 가장자리 지도의 상관, 벡터 재현 확인과 `params.txt` |
| [yaw_world.npy](yaw_world.npy) · [yaw_world.png](yaw_world.png) | 녹화 전체를 정수 모델로 쌓은 월드 메모리(1024 × 192칸, uint16) |
| [vectors/](vectors/params.txt) | 프레임 1000 ~ 1039의 `words.hex`(원시 워드), `phase.hex`(프레임별 가로 이동 PHI), `lut.hex`(열마다 A0 · A2 · C1 · C3), `expected.hex`(누적 메모리), `params.txt`(사양 상수와 수) |

스크립트가 실행할 때마다 확인하는 것(어긋나면 assert로 멈춘다)

- 8행 묶음 차분 계산(`group_eval`)이 모든 열(960) · 묶음(90) · 행(8)에서 이벤트마다 직접 계산한 U · V와 비트 단위로 같다.
- `check_vectors`: 벡터 폴더만으로 `words.hex`를 워드 단위로 풀고, 묶음 차분 계산으로 칸을 내 누적한 메모리가
  `expected.hex`와 칸 단위로 같다.

개발 서버(numpy 2.4.6)에서 41 s, 최대 메모리 3.4 GB.
