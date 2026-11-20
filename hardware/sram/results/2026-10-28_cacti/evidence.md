# 2026-10-28 CACTI SRAM 에너지 · leakage 근거

원자료다. 월드 메모리 macro의 접근 에너지와 leakage를 CACTI 7로 낸 값이다. OpenRAM은 면적
([2026-10-28_openram](../2026-10-28_openram/evidence.md))을 주지만, ngspice 특성화가 4,096 word에서 몇 시간 걸리고
13비트 1,024 word에서는 "Sense amp enable timing error"로 멈춰 에너지를 아직 주지 못했다. CACTI는 같은 45 nm 세대의
다른 모델이라 에너지 · leakage의 교차 확인과 대체값으로 쓴다. 모두 SRAM assumptions이고 합성 리포트와 섞지 않는다.

| 파일 | 내용 |
|---|---|
| `ram_<word>_<셀>.cfg`, `ram1r1w_4096_itrs_lstp.cfg` | CACTI 입력. 함께 들어 있는 `cache.cfg`에서 크기 · word 폭 · 포트 · 공정 · 셀 종류만 바꿨다 |
| `out_*.txt` | CACTI 출력 그대로 |
| [summary.csv](summary.csv) | macro마다 접근 시간, 읽기 · 쓰기 에너지(pJ/접근), leakage(µW), 면적(µm²) |

CACTI는 HP의 공개 저장소(`github.com/HewlettPackard/cacti`, 커밋 `1ffd8df`)를 개발 서버의 `~/wl_build/ext/cacti`에 받아
`make`로 빌드했다. 재현은 그 폴더에서 `./cacti -infile <cfg>`다.

## 설정

- 공정 0.045 µm, `ram`(태그 없음), 뱅크 1개, 360 K. CACTI는 word 폭을 바이트 단위로만 받아서 13비트 대신 16비트
  word로 잡았다(4,096 × 16비트, 1,024 × 16비트). 그만큼 에너지가 위로 잡힌다.
- 셀 종류는 `itrs-lstp`(저누설)와 `itrs-hp`(고성능) 둘을 돌렸다. hp는 4,096 word 하나의 leakage가 11.1 mW라 16개 뱅크면
  수백 mW가 된다. lstp도 접근 시간이 1.07 ns로 합성에서 가정한 읽기 시간 2.0 ns([wl_sram.lib](../../../syn/wl_sram.lib)) 안에
  들어서, 설계 값은 lstp로 정했다.
- 1R1W(읽기 포트 1 + 쓰기 포트 1) 4,096 word는 바꾸기 전 구조와 견주려고 돌렸다.

## 결과 (lstp)

| macro | 접근 시간 | 읽기 | 쓰기 | leakage | CACTI 면적 |
|---|---:|---:|---:|---:|---:|
| 1RW 4,096 × 16 | 1.07 ns | 5.41 pJ | 3.66 pJ | 2.48 µW | 28,558 µm² |
| 1RW 1,024 × 16 | 0.83 ns | 2.19 pJ | 2.04 pJ | 0.67 µW | 7,852 µm² |
| 1R1W 4,096 × 16 | 1.18 ns | 6.55 pJ | 4.36 pJ | 3.08 µW | 56,697 µm² |

CACTI 면적은 OpenRAM layout보다 작다(4,096 word 0.029 대 0.081 mm²). 면적은 OpenRAM 값을 쓴다. 다만 1R1W와 1RW의
비는 CACTI 1.99배, OpenRAM 2.05배로 비슷하다.

## 이벤트당 SRAM 에너지 (장면 1)

칸 갱신 하나는 뱅크 읽기 한 번과 쓰기 한 번이다. 장면 1 전체에서 칸 갱신은 이벤트당 0.727번이다
([merge_bank 근거](../../../sim/results/2026-10-28_merge_bank/evidence.md)). 뱅크 안 주소가 4,096보다 작은 칸(4,096 word
macro)에 장면 1 이벤트의 81.6 %가 떨어진다(골든 지도 `world.npy`와 저장 영역 표로 셈).

| | 지금 설계 (1RW 뱅크 16개, 합치기) | 바꾸기 전 (1R1W, 합치지 않음) |
|---|---:|---:|
| 칸 갱신 하나 | 0.816 × 9.07 + 0.184 × 4.24 = 8.17 pJ | 6.55 + 4.36 = 10.91 pJ |
| 칸 갱신 / 이벤트 | 0.727 | 1 |
| 동적 에너지 / 이벤트 | 5.94 pJ | 10.91 pJ |
| leakage 합 | 16 × (2.48 + 2 × 0.67) = 61.3 µW | 31 × 3.08 = 95.5 µW |

바꾸기 전 구조의 macro 수 31은 1R1W 4,096 word로 123,904칸을 채운 수다([설계 근거 8절](../../../../docs/reports/01_설계_근거.md#8-같은-칸-업데이트-합치기와-sram-뱅크)).
