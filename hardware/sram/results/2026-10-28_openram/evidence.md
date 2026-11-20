# 2026-10-28 OpenRAM macro 면적 근거

[설계 근거 8절](../../../../docs/reports/01_설계_근거.md#8-같은-칸-업데이트-합치기와-sram-뱅크) 2절의 원자료다.
개발 서버에서 OpenRAM 1.2.48로 FreePDK45, TT · 1.0 V · 25 °C 조건의 14비트 macro를 만들었다. 대회 공정
GPDK045와 다른 공정이므로 합성 리포트 수치와 섞지 않고 SRAM assumptions로만 쓴다.

| 파일 | 만든 도구 | 내용 |
|---|---|---|
| [summary.csv](summary.csv) | OpenRAM 출력에서 뽑음 | macro마다 포트, 비트 폭, word 수, layout 면적(µm²), 비트당 면적, 분석 모델의 최소 클럭 주기 |
| `datasheet_<macro>.html` | OpenRAM | 생성된 datasheet 그대로 |

재현 명령. 저장소 경로에 공백이 있어 빌드 폴더에서 돌린다. 하나에 약 10분 걸린다.

    cd ~/wl_build/openram
    ~/openram_venv/bin/python <저장소>/hardware/sram/gen_openram.py 1rw 14 4096
    ~/openram_venv/bin/python <저장소>/hardware/sram/gen_openram.py 1rw1r 14 4096    # 1,024 word도 같은 식

면적은 생성된 layout의 외곽 면적이다. 최소 클럭 주기와 datasheet의 전력 · leakage는 ngspice를 돌리지 않은 OpenRAM
분석 모델 값이라 읽기 · 쓰기 전력이 같은 숫자로 나오는 등 거칠다. 그래서 면적만 쓰고 에너지는 ngspice 특성화
결과를 기다린다.

16,384 word는 1RW · 1R1W 모두 OpenRAM 설정 검사에서 "Number of words exceeds 2048"로 멈췄다. 검사 코드
(`sram_config.py`의 `amend_words_per_row`)가 실제로 막는 것은 행 수 × 행당 word 수가 4,096을 넘는 경우라서,
메시지와 달리 이 판에서 만들 수 있는 가장 큰 macro는 4,096 word다.

## 13비트 macro

칸을 13비트로 줄인 뒤 같은 방법으로 1RW 13비트 macro를 만들었다. `_w8`은 한 행에 둘 word 수를 8로 정해 만든
것이다(`gen_openram.py ... wpr=8`).

| macro | 한 행 word 수 | layout 면적 (µm²) |
|---|---:|---:|
| 1RW 13비트 × 1,024 (OpenRAM이 고름) | 16 | 28,492 |
| 1RW 13비트 × 1,024 (`wpr=8`) | 8 | 24,849 |
| 1RW 13비트 × 4,096 (OpenRAM이 고름) | 16 | 81,376 |

OpenRAM이 스스로 고른 13비트 1,024 word는 한 행을 16 word로 잡아 14비트 1,024 word(한 행 8 word, 26,153 µm²)보다
커졌다. 14비트와 같은 모양(한 행 8 word)으로 다시 만들면 24,849 µm²다. 설계에는 이 macro를 쓴다.

지금 설계(뱅크 16개, 뱅크마다 저장 영역 6,132칸을 4,096 word 하나와 1,024 word 둘에)의 SRAM 면적은
16 × (81,376 + 2 × 24,849) = 2,097,184 µm²(2.10 mm²)다.
