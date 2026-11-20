# 2026-10-28 합치기 · 1RW 뱅크 RTL 근거

[설계 근거 8절](../../../../docs/reports/01_설계_근거.md#8-같은-칸-업데이트-합치기와-sram-뱅크) 3.2 · 3.3절의 원자료다. 개발 서버에서
2026-10-28에 Verilator 5.020과 Vivado 2026.1 xsim으로 돌렸다. RTL은 `hardware/src/wl_*.v`(합치기 폭 `CW` = 4, 뱅크
16개가 기본값), 테스트벤치는 `hardware/testbench/tb_worldlock.v`다. 이전 RTL(1R1W 하나, 합치지 않음)의 같은 시험은
`2026-10-28_rtl_check`에 있다.

| 파일 | 만든 도구 | 내용 |
|---|---|---|
| [result.txt](result.txt) | `hardware/sim/regress.sh` | 회귀 시험 전체. 시험마다 클럭 수, 칸 갱신 수(`sram` 줄), 멈춘 클럭, 메모리 비교 결과 |
| [ablation.txt](ablation.txt) | 아래 반복문 | 기본 벡터에서 `CW` 1 ~ 4 × 뱅크 4 · 16 · 64개, 12가지의 클럭 수 · 칸 갱신 수 · 멈춘 클럭 |
| [frame_cycles.txt](frame_cycles.txt) | `hardware/sim/frame_cycles.py` | 장면 1 전체에서 프레임마다 걸린 클럭과, 프레임 간격 안에 끝내는 데 필요한 클럭 주파수 |

`sram` 줄의 칸 갱신 하나는 뱅크 읽기 한 번과 쓰기 한 번이다. 멈춘 클럭은 누적 블록이 `ev_ready`를 내려 좌표 변환이
선 클럭이다. 둘 다 이벤트가 흐르는 동안만 센다.

재현 명령. `ablation.txt`는 `run_sim.sh`와 빌드 폴더가 겹치지 않게 따로 빌드했다. `~/wl_build/ablation/src`에
`hardware/src/wl_*.v`와 `tb_worldlock.v`를 복사하고, 기본 벡터 폴더를 `~/wl_build/ablation/vec`로 링크한 뒤 그
폴더에서 반복문을 돌린다.

    hardware/sim/regress.sh > result.txt
    for bk in 1 2 3; do for cw in 1 2 3 4; do
      verilator --binary --timing -O3 -Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC --top-module tb_worldlock \
        -GBK=$bk -GCW=$cw --Mdir b${bk}c${cw} -o sim src/*.v > /dev/null
      echo "== CW=$cw BK=$bk"; ./b${bk}c${cw}/sim +vec=vec | grep -E '^(cycles|sram|RESULT)'
    done; done > ablation.txt
    VFLAGS="-GMAXW=33554432 -GBITS=10" hardware/sim/run_sim.sh verilator tb_worldlock ~/wl_build/vec_full +trace > trace.txt
    python3 hardware/sim/frame_cycles.py trace.txt software/datasets/2026-09-22-16-42-44-DELTA.dvs > frame_cycles.txt

`result.txt`의 마지막 시험(장면 1 전체, xsim)은 첫 회귀가 세션 종료로 끊겨 같은 명령을 따로 다시 돌려 이어 붙였다.
`frame_cycles.txt`는 칸 10비트로 빌드한 RTL의 trace에서 뽑았다. `result.txt`에서 장면 1 전체의 14비트 · 10비트
클럭 수가 같다. xsim과 Verilator의 클럭 수가 1 다른 것은 이전 묶음과 같은 이유(테스트벤치가 시작 클럭을 읽는 시점)다.
