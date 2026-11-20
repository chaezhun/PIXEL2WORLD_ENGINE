# 2026-10-29 장면 2를 지금 칩에 넣은 RTL 시험

설계 대상 `hardware/`의 `wl_top`을 고치지 않고, 장면 2 설정값(회전 중심 (460, 439), 처음 회전각, 위쪽 보정 0)을 입력으로 줘서
장면 2를 돌린 결과다. 설정값은 [2026-10-29_scene2_golden](../../../../software/golden_data/2026-10-29_scene2_golden/2026-10-29_scene2_golden.md),
고른 근거는 [설계 근거 10.1절](../../../../docs/reports/01_설계_근거.md#101-장면-2-회전판)에 있다. 개발 서버에서
2026-10-29에 Verilator 5.020과 xsim으로 돌렸다.

    DVS=software/datasets/2026-09-22-16-42-32-DELTA.dvs
    E=software/golden_data/2026-10-29_scene2_golden
    python3 software/golden_develop/golden.py vectors $DVS $E 0 all ~/wl_build/vec_scene2_bar      # 장면 2 전체, 약 250 MB라 저장소 밖
    VFLAGS=-GMAXW=33554432 hardware/sim/run_sim.sh verilator tb_worldlock ~/wl_build/vec_scene2_bar +cx=460 +cy=439
    XFLAGS="-generic_top MAXW=33554432" hardware/sim/run_sim.sh xsim tb_worldlock ~/wl_build/vec_scene2_bar +cx=460 +cy=439
    hardware/sim/run_sim.sh verilator tb_worldlock $E/vectors +cx=460 +cy=439                     # 프레임 600 ~ 639

| 파일 | 만든 도구 | 내용 |
|---|---|---|
| [result.txt](result.txt) | `hardware/sim/run_sim.sh` | 장면 2 전체(Verilator · xsim)와 프레임 600 ~ 639의 입력 수 · 클럭 수 · SRAM 접근 · 메모리 합 · 판정 |

확인한 것

- 세 실행 모두 `RESULT PASS`다. 장면 2 전체 2,056프레임, 이벤트 75,329,889개가 골든 모델과 칸 단위로 같고, 범위 밖으로 버린 이벤트가 없다.
- 장면 2 전체가 7,791만 클럭이다. 200 MHz면 0.39 s로, 녹화 길이 1.59 s보다 짧다. 한 프레임에 가장 오래 걸린 것이 81,676클럭(408 µs)이라
  프레임 간격 776 µs 안에 끝난다.
- RTL · 테스트벤치는 고치지 않았다. 회전 중심은 테스트벤치의 `+cx` · `+cy`로 준다.
