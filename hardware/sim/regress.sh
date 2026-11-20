#!/usr/bin/env bash
# RTL 회귀 시험. CORDIC 전수 시험, 아래 벡터 묶음과 바꾼 파라미터(셀 폭, 병합 폭, 워드 사이 빈
# 사이클)로 돌리는 wl_top, 이벤트 단위 기준 설계 wl_base_top("reference design" 실행)을 돌리고
# 실행마다 블록 하나를 출력한다.
#
#   hardware/sim/regress.sh | tee hardware/sim/results/<YYYY-MM-DD_topic>/result.txt
#
# 벡터 묶음
#   default    : software/golden_data/2026-10-28_golden_bar_center/vectors (장면 1, 프레임 500-539)
#   full       : 파일 첫 워드부터 장면 1 전체, ~250 MB. 없으면 golden.py 가
#                ~/wl_build/vec_full 에 만든다
#   offcentre  : 회전 중심을 (40, 60) 으로 옮겨 이벤트가 월드 메모리 밖으로 떨어지게 한
#                프레임 500-539, ~/wl_build/vec_offcentre
#   scene2     : 파일 첫 워드부터 장면 2 의 처음 40 프레임, 모든 프레임의 위상은 0.
#                장면 2 는 열 주소가 나오기 전의 이벤트로 시작한다.
#   base       : 기준 설계 wl_base_top(tb_base)용으로 프레임 500-539 의 이벤트를 프레임 위상과
#                함께 한 줄에 하나씩 쓴 것, ~/wl_build/vec_base. expected.hex 는 default
#                묶음의 것을 가리키는 링크다
#   empty      : 프레임 500, 510, 520, 521, 539 에서 이벤트 · 열 워드를 모두 지운 장면 1 사본의
#                프레임 500-539, ~/wl_build/vec_empty
# 테스트벤치가 "RESULT PASS" 를 출력하면 그 실행은 통과다. 실행들이 빌드 디렉터리를 같이
# 쓰므로, 이것이 도는 동안 다른 run_sim.sh 를 시작하지 않는다.
set -uo pipefail

here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$here/../.." && pwd)
gd=$root/software/golden_develop
bundle=$root/software/golden_data/2026-10-28_golden_bar_center
dvs=$root/software/datasets/2026-09-22-16-42-44-DELTA.dvs
full=$HOME/wl_build/vec_full
off=$HOME/wl_build/vec_offcentre
s2=$HOME/wl_build/vec_scene2_start
base=$HOME/wl_build/vec_base
empty=$HOME/wl_build/vec_empty
big=33554432

[ -f "$full/words.hex" ] || python3 "$gd/golden.py" vectors "$dvs" "$bundle" 0 all "$full" > /dev/null
[ -f "$off/words.hex" ]  || python3 "$gd/golden.py" vectors "$dvs" "$bundle" 500 540 "$off" 40 60 > /dev/null
[ -f "$s2/words.hex" ]   || python3 "$gd/golden.py" vectors "${dvs/16-42-44/16-42-32}" - 0 40 "$s2" > /dev/null
[ -f "$base/events.hex" ] || { python3 "$gd/golden.py" events "$dvs" "$bundle" 500 540 "$base" > /dev/null &&
                               ln -sfn "$bundle/vectors/expected.hex" "$base/expected.hex"; }
[ -f "$empty/words.hex" ] || { mkdir -p "$empty" &&
    python3 "$gd/golden.py" empty "$dvs" "$empty/empty.dvs" 500 510 520 521 539 > /dev/null &&
    python3 "$gd/golden.py" vectors "$empty/empty.dvs" "$bundle" 500 540 "$empty" > /dev/null; }

run() {   # run <label> <VAR=value 하나 또는 ""> <run_sim.sh 인자...>
    local label=$1 envs=$2; shift 2
    echo "== $label"
    if [ -n "$envs" ]; then env "$envs" "$here/run_sim.sh" "$@"; else "$here/run_sim.sh" "$@"; fi 2>&1 \
        | grep -E '^(loaded|cycles|sram|memory|phases|MISMATCH|TIMEOUT|RESULT)'
}

run "cordic, all phases, verilator"            ""                                  verilator tb_cordic
run "cordic, all phases, xsim"                 ""                                  xsim tb_cordic
run "frames 500-539, verilator"                ""                                  verilator tb_worldlock
run "frames 500-539, xsim"                     ""                                  xsim tb_worldlock
run "frames 500-539, 3 idle cycles per word"   ""                                  verilator tb_worldlock "$bundle/vectors" +gap=3
run "frames 500-539, 8-bit cells"              "VFLAGS=-GBITS=8"                   verilator tb_worldlock
run "frames 500-539, no merging"              "VFLAGS=-GCW=1"                     verilator tb_worldlock
run "frames 500-539, merge width 2"            "VFLAGS=-GCW=2"                     verilator tb_worldlock
run "frames 500-539, centre (40, 60)"          ""                                  verilator tb_worldlock "$off" +cx=40 +cy=60
run "scene 2 frames 0-39 from file start"      ""                                  verilator tb_worldlock "$s2" +cx=480 +cy=360
run "frames 500-539, five frames emptied"      ""                                  verilator tb_worldlock "$empty"
run "reference design, frames 500-539, verilator" ""                         verilator tb_base "$base"
run "reference design, frames 500-539, xsim"    ""                                  xsim tb_base "$base"
run "whole scene 1, verilator"                 "VFLAGS=-GMAXW=$big"                verilator tb_worldlock "$full"
run "whole scene 1, 10-bit cells"              "VFLAGS=-GMAXW=$big -GBITS=10"      verilator tb_worldlock "$full"
run "whole scene 1, xsim"                      "XFLAGS=-generic_top MAXW=$big"     xsim tb_worldlock "$full"
