#!/bin/bash
# 대회 서버에서 Genus 넷리스트를 Xcelium 으로 게이트 레벨 시뮬레이션해 골든 기댓값과 견준다.
#
#   cd ~/worldlock && hardware/sim/run_gls.sh <벡터 폴더> [넷리스트] [+plusarg ...]
#
# 벡터 폴더에는 golden.py 가 쓴 words.hex · phase.hex · expected.hex 가 있어야 한다. 넷리스트를 주지 않으면
# run_genus.sh 가 쓴 hardware/syn/out/wl_top/netlist.v 를 쓴다. 테스트벤치는 RTL 시험과 같은 tb_worldlock.v 를
# GLS 정의로 컴파일한다. 월드 메모리 뱅크(wl_sram)는 넷리스트에서도 블랙박스 셀이라 RTL 동작 모델을 붙인다.
# 표준 셀은 slow_vdd1v0 의 Verilog 모델이다. 지연 없이(zero delay) 기능만 본다. 타이밍은 합성 · 배치 · 배선의 STA 로 본다.
# 셋째 인자부터는 테스트벤치 plusarg 로 넘긴다. 배치 · 배선 뒤 넷리스트로 전력용 VCD 를 뽑을 때
# +vcd=<파일> +vcd_from=<n> [+vcd_len=<n>] 을 준다. 첫 워드를 내준 뒤 vcd_from 클럭부터 vcd_len 클럭
# (기본 20000) 동안 설계의 모든 신호를 적는다.
# 결과는 hardware/sim/out/gls/xrun.log 에 쓴다. "RESULT PASS" 가 나와야 한다.
set -euo pipefail

VEC=${1:?벡터 폴더를 주십시오}
NET=${2:-hardware/syn/out/wl_top/netlist.v}
shift $(( $# < 2 ? $# : 2 ))
CELLS=$HOME/worldlock/lib/gsclib045_all_v4.7/gsclib045/verilog/slow_vdd1v0_basicCells.v
OUT=hardware/sim/out/gls
mkdir -p "$OUT"

# -access +r: $dumpvars 가 넷리스트 신호를 읽을 수 있게 한다(없으면 VCD 가 거의 빈다).
csh -f -c "source $HOME/control_digi.cshrc; xrun -64bit -sv -define GLS -delay_mode zero -notimingchecks -access +r \
    -top tb_worldlock -xmlibdirname $OUT/xcelium.d -l $OUT/xrun.log \
    hardware/testbench/tb_worldlock.v $NET hardware/src/wl_sram.v -v $CELLS +vec=$VEC $*" > /dev/null || true
grep -E '^(loaded|cycles|memory|vcd|MISMATCH|TIMEOUT|RESULT)' "$OUT/xrun.log"
