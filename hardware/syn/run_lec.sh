#!/bin/bash
# 대회 서버에서 Conformal LEC 로 RTL 과 Genus 넷리스트가 같은 논리인지 확인한다.
#
#   cd ~/worldlock && hardware/syn/run_lec.sh <top> [이름]
#
# run_genus.sh 가 쓴 hardware/syn/out/<top>[_<이름>]/netlist.v 를 RTL(hardware/src, wl_sram.v 제외)과 견준다.
# 이름을 pnr 로 주면 run_innovus.sh 가 쓴 배치 · 배선 뒤 넷리스트를 견준다.
# 월드 메모리 뱅크 wl_sram 은 양쪽 모두 hardware/syn/wl_sram.lib 의 블랙박스 셀이다. 결과(리포트 포함)는 같은
# 폴더의 lec.log 에 쓴다.
set -euo pipefail

TOP=${1:?top 모듈 이름을 주십시오}
OUT=hardware/syn/out/$TOP${2:+_$2}
LIB=$(grep -m1 '/slow_vdd1v0_basicCells\.lib$' "hardware/syn/out/$TOP/libs.txt")
[ -f "$OUT/netlist.v" ] || { echo "$OUT/netlist.v 가 없습니다. run_genus.sh 를 먼저 돌리십시오." >&2; exit 1; }

# -gated_clock: Genus 가 넣은 clock gating 셀을 RTL 의 enable 레지스터와 같은 것으로 본다.
# analyze datapath / analyze abort: 곱셈기 출력은 그냥 비교하면 판정이 나지 않고 abort 로 남으므로,
# 산술 블록을 묶어 비교하고(analyze datapath -merge) 남은 abort 를 다시 깊게 비교한다(analyze abort -compare).
RTL=$(ls hardware/src/*.v | grep -v '/wl_sram\.v$' | tr '\n' ' ')
cat > "$OUT/lec.do" <<EOF
set dofile abort exit
set log file $OUT/lec.log -replace
read library -both -liberty $LIB hardware/syn/wl_sram.lib
read design -golden -verilog -define WL_SRAM_LIBCELL $RTL -root $TOP
read design -revised -verilog $OUT/netlist.v -root $TOP
set flatten model -gated_clock
set system mode lec
analyze datapath -merge
add compared points -all
compare
analyze abort -compare
report verification
report unmapped points -summary
report compare data -class nonequivalent -class abort
exit -force
EOF

csh -f -c "source $HOME/control_digi.cshrc; lec -XL -nogui -dofile $OUT/lec.do"
echo "리포트: $OUT/lec.log"
