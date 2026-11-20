#!/bin/bash
# 대회 서버에서 Innovus 로 합성 넷리스트를 배치 · 배선하고 리포트를 남긴다.
#
#   cd ~/worldlock && hardware/syn/run_innovus.sh <top> [제약 .sdc] [이름]
#
# 넷리스트는 run_genus.sh 가 쓴 hardware/syn/out/<top>/netlist.v 를 읽는다. 제약을 주지 않거나 - 로 주면
# hardware/constraints/<top>.sdc 를 쓴다. 결과는 hardware/syn/out/<top>_pnr/ (이름을 주면 <top>_pnr_<이름>/).
# VCD=<파일> 을 앞에 붙이면 배선 뒤 그 활동도로 전력을 한 번 더 낸다(VCD_SCOPE 는 run_genus.sh 와 같다).
# PWR=1 VCD=<게이트 레벨 VCD> VCD_SCOPE=tb_worldlock/dut 를 붙이면 배치 · 배선은 다시 하지 않고 저장한 결과로
# 전력만 낸다(power_gls.rpt). VCD 는 run_gls.sh 로 배선 뒤 넷리스트를 돌려 뽑는다.
# 월드 메모리 뱅크(wl_sram)는 wl_sram.lef 의 자리만 차지하는 블록이다.
set -euo pipefail

TOP=${1:?top 모듈 이름을 주십시오}
SDC=${2:--}
[ "$SDC" = - ] && SDC=hardware/constraints/$TOP.sdc
OUT=hardware/syn/out/${TOP}_pnr${3:+_$3}
NET=hardware/syn/out/$TOP/netlist.v
LIBDIR=$HOME/worldlock/lib/gsclib045_all_v4.7/gsclib045

if [ ! -e "$NET" ]; then
    echo "$NET 이 없습니다. 먼저 run_genus.sh $TOP 를 돌리십시오." >&2
    exit 1
fi
mkdir -p "$OUT"

# 빠른 코너(PVT_1P1V_0C)의 뱅크 타이밍. 느린 코너 모델에서 조건 이름만 바꾸고, 읽기 출력 지연을 2.0 ns 에서
# 0.5 ns 로 줄인다. hold 는 출력이 빨리 바뀔수록 나쁘므로 작게 잡는다.
sed -e 's/wl_sram_lib/wl_sram_lib_fast/' \
    -e 's/PVT_0P9V_125C/PVT_1P1V_0C/g' \
    -e 's/temperature : 125/temperature : 0/' \
    -e 's/voltage : 0\.9/voltage : 1.1/' \
    -e 's/values ("2\.0")/values ("0.5")/' \
    hardware/syn/wl_sram.lib > "$OUT/wl_sram_fast.lib"

VCD=${VCD:-}
VCD_SCOPE=${VCD_SCOPE:-TOP/tb_worldlock/dut}
PWR=${PWR:-0}
LOG=$OUT/innovus
[ "$PWR" = 1 ] && LOG=$OUT/innovus_pwr      # 배치 · 배선 로그를 덮어쓰지 않는다
export TOP SDC OUT NET LIBDIR VCD VCD_SCOPE PWR
csh -f -c "source $HOME/control_digi.cshrc; innovus -stylus -no_gui -batch -overwrite -files hardware/syn/innovus.tcl -log $LOG"

echo "리포트: $OUT"
