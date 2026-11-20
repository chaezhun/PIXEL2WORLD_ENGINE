#!/bin/bash
# 대회 서버에서 Genus 로 RTL 을 합성하고 리포트를 남긴다.
#
#   cd ~/worldlock && hardware/syn/run_genus.sh <top> [제약 .sdc] [이름]
#
# 제약을 주지 않거나 - 로 주면 hardware/constraints/<top>.sdc 를 쓴다. 리포트는
# hardware/syn/out/<top>/ (이름을 주면 <top>_<이름>/) 에 쓴다. CG=0 을 앞에 붙이면 clock gating 을
# 넣지 않는다(기본 1). VCD=<파일> 을 앞에 붙이면 그 활동도로 전력을 한 번 더
# 낸다(설계 범위는 VCD_SCOPE, 기본 TOP/tb_worldlock/dut). 서버의 로그인 셸이 csh 라 이 파일은 bash 로 돌리고,
# 툴 환경(control_digi.cshrc)만 csh 안에서 읽는다.
set -euo pipefail

TOP=${1:?top 모듈 이름을 주십시오}
SDC=${2:--}
[ "$SDC" = - ] && SDC=hardware/constraints/$TOP.sdc
OUT=hardware/syn/out/$TOP${3:+_$3}
CG=${CG:-1}
LIBDIR=$HOME/worldlock/lib

# 표준 셀 라이브러리는 홈의 압축본(gsclib045_all_v4.7.tgz)을 $LIBDIR 에 풀어 쓴다. 푼 뒤 .unpacked 표지를
# 남겨 다음 실행부터는 다시 풀지 않는다.
if [ ! -e "$LIBDIR/.unpacked" ]; then
    mkdir -p "$LIBDIR"
    tar xzf "$HOME/gsclib045_all_v4.7.tgz" -C "$LIBDIR"
    touch "$LIBDIR/.unpacked"
fi

mkdir -p "$OUT"
# 들어 있는 .lib 전부를 적어 두어 다른 코너를 고를 때 참고한다.
find "$LIBDIR" -name '*.lib' | sort > "$OUT/libs.txt"

# 합성은 가장 느린 코너 slow_vdd1v0(PVT_0P9V_125C, 1.0 V 공급의 -10 %)로 한다.
# 이 코너에서 타이밍이 맞으면 나머지 코너에서도 맞는다.
LIB=$(grep -m1 '/slow_vdd1v0_basicCells\.lib$' "$OUT/libs.txt" || true)
if [ -z "$LIB" ]; then
    echo "slow_vdd1v0_basicCells.lib 를 찾지 못했습니다. $OUT/libs.txt 를 보십시오." >&2
    exit 1
fi
# .lib 머리의 공정 · 전압 · 온도 조건을 리포트 옆에 남긴다.
{
    echo "lib: $LIB"
    grep -m6 -E 'nom_(process|voltage|temperature)|default_operating_conditions' "$LIB"
} > "$OUT/corner.txt"

VCD=${VCD:-}
VCD_SCOPE=${VCD_SCOPE:-TOP/tb_worldlock/dut}
export TOP SDC OUT LIB CG VCD VCD_SCOPE
csh -f -c "source $HOME/control_digi.cshrc; genus -batch -files hardware/syn/genus.tcl -log $OUT/genus"

echo "리포트: $OUT"
