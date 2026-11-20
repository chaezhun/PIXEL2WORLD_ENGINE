#!/usr/bin/env bash
# RTL 시뮬레이션 실행기.
#
#   hardware/sim/run_sim.sh <verilator|xsim> <tb_worldlock|tb_base|tb_cordic> [vector dir] [extra plusargs...]
#
# Verilator 와 Vivado 는 공백이 든 경로를 잘 다루지 못하고 저장소가 그런 경로 아래 있을 수 있어서,
# 소스를 ~/wl_build/<tool>_<tb> 아래로 복사해 빌드하고 벡터 디렉터리는 거기에 "vec" 로
# 링크한다. 기본 벡터 묶음은 골든 모델 묶음(장면 1 의 프레임 500-539)이다. 남는 인자는
# plusarg 로 시뮬레이션에 넘긴다. 예: +gap=3, +dump=mem.hex. 테스트벤치 파라미터는
# VFLAGS(Verilator)나 XFLAGS(xelab)로 정할 수 있다. 예를 들어 녹화 전체는
#   VFLAGS=-GMAXW=33554432   XFLAGS="-generic_top MAXW=33554432"
set -euo pipefail

tool=${1:?tool: verilator or xsim}
tb=${2:?testbench: tb_worldlock or tb_cordic}
here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$here/../.." && pwd)
vec=$(cd "${3:-$root/software/golden_data/2026-10-28_golden_bar_center/vectors}" && pwd)
shift $(( $# < 3 ? $# : 3 ))

build=$HOME/wl_build/${tool}_${tb}
mkdir -p "$build/src"
cp "$root"/hardware/src/wl_*.v "$root/hardware/testbench/$tb.v" "$build/src/"
ln -sfn "$vec" "$build/vec"
cd "$build"

srcs=(src/wl_*.v "src/$tb.v")
case $tool in
  verilator)
    verilator --binary --timing -O3 -Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
      --top-module "$tb" -o sim ${VFLAGS:-} "${srcs[@]}" > build.log 2>&1 \
      || { tail -40 build.log; exit 1; }
    ./obj_dir/sim +vec=vec "$@"
    ;;
  xsim)
    xvlog "${srcs[@]}" > build.log 2>&1 || { tail -40 build.log; exit 1; }
    xelab "$tb" -s sim --timescale 1ns/1ps -O3 ${XFLAGS:-} >> build.log 2>&1 || { tail -40 build.log; exit 1; }
    plus=(-testplusarg vec=vec)
    for a in "$@"; do plus+=(-testplusarg "${a#+}"); done
    xsim sim -R "${plus[@]}" | grep -v '^#\|^ *\*\*\*\*\|^ *$\|Vivado Simulator\|Copyright\|Time resolution\|^INFO\|^source\|^xsim\|exit' || true
    ;;
  *) echo "unknown tool $tool"; exit 1 ;;
esac
