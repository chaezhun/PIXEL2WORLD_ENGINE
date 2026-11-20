# Innovus(Common UI) 배치 · 배선 스크립트. run_innovus.sh 가 환경변수를 채워 부른다.
#   TOP    : 최상위 모듈 이름
#   NET    : Genus 넷리스트
#   SDC    : 타이밍 제약
#   OUT    : 리포트와 결과를 쓸 폴더
#   LIBDIR : gsclib045 폴더(timing/ lef/ qrc/)
#   VCD    : (있으면) 활동도 전력에 쓸 시뮬레이션 VCD, 범위는 VCD_SCOPE
#
# 흐름은 floorplan(뱅크 16개 고정 배치) → 전원망 → 배치 · 최적화 → CTS → hold 고침 → 배선 → 배선 뒤 최적화
# → 필러 → 리포트다. 셋업은 느린 코너(slow_vdd1v0, PVT_0P9V_125C), hold 는 빠른 코너(fast_vdd1v0,
# PVT_1P1V_0C)로 본다. 배선 RC 는 gsclib045 의 QRC 기술 파일로 뽑는다.

set top $env(TOP)
set out $env(OUT)
set lib $env(LIBDIR)
set sram_lib [file normalize hardware/syn/wl_sram.lib]

set_multi_cpu_usage -local_cpu 8

# PWR=1 이면 배치 · 배선을 다시 하지 않고, 저장해 둔 결과(<top>.db)를 읽어 VCD 전력만 낸다. 배선 뒤 넷리스트를
# 게이트 레벨로 시뮬레이션한 VCD 를 주면 넷 이름이 넷리스트와 같아 활동도가 모두 붙는다(RTL VCD 는 일부만 붙는다).
if {[info exists env(PWR)] && $env(PWR)} {
    read_db $out/$top.db
    read_activity_file -format VCD -scope $env(VCD_SCOPE) $env(VCD)
    report_power -out_file $out/power_gls.rpt
    catch {report_power -hierarchy all -out_file $out/power_gls_hier.rpt}
    exit
}

# ---------------------------------------------------------------- MMMC
set fh [open $out/mmmc.tcl w]
puts $fh "create_library_set -name slow -timing {$lib/timing/slow_vdd1v0_basicCells.lib $sram_lib}"
puts $fh "create_library_set -name fast -timing {$lib/timing/fast_vdd1v0_basicCells.lib [file normalize $out/wl_sram_fast.lib]}"
puts $fh "create_rc_corner -name rc_slow -qrc_tech $lib/qrc/qx/gpdk045.tch -temperature 125"
puts $fh "create_rc_corner -name rc_fast -qrc_tech $lib/qrc/qx/gpdk045.tch -temperature 0"
puts $fh "create_timing_condition -name tc_slow -library_sets slow"
puts $fh "create_timing_condition -name tc_fast -library_sets fast"
puts $fh "create_delay_corner -name dc_slow -timing_condition tc_slow -rc_corner rc_slow"
puts $fh "create_delay_corner -name dc_fast -timing_condition tc_fast -rc_corner rc_fast"
puts $fh "create_constraint_mode -name func -sdc_files [file normalize $env(SDC)]"
puts $fh "create_analysis_view -name view_setup -constraint_mode func -delay_corner dc_slow"
puts $fh "create_analysis_view -name view_hold -constraint_mode func -delay_corner dc_fast"
puts $fh "set_analysis_view -setup view_setup -hold view_hold"
close $fh

set_db init_power_nets VDD
set_db init_ground_nets VSS
read_mmmc $out/mmmc.tcl
read_physical -lefs [list $lib/lef/gsclib045_tech.lef $lib/lef/gsclib045_macro.lef \
                          [file normalize hardware/syn/wl_sram.lef]]
read_netlist $env(NET) -top $top
init_design

set_db design_process_node 45
# 두 줄 높이(CoreSiteDouble) 셀은 저전력 설계용(리텐션 · 격리 · 레벨 시프터)이라 이 설계에 쓰지 않는다.
# 그 줄을 만들지 않으므로 최적화가 고르지 못하게 막는다.
set_db [get_db base_cells -if {.site.name == CoreSiteDouble}] .dont_use true
# Metal10 · Metal11 은 전원망에 남기고 신호는 Metal9 까지 쓴다.
set_db design_top_routing_layer Metal9
set_db design_bottom_routing_layer Metal1

connect_global_net VDD -type pg_pin -pin_base_name VDD -all
connect_global_net VSS -type pg_pin -pin_base_name VSS -all
connect_global_net VDD -type tie_hi -all
connect_global_net VSS -type tie_lo -all

# ---------------------------------------------------------------- 플로어플랜
# 뱅크 16개를 4 x 4 로 놓고, 가운데 열과 행 사이에 로직이 들어갈 십자 통로(폭 CH)를 둔다. 다른 뱅크 사이는
# 배선이 지나갈 틈(GAP)만 둔다. 뱅크의 핀은 모두 오른쪽 아래 모서리(wl_sram.lef)에 있으므로, 왼쪽 위
# 사분면은 그대로(r0), 왼쪽 아래는 위아래로(mx), 오른쪽 위는 좌우로(my), 오른쪽 아래는 둘 다(r180)
# 뒤집어 핀이 모두 십자 통로 쪽을 보게 한다.
set row_h 1.71
set site_w 0.2
set bbox [lindex [get_db base_cell:wl_sram .bbox] 0]
set W [lindex $bbox 2]
set H [lindex $bbox 3]
set GAP  20.0
set CH   60.0
set EDGE 10.0
set MARGIN 20.0
proc snap {v q} { return [expr {ceil($v / $q - 1e-6) * $q}] }
set core_w [snap [expr {2*$EDGE + 4*$W + 2*$GAP + $CH}] $site_w]
set core_h [snap [expr {2*$EDGE + 4*$H + 2*$GAP + $CH}] $row_h]
create_floorplan -site CoreSite -core_size [list $core_w $core_h $MARGIN $MARGIN $MARGIN $MARGIN]

set core [lindex [get_db current_design .core_bbox] 0]
set cx0 [lindex $core 0]
set cy0 [lindex $core 1]
set xs {}
set ys {}
foreach d [list $EDGE [expr {$EDGE + $W + $GAP}] [expr {$EDGE + 2*$W + $GAP + $CH}] [expr {$EDGE + 3*$W + 2*$GAP + $CH}]] {
    lappend xs [expr {$cx0 + [snap $d $site_w]}]
}
foreach d [list $EDGE [expr {$EDGE + $H + $GAP}] [expr {$EDGE + 2*$H + $GAP + $CH}] [expr {$EDGE + 3*$H + 2*$GAP + $CH}]] {
    lappend ys [expr {$cy0 + [snap $d $row_h]}]
}
# 뱅크 번호 i = {wy[1:0], wx[1:0]} 는 배치와 상관이 없다(어느 뱅크든 같은 빈도로 쓰인다). 번호 순서로 채운다.
set banks [get_db insts -if {.base_cell.name == wl_sram}]
if {[llength $banks] != 16} {
    error "wl_sram 인스턴스가 16개가 아니라 [llength $banks]개다"
}
foreach b $banks {
    set n [get_db $b .name]
    if {![regexp {g_bank\[(\d+)\]} $n -> i]} { error "뱅크 번호를 못 읽음: $n" }
    set col [expr {$i % 4}]
    set row [expr {$i / 4}]
    set left  [expr {$col < 2}]
    set lower [expr {$row < 2}]
    set orient [expr {$left ? ($lower ? "mx" : "r0") : ($lower ? "r180" : "my")}]
    set_db $b .location [list [lindex $xs $col] [lindex $ys $row]]
    set_db $b .orient $orient
    set_db $b .place_status fixed
    # 뱅크 핀 바로 옆에 셀이 붙지 않게 2 um 여유를 둔다. 실패해도 흐름은 계속한다.
    if {[catch {create_place_halo -insts $n -halo_deltas {2 2 2 2}} msg]} { puts "halo $n: $msg" }
}
# 입출력 핀은 배치기가 연결된 셀 가까이에 놓게 한다.
set_db place_global_place_io_pins true

# ---------------------------------------------------------------- 전원망
# 코어 둘레에 VDD · VSS 링(Metal11 가로, Metal10 세로), 세로 줄무늬(Metal10)를 80 um 마다, 표준 셀 행의
# Metal1 전원선을 줄무늬와 링에 잇는다. 뱅크는 전원 핀이 없는 자리 모델이라 잇지 않는다.
add_rings -nets {VDD VSS} -type core_rings -follow core \
    -layer {top Metal11 bottom Metal11 left Metal10 right Metal10} \
    -width 3 -spacing 2 -offset 2
add_stripes -nets {VDD VSS} -layer Metal10 -direction vertical -width 2 -spacing 2 \
    -set_to_set_distance 80 -start_from left -start_offset 40
route_special -connect {core_pin} -nets {VDD VSS}

# ---------------------------------------------------------------- 배치 · CTS · 배선
set_db add_tieoffs_cells {TIEHI TIELO}
place_opt_design -report_dir $out/rpt_place
add_tieoffs

set_db cts_buffer_cells   {CLKBUFX2 CLKBUFX3 CLKBUFX4 CLKBUFX6 CLKBUFX8 CLKBUFX12 CLKBUFX16 CLKBUFX20}
set_db cts_inverter_cells {CLKINVX1 CLKINVX2 CLKINVX3 CLKINVX4 CLKINVX6 CLKINVX8 CLKINVX12 CLKINVX16 CLKINVX20}
# Innovus 23 의 place_opt_design 뒤에는 CTS 를 clock_opt_design 으로 한다(ccopt_design 은 받지 않는다).
create_clock_tree_spec
clock_opt_design -report_dir $out/rpt_cts
opt_design -post_cts -hold -report_dir $out/rpt_cts_hold

# 배선 뒤 최적화(SI 포함)는 OCV 분석 모드를 요구한다. 같은 클럭 경로를 공유하는 몫(CPPR)은 빼고 본다.
set_db timing_analysis_type ocv
set_db timing_analysis_cppr both
route_design
set_db extract_rc_engine post_route
opt_design -post_route -setup -hold -report_dir $out/rpt_route

set_db add_fillers_cells {FILL64 FILL32 FILL16 FILL8 FILL4 FILL2 FILL1}
add_fillers

# ---------------------------------------------------------------- 리포트
time_design -post_route -report_dir $out/rpt_final -report_prefix setup
time_design -post_route -hold -report_dir $out/rpt_final -report_prefix hold
report_timing -max_paths 10 > $out/timing_setup.rpt
report_timing -early -max_paths 10 > $out/timing_hold.rpt
# 뱅크 출력에서 나가는 경로와 뱅크 쓰기 데이터로 들어가는 경로(합성에서 본 것과 같은 경로)
catch {report_timing -from [get_pins -hierarchical *u_mem/rdata*] > $out/timing_sram_out.rpt}
catch {report_timing -to [get_pins -hierarchical *u_mem/wd*] > $out/timing_sram_wd.rpt}
catch {report_timing -to [get_pins -hierarchical *u_mem/addr*] > $out/timing_sram_addr.rpt}
report_area > $out/area.rpt
catch {report_area -min_count 1 > $out/area_hier.rpt}
catch {report_clock_trees -summary -out_file $out/clock_tree.rpt}
catch {report_ccopt_skew_groups -summary -file $out/skew_groups.rpt}
check_drc -out_file $out/drc.rpt
check_connectivity -out_file $out/connectivity.rpt
catch {check_place $out/place.rpt}
report_power -out_file $out/power.rpt

if {[info exists env(VCD)] && $env(VCD) ne ""} {
    set scope $env(VCD_SCOPE)
    if {[catch {read_activity_file -format VCD -scope $scope $env(VCD)} msg]} {
        puts "read_activity_file -scope $scope: $msg"
    } else {
        report_power -out_file $out/power_vcd.rpt
        catch {report_power -hierarchy all -out_file $out/power_vcd_hier.rpt}
    }
}

# 다이 · 코어 크기, 표준 셀 면적과 이용률, 뱅크 위치를 한 파일로 남긴다.
set fh [open $out/floorplan.txt w]
puts $fh "die  [get_db current_design .bbox]"
puts $fh "core [get_db current_design .core_bbox]"
set std [get_db insts -if {.base_cell.class == core && !.is_physical}]
set a 0.0
foreach c $std { set a [expr {$a + [get_db $c .area]}] }
puts $fh "std_cells [llength $std] area_um2 [format %.1f $a]"
foreach b $banks {
    puts $fh "bank [get_db $b .name] [get_db $b .location] [get_db $b .orient]"
}
close $fh

write_netlist $out/netlist.v
catch {write_sdf -view view_setup $out/$top.sdf}
write_def -routing $out/$top.def
write_db $out/$top.db

exit
