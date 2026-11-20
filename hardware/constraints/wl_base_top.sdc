# wl_base_top(기준 설계, 이벤트마다 입력) 합성 제약. wl_top과 견주려고 클럭 · 입출력 지연을 wl_top.sdc와 같게 둔다.
create_clock -name clk -period 5.0 [get_ports clk]

set_input_delay  1.0 -clock clk [get_ports {in_x in_y in_phase in_valid clr rd_en rd_x rd_y}]
set_output_delay 1.0 -clock clk [all_outputs]

# 회전 중심 cx, cy는 도는 동안 바꾸지 않는 설정값이다. 리셋은 비동기다.
set_false_path -from [get_ports {cx cy rst_n}]
