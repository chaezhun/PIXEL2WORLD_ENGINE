# wl_region_sq 합성 제약. 원 영역만 저장하는 주소(wl_region)와 정사각형 전체를 저장하는 주소(wl_region_sq)의
# 로직 비용을 같은 조건에서 견주려는 것이라 둘 다 같은 클럭을 쓴다. wl_top 의 처리율 요구량 111.1 MHz보다
# 여유 있게 8 ns(125 MHz)로 둔다.
create_clock -name clk -period 8.0 [get_ports clk]

# 입출력은 같은 클럭에 붙은 외부 레지스터와 주고받는다고 보고 지연 0 으로 둔다.
set_input_delay  0.0 -clock clk [get_ports {in_valid wx wy}]
set_output_delay 0.0 -clock clk [all_outputs]
