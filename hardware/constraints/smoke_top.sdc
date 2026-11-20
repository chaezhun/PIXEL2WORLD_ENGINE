# smoke_top 합성 제약. 흐름 점검용이라 클럭 주기는 여유 있게 10 ns(100 MHz)로 둔다.
create_clock -name clk -period 10.0 [get_ports clk]

# 입출력은 같은 클럭에 붙은 외부 레지스터와 주고받는다고 보고 지연 0 으로 둔다.
set_input_delay  0.0 -clock clk [get_ports {rst_n a b}]
set_output_delay 0.0 -clock clk [all_outputs]
