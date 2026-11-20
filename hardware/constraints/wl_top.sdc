# wl_top 합성 제약.
#
# 목표 200 MHz(주기 5 ns). 필요한 클럭은 프레임 하나를 처리하는 데 드는 클럭 수를 그 프레임의 간격으로 나눈
# 값이다. 장면 1에서 가장 무거운 보통 프레임(790)을 프레임 간격 안에 끝내려면 111.1 MHz,
# 열 훑기가 두 번 든 프레임 513을 그 간격 안에 끝내려면 164.2 MHz가 든다. 200 MHz는 둘 다
# 넘고 보통 프레임에 1.8배 여유를 둔다.
create_clock -name clk -period 5.0 [get_ports clk]

# 입출력은 같은 클럭에 붙은 외부 레지스터와 주고받는다고 보고, 주기의 20 %를 바깥 몫으로 남긴다.
set_input_delay  1.0 -clock clk [get_ports {in_word in_valid ph_data ph_valid clr rd_en rd_x rd_y}]
set_output_delay 1.0 -clock clk [all_outputs]

# 회전 중심 cx, cy는 도는 동안 바꾸지 않는 설정값이다. 리셋은 비동기다.
set_false_path -from [get_ports {cx cy rst_n}]
