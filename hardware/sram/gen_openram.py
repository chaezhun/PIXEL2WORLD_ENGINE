#!/usr/bin/env python3
"""OpenRAM 으로 FreePDK45 SRAM macro 하나를 만들어 면적 · 지연 · 전력을 얻는다.

    ~/openram_venv/bin/python hardware/sram/gen_openram.py <1rw|1rw1r> <word_size> <num_words> [spice] [wpr=<n>] [dcs=<n>]

대회 공정(GPDK045)에는 SRAM compiler 가 없어서, 같은 45 nm 세대의 공개 PDK(FreePDK45)용 OpenRAM 결과를
SRAM assumptions 로 쓴다. 설정 파일 cfg_<이름>.py 와 결과 out/<이름>/ 는 현재 폴더에 생긴다. 저장소 경로에는
공백이 있을 수 있으므로 ~/wl_build/openram 처럼 공백 없는 폴더에서 돌린다. 중요한 파일은
<이름>.html(datasheet), *.lib(면적 · 타이밍 · 전력), *.lef(macro 외곽), *.gds(layout).

  1rw    : 포트 하나(읽기 또는 쓰기). 6T bitcell(cell_1rw)
  1rw1r  : 읽기 · 쓰기 포트 하나 + 읽기 포트 하나. 8T bitcell(cell_2rw). 읽기와 쓰기를 한 클럭에 같이 할 수 있다
  wpr=n  : 한 행에 둘 word 수(열 다중화)를 n 으로 정한다. 빼면 OpenRAM 이 고르는데, 1,024 word 에서 14비트는 8,
           13비트는 16 을 골라 13비트 macro 가 더 크게 나온다. word 크기가 달라도 같은 모양으로 견주려고 둔다
  dcs=n  : sense amp enable 지연 사슬 단 수(OpenRAM 기본 9). 13비트 1,024 word(wpr=8)는 기본값으로 특성화하면
           "Sense amp enable timing error"로 멈추므로, 이 값을 늘려 볼 때 쓴다
  spice  : 넣으면 ngspice 로 타이밍 · 전력을 시뮬레이션한다. 빼면 OpenRAM 분석 모델이라 지연 · 전력은 대략값이다.
           면적은 두 경우 모두 생성된 layout 에서 잰 값이다. 기본 부하 · slew 3 x 3 점을 다 돌리면 4,096 word 에서
           몇 시간이 걸려서 가운데 한 점(배율 1)만 돌린다. 접근 에너지 · leakage 는 부하 · slew 에 거의 매이지 않는다.

설치: python3 -m venv ~/openram_venv; pip install openram "numpy<2" "scipy<1.14".
numpy 2 에서는 layout 단계에서 TypeError 로 죽는다. 처음 실행할 때 OpenRAM 이 자기 폴더에 miniconda 와
ngspice · Xyce · magic · netgen · klayout 을 설치한다(약 6분).
"""
import sys

ports, ws, nw = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
spice = 'spice' in sys.argv[4:]
wpr = next((int(a[4:]) for a in sys.argv[4:] if a.startswith('wpr=')), None)
dcs = next((int(a[4:]) for a in sys.argv[4:] if a.startswith('dcs=')), None)
name = f"{ports}_{ws}x{nw}" + (f"_w{wpr}" if wpr else "") + (f"_d{dcs}" if dcs else "") + ("_sp" if spice else "")
cfg = f"cfg_{name}.py"
with open(cfg, "w") as f:
    f.write(f"""
word_size = {ws}
num_words = {nw}
num_rw_ports = 1
num_r_ports = {1 if ports == '1rw1r' else 0}
num_w_ports = 0
tech_name = "freepdk45"
nominal_corner_only = True
process_corners = ["TT"]
supply_voltages = [1.0]
temperatures = [25]
route_supplies = False
check_lvsdrc = False
analytical_delay = {not spice}
load_scales = [1]
slew_scales = [1]
spice_name = "ngspice"
output_name = "{name}"
output_path = "out/{name}"
""" + (f"words_per_row = {wpr}\n" if wpr else "")
            + (f"delay_chain_stages = {dcs}\n" if dcs else ""))

import openram                                    # noqa: E402  (설정 파일을 먼저 써야 한다)
openram.init_openram(config_file=cfg, is_unit_test=False)
from openram import sram                          # noqa: E402
sram().save()
