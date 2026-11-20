#!/usr/bin/env python3
"""배치 · 배선 결과 DEF 에서 다이, 월드 메모리 뱅크, 표준 셀 자리를 그림으로 그린다.

    python3 hardware/sim/plot_layout.py <DEF> <출력 .png>

뱅크(wl_sram)는 회색 사각형에 뱅크 번호를, 표준 셀은 최상위 아래 첫 계층(u_dec, u_xf, u_acc …)별 색의 점으로
그린다. 필러는 빼고, 계층 밖에 있는 셀(클럭 트리 버퍼, 최적화가 넣은 버퍼, tie 셀)은 "top" 으로 묶는다.
뱅크 크기는 hardware/syn/wl_sram.lef 의 SIZE 를 읽는다.
"""
import os
import re
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

LEF = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'syn', 'wl_sram.lef')


def read_def(path):
    """(단위, 다이 박스, [(인스턴스, 셀, x, y, 방향)]) 을 DEF 단위 그대로 돌려준다."""
    text = open(path).read()
    unit = int(re.search(r'UNITS DISTANCE MICRONS (\d+)', text).group(1))
    die = [int(v) for v in re.search(r'DIEAREA \( (-?\d+) (-?\d+) \) \( (-?\d+) (-?\d+) \)', text).groups()]
    body = text[text.index('\nCOMPONENTS'):text.index('END COMPONENTS')]
    comps = []
    for rec in body.split(';')[1:]:
        m = re.search(r'-\s+(\S+)\s+(\S+).*?\+\s+(?:PLACED|FIXED)\s+\(\s*(-?\d+)\s+(-?\d+)\s*\)\s+(\S+)', rec, re.S)
        if m:
            comps.append((m.group(1), m.group(2), int(m.group(3)), int(m.group(4)), m.group(5)))
    return unit, die, comps


def main(def_path, png):
    unit, die, comps = read_def(def_path)
    w, h = (float(v) for v in re.search(r'SIZE (\S+) BY (\S+) ;', open(LEF).read()).groups())
    fig, ax = plt.subplots(figsize=(7, 7 * (die[3] - die[1]) / (die[2] - die[0])))
    ax.add_patch(Rectangle((die[0] / unit, die[1] / unit), (die[2] - die[0]) / unit, (die[3] - die[1]) / unit,
                           fill=False, lw=1.0, ec='black'))
    groups = {}
    for name, cell, x, y, _ in comps:
        if cell == 'wl_sram':
            ax.add_patch(Rectangle((x / unit, y / unit), w, h, fc='0.85', ec='0.4', lw=0.6))
            bank = re.search(r'g_bank\\?\[(\d+)\\?\]', name).group(1)
            ax.text(x / unit + w / 2, y / unit + h / 2, f'bank {bank}', ha='center', va='center', fontsize=7)
        elif not cell.startswith(('FILL', 'DECAP')):
            g = name.split('/')[0] if '/' in name else 'top'
            groups.setdefault(g, []).append((x / unit, y / unit))
    for g in sorted(groups):
        xs, ys = zip(*groups[g])
        ax.scatter(xs, ys, s=0.3, label=f'{g} ({len(xs)})', rasterized=True)
    ax.set_xlim(die[0] / unit, die[2] / unit)
    ax.set_ylim(die[1] / unit, die[3] / unit)
    ax.set_aspect('equal')
    ax.set_xlabel('x (um)')
    ax.set_ylabel('y (um)')
    ax.legend(loc='upper right', fontsize=7, markerscale=12, framealpha=0.9)
    fig.tight_layout()
    fig.savefig(png, dpi=160)
    print(f'{len(comps)} components, std cells by block:', {g: len(v) for g, v in sorted(groups.items())})


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
