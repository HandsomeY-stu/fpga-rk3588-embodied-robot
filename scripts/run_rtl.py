#!/usr/bin/env python3
"""Compile and run existing RTL tests in a temporary directory; no programming hardware."""
import argparse
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TESTS = [
    ('tb_task4_seg1_tail', ['task4/task4_seg1_fsm.v'], 'RESULT: TAIL SEQUENCE PASSED'),
    ('tb_task4_controls', ['task4/task4_seg1_fsm.v'], 'PASS Start latch'),
    ('tb_remote34', ['comm/vis_uart_bridge.v', 'comm/car_cmd_parse.v', 'common/car_crc8.v'], 'RESULT: ALL_PASS'),
    ('tb_osd_mode_cdc', ['osd/osd_mode_cdc.v'], 'PASS OSD mailbox'),
    ('tb_trim_equiv', ['task4/task4_seg1_fsm.v'], 'PASS all 262144'),
    ('tb_rgb_neutral_trim', ['rgb_neutral_trim.v'], 'PASS all 65536'),
]

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--require-gowin', action='store_true', help='Fail if the Gowin simulation library is unavailable; require all six RTL tests.')
    args = parser.parse_args()
    iv = os.environ.get('IVERILOG') or shutil.which('iverilog')
    vp = os.environ.get('VVP') or shutil.which('vvp')
    if not iv or not vp:
        raise SystemExit('Install Icarus Verilog or set IVERILOG and VVP.')
    src = ROOT / 'fpga/src'
    env = dict(os.environ)
    env['PATH'] = str(Path(iv).resolve().parent) + os.pathsep + env.get('PATH', '')
    passed = skipped = 0
    with tempfile.TemporaryDirectory(prefix='robot-rtl-') as tmp:
        for top, duts, marker in TESTS:
            extra = []
            if top == 'tb_rgb_neutral_trim':
                lib = env.get('GOWIN_SIMLIB')
                if not lib or not Path(lib).is_file():
                    if args.require_gowin:
                        raise SystemExit('RGB test is required: set GOWIN_SIMLIB to Gowin prim_sim.v')
                    print('SKIP RGB test: set GOWIN_SIMLIB to Gowin prim_sim.v')
                    skipped += 1
                    continue
                extra.append(lib)
            out = str(Path(tmp) / (top + '.vvp'))
            cmd = [iv, '-g2012', '-I', str(src / 'common'), '-s', top, '-o', out,
                   str(ROOT / 'sim/rtl' / (top + '.v'))] + [str(src / p) for p in duts] + extra
            subprocess.run(cmd, check=True, env=env, timeout=120)
            result = subprocess.run([vp, out], check=True, env=env, timeout=120, capture_output=True, text=True, errors='replace')
            if marker not in result.stdout:
                print(result.stdout)
                raise SystemExit('Assertion summary failed: ' + top)
            print(top + ': PASS')
            passed += 1
    print(f'RTL tests: passed={passed}, skipped={skipped}')

if __name__ == '__main__':
    main()
