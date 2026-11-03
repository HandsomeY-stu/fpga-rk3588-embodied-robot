#!/usr/bin/env python3
"""Unified host-console entry point; --check validates files without opening serial ports."""
import argparse
import ast
import os
from pathlib import Path

def main():
    parser = argparse.ArgumentParser(description='FPGA / RK3588 robot host console')
    parser.add_argument('--check', action='store_true', help='Check source/assets and exit without loading the hardware service')
    args = parser.parse_args()
    here = Path(__file__).resolve().parent
    if args.check:
        for name in ('server.py', 'assets.py', 'fpga_web_modes.py'):
            ast.parse((here / name).read_text(encoding='utf-8'), filename=name)
        for name in ('index.html', 'style.css', 'app.js'):
            if not (here / 'static' / name).is_file():
                raise SystemExit('Missing frontend resource: ' + name)
        root = here.parent
        for name in ('software/task6_red_bottle/run_red_bottle.py',
                     'software/task6_red_bottle/red_bottle_target.py',
                     'software/task6_red_bottle/prop_ctrl.py',
                     'software/runtime/grab_loop2.sh',
                     'software/task5_follow/frame_pub.py'):
            if not (root / name).is_file():
                raise SystemExit('Missing runtime source: ' + name)
        print('PASS web source, frontend assets and runtime dependencies; no serial/network service started.')
        return
    if os.name != 'posix':
        raise SystemExit('Run the hardware console on RK3588 Linux. Use --check for offline validation.')
    # Importing the backend opens the command UART, so the offline check exits above.
    from server import main as serve
    serve()

if __name__ == '__main__':
    main()
