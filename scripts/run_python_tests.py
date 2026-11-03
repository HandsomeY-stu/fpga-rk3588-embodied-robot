#!/usr/bin/env python3
"""Run the existing policy and mock-service regression suites without board access."""
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TESTS = ('test_follow_policy.py', 'test_runner.py', 'test_continuity.py',
         'test_speedup.py', 'test_qualify_phase.py')

def main():
    cwd = ROOT / 'software/task5_follow'
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONUTF8='1')
    failed = []
    for name in TESTS:
        print(f'Running {name}', flush=True)
        result = subprocess.run([sys.executable, str(cwd / name)], cwd=cwd, env=env, timeout=180)
        if result.returncode:
            failed.append(name)
    print(f'{len(TESTS) - len(failed)}/{len(TESTS)} suites passed; failed={failed}')
    return bool(failed)

if __name__ == '__main__':
    sys.exit(main())
