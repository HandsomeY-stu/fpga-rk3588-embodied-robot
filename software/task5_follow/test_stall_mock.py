#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线回归测试：检查stall_mock相关逻辑和异常处理。"""
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nav_layer import NavCtl   # noqa: E402


class FakeSender(object):
    """live=False 模拟 DRY 模式（semantic_nav 的 Sender(live)）。"""
    def __init__(self, live=True):
        self.hist = []
        self.live = live

    def send(self, d, spd):
        self.hist.append((d, spd))


class FakeTlm(object):
    """可编程假遥测：vell/velr 初值 0，lag_s 秒后切到 v。yaw 恒定。"""
    def __init__(self, v=0.0, lag_s=0.0, yaw=0.0):
        self.vell = 0.0
        self.velr = 0.0
        self.yaw_deg = yaw
        if lag_s <= 0:
            self.vell = self.velr = v
        else:
            def _flip():
                time.sleep(lag_s)
                self.vell = self.velr = v
            threading.Thread(target=_flip, daemon=True).start()

    def fresh_range(self):
        return None


def run(name, dist_mm, v, lag_s, deadline_s, expect, max_s=None, live=True):
    snd = FakeSender(live=live)
    tlm = FakeTlm(v, lag_s)
    logs = []
    ctl = NavCtl(snd, tlm, log=logs.append)
    t0 = time.time()
    st, mv = ctl.fwd(dist_mm, 0.0, spd=16,
                     deadline=time.time() + deadline_s, obs_mm=0)
    dt = time.time() - t0
    ok = (st == expect)
    if max_s is not None and dt > max_s:
        ok = False
    print('[%s] %-14s -> %-8s moved=%7.1fmm 用时=%5.1fs  (期望 %s%s)'
          % ('PASS' if ok else 'FAIL', name, st, mv, dt, expect,
             '' if max_s is None else ', ≤%.1fs' % max_s))
    for l in logs:
        print('        ' + l.strip())
    return ok


def main():
    print('=' * 72)
    print(' fwd() 进度看门狗离线验证（假 sender / 假遥测，不发真命令）')
    print('=' * 72)
    res = []

    res.append(run('A 堵转', 500, 0.0, 0.0, 300.0, 'STALL', max_s=6.0))

    res.append(run('B 正常', 200, 100.0, 0.0, 30.0, 'DONE'))

    res.append(run('C 滞后2s起步', 200, 100.0, 2.0, 30.0, 'DONE'))

    res.append(run('D 滞后4s起步', 500, 100.0, 4.0, 300.0, 'STALL', max_s=6.0))

    res.append(run('E DRY不判STALL', 500, 0.0, 0.0, 5.0, 'TIMEOUT', live=False))
    print('=' * 72)
    n = sum(1 for r in res if r)
    print(' 通过 %d / %d' % (n, len(res)))
    return 0 if n == len(res) else 1


if __name__ == '__main__':
    sys.exit(main())
