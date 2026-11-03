#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""跟随策略阶段回归，复用离线输入检查控制输出。"""
import os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
_argv = sys.argv[:]
sys.argv = ['nav']
import nav_fsm_sw as nav
sys.argv = _argv


def load_relay_frame():
    """cmd_relay.py 依赖 Linux 的 termios，Windows 上不能 import。"""
    import ast
    p = os.path.join(HERE, 'cmd_relay.py')
    tree = ast.parse(open(p, encoding='utf-8').read())
    keep = [n for n in tree.body
            if isinstance(n, ast.FunctionDef) and n.name in ('crc8', 'frame')]
    ns = {}
    exec(compile(ast.Module(body=keep, type_ignores=[]), p, 'exec'), ns)
    return ns['frame']


relay_frame = load_relay_frame()
from prop_ctrl import PropFollow, DIR_STOP, DIR_FWD, DIR_SPIN_L, DIR_SPIN_R

SPIN = (DIR_SPIN_L, DIR_SPIN_R)
results = []


def check(name, ok, detail):
    results.append((name, ok, detail))
    print('  [%s] %-34s %s' % ('PASS' if ok else 'FAIL', name, detail))


def b1_center_no_turn():
    print('\nB1 · cx=520 正前方不产生持续转向')

    f = PropFollow(cx_center=520)
    spins = 0
    for i in range(20):
        c = f.step((1, 520, 360, 400, 500, 95), i * 77)
        spins += 1 if c.dir in SPIN else 0
    check('静止 20 帧无转向', spins == 0, '转向帧 %d/20' % spins)


    import random
    rng = random.Random(7)
    f = PropFollow(cx_center=520)
    spins = 0
    for i in range(30):
        cx = 520 + rng.randint(-5, 5)
        c = f.step((1, cx, 360, 400, 500, 95), i * 77)
        spins += 1 if c.dir in SPIN else 0
    check('±5px 噪声 30 帧无转向', spins == 0, '转向帧 %d/30' % spins)


    f = PropFollow(cx_center=640)
    spins = 0
    for i in range(20):
        c = f.step((1, 520, 360, 400, 500, 95), i * 77)
        spins += 1 if c.dir in SPIN else 0
    check('反证：cx_center=640 必然转', spins > 0,
          '转向帧 %d/20（err 恒 -120，这正是 P2 的离线复现）' % spins)


    f = PropFollow(cx_center=520, db_in=40, db_out=40)
    spins = 0
    for i in range(20):
        c = f.step((1, 569, 360, 280, 350, 64), i * 77)
        spins += 1 if c.dir in SPIN else 0
    check('反证：死区 40 + err=49 会转', spins > 0,
          '转向帧 %d/20（run_b 当前默认行为，P4 的开关式两难）' % spins)


def b2_no_deadband250():
    print('\nB2 · deadband=250 不再作为默认')
    import inspect, re
    src_c = open(os.path.join(HERE, 'run_c.py'), encoding='utf-8').read()
    src_b = open(os.path.join(HERE, 'run_b.py'), encoding='utf-8').read()

    has_dband_c = ("add_argument('--deadband'" in src_c) or ('add_argument("--deadband"' in src_c)
    m = re.search(r"--deadband',\s*type=int,\s*default=(\d+)", src_b)
    dflt_b = int(m.group(1)) if m else None
    f = PropFollow()
    check('run_c 未注册 --deadband 参数', not has_dband_c,
          'run_c.py add_argument(--deadband) = %s' % has_dband_c)
    check('run_b 的 --deadband 默认值', dflt_b == 0, 'run_b 默认 %s（062 实车用了 250，已废）' % dflt_b)
    check('prop_ctrl 迟滞 30/60', (f.db_in, f.db_out) == (30, 60),
          'db_in=%d db_out=%d（替代 250 吸附）' % (f.db_in, f.db_out))


def b3_no_diff_turn():
    print('\nB3 · DIR_LEFT/RIGHT 不用于对准')
    dirs = set()

    import red_nav_e2e as rd
    f = PropFollow(cx_center=520)
    seq = ['look.jpg', 'now.jpg', 't.jpg', 'f0.jpg', 'f1.jpg', 'chk0.jpg', 'chk.jpg', 'diag.jpg']
    t = 0
    for fn in seq:
        d = rd.detect_red(os.path.join(HERE, fn))
        if d:
            c = f.step((d['cls'], d['cx'], d['cy'], d['w'], d['h'], d['conf']), t)
            dirs.add(c.dir)
        t += 77
    for i in range(20):
        c = f.step(None, t); dirs.add(c.dir); t += 77
    for cx in (0, 200, 400, 520, 700, 900, 1100, 1279):
        c = f.step((1, cx, 360, 300, 380, 90), t); dirs.add(c.dir); t += 77
    bad = dirs & {3, 4}
    check('全场景方向码 ⊆ {0,1,5,6}', not bad,
          '实际用到 %s，差速码 3/4 出现 %s' % (sorted(dirs), sorted(bad) or '无'))


def b4_protocol():
    print('\nB4 · 协议仍为 LEN+5')
    fr = relay_frame(1, 5, 20)               # 中继实际发送的帧
    check('中继 MOTION 帧长 = LEN+5', len(fr) == fr[2] + 5 == 8,
          'len=%d LEN=%d 期望 8' % (len(fr), fr[2]))
    nf = nav.motion_frame(1, 5, 20)
    ok, errs = nav.check_frame(nf)
    check('nav.check_frame 通过', ok and not errs, 'errs=%s' % (errs or '无'))
    check('中继帧与状态机帧逐字节相同', fr == nf, fr.hex(' '))


def main():
    print('=' * 84)
    print('063 第一阶段 · 062 实车基线回归（4 条）')
    print('=' * 84)
    b1_center_no_turn()
    b2_no_deadband250()
    b3_no_diff_turn()
    b4_protocol()
    npass = sum(1 for _, ok, _ in results if ok)
    print('\n' + '=' * 84)
    print('回归项: %d/%d PASS' % (npass, len(results)))
    print('总判定: %s' % ('PASS' if npass == len(results) else 'FAIL'))
    print('=' * 84)
    return 0 if npass == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
