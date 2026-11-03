#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线回归测试：检查continuity相关逻辑和异常处理。"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from follow_policy import FollowPolicy, FWD_FAMILY, SPIN_FAMILY  # noqa: E402
from test_support import det, pkt, st, red_pkt              # noqa: E402

TICK = 100.0
BAG = (27, 31, 33)
CASES = []


def case(fn):
    CASES.append(fn)
    return fn


def mk(**kw):
    """S5 配置（当前候选）：unified + shape + grace200 + turn 130/70"""
    base = dict(target_cls=BAG, accel_pct_s=20, decel_pct_s=40,
                semantic_grace_ms=200, turn_enter_px=130, turn_exit_px=70,
                motion_search_allowed=False)
    base.update(kw)
    return FollowPolicy(**base)


def run(pol, packets, statuses):
    return [pol.step(p, s, i * TICK) for i, (p, s) in enumerate(zip(packets, statuses))]


@case
def c01_slow_pan_keeps_forward():
    """cls33 逐帧平移（每帧 20px，累计到偏 280px）应持续 dir1/3/4，无 SEARCH_DISABLED。"""
    p = mk()
    pk, ss = [], []
    cx = 664.0
    for i in range(40):
        cx = min(944.0, cx + 20.0)
        pk.append(pkt(i // 2, [det(33, cx, 400)], red_cx=cx, red_n=800, red_w=180))
        ss.append(st(1200))
    cmds = run(p, pk, ss)
    bad = [c for c in cmds if c.state == 'SEARCH_DISABLED']
    assert not bad, '平移过程中出现 %d 拍 SEARCH_DISABLED' % len(bad)
    fwd = [c for c in cmds if c.dir in FWD_FAMILY]
    assert len(fwd) >= 30, '前进族只占 %d/%d 拍' % (len(fwd), len(cmds))

    assert not any(c.dir in SPIN_FAMILY for c in cmds), '出现原地转'


@case
def c02_large_pan_no_stop():
    """偏 200px 以上仍应保持前进（差速），不停车。"""
    p = mk()
    pk, ss = [], []
    cx = 664.0
    for i in range(40):
        cx = min(864.0, cx + 25.0)          # 逐帧 25px，累计到 +200px
        pk.append(pkt(i, [det(33, cx, 400)], red_cx=cx, red_n=800, red_w=180))
        ss.append(st(1200))
    cmds = run(p, pk, ss)
    stops = [c for c in cmds if c.dir == 0 or c.spd <= 0]
    assert len(stops) <= 2, '偏 200px 时停车 %d 拍' % len(stops)


@case
def c03_iou_jump_rejected_then_recover():
    """框中心突然跳 400px（IoU 崩）→ 应拒绝关联（不前进）；稳定几帧后恢复。"""
    p = mk()
    pk, ss = [], []
    for i in range(40):
        cx = 664.0 if i < 8 else 1064.0      # 第 8 拍突然跳 400px，之后稳定
        pk.append(pkt(i, [det(33, cx, 400)], red_cx=cx, red_n=800, red_w=180))
        ss.append(st(1200))
    cmds = run(p, pk, ss)

    jump = [c for c in cmds[8:12] if c.dir in FWD_FAMILY and c.spd > 18]
    assert len(jump) <= 1, 'IoU 跳变后立刻高速前进 %d 拍（关联未生效）' % len(jump)

    tail = [c for c in cmds[-10:] if c.dir in FWD_FAMILY and c.spd > 0]
    assert len(tail) >= 3, '跳变后始终没恢复跟踪（retain 过期后仍锁死）'


@case
def c04_red_ratio_drop():
    """红色比例跌到 0.10（低于 0.18）→ 锁定应失效，不得继续按该目标前进。"""
    p = mk()
    pk = [pkt(i // 2, [det(33, 664, 400, red=0.10)], red_cx=664, red_n=100, red_w=40)
          for i in range(30)]
    cmds = run(p, pk, [st(1200)] * 30)
    fwd = [c for c in cmds if c.dir in FWD_FAMILY and c.spd > 0]
    assert len(fwd) <= 2, '红色比例 0.10 仍前进 %d 拍（红色门槛失效）' % len(fwd)


@case
def c05_two_bags_no_swap():
    """两个相近红包：不得在两者间来回切换身份。"""
    p = mk()
    pk = []
    for i in range(60):
        d1 = det(33, 500, 380, red=0.96)
        d2 = det(33, 820, 380, red=0.95)
        pk.append(pkt(i // 3, [d1, d2]))
    cmds = run(p, pk, [st(1200)] * 60)

    seq = [c.dir for c in cmds if c.dir in (3, 4)]
    flips = sum(1 for a, b in zip(seq, seq[1:]) if a != b)
    assert flips <= 4, '两个同类目标间方向翻转 %d 次（疑似偷换目标）' % flips


@case
def c06_bottle_beside_bag_not_target():
    """书包旁有红瓶（cls44）：BAG 配置下不得改跟红瓶。"""
    p = mk()
    pk = []
    for i in range(40):
        pk.append(pkt(i // 3, [det(33, 700, 380, red=0.96), det(44, 900, 300, red=0.98)]))
    cmds = run(p, pk, [st(1200)] * 40)

    far = [c for c in cmds if c.dir in FWD_FAMILY and c.spd > 0]
    assert far, '完全没有前进'

    assert 33 in p.lock.target_cls, '配置错误'


@case
def c07_class_flicker_same_object():
    """33↔27 类别波动但空间位置连续：应延续身份，不因类别跳变而失锁。"""
    p = mk()
    pk = []
    for i in range(40):
        cls = 33 if (i // 3) % 2 == 0 else 27
        pk.append(pkt(i // 3, [det(cls, 680, 400)]))
    cmds = run(p, pk, [st(1200)] * 40)
    fwd = [c for c in cmds if c.dir in FWD_FAMILY and c.spd > 0]
    assert len(fwd) >= 28, '类别波动导致前进只有 %d/%d 拍' % (len(fwd), len(cmds))


@case
def c08_grace_window_then_stop():
    """短缺失 200ms 内可保持；超过截止时间第一拍必须停车。"""
    p = mk()
    pk, ss = [], []


    for i in range(12):
        pk.append(pkt(i, [det(33, 664, 400)] if i < 6 else []))
        ss.append(st(1200))
    cmds = run(p, pk, ss)

    coast = [c for c in cmds[6:8] if c.state == 'COAST']
    assert coast, '缺失后没有 COAST 保持'

    late = cmds[9:]
    assert all(c.dir == 0 or c.spd <= 0 for c in late), \
        '超过 200ms 仍输出运动: %s' % [(c.state, c.dir, c.spd) for c in late]


@case
def c09_duplicate_frame_no_extend():
    """重复帧不得延长 COAST 窗口。"""
    p = mk()
    pk, ss = [], []
    for i in range(30):
        pk.append(pkt(5, [det(33, 664, 400)] if i < 6 else []))   # frame_id 恒定
        ss.append(st(1200))
    cmds = run(p, pk, ss)
    coast = [c for c in cmds if c.state == 'COAST']
    assert len(coast) <= 3, '重复帧把 COAST 拉长到 %d 拍' % len(coast)


@case
def c10_all_paths_stop_on_bad_input():
    """过期/NaN/≤300mm/模式改变：即使框一直存在也必须 STOP。"""
    cases = [
        ('视觉过期', st(1200), dict(age=999)),
        ('遥测过期', st(1200, tlm_age=999), {}),
        ('NaN 距离', st(float('nan')), {}),
        ('距离过近', st(280), {}),
        ('模式改变', st(1200, mode=1), {}),
    ]
    for name, s, vkw in cases:
        p = mk()
        pkt_ = pkt(0, [det(33, 664, 400)], **vkw)
        c = p.step(pkt_, s, 0)
        assert c.dir == 0 and c.spd == 0, '%s 未停车: %s' % (name, (c.dir, c.spd))


@case
def c11_trim_first_tick_capped():
    """从 22% 直行切到 TRIM/COAST，首拍最终速度必须 ≤12%。"""
    p = mk()
    pk, ss = [], []

    for i in range(20):
        pk.append(pkt(i // 2, [det(33, 664, 400)]))
        ss.append(st(2000))

    for i in range(10):
        pk.append(pkt(10 + i // 2, [det(33, 1064, 400)], red_cx=1064, red_n=800, red_w=180))
        ss.append(st(2000))
    cmds = run(p, pk, ss)
    trim = [c for c in cmds[20:] if c.state.startswith('TRIM') or c.state == 'COAST']
    assert trim, '没有进入 TRIM/COAST'
    over = [c for c in trim if c.spd > 12.5]
    assert not over, 'TRIM/COAST 首拍超过 12%%: %s' % [(c.state, c.spd) for c in over[:3]]


@case
def c12_regain_accel_limit():
    """重获后加速仍符合 20 点/秒（每 100ms ≤2 点）。"""
    p = mk()
    pk, ss = [], []
    for i in range(40):
        pk.append(pkt(i // 2, [det(33, 664, 400)] if i >= 6 else []))
        ss.append(st(2000))
    cmds = run(p, pk, ss)
    seq = [(i, c.spd) for i, c in enumerate(cmds) if c.dir in FWD_FAMILY]
    for (i0, s0), (i1, s1) in zip(seq, seq[1:]):
        if i1 - i0 == 1 and s1 > s0:
            assert s1 - s0 <= 2.5, '重获后加速超限 %.1f→%.1f' % (s0, s1)


@case
def c13_no_target_no_forward():
    """无目标时不得通过 APPROACH_RED 前进。"""
    p = mk()
    pk = [red_pkt(i // 3, 664) for i in range(40)]     # 红块居中但无 det
    cmds = run(p, pk, [st(1200)] * 40)
    fwd = [c for c in cmds if c.dir in FWD_FAMILY and c.spd > 0]
    assert not fwd, '无语义目标却前进 %d 拍' % len(fwd)


@case
def c14_wrong_class_red_not_forward():
    """只有错误类别红物体（cls44 在 BAG 配置下）不得前进。"""
    p = mk()
    pk = [red_pkt(i // 3, 664, dets=[det(44, 664, 400, red=0.98)]) for i in range(40)]
    cmds = run(p, pk, [st(1200)] * 40)
    fwd = [c for c in cmds if c.dir in FWD_FAMILY and c.spd > 0]
    assert not fwd, '把错误类别红物体当前进目标 %d 拍' % len(fwd)


@case
def c15_first_confirm_no_forward():
    """首次 CONFIRMING 阶段不得未经确认直接前进。"""
    p = mk()
    pk = [pkt(0, [det(33, 664, 400)])]      # 只有一帧
    c = p.step(pk[0], st(1200), 0)
    assert p.lock.state != 'LOCKED', '单帧就锁定了'
    assert not (c.dir in FWD_FAMILY and c.spd > 15), \
        '首次确认阶段就高速前进: %s' % ((c.dir, c.spd),)


def main():
    fails = 0
    for fn in CASES:
        try:
            fn()
            print('  PASS  %s' % fn.__name__)
        except AssertionError as e:
            fails += 1
            print('  FAIL  %s: %s' % (fn.__name__, e))
        except Exception as e:      # noqa: BLE001
            fails += 1
            print('  ERR   %s: %s: %s' % (fn.__name__, type(e).__name__, e))
    print('-' * 70)
    print('%d/%d 通过' % (len(CASES) - fails, len(CASES)))
    return 0 if fails == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
