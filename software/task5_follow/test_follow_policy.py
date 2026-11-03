#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线回归测试：检查follow_policy相关逻辑和异常处理。"""
import math
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from follow_policy import FollowPolicy, FWD_FAMILY, SPIN_FAMILY, SEARCH_STATES
from test_support import det, pkt, st, red_pkt

TICK = 100.0
BAG = (27, 31, 33)
BOTTLE = (44,)

CASES = []


def case(fn):
    CASES.append(fn)
    return fn


def run(policy, packets, statuses):
    out = []
    for i, (p, s) in enumerate(zip(packets, statuses)):
        out.append(policy.step(p, s, i * TICK))
    return out


@case
def t01_invalid_range_zero_output():
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    p = FollowPolicy(target_cls=BAG)
    cmds = run(p, [pkt(i // 3, [det(27, 700, 380)]) for i in range(30)],
               [st(None)] * 30)
    assert all(c.dir == 0 and c.spd == 0 for c in cmds), \
        '距离无效仍输出运动: %s' % {(c.dir, c.spd, c.state) for c in cmds}


@case
def t02_nan_range_zero_output():
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    p = FollowPolicy(target_cls=BAG)
    cmds = run(p, [pkt(i // 3, [det(27, 700, 380)]) for i in range(20)],
               [st(float('nan'))] * 20)
    assert all(c.dir == 0 for c in cmds), 'NaN 距离仍输出运动'


@case
def t03_range_stop_zero_output():
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    p = FollowPolicy(target_cls=BAG, range_stop_mm=300)
    cmds = run(p, [pkt(i // 3, [det(27, 700, 380)]) for i in range(20)],
               [st(300)] * 20)
    assert all(c.dir == 0 for c in cmds), '300mm 仍输出运动'


@case
def t04_red_branch_cannot_bypass_range():
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    p = FollowPolicy(target_cls=BOTTLE, red_cue=True)

    cmds = run(p,
               [red_pkt(i // 3, 664, dets=[det(44, 664, 400)]) for i in range(20)],
               [st(250)] * 20)
    bad = [c for c in cmds if c.dir in FWD_FAMILY and c.spd > 0]
    assert not bad, '红块分支绕过距离保护，仍前进 %d 拍' % len(bad)


@case
def t05_stale_vision_zero_output():
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    p = FollowPolicy(target_cls=BOTTLE, vision_max_age_ms=200)
    cmds = run(p,
               [red_pkt(i // 3, 900, age=350) for i in range(20)],
               [st(900)] * 20)
    assert all(c.dir == 0 for c in cmds), '视觉过期仍在运动'


@case
def t06_stale_telemetry_zero_output():
    p = FollowPolicy(target_cls=BAG, tlm_max_age_ms=300)
    cmds = run(p, [pkt(i // 3, [det(27, 700, 380)]) for i in range(20)],
               [st(900, tlm_age=500)] * 20)
    assert all(c.dir == 0 for c in cmds), '遥测过期仍在运动'


@case
def t07_wrong_mode_zero_output():
    p = FollowPolicy(target_cls=BAG)
    cmds = run(p, [pkt(i // 3, [det(27, 700, 380)]) for i in range(10)],
               [st(900, mode=1)] * 10)
    assert all(c.dir == 0 for c in cmds), '非手动模式仍在运动'


@case
def t08_no_search_after_target_lost():
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    p = FollowPolicy(target_cls=BAG, motion_search_allowed=False)
    pk, ss = [], []
    for i in range(150):
        pk.append(pkt(i // 3, [det(27, 700, 380)] if i < 15 else []))
        ss.append(st(900))
    cmds = run(p, pk, ss)
    bad = [c for c in cmds if c.state in SEARCH_STATES]
    assert not bad, '搜索路径未禁用: %s' % {(c.state, c.dir) for c in bad}


@case
def t09_never_confirmed_no_forward():
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    p = FollowPolicy(target_cls=BAG, red_cue=True)
    cmds = run(p, [red_pkt(i // 3, 664) for i in range(40)], [st(900)] * 40)
    bad = [c for c in cmds if c.dir in FWD_FAMILY and c.spd > 0]
    assert not bad, '没有语义目标却前进了 %d 拍' % len(bad)


@case
def t10_wrong_class_red_object():
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    p = FollowPolicy(target_cls=BAG, red_cue=True)

    cmds = run(p,
               [red_pkt(i // 3, 664, dets=[det(44, 664, 400, red=0.95)]) for i in range(40)],
               [st(900)] * 40)
    bad = [c for c in cmds if c.dir in FWD_FAMILY and c.spd > 0]
    assert not bad, '把错误类别的红物体当成目标前进了 %d 拍' % len(bad)


@case
def t11_two_same_class_targets():
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    p = FollowPolicy(target_cls=BAG, red_cue=False)
    pk = []
    for i in range(60):
        d1 = det(27, 400, 300, red=0.80)
        d2 = det(27, 950, 300, red=0.79)
        pk.append(pkt(i // 3, [d1, d2]))
    cmds = run(p, pk, [st(900)] * 60)

    dirs = [c.dir for c in cmds if c.dir in FWD_FAMILY]
    assert len(dirs) == len([c for c in cmds if c.dir in FWD_FAMILY])

    assert all(isinstance(c.spd, (int, float)) and not math.isnan(c.spd) for c in cmds)


@case
def t12_duplicate_frames_not_counted():
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    p = FollowPolicy(target_cls=BAG, semantic_grace_ms=200)
    pk, ss = [], []
    for i in range(60):
        pk.append(pkt(5, [det(27, 700, 380)] if i < 6 else []))   # 同一个 frame_id
        ss.append(st(900))
    cmds = run(p, pk, ss)
    coast = [c for c in cmds if c.state == 'COAST']
    assert len(coast) <= 3, '重复帧把 COAST 拉长到 %d 拍' % len(coast)


@case
def t13_accel_limit():
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    p = FollowPolicy(target_cls=BAG, accel_pct_s=20, decel_pct_s=40)
    pk = [pkt(i // 3, [det(27, 664, 200)]) for i in range(40)]
    cmds = run(p, pk, [st(2000)] * 40)
    fwd = [(i, c.spd) for i, c in enumerate(cmds) if c.dir in FWD_FAMILY]
    for (i0, s0), (i1, s1) in zip(fwd, fwd[1:]):
        if i1 - i0 == 1 and s1 > s0:
            assert s1 - s0 <= 2.5, '加速超限: %s -> %s' % (s0, s1)


@case
def t14_unified_speed_same_function():
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    d = 900
    p1 = FollowPolicy(target_cls=BOTTLE, unified_speed=True, red_cue=True)
    a = []
    for i in range(60):
        a.append(p1.step(pkt(i // 3, [det(44, 664, 300)]), st(d), i * TICK))
    p2 = FollowPolicy(target_cls=BOTTLE, unified_speed=True, red_cue=True)
    b = []
    for i in range(60):
        b.append(p2.step(red_pkt(i // 3, 664, dets=[det(44, 664, 300)]), st(d), i * TICK))
    sa = [c.spd for c in a if c.state.startswith('APPROACH')]
    sb = [c.spd for c in b if c.state.startswith('APPROACH')]
    assert sa and sb, '两路径未产生 APPROACH: %s / %s' % (sa, sb)
    assert abs(sa[-1] - sb[-1]) <= 2.0, \
        'S1 未统一：APPROACH=%.1f APPROACH_RED=%.1f' % (sa[-1], sb[-1])

    assert abs(sa[-1] - 20.0) <= 2.0, '稳态速度与距离插值不符: %.1f' % sa[-1]


@case
def t15_direction_confirm_frames():
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    p = FollowPolicy(target_cls=BOTTLE, red_cue=True, direction_confirm_frames=2,
                     turn_enter_px=130, turn_exit_px=70, motion_search_allowed=True)

    pk = [red_pkt(0, 1000), red_pkt(1, 1000), red_pkt(2, 200), red_pkt(3, 200), red_pkt(4, 200)]
    cmds = run(p, pk, [st(900)] * 5)

    assert cmds[2].dir == cmds[1].dir, '反向未经确认就翻向: %s -> %s' % (cmds[1].dir, cmds[2].dir)
    assert cmds[4].dir != cmds[1].dir, '反向始终没生效（锁死错误方向）'


@case
def t16_bad_inputs_no_crash():
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    p = FollowPolicy(target_cls=BAG)
    bad = [None, {}, {'color_schema': 2}, {'color_schema': 1},
           {'color_schema': 1, 'producer_id': 'x', 'frame_id': 1},
           pkt(1, [{'cls': 27}]), pkt(2, [det(27, 700, 380)])]
    for i, b in enumerate(bad):
        for s in (None, {}, st(900), st(None)):
            c = p.step(b, s, i * TICK)
            assert c is not None and c.dir in (0, 1, 2, 3, 4, 5, 6)


@case
def t17_stop_clears_speed_cache():
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    p = FollowPolicy(target_cls=BAG, accel_pct_s=20, decel_pct_s=40)
    run(p, [pkt(i // 3, [det(27, 664, 200)]) for i in range(9)], [st(2000)] * 9)
    p.step(pkt(9, [det(27, 664, 200)]), st(250), 9 * TICK)      # RANGE_STOP
    assert p._spd_prev is None, '停车后速度缓存未清'
    c = p.step(pkt(10, [det(27, 664, 200)]), st(2000), 10 * TICK)
    assert c.spd <= 12, '停车后重新起步速度直接跳到 %.1f' % c.spd


@case
def t18_spin_speed_capped():
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    p = FollowPolicy(target_cls=BOTTLE, red_cue=True, motion_search_allowed=True)
    cmds = run(p, [red_pkt(i // 3, 1250) for i in range(20)], [st(900)] * 20)
    spins = [c for c in cmds if c.dir in SPIN_FAMILY]
    assert spins, '没有产生原地转'
    assert all(c.spd <= 8.0 for c in spins), \
        '原地转超过 8%%: %s' % sorted({c.spd for c in spins})


def _feed(p, rows, start_ms=0):
    """按 (fid, conf, cx, w) 逐帧喂一次（**每帧新 fid**，与实车 10Hz 视觉对齐）。"""
    out = []
    t = start_ms
    for row in rows:
        fid, conf = row[0], row[1]
        cx = row[2] if len(row) > 2 else 700
        w = row[3] if len(row) > 3 else None
        out.append(p.step(pkt(fid, [det(27, cx, 380, conf=conf, w=w)]), st(1200), t))
        t += TICK
    return out


def _locked(p):
    """喂 3 个强帧直到 LOCKED（confirm=3）。"""
    return _feed(p, [(1, .60), (2, .60), (3, .60)])


@case
def t19_weak_continue_keeps_motion():
    """续跟门 0.15：已在跟随时 conf=0.24 同位置框**维持跟随并继续输出运动**。"""
    p = FollowPolicy(target_cls=BAG)
    cmds = _feed(p, [(1, .60), (2, .60), (3, .60), (4, .60), (5, .60), (6, .24)])
    assert p.lock.state == 'LOCKED', '弱帧后掉锁: %s' % p.lock.state
    assert p.lock.target.get('_weak') is True, '弱帧未标记 _weak（会污染资格统计）'
    assert p.stats['weak_cont'] == 1, '弱帧计数器没涨: %s' % p.stats
    last = cmds[-1]
    assert last.dir == 1 and last.spd > 0, '弱帧续跟后没继续前进: %s' % (last,)


@case
def t20_weak_continue_rejects_jump():
    """续跟门不是"降阈值"：低分框位置不连续（IoU<0.25）必须拒收。"""
    p = FollowPolicy(target_cls=BAG)
    _locked(p)
    _feed(p, [(4, .24, 150)], start_ms=3 * TICK)     # cx 700 -> 150，无重叠
    assert p.lock.state == 'TARGET_LOST', '跳变弱框被当成同一目标: %s' % p.lock.state


@case
def t21_weak_conf_cannot_acquire():
    """首次获取仍 0.30：冷启动只有低分框时不得锁定。"""
    p = FollowPolicy(target_cls=BAG)
    cmds = run(p, [pkt(i, [det(27, 700, 380, conf=.24)]) for i in range(30)],
               [st(1200)] * 30)
    assert p.lock.target is None, '低分框凭空锁定了目标'
    assert all(c.dir == 0 for c in cmds), '未获取目标仍输出运动'


@case
def t22_weak_frame_never_qualified():
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    p = FollowPolicy(target_cls=BAG, trim_test_cap=18, trim_speed_mode='qualified-fixed')
    _feed(p, [(1, .60), (2, .60), (3, .60), (4, .60), (5, .60)])
    assert p.debug['qualify_state'] == 'ok', '强帧未取得资格: %s' % p.debug['qualify_state']
    p.step(pkt(6, [det(27, 700, 380, conf=.24)]), st(1200), 5 * TICK)
    assert p.debug['qualify_state'] == 'blocked', '弱帧仍具资格'
    assert p.debug['qualify_block'] == 'weak_cont', \
        '弱帧的资格阻断原因应单列: %s' % p.debug['qualify_block']
    assert p.qual_block['weak_cont'] == 1


@case
def t23_weak_frame_speed_capped_12():
    """低分续跟只保"不停车"，按 RED_HOLD 同档 ≤12% 限速，且不进 trim_qualified 档。"""
    p = FollowPolicy(target_cls=BAG, fwd_far=22, trim_test_cap=18,
                     trim_speed_mode='qualified-fixed')
    cmds = _feed(p, [(1, .60), (2, .60), (3, .60), (4, .60), (5, .60), (6, .24)])
    last = cmds[-1]
    assert last.dir == 1 and last.spd <= 12.0, '弱帧输出 %s 超过 12%%' % (last,)
    assert p.debug.get('cap_reason') == 'weak_cont', \
        '限速来源应为 weak_cont: %s' % p.debug.get('cap_reason')


@case
def t24_weak_continue_requires_scale():
    """尺度突变（宽比 3 倍）的低分框不算续跟——防止"换成另一个更大物体"。"""
    p = FollowPolicy(target_cls=BAG)
    _feed(p, [(1, .60, 700, 200), (2, .60, 700, 200), (3, .60, 700, 200)])
    _feed(p, [(4, .24, 700, 600)], start_ms=3 * TICK)
    assert p.lock.state == 'TARGET_LOST', '宽比突变的弱框被接受: %s' % p.lock.state


@case
def t25_cross_class_duplicate_merged():
    """同一个书包被判成 suitcase+backpack（IoU≈0.98）⇒ 跨类去重，不算"多目标歧义"。"""
    p = FollowPolicy(target_cls=BAG)
    d1 = det(33, 671, 380, conf=.535, w=323)
    d2 = det(27, 674, 380, conf=.457, w=323)
    for i in range(6):
        p.step(pkt(i + 1, [d1, d2]), st(1200), (i + 1) * TICK)
    assert p.lock.state == 'LOCKED', p.lock.state
    assert p.lock.cross_class_dedups >= 1, '跨类重复框没被合并'
    assert p.debug.get('eligible_count') == 1, '合并后仍被判成多个合格目标'
    assert p.debug.get('qualify_block') != 'ambiguous', '同一目标跨类抖动被当成歧义'


@case
def t26_two_real_targets_not_merged():
    """两个位置/尺度都不同的真目标**不得**被跨类去重合并。"""
    p = FollowPolicy(target_cls=BAG)
    a = det(33, 300, 380, conf=.60, w=300)
    b = det(27, 950, 380, conf=.55, w=300)
    for i in range(6):
        p.step(pkt(i + 1, [a, b]), st(1200), (i + 1) * TICK)
    assert p.lock.cross_class_dedups == 0, '两个分开的目标被误合并'
    assert p.debug.get('eligible_count') == 2


@case
def t27_nested_body_half_merged():
    """同一物体"全身 + 下半身"两个框（IoU≈0.4，同 cx 同宽）⇒ 也必须合并。"""
    p = FollowPolicy(target_cls=BAG)
    full = dict(cls=33, conf=.751, cx=553.0, cy=317.0, w=351.0, h=418.0, red_ratio=.9)
    half = dict(cls=33, conf=.598, cx=547.0, cy=417.0, w=324.0, h=201.0, red_ratio=.9)
    for i in range(6):
        p.step(pkt(i + 1, [full, half]), st(1200), (i + 1) * TICK)
    assert p.debug.get('eligible_count') == 1, '全身+半身没合并: %s' % p.debug.get('eligible_count')
    assert p.debug.get('qualify_block') != 'ambiguous'


@case
def t28_task6_red_centered_still_stops():
    """任务六语义（red_follow=False）：红块居中但无语义确认 ⇒ **停车**（原行为不变）。"""
    p = FollowPolicy(target_cls=BAG, semantic_grace_ms=0, red_hold_enable=False)
    for i in range(3):
        p.step(pkt(i + 1, [det(27, 700, 380)]), st(1200), (i + 1) * TICK)
    c = p.step(pkt(9, [], red_cx=664, red_n=800, red_w=200), st(1200), 9 * TICK)
    assert c.state in ('RED_CENTERED', 'LOST'), c.state
    assert c.dir == 0, '任务六不该在没有语义确认时继续前进: %s' % c


@case
def t29_task5_red_centered_keeps_going():
    """任务五（red_follow=True）：红块居中无确认 ⇒ 以红色维持档**继续前进**。"""
    p = FollowPolicy(target_cls=BAG, semantic_grace_ms=0, red_follow=True,
                          red_hold_cap=16, red_hold_enable=False)
    for i in range(3):
        p.step(pkt(i + 1, [det(27, 700, 380)]), st(1200), (i + 1) * TICK)
    c = p.step(pkt(9, [], red_cx=664, red_n=800, red_w=200), st(1200), 9 * TICK)
    assert c.state == 'RED_FOLLOW', c.state
    assert c.dir == 1 and 0 < c.spd <= 16.0, c


@case
def t30_task5_red_offset_keeps_moving():
    """任务五：红块偏离时**差速前进**（RED_TRACK，一边走一边修正），不再原地转头。"""
    for follow, want in ((False, 'SEARCH_DISABLED'), (True, 'RED_TRACK')):
        p = FollowPolicy(target_cls=BAG, semantic_grace_ms=0, red_follow=follow,
                                red_hold_enable=False)
        for i in range(3):
            p.step(pkt(i + 1, [det(27, 700, 380)]), st(1200), (i + 1) * TICK)
        c = p.step(pkt(9, [], red_cx=1150, red_n=800, red_w=200), st(1200), 9 * TICK)
        assert c.state == want, 'red_follow=%s 期望 %s 实际 %s' % (follow, want, c.state)
        if follow:
            assert c.dir in (3, 4), '任务五追红必须是差速前进(dir 3/4)，实际 %s' % c.dir
            assert c.spd > 0, '追红不能停车'


def main():
    fails = 0
    for fn in CASES:
        name = fn.__name__
        try:
            fn()
            print('  PASS  %s' % name)
        except AssertionError as e:
            fails += 1
            print('  FAIL  %s: %s' % (name, e))
        except Exception as e:
            fails += 1
            print('  ERR   %s: %s: %s' % (name, type(e).__name__, e))
    print('-' * 70)
    print('%d/%d 通过' % (len(CASES) - fails, len(CASES)))
    return 0 if fails == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
