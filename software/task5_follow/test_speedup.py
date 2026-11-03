#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线回归测试：检查speedup相关逻辑和异常处理。"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from follow_policy import FollowPolicy, FWD_FAMILY, DIR_LEFT, DIR_RIGHT  # noqa: E402
from test_support import det, pkt, st, red_pkt               # noqa: E402

TICK = 100.0
BAG = (27, 31, 33)
CX = 664

OFF = 200
FAR_RNG = 1200        # ≥ d_far(1000) 且 ≥ qualify_rng_min(1000)

CASES = []


def case(fn):
    CASES.append(fn)
    return fn


def mk(cap=12.0, mode='qualified-fixed', **kw):
    """与 runner S5 实车配置一致的基线（far22 / 130-70 / 20-40）。"""
    base = dict(target_cls=BAG, min_conf=0.30, red_min=0.18,
                range_stop_mm=300, close_h=0.96,
                d_far=1000, d_near=400,
                fwd_far=22.0, fwd_near=10.0, max_spd=24.0,
                trim_cap=12.0, trim_test_cap=cap, trim_speed_mode=mode,
                red_hold_cap=12.0, coast_cap=12.0,
                accel_pct_s=0, decel_pct_s=0,
                semantic_grace_ms=200,
                turn_enter_px=130, turn_exit_px=70,
                motion_search_allowed=False)
    base.update(kw)
    return FollowPolicy(**base)


def trims(policy, n, start=0, cx=CX - OFF, conf=0.55, age=30, rng=FAR_RNG, **kw):
    """连续 n 个**新帧**的偏心目标（每帧 fid 递增）。"""
    out = []
    for i in range(n):
        p = pkt(start + i, [det(27, cx, 380, conf=conf)], age=age, **kw)
        out.append(policy.step(p, st(rng), (start + i) * TICK))
    return out


def cap_reasons(policy):
    return policy.debug.get('cap_reason'), policy.debug.get('final_cap')


@case
def s01_trim_cap_starts_at_base_then_rises():
    """资格满足前必须是 12，满 3 个不同新鲜帧后才用候选 18。"""
    p = mk(cap=18.0)
    rows = []
    for i in range(8):
        c = p.step(pkt(i, [det(27, CX - OFF, 380)]), st(FAR_RNG), i * TICK)
        rows.append((c, p.debug.get('cap_reason'), p.debug.get('final_cap')))
    tr = [r for r in rows if r[0].dir in (DIR_LEFT, DIR_RIGHT)]
    assert len(tr) >= 4, 'TRIM 拍太少，用例前提不成立: %d' % len(tr)
    base = [r for r in tr if str(r[1]).startswith('trim_base')]
    qual = [r for r in tr if str(r[1]).startswith('trim_qualified')]
    assert base, '没有任何一拍走 trim_base（§5.1 的资质门槛没生效？）'
    assert all(r[0].spd <= 12.0 for r in base), \
        '资格未满就提速: %s' % [(r[0].state, r[0].spd) for r in base]
    assert qual, '攒满 3 帧后仍未拿到 qualified 档'
    assert all(r[0].spd <= 18.0 for r in qual), \
        '最终发令越过候选 cap: %s' % [r[0].spd for r in qual]
    assert any(abs(r[0].spd - 18.0) < 1e-6 for r in qual), \
        '资格满足后仍拿不到 18（请求侧钳位没同步？）: %s' % [r[0].spd for r in qual]
    assert all(r[2] == 18.0 for r in qual), 'final_cap 与 manifest 不一致'


@case
def s02_off_mode_never_exceeds_trim_cap():
    """B0 基线（mode=off）：即使传了候选 18，也必须恒为 12。"""
    p = mk(cap=18.0, mode='off')
    cmds = trims(p, 12)
    assert all(c.spd <= 12.0 for c in cmds), \
        'B0 越界: %s' % sorted({c.spd for c in cmds})
    assert p.debug.get('cap_reason', '').startswith('trim_base'), \
        'B0 的 cap_reason 应为 trim_base，实际 %s' % p.debug.get('cap_reason')
    assert p.debug.get('final_cap') == 12.0, 'B0 的 final_cap 应为 12'


@case
def s03_ramp_never_exceeds_state_cap():
    """整形打开（20/40）时，从 22 直行切到纠偏也要被 cap 立即压住。"""
    p = mk(cap=18.0)
    out = []
    for i in range(12):                    # 先居中直行，攒速度
        out.append(p.step(pkt(i, [det(27, CX, 380)]), st(FAR_RNG), i * TICK))
    for i in range(12, 30):                # 目标突然偏出 ⇒ TRIM
        out.append(p.step(pkt(i, [det(27, CX - OFF, 380)]), st(FAR_RNG), i * TICK))
    assert out[10].spd > 18.0, '前置条件不成立：直行没跑起来（%.2f）' % out[10].spd

    trim_cmds = [c for c in out[12:] if c.dir in (DIR_LEFT, DIR_RIGHT)]
    assert len(trim_cmds) >= 4, 'TRIM 拍太少，用例前提不成立: %d' % len(trim_cmds)
    bad = [c.spd for c in trim_cmds if c.spd > 18.0 + 1e-9]
    assert not bad, 'TRIM 期间越过 18: %s' % bad
    assert all(c.spd <= 24.0 for c in out), '越过全局 max_spd'


@case
def s04_same_frame_repeat_no_qualify():
    """同帧重复（frame_id 不变）不得累积资格。"""
    p = mk(cap=18.0)
    cmds = [p.step(pkt(777, [det(27, CX - OFF, 380)]), st(FAR_RNG), i * TICK)
            for i in range(15)]
    assert all(c.spd <= 12.0 for c in cmds), \
        '重复帧累积出资格: %s' % sorted({c.spd for c in cmds})


@case
def s05_low_conf_follows_but_no_speedup():
    """conf 0.31~0.39：仍要能跟随（不重新引入两道门槛），但不许提速。"""
    p = mk(cap=18.0)
    cmds = trims(p, 12, conf=0.35)

    tail = cmds[4:]
    assert all(c.dir in FWD_FAMILY for c in tail), \
        '低置信目标不再跟随（A1 修复被破坏）: %s' % [(c.dir, c.state) for c in tail]
    assert all(c.spd <= 12.0 for c in cmds), \
        '低置信目标拿到了高速资格: %s' % sorted({c.spd for c in cmds})


@case
def s06_near_range_follows_but_no_speedup():
    """rng 900 < qualify_rng_min(1000)：可跟随，不提速。"""
    p = mk(cap=18.0)
    cmds = trims(p, 12, rng=900)
    tail = cmds[4:]
    assert all(c.dir in FWD_FAMILY and c.spd > 0 for c in tail), \
        '近距不再跟随: %s' % [(c.dir, c.state, c.spd) for c in tail]
    assert all(c.spd <= 12.0 for c in cmds), '近距拿到高速资格: %s' % sorted({c.spd for c in cmds})


@case
def s07_stale_vision_no_speedup():
    """视觉 age 150（>120 但未过期）：可跟随，不提速。"""
    p = mk(cap=18.0)
    cmds = trims(p, 12, age=150)
    assert all(c.spd <= 12.0 for c in cmds), '过期边缘拿到高速资格: %s' % sorted({c.spd for c in cmds})


@case
def s08_qualify_decays_immediately():
    """资格掉一帧立即回落 12（不等 3 帧），避免好坏交替时来回跳。"""
    p = mk(cap=18.0)
    trims(p, 5)
    assert p.debug.get('cap_reason', '').startswith('trim_qualified')
    c = p.step(pkt(99, [det(27, CX - OFF, 380, conf=0.35)]), st(FAR_RNG), 99 * TICK)
    assert c.spd <= 12.0, '资格掉了当拍仍输出 %.2f（应 ≤12）' % c.spd
    assert p.debug.get('cap_reason', '').startswith('trim_base'), \
        '资格掉了 cap_reason 仍是 %s' % p.debug.get('cap_reason')


@case
def s09_red_hold_gets_no_speedup():
    """RED_HOLD 是虚拟目标，不能冒充语义目标拿高速。"""
    p = mk(cap=18.0)
    trims(p, 6)
    out = []
    for i in range(6, 14):
        out.append(p.step(red_pkt(i, CX - OFF, h=300, n=800, w=110), st(FAR_RNG), i * TICK))
    states = {c.state for c in out}
    assert any(s in ('RED_HOLD', 'TRIM_L', 'TRIM_R') for s in states), \
        '没进红块维持分支，用例前提不成立: %s' % states
    assert all(c.spd <= 12.0 for c in out), \
        'RED_HOLD 拿到高速: %s' % [(c.state, c.spd) for c in out]


@case
def s10_coast_gets_no_speedup():
    """COAST（漏检宽容）不得比上一拍更快，且恒 ≤12。"""
    p = mk(cap=18.0)
    trims(p, 6)
    prev = p.last_cmd.spd if p.last_cmd else 0
    out = []
    for i in range(6, 10):
        c = p.step(pkt(i, []), st(FAR_RNG), i * TICK)
        out.append(c)
        assert c.spd <= max(12.0, prev) + 1e-9, 'COAST 比上一拍更快: %.2f > %.2f' % (c.spd, prev)
        prev = c.spd
    assert any(c.state == 'COAST' for c in out), '没进 COAST，用例前提不成立: %s' % [c.state for c in out]


@case
def s11_lost_with_red_background_stops():
    """目标消失但背景仍有红物体：必须**有界**停车，不能无限续行。"""
    p = mk(cap=18.0)
    trims(p, 6)
    cmds = []
    for i in range(6, 46):
        cmds.append(p.step(red_pkt(i, CX, h=300, n=800, w=110), st(FAR_RNG), i * TICK))
    moving = [i for i, c in enumerate(cmds) if c.spd > 0]
    assert not moving or moving[-1] <= 20, \
        '目标消失后仍在前进 %d 拍（%.1fs），红块维持没有被硬截止' % (
            (moving[-1] + 1) if moving else 0, ((moving[-1] + 1) if moving else 0) * 0.1)
    assert all(c.spd == 0 for c in cmds[22:]), '2.2s 后仍输出非零速度'


@case
def s12_lost_no_red_stops_quickly():
    """目标消失且无红背景：200ms 级停车，不许靠重复帧续期。"""
    p = mk(cap=18.0)
    trims(p, 6)
    cmds = [p.step(pkt(i, []), st(FAR_RNG), i * TICK) for i in range(6, 30)]
    moving = [i for i, c in enumerate(cmds) if c.spd > 0]
    assert not moving or moving[-1] <= 4, \
        '无红背景时仍续行 %d 拍' % ((moving[-1] + 1) if moving else 0)


@case
def s13_invalid_range_and_close_still_stop():
    """距离无效 / ≤300mm：即使资格满足也必须 0（提速不得架空保护）。"""
    for bad in (None, float('nan'), 250, 300):
        p = mk(cap=18.0)
        trims(p, 6)                      # 先攒满资格
        cmds = [p.step(pkt(100 + i, [det(27, CX - OFF, 380)]), st(bad), (100 + i) * TICK)
                for i in range(5)]
        assert all(c.spd == 0 for c in cmds), \
            'rng=%r 时仍输出速度 %s' % (bad, [c.spd for c in cmds])


def main():
    fails = 0
    for fn in CASES:
        try:
            fn()
            print('  PASS  %s' % fn.__name__)
        except AssertionError as e:
            fails += 1
            print('  FAIL  %s: %s' % (fn.__name__, e))
        except Exception as e:          # noqa: BLE001
            fails += 1
            print('  ERR   %s: %s: %s' % (fn.__name__, type(e).__name__, e))
    print('-' * 70)
    print('%d/%d 通过' % (len(CASES) - fails, len(CASES)))
    return 0 if fails == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
