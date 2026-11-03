#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线回归测试：检查qualify_phase相关逻辑和异常处理。"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from follow_policy import FollowPolicy
from test_support import det, pkt, st

TICK = 100.0
BAG = (27, 31, 33)
CASES = []


def case(fn):
    CASES.append(fn)
    return fn


class Seq:
    """自动递增 frame_id 与时间，避免"同 fid 连喂"把状态卡在 *_DUPLICATE。"""

    def __init__(self, policy, start_fid=1):
        self.p = policy
        self.fid = start_fid - 1

    def feed(self, conf=.60, tlm_age=40, rng=1200.0, age=30, producer=1, cx=700.0,
             red_cx=None, dets=None):
        self.fid += 1
        dets = [det(27, cx, 380, conf=conf)] if dets is None else dets
        pk = pkt(self.fid, dets, red_cx=red_cx, red_n=(800 if red_cx else 0),
                 red_w=(200 if red_cx else 0), age=age, producer=producer)
        self.p.step(pk, st(rng, tlm_age=tlm_age), self.fid * TICK)
        return self.p.last_cmd

    def warm(self, n=5):
        """锁定(3 帧确认)+资格(3 个新鲜帧) 至少需要 5 帧。"""
        for _ in range(n):
            self.feed()

    def state(self):
        return self.p.debug.get('qualify_state'), self.p.debug.get('qualify_block')


@case
def q01_tlm_phase_history_kept():
    """4Hz 遥测相位下滑动：保留视觉历史必须让 ok 拍数显著增加。"""
    def sim(keep_ms, ticks=40, period=267.0):
        p = FollowPolicy(target_cls=BAG, qual_hist_keep_ms=keep_ms)
        s = Seq(p)
        ok = 0
        for i in range(ticks):
            s.feed(tlm_age=(i * TICK) % period)
            if p.debug.get('qualify_state') == 'ok':
                ok += 1
        return ok, p

    ok0, _ = sim(0)
    ok1, p1 = sim(300)
    assert ok1 >= 10, '保留历史后 ok 拍只有 %d（期望 ≥10）' % ok1
    assert ok1 > ok0 * 2, '没有明显改善: keep=0 → %d, keep=300 → %d' % (ok0, ok1)
    assert p1.stats['qual_kept'] > 0, '没有任何"保留历史"的拍'


@case
def q02_tlm_block_still_downgrades():
    """关键安全边界：tlm_age 超限那一拍**必须立即降速**，不能靠历史继续提速。"""
    p = FollowPolicy(target_cls=BAG, qual_hist_keep_ms=300)
    s = Seq(p)
    s.warm(5)
    assert s.state()[0] == 'ok', '预热未取得资格: %s' % (s.state(),)
    s.feed(tlm_age=250)
    _, blk = s.state()
    assert blk == 'tlm_age', '阻断原因应为 tlm_age，实际 %s' % blk
    assert p._qualified is False, 'tlm_age 超限仍保持资格'
    assert len(p._qual_hist) >= 1, '窗口内的视觉历史被整段清掉了'


@case
def q03_history_expires_after_300ms():
    """遥测中断 500ms 后再恢复：旧框必须过期，不得立刻换回资格。"""
    p = FollowPolicy(target_cls=BAG, qual_hist_keep_ms=300)
    s = Seq(p)
    s.warm(5)
    assert s.state()[0] == 'ok'

    for _ in range(5):
        s.fid += 1
        p.step(pkt(s.fid, [det(27, 700, 380)]),
               dict(tlm_ok=False, tlm_age_ms=999, fpga_mode=0, rng=1200.0,
                    yaw=90.0, velL=0, velR=0), s.fid * TICK)
    s.feed()
    assert s.state()[0] != 'ok', '旧视觉证据仍能换取提速资格'
    assert len(p._qual_hist) <= 1, '过期历史未淘汰: %d 条' % len(p._qual_hist)


@case
def q04_vision_stale_clears_history():
    """视觉过期是**视觉证据失效**，必须清空历史（不是保留）。"""
    p = FollowPolicy(target_cls=BAG)
    s = Seq(p)
    s.warm(5)
    s.feed(age=121)
    assert s.state()[1] == 'vision_age', s.state()
    assert p._qual_hist == [], '视觉过期后历史没清空'


@case
def q05_low_conf_clears_history():
    """conf<0.40 是视觉证据不足，必须清空历史。"""
    p = FollowPolicy(target_cls=BAG)
    s = Seq(p)
    s.warm(5)
    s.feed(conf=.35)
    assert s.state()[1] == 'conf', s.state()
    assert p._qual_hist == [], '低分帧后历史没清空'


@case
def q06_duplicate_frame_not_counted():
    """重复包（同 frame_id）不得被当成新的视觉确认帧。"""
    p = FollowPolicy(target_cls=BAG)
    s = Seq(p)
    s.warm(5)
    n0 = len(p._qual_hist)
    dup = pkt(s.fid, [det(27, 700, 380)])
    for _ in range(3):
        p.step(dup, st(1200), s.fid * TICK + 10)
    assert len(p._qual_hist) == n0, '重复帧把历史长度从 %d 涨到 %d' % (n0, len(p._qual_hist))


@case
def q07_producer_change_resets():
    """producer 变化（抓帧进程重启）⇒ 旧确认链作废。"""
    p = FollowPolicy(target_cls=BAG)
    s = Seq(p)
    s.warm(5)
    assert s.state()[0] == 'ok'
    s.feed(producer=2)
    assert len(p._qual_hist) <= 1, 'producer 变了还沿用旧历史: %d 条' % len(p._qual_hist)


@case
def q08_target_lost_clears():
    """真实目标丢失 ⇒ off/no_target，历史清空。"""
    p = FollowPolicy(target_cls=BAG)
    s = Seq(p)
    s.warm(5)
    for _ in range(3):
        s.feed(dets=[])
    assert s.state() == ('off', 'no_target'), s.state()
    assert p._qual_hist == []


@case
def q09_red_hold_blocks_qualify():
    """RED_HOLD 虚拟目标不得获得提速资格（实现不变，仅回归）。"""
    p = FollowPolicy(target_cls=BAG, red_hold_enable=True)
    s = Seq(p)
    for _ in range(3):
        s.feed(red_cx=700)
    s.feed(dets=[], red_cx=700)
    assert s.state() == ('off', 'red_hold'), s.state()


@case
def q10_stop_gap_requires_rewarm():
    """停车/撤权造成的时间空档 ⇒ 恢复后必须重新凑 3 帧，不得瞬间 ok。"""
    p = FollowPolicy(target_cls=BAG, qual_hist_keep_ms=300)
    s = Seq(p)
    s.warm(5)
    assert s.state()[0] == 'ok'
    for _ in range(5):
        s.feed(rng=250)       # 近距离 ⇒ _precheck RANGE_STOP，绕过 _req_track
    assert p._qual_hist == [], '停车期间历史没被时间窗淘汰: %d 条' % len(p._qual_hist)
    s.feed()
    assert s.state()[0] != 'ok', '停车后立刻就拿到资格'


def main():
    fails = 0
    for fn in CASES:
        try:
            fn()
            print('  PASS  %s' % fn.__name__)
        except AssertionError as e:
            fails += 1
            print('  FAIL  %s: %s' % (fn.__name__, e))
        except Exception as e:
            fails += 1
            print('  ERR   %s: %s: %s' % (fn.__name__, type(e).__name__, e))
    print('-' * 70)
    print('%d/%d 通过' % (len(CASES) - fails, len(CASES)))
    return 0 if fails == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
