#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""软件导航状态机，描述导航阶段及完成/中止条件。"""
import argparse, csv, os, sys


CX_CENTER   = 640
DEADBAND    = 40          # |cx-640| <= 40 判居中          == vision_follow_ctrl.DEADBAND
NEAR_W      = 400         # 框宽 >= 400 判接近            == vision_follow_ctrl.NEAR_W
CONF_MIN    = 40          #                            == vision_follow_ctrl.CONF_MIN
VIS_TIMEOUT_MS = 500      #                            == vis_uart_bridge.VIS_TIMEOUT_MS
RANGE_STOP_MM  = 300      # 前方 < 300mm 避障停车（任务三优先级）
JUMP_PX     = 300
ACCEPT_N    = 3
SEARCH_SPIN_SPD = 20
TURN_SPD    = 35
FWD_SPD     = 45
KEEPALIVE_MS = 300        # 无变化时每 300ms 重发（< gate 500ms 新鲜度，防超时停车）

DIR_STOP, DIR_FWD, DIR_BACK, DIR_LEFT, DIR_RIGHT, DIR_SPIN_L, DIR_SPIN_R = 0, 1, 2, 3, 4, 5, 6
DIR_NAME = {0: 'STOP', 1: 'FWD', 2: 'BACK', 3: 'LEFT', 4: 'RIGHT', 5: 'SPIN_L', 6: 'SPIN_R'}
CMD_MOTION = 0x01

ST = ['SEARCH', 'ALIGN_LEFT', 'ALIGN_RIGHT', 'APPROACH', 'ARRIVED',
      'LOST', 'OBSTACLE_STOP', 'ESTOP']


# 协议使用CRC-8多项式0x07；字段顺序需与FPGA解析器一致。
def crc8(d):
    c = 0
    for b in d:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def motion_frame(seq, direction, speed):
    """CMD_MOTION(0x01)，LEN=3：DATA = dir(1), speed(1)"""
    body = bytes([3, seq & 0xFF, CMD_MOTION, direction & 0xFF, speed & 0xFF])
    return b'\xA5\x5A' + body + bytes([crc8(body)])


def check_frame(f):
    """返回 (ok, [问题列表])，检查头/LEN/CMD/DATA 范围/CRC"""
    errs = []
    if len(f) < 3:
        errs.append('帧过短 %d' % len(f))
        return False, errs
    if len(f) != f[2] + 5:
        errs.append('长度 %d != LEN(%d)+5=%d' % (len(f), f[2], f[2] + 5))
        return False, errs
    if f[0] != 0xA5 or f[1] != 0x5A:
        errs.append('帧头错 %02X%02X' % (f[0], f[1]))
    if f[2] != 3:
        errs.append('LEN=%d != 3' % f[2])
    if f[4] != CMD_MOTION:
        errs.append('CMD=%02X != 01' % f[4])
    if f[5] not in DIR_NAME:
        errs.append('dir=%d 越界' % f[5])
    if f[6] > 60:
        errs.append('speed=%d 超上限 60' % f[6])
    if crc8(f[2:-1]) != f[-1]:
        errs.append('CRC 错 %02X != %02X' % (crc8(f[2:-1]), f[-1]))
    return (not errs), errs


class NavFsm:
    def __init__(self):
        self.state = 'SEARCH'
        self.seq = 1
        self.last_cx = None
        self.accept_cnt = 0
        self.lost_ms = 0
        self.estop_latched = False
        self.log = []
        self.scenes = []
        self.frames = []
        self.last_dir = None
        self.last_spd = None
        self.last_send_ms = -10 ** 9
        self.dup_suppressed = 0

    def step(self, vis, range_mm, s1, now_ms, force_send=False):
        if s1:
            self.estop_latched = True
        if self.estop_latched:
            return self._emit('ESTOP', DIR_STOP, 0, now_ms, 'S1_latched', force_send)
        if range_mm is not None and range_mm < RANGE_STOP_MM:
            return self._emit('OBSTACLE_STOP', DIR_STOP, 0, now_ms,
                              'range=%dmm' % range_mm, force_send)

        valid = False
        vis_ok = False
        if vis is not None:
            cls_, cx, cy, w, h, conf = vis
            vis_ok = (cls_ != 0 and conf >= CONF_MIN and w > 0 and h > 0)
            if vis_ok and self.last_cx is not None and abs(cx - self.last_cx) > JUMP_PX:
                self.log.append('  [DROP] cx 跳变 %d->%d，丢弃本帧并重建基线' % (self.last_cx, cx))
                self.last_cx = cx
                vis_ok = False
            if vis_ok:
                self.accept_cnt = min(self.accept_cnt + 1, 99)
                self.last_cx = cx
            valid = vis_ok and self.accept_cnt >= ACCEPT_N
            if valid:
                self.lost_ms = 0

        if not valid:
            if vis is None or not vis_ok:
                self.accept_cnt = 0
                self.lost_ms += 100
            if self.lost_ms >= VIS_TIMEOUT_MS:
                if self.lost_ms >= VIS_TIMEOUT_MS + 100:
                    return self._emit('SEARCH', DIR_SPIN_L, SEARCH_SPIN_SPD, now_ms,
                                      'lost %dms' % self.lost_ms, force_send)
                self.state = 'LOST'
                return self._emit('LOST', DIR_STOP, 0, now_ms,
                                  'lost %dms' % self.lost_ms, force_send)
            return self._emit(self.state if self.state in
                              ('APPROACH', 'ALIGN_LEFT', 'ALIGN_RIGHT', 'ARRIVED')
                              else 'SEARCH', DIR_STOP, 0, now_ms, 'no_valid_vis', force_send)

        cx, w = vis[1], vis[3]
        err = cx - CX_CENTER
        if w >= NEAR_W:
            return self._emit('ARRIVED', DIR_STOP, 0, now_ms, 'w=%d' % w, force_send)
        if err < -DEADBAND:
            return self._emit('ALIGN_LEFT', DIR_LEFT, TURN_SPD, now_ms, 'err=%d' % err, force_send)
        if err > DEADBAND:
            return self._emit('ALIGN_RIGHT', DIR_RIGHT, TURN_SPD, now_ms, 'err=%d' % err, force_send)
        return self._emit('APPROACH', DIR_FWD, FWD_SPD, now_ms, 'err=%d w=%d' % (err, w), force_send)

    def _emit(self, state, direction, speed, now_ms, reason='', force_send=False):
        """变化即发；无变化时按 KEEPALIVE_MS 重发（< gate 500ms 新鲜度）；否则抑制"""
        self.state = state
        unchanged = (direction == self.last_dir and speed == self.last_spd)
        if unchanged and not force_send and (now_ms - self.last_send_ms) < KEEPALIVE_MS:
            self.dup_suppressed += 1
            self.log.append('[%6dms] %-13s dir=%d spd=%-2d  (保持，不重发)  (%s)'
                            % (now_ms, state, direction, speed, reason))
            return None
        f = motion_frame(self.seq, direction, speed)
        ok, errs = check_frame(f)
        self.frames.append((now_ms, state, direction, speed, f, ok, errs))
        self.seq = self.seq % 250 + 1
        self.last_dir, self.last_spd, self.last_send_ms = direction, speed, now_ms
        self.log.append('[%6dms] %-13s dir=%d spd=%-2d  frame=%s%s  (%s)'
                        % (now_ms, state, direction, speed, f.hex(' '),
                           '' if ok else '  X' + ';'.join(errs), reason))
        return f

    def reset_estop(self):
        self.estop_latched = False


SCENES = [
    ('S1 无目标', [(None, 999, 0, 100)] * 12, 'SEARCH'),
    ('S2 偏左 cx=200', [((1, 200, 360, 150, 80, 90), 999, 0, 100)] * 8, 'ALIGN_LEFT'),
    ('S3 居中远 w=150', [((1, 640, 360, 150, 80, 90), 999, 0, 100)] * 8, 'APPROACH'),
    ('S4 接近 w=450', [((1, 640, 360, 450, 80, 90), 999, 0, 100)] * 8, 'ARRIVED'),
    ('S5 偏右 cx=1050', [((1, 1050, 360, 150, 80, 90), 999, 0, 100)] * 8, 'ALIGN_RIGHT'),
    ('S6 丢失 1.0s', [(None, 999, 0, 100)] * 10, 'LOST'),
    ('S7 近障 200mm', [((1, 640, 360, 150, 80, 90), 200, 0, 100)] * 5, 'OBSTACLE_STOP'),
    ('S8 跳变 cx640->1100', [((1, 640, 360, 150, 80, 90), 999, 0, 100)] * 3
                            + [((1, 1100, 360, 150, 80, 90), 999, 0, 100)] * 2, 'SEARCH'),
    ('S9 S1 急停', [((1, 640, 360, 150, 80, 90), 999, 1, 100)] * 3, 'ESTOP'),
    ('S10 松S1保持锁存', [((1, 640, 360, 150, 80, 90), 999, 0, 100)] * 3, 'ESTOP'),
    ('S11 人工解锁恢复', [((1, 640, 360, 150, 80, 90), 999, 0, 100)] * 5, 'APPROACH'),
]


def run():
    print('=' * 78)
    print('053/055 · 任务六导航状态机离线回放（dry-run，绝不打开串口）')
    print('=' * 78)
    fsm = NavFsm()
    t = 0
    for name, seq, expect in SCENES:
        print('\n--- %s ---' % name)
        if name.startswith('S11') and fsm.estop_latched:
            fsm.reset_estop()
            print('  [RESET] 人工解除 ESTOP 锁存')
        start_t = t
        for vis, rng, s1, dt in seq:
            if name.startswith('S11') and fsm.estop_latched:
                fsm.reset_estop()
            fsm.step(vis, rng, s1, t)
            t += dt
        last = fsm.log[-1]
        print('  ' + last)
        ok_state = (fsm.state == expect) or (expect == 'LOST' and fsm.state in ('LOST', 'SEARCH'))
        fsm.scenes.append({
            'scene': name, 'expect': expect, 'final_state': fsm.state,
            'verdict': 'PASS' if ok_state else 'FAIL',
            'duration_ms': t - start_t, 'last_frame': last,
        })
    print('\n' + '=' * 78)

    bad = [x for x in fsm.frames if not x[5]]
    print('协议检查：共发帧 %d，非法帧 %d' % (len(fsm.frames), len(bad)))
    for x in bad[:5]:
        print('  X t=%d %s %s' % (x[0], x[1], ';'.join(x[6])))
    seqs = [x[4][3] for x in fsm.frames]
    gaps = [(seqs[i], seqs[i + 1]) for i in range(len(seqs) - 1)
            if not (seqs[i + 1] == seqs[i] + 1 or (seqs[i] == 250 and seqs[i + 1] == 1))]
    print('序号连续性：%d 处跳变（共 %d 帧）' % (len(gaps), len(seqs)))
    print('保活抑制重复帧：%d 次（间隔 < %dms 不重发）' % (fsm.dup_suppressed, KEEPALIVE_MS))

    print('\n%-22s %-14s %-14s %-6s %s' % ('场景', '期望', '实际', '判定', '时长'))
    print('-' * 78)
    allpass = True
    for s in fsm.scenes:
        print('%-22s %-14s %-14s %-6s %dms'
              % (s['scene'], s['expect'], s['final_state'], s['verdict'], s['duration_ms']))
        allpass = allpass and (s['verdict'] == 'PASS')

    with open('nav_fsm_replay.log', 'w', encoding='utf-8') as f:
        f.write('\n'.join(fsm.log))
    with open('nav_fsm_scenarios.csv', 'w', newline='', encoding='utf-8-sig') as f:
        w = csv.DictWriter(f, fieldnames=['scene', 'expect', 'final_state', 'verdict',
                                          'duration_ms', 'last_frame'])
        w.writeheader()
        w.writerows(fsm.scenes)

    print('-' * 78)
    npass = sum(1 for s in fsm.scenes if s['verdict'] == 'PASS')
    print('场景通过: %d/%d' % (npass, len(fsm.scenes)))
    print('协议非法帧: %d' % len(bad))
    print('串口副作用: 无（未 import serial、未 open 任何设备）')
    ok = allpass and not bad and 'serial' not in sys.modules
    print('总判定: %s' % ('PASS' if ok else 'FAIL'))
    print('输出: nav_fsm_replay.log / nav_fsm_scenarios.csv')
    return ok


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--replay', action='store_true')
    ap.add_argument('--verify', action='store_true', help='055 阶段三：回放+协议检查+场景表')
    ap.add_argument('--dev', default=None)
    a = ap.parse_args()
    if a.dev:
        print('禁止：真实发送在本阶段被禁用（055 阶段三.3：dry-run 不得打开真实串口）。')
        sys.exit(2)
    ok = run()
    sys.exit(0 if ok else 1)
