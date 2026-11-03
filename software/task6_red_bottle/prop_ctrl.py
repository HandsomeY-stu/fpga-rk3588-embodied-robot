#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""图像横向误差和距离的跟随控制器，提供迟滞纠偏、速度曲线和丢失处理。"""

DIR_STOP, DIR_FWD, DIR_BACK, DIR_LEFT, DIR_RIGHT, DIR_SPIN_L, DIR_SPIN_R = 0, 1, 2, 3, 4, 5, 6
DIR_NAME = {0: 'STOP', 1: 'FWD', 2: 'BACK', 3: 'LEFT', 4: 'RIGHT', 5: 'SPIN_L', 6: 'SPIN_R'}


class Cmd(object):
    __slots__ = ('dir', 'spd', 'state', 'reason', 'err', 'w', 'smooth_cx')

    def __init__(self, d, spd, state, reason='', err=None, w=None, smooth_cx=None):
        self.dir, self.spd, self.state, self.reason = d, spd, state, reason
        self.err, self.w, self.smooth_cx = err, w, smooth_cx

    def __repr__(self):
        return 'Cmd(dir=%d %s, spd=%d, %s, %s)' % (
            self.dir, DIR_NAME.get(self.dir, '?'), self.spd, self.state, self.reason)

    def key(self):
        return (self.dir, self.spd)


class PropFollow(object):
    """图像误差到差速运动命令的控制器，带迟滞和速度变化率限制。"""

    def __init__(self,
                 cx_center=520,
                 near_w=600,        # 到达阈值：框宽 >= near_w 视为到目标附近
                 far_w=140,         # 远距离下限：w <= far_w 用最高前进速度
                 db_in=90,          # 进入"居中"的阈值 px（90px≈4.3°）
                 db_out=170,        # 退出"居中"的阈值 px（170px≈8.2°）
                 err_span=500,      # 偏差达到该值时转向速度到顶
                 spd_fwd_far=22,    # 远距离前进速度 %
                 spd_fwd_near=10,   # 近距离前进速度 %（不能再低，低于 8% 电机可能不转）
                 spd_spin_min=9,    # 最小转向速度 %（低于此值轮子不转，转了也白转）
                 spd_spin_max=24,   # 最大转向速度 %（中继硬限速 25）
                 conf_min=40,
                 accept_n=2,        # 连续确认帧数
                 jump_px=300,       # cx 跳变过滤
                 grace_ms=300,      # 丢帧保持窗：短于此时长不打断动作
                 hold_ms=700,       # 保持窗结束后先停车等待（等检测恢复）
                 sweep_ms=900,      # 小范围扫视：单向摆动时长（900ms ≈ 24°，现场需按 --sweep-deg 标定）
                 sweep_cycles=2,    # 摆几轮还没找到 → 转全圈搜索
                 search_switch_ms=3000,  # 全圈搜索每 3s 换向
                 sweep_spd=0,
                 search_spd=0,
                 search_when_never_ms=0,
                 damp_gain=0.55,    # 检测到快速换向时的增益衰减
                 damp_ms=600,
                 range_stop_mm=300,
                 range_max_mm=4000,     # 超声波有效上限（car_defines.vh: US_MAX_MM=4000）
                 follow=False,      # 跟随模式：到达后不彻底停，保持距离
                 mirror=True,
                 burst=False,
                 burst_pct=14,
                 burst_settle_ms=420,
                 burst_min_ms=100,
                 burst_max_ms=700,
                 burst_px_per_100ms=None,  # None ⇒ 按 burst_pct 自动换算（标定常数见下）
                 burst_start_ms=275,
                 trim=True,
                 spin_at=300,       # |err| 超过此值才改用原地转（px）
                 d_far_mm=1500,     # 距离 >= 此值用最高前进速度
                 d_near_mm=500,     # 距离 <= 此值用最低前进速度（线性插值）
                 close_h_frac=0.90, # 框高 ≥ 此比例×720 ⇒ 判定"已到目标跟前"（0=禁用）
                 close_h_release=0.78,  # 迟滞：框高 < 此比例才解除"已近"状态
                 trim_spd_cap=12,   # 差速纠偏时的速度上限%（限制转向速率，防来回摆）
                 trim_full_px=250,
                 slew_pct_s=80.0,   # 前进速度变化率上限 %/s（0=关闭）。防 16%↔55% 硬跳变
                 rng_gate=True,
                 rng_gate_mm=400,   # 单次跳变超过此值视为可疑（下限 400mm）
                 rng_confirm=2,     # 可疑读数需连续出现 N 次才采信
                 smooth_n=3):       # cx 中位数平滑窗口
        self.cx_center = cx_center
        self.near_w = near_w
        self.far_w = far_w
        self.db_in, self.db_out = db_in, db_out
        self.err_span = err_span
        self.spd_fwd_far, self.spd_fwd_near = spd_fwd_far, spd_fwd_near
        self.spd_spin_min, self.spd_spin_max = spd_spin_min, spd_spin_max
        self.conf_min = conf_min
        self.accept_n = accept_n
        self.jump_px = jump_px
        self.grace_ms = grace_ms
        self.hold_ms = hold_ms
        self.sweep_ms = sweep_ms
        self.sweep_cycles = sweep_cycles
        self.search_switch_ms = search_switch_ms
        self.search_when_never_ms = search_when_never_ms


        self.sweep_spd  = sweep_spd  if sweep_spd  > 0 else (spd_spin_min + 3)
        self.search_spd = search_spd if search_spd > 0 else (spd_spin_min + 6)
        self.damp_gain = damp_gain
        self.damp_ms = damp_ms
        self.range_stop_mm = range_stop_mm
        self.range_max_mm = range_max_mm
        self.follow = follow
        self.smooth_n = smooth_n


        self.mirror = mirror


        self.burst = burst
        self.burst_pct = burst_pct
        self.burst_settle_ms = burst_settle_ms
        self.burst_min_ms = burst_min_ms
        self.burst_max_ms = burst_max_ms


        self.burst_px_per_100ms = (burst_px_per_100ms if burst_px_per_100ms
                                   else 18.5 * burst_pct)
        self.burst_start_ms = burst_start_ms
        self.burst_until_ms = -10 ** 9
        self.settle_until_ms = -10 ** 9
        self.burst_dir = DIR_STOP

        self.trim = trim
        self.spin_at = spin_at
        self.d_far_mm = d_far_mm
        self.d_near_mm = d_near_mm
        self.close_h_frac = close_h_frac
        self.close_h_release = close_h_release
        self._size_close = False
        self._h_buf = []        # 框高中位数窗口（防单帧误判"占满画面"）
        self.trim_spd_cap = trim_spd_cap
        self.trim_full_px = trim_full_px

        self.slew_pct_s = slew_pct_s
        self.last_fwd_spd = 0.0
        self.last_fwd_ms = None

        self.rng_gate = rng_gate
        self.rng_gate_mm = rng_gate_mm
        self.rng_confirm = rng_confirm
        self.last_rng = None
        self.rng_suspect = 0


        self.w_ref = None
        self.d_ref = None
        self.last_dist_src = 'none'

        self.estop = False
        self.state = 'SEARCH'

        self.cx_hist = []
        self.hit_cnt = 0
        self.ever_valid = False
        self.last_cx = None
        self.last_side = DIR_SPIN_L      # 最后一次看到目标时该往哪边转
        self.last_valid_ms = -10 ** 9
        self.lost_since_ms = None
        self.centered = False
        self.turn_sign = None
        self.turn_change_ms = -10 ** 9
        self.flip_ms = -10 ** 9
        self.last_cmd = Cmd(DIR_STOP, 0, 'INIT')
        self.stats = {'frames': 0, 'valid': 0, 'blips': 0, 'jumps': 0,
                      'turns': 0, 'flips': 0, 'sweeps': 0, 'searches': 0,
                      'bursts': 0, 'trims': 0, 'dir_changes': 0}


    def step(self, vis, now_ms, range_mm=None, s1=0):
        """vis = (cls, cx, cy, w, h, conf) 或 None；now_ms 单调递增毫秒"""
        self.stats['frames'] += 1
        if s1:
            self.estop = True
        if self.estop:
            return self._emit(DIR_STOP, 0, 'ESTOP', 'S1_latched')

        cx, w, err = self._ingest(vis, now_ms)
        if cx is None:


            if range_mm is not None and 20 <= range_mm < self.range_max_mm \
                    and range_mm < self.range_stop_mm:
                return self._emit(DIR_STOP, 0, 'OBSTACLE_STOP', 'range=%dmm' % range_mm)
            return self._lost(now_ms)


        d_mm, d_src = self._fuse_distance(w, range_mm)
        self.last_dist_src = d_src


        if d_mm is not None:
            reached = (d_mm <= self.range_stop_mm)
            why = '%s=%dmm' % (d_src, d_mm)
        else:
            reached = (w >= self.near_w)
            why = 'w=%d' % w


        h_px = 0
        if vis is not None and len(vis) > 4 and vis[4]:


            self._h_buf.append(int(vis[4]))
            if len(self._h_buf) > 5:
                self._h_buf.pop(0)
            h_px = sorted(self._h_buf)[len(self._h_buf) // 2]
        if self.close_h_frac > 0 and h_px:
            if h_px >= self.close_h_frac * 720.0:
                self._size_close = True
            elif h_px < self.close_h_release * 720.0:
                self._size_close = False
        if self._size_close:
            reached = True
            why = 'h=%d(占画面)' % h_px

        if reached:
            if not self.follow:
                return self._emit(DIR_STOP, 0, 'ARRIVED', why, err, w, cx)


            ae_r = abs(err)


            if self._size_close:


                return self._emit(DIR_STOP, 0, 'HOLD',
                                  'hold %s err=%+d(已到跟前不再校)' % (why, err),
                                  err, w, cx)
            if ae_r <= self.db_out:
                return self._emit(DIR_STOP, 0, 'HOLD',
                                  'hold %s err=%+d' % (why, err), err, w, cx)
            if self.burst:
                return self._burst_turn(err, ae_r, now_ms, w, cx)
            d = self._turn_dir(err)
            self.stats['turns'] += 1
            return self._emit(d, self.spd_spin_min, 'HOLD_ALIGN',
                              'hold %s err=%+d' % (why, err), err, w, cx)


        ae = abs(err)
        if self.centered:
            if ae > self.db_out:
                self.centered = False
        else:
            if ae <= self.db_in:
                self.centered = True

        if self.centered:
            spd = self._slew(self._fwd_speed(w, d_mm), now_ms)
            return self._emit(DIR_FWD, spd, 'APPROACH',
                              'err=%+d w=%d' % (err, w), err, w, cx)


        if self.trim and ae <= self.spin_at:
            d = self._turn_dir(err)          # SPIN_L / SPIN_R
            d2 = DIR_LEFT if d == DIR_SPIN_L else DIR_RIGHT


            if ae <= self.trim_full_px:
                cap = self.trim_spd_cap
            else:
                span_t = max(1.0, float(self.err_span - self.trim_full_px))
                f_t = min(1.0, (ae - self.trim_full_px) / span_t)
                cap = self.trim_spd_cap + (self.spd_spin_max - self.trim_spd_cap) * f_t
            spd = self._slew(min(self._fwd_speed(w, d_mm), cap), now_ms)
            self.stats['trims'] += 1
            return self._emit(d2, spd, 'TRIM_L' if d2 == DIR_LEFT else 'TRIM_R',
                              'err=%+d ae=%d spd=%d' % (err, ae, spd), err, w, cx)


        if self.burst:
            return self._burst_turn(err, ae, now_ms, w, cx)

        d = self._turn_dir(err)
        self._update_flip(d, now_ms)
        spd = self._spin_speed(ae, now_ms)
        self.stats['turns'] += 1
        return self._emit(d, spd, 'ALIGN_L' if d == DIR_SPIN_L else 'ALIGN_R',
                          'err=%+d ae=%d' % (err, ae), err, w, cx)

    def _burst_turn(self, err, ae, now_ms, w, cx):
        """脉冲转向：转 burst_ms 毫秒 → 停 burst_settle_ms 毫秒 → 重新测量"""


        if now_ms < self.burst_until_ms:
            self.stats['turns'] += 1
            return self._emit(self.burst_dir, self.burst_pct,
                              'PULSE_L' if self.burst_dir == DIR_SPIN_L else 'PULSE_R',
                              'err=%+d ae=%d left=%dms'
                              % (err, ae, int(self.burst_until_ms - now_ms)),
                              err, w, cx)

        if now_ms < self.settle_until_ms:
            return self._emit(DIR_STOP, 0, 'SETTLE',
                              'settle %dms' % int(self.settle_until_ms - now_ms),
                              err, w, cx)


        ms = self.burst_start_ms + float(ae) / max(1.0, self.burst_px_per_100ms) * 100.0
        ms = max(self.burst_min_ms, min(self.burst_max_ms, ms))
        self.burst_dir = self._turn_dir(err)
        self.burst_until_ms = now_ms + ms
        self.settle_until_ms = self.burst_until_ms + self.burst_settle_ms
        self.stats['bursts'] += 1
        self.stats['turns'] += 1
        return self._emit(self.burst_dir, self.burst_pct,
                          'PULSE_L' if self.burst_dir == DIR_SPIN_L else 'PULSE_R',
                          'err=%+d ae=%d pulse=%dms' % (err, ae, int(ms)),
                          err, w, cx)

    def reset_estop(self):
        self.estop = False


    def _ingest(self, vis, now_ms):
        """返回 (smooth_cx, w, err)；本帧不可用则 (None, None, None)"""
        if vis is None:
            return None, None, None
        try:
            cls_, cx, cy, w, h, conf = vis
        except Exception:
            return None, None, None
        if cls_ == 0 or conf < self.conf_min or w <= 0 or h <= 0:
            return None, None, None

        if self.last_cx is not None and abs(cx - self.last_cx) > self.jump_px:
            self.stats['jumps'] += 1
            self.last_cx = cx
            return None, None, None
        self.last_cx = cx
        self.cx_hist.append(cx)
        if len(self.cx_hist) > self.smooth_n:
            self.cx_hist.pop(0)
        smooth = sorted(self.cx_hist)[len(self.cx_hist) // 2]


        self.lost_since_ms = None
        self.hit_cnt = min(self.hit_cnt + 1, 99)
        if self.hit_cnt < self.accept_n:

            return None, None, None
        self.last_valid_ms = now_ms
        self.ever_valid = True
        self.stats['valid'] += 1
        err = smooth - self.cx_center
        if err != 0:
            self.last_side = self._turn_dir(err)
        return smooth, w, err

    def _turn_dir(self, err):
        """偏差符号 → 转向方向（考虑画面是否左右镜像）"""
        if self.mirror:
            return DIR_SPIN_L if err > 0 else DIR_SPIN_R
        return DIR_SPIN_L if err < 0 else DIR_SPIN_R

    # 融合前先检查测距有效性和跳变；原始近距离保护仍独立执行。
    def _fuse_distance(self, w, range_mm):
        """融合前先检查测距有效性和跳变；原始近距离保护仍独立执行。"""
        rng_ok = (range_mm is not None and
                  20 <= range_mm < self.range_max_mm)


        suspicious = False
        if self.rng_gate and rng_ok and self.last_rng is not None:
            step = abs(range_mm - self.last_rng)
            tol = max(float(self.rng_gate_mm), 0.4 * self.last_rng)
            if step > tol:
                self.rng_suspect += 1
                if self.rng_suspect <= self.rng_confirm:
                    rng_ok = False
                    suspicious = True
            else:
                self.rng_suspect = 0
        if rng_ok:
            self.last_rng = range_mm
            self.rng_suspect = 0

        d_vis = None
        if self.w_ref and self.d_ref and w and w > 0:
            d_vis = self.d_ref * self.w_ref / float(w)

        if rng_ok:

            if w and w > 0:
                self.w_ref = w
                self.d_ref = range_mm
            return range_mm, 'range'
        if d_vis is not None:
            return int(round(d_vis)), 'visual'
        return None, 'none'

    # 按经过时间约束速度变化率，避免不同控制调用周期改变加速度。
    def _slew(self, spd, now_ms):
        """按经过时间约束速度变化率，避免不同控制调用周期改变加速度。"""
        target = float(spd)
        if self.slew_pct_s <= 0:
            self.last_fwd_spd, self.last_fwd_ms = target, now_ms
            return int(round(target))
        if self.last_fwd_ms is None:
            self.last_fwd_ms = now_ms - 1        # 让第一次也走斜坡，从 0 平滑起步
        dt = max(0.001, (now_ms - self.last_fwd_ms) / 1000.0)
        max_step = self.slew_pct_s * dt
        d = target - self.last_fwd_spd
        if d > max_step:
            target = self.last_fwd_spd + max_step
        elif d < -max_step:
            target = self.last_fwd_spd - max_step
        self.last_fwd_spd, self.last_fwd_ms = target, now_ms
        return int(round(target))

    # 按距离从近距低速插值到远距速度；不能用框存在替代距离有效性。
    def _fwd_speed(self, w, d_mm=None):
        """按距离从近距低速插值到远距速度；不能用框存在替代距离有效性。"""
        if d_mm is not None:
            if d_mm >= self.d_far_mm:
                return self.spd_fwd_far
            if d_mm <= self.d_near_mm:
                return self.spd_fwd_near
            f = (d_mm - self.d_near_mm) / float(self.d_far_mm - self.d_near_mm)
            return int(round(self.spd_fwd_near + f * (self.spd_fwd_far - self.spd_fwd_near)))
        if w <= self.far_w:
            return self.spd_fwd_far
        if w >= self.near_w:
            return self.spd_fwd_near
        f = (w - self.far_w) / float(self.near_w - self.far_w)
        return int(round(self.spd_fwd_far - f * (self.spd_fwd_far - self.spd_fwd_near)))

    def _spin_speed(self, ae, now_ms):
        span = max(1.0, float(self.err_span - self.db_out))
        frac = min(1.0, max(0.0, (ae - self.db_out) / span))
        g = 1.0
        if now_ms - self.flip_ms < self.damp_ms:
            g = self.damp_gain          # 刚发生过快速换向 ⇒ 降增益，防来回摆
        spd = self.spd_spin_min + (self.spd_spin_max - self.spd_spin_min) * frac * g
        return int(round(spd))

    def _update_flip(self, d, now_ms):
        if self.turn_sign is not None and d != self.turn_sign:
            if now_ms - self.turn_change_ms < self.damp_ms:
                self.flip_ms = now_ms
                self.stats['flips'] += 1
        self.turn_sign = d
        self.turn_change_ms = now_ms

    def _lost(self, now_ms):
        if self.lost_since_ms is None:
            self.lost_since_ms = now_ms
        dt = now_ms - self.lost_since_ms


        if not self.ever_valid:
            if self.search_when_never_ms <= 0 or dt < self.search_when_never_ms:
                return self._emit(DIR_STOP, 0, 'GRACE', 'no_vis_yet %dms' % dt)

        if dt < self.grace_ms:
            self.stats['blips'] += 1
            last = self.last_cmd
            if last.dir in (DIR_FWD, DIR_SPIN_L, DIR_SPIN_R) and \
                    last.state not in ('LOST', 'ESTOP', 'OBSTACLE_STOP'):
                return self._emit(last.dir, last.spd, 'GRACE', 'blip %dms' % dt,
                                  last.err, last.w, last.smooth_cx)
            return self._emit(DIR_STOP, 0, 'GRACE', 'blip %dms (last=stop)' % dt)

        if dt < self.grace_ms + self.hold_ms:
            self.hit_cnt = 0
            return self._emit(DIR_STOP, 0, 'LOST', 'lost %dms' % dt)
        t = dt - self.grace_ms - self.hold_ms
        cycle = 4 * self.sweep_ms
        if t < self.sweep_cycles * cycle:
            self.stats['sweeps'] += 1
            ph = t % cycle
            if ph < self.sweep_ms or ph >= 3 * self.sweep_ms:
                d = self.last_side                      # 朝最后已知方向
            else:
                d = DIR_SPIN_R if self.last_side == DIR_SPIN_L else DIR_SPIN_L
            return self._emit(d, self.sweep_spd, 'SWEEP',
                              'sweep %dms side=%s' % (t, DIR_NAME[self.last_side]))

        self.stats['searches'] += 1
        t2 = t - self.sweep_cycles * cycle
        d = DIR_SPIN_L if int(t2 / self.search_switch_ms) % 2 == 0 else DIR_SPIN_R
        return self._emit(d, self.search_spd, 'SEARCH', 'full_search %dms' % t2)

    def _emit(self, d, spd, state, reason, err=None, w=None, cx=None):
        if self.last_cmd.dir != d:
            self.stats['dir_changes'] += 1


        if d not in (DIR_FWD, DIR_LEFT, DIR_RIGHT):
            self.last_fwd_spd = 0.0
        cmd = Cmd(d, spd, state, reason, err, w, cx)
        self.last_cmd = cmd
        self.state = state
        return cmd


if __name__ == '__main__':
    import sys
    f = PropFollow()
    t = 0
    print('--- 静态居中（w=300）应持续 FWD ---')
    for _ in range(3):
        c = f.step((1, 520, 360, 300, 380, 90), t); t += 77
        print('  %s' % c)
    print('--- 偏右 err=+180 应 SPIN_R 中速 ---')
    for _ in range(3):
        c = f.step((1, 700, 360, 300, 380, 90), t); t += 77
        print('  %s' % c)
    print('--- 偏右 err=+420 应 SPIN_R 高速 ---')
    for _ in range(2):
        c = f.step((1, 940, 360, 300, 380, 90), t); t += 77
        print('  %s' % c)
    print('--- 丢 2 帧应 GRACE 保持 ---')
    for _ in range(2):
        c = f.step(None, t); t += 77
        print('  %s' % c)
    print('--- 丢 1.2s 应 LOST 停车 ---')
    for _ in range(8):
        c = f.step(None, t); t += 77
    print('  %s' % c)
    print('--- 丢 3s 应 SWEEP ---')
    for _ in range(25):
        c = f.step(None, t); t += 77
    print('  %s' % c)
    sys.exit(0)
