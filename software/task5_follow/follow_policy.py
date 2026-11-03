"""任务五跟随策略：目标身份、可靠性判定、速度整形和最终保护按统一顺序执行。"""
import collections
import math
from prop_ctrl import PropFollow, Cmd, DIR_FWD, DIR_LEFT, DIR_RIGHT, DIR_SPIN_L, DIR_SPIN_R
from red_bottle_target import RedBottleLock, collapse_same_object

DIR_STOP = 0
FWD_FAMILY = (DIR_FWD, DIR_LEFT, DIR_RIGHT)      # 前进族：dir 1/3/4
SPIN_FAMILY = (DIR_SPIN_L, DIR_SPIN_R)           # 原地转：dir 5/6
SEARCH_STATES = ('SEARCH', 'SWEEP', 'SEEK_BEARING', 'SEEK_RED')


class FollowPolicy(object):
    """统一跟随决策层，集中处理目标可靠性、速度请求和最终安全限制。"""

    def __init__(self, cx_center=664, follow=True,
                 target_cls=(27, 31, 33), min_conf=.30, red_min=.18, grace_frames=3,
                 continue_conf=0.15, continue_iou=0.25,
                 range_stop_mm=300, close_h=0.96, d_far=1000, d_near=400,
                 fwd_far=22, fwd_near=10, max_spd=24, trim_cap=12,
                 red_min_spd=4, red_cue_spd=8,
                 red_cue=True, red_min_n=60, red_min_w=24,
                 unified_speed=False,
                 accel_pct_s=0, decel_pct_s=0,
                 semantic_grace_ms=0,
                 red_cue_after_ms=0,
                 turn_enter_px=170, turn_exit_px=90,
                 direction_confirm_frames=2,
                 motion_search_allowed=False,
                 red_follow=False,
                 vision_max_age_ms=200, tlm_max_age_ms=300,
                 predict_window_ms=200, predict_max_frac=0.75,
                 trim_test_cap=12, trim_speed_mode='off',
                 red_hold_cap=12, coast_cap=12,
                 qualify_conf=0.40, qualify_frames=3,
                 qualify_vision_age=120, qualify_tlm_age=200, qualify_rng_min=1000,
                 qual_hist_keep_ms=0,
                 red_hold_enable=True, red_hold_max_ms=800,
                 red_hold_max_frac=0.60, red_hold_min_n=200,
                 deg_per_px=0.0413, bearing_tol_deg=5.0, bearing_spd=8,
                 sweep_spd=8, search_spd=8, sweep_ms=900, search_switch_ms=3000,
                 trim_closed_loop=True, trim_progress_px=3, trim_stall_frames=8,
                 trim_max_ms=1500, trim_overshoot_px=60, trim_settle_ms=400,
                 burst_start_ms=0, burst_min_ms=100, burst_max_ms=100,
                 burst_pct=14, burst_settle=420,
                 grace_ms=300, hold_ms=700, coast_ms=0,
                 still_confirm=False, still_vel_max=15, still_frames=3,
                 search_enable=True):
        self.cx_center = cx_center
        self.follow = follow

        if isinstance(target_cls, (list, tuple, set)):
            self.target_cls = set(int(x) for x in target_cls)
        else:
            self.target_cls = {int(target_cls)}


        self.min_conf = float(min_conf)
        self.continue_conf = float(continue_conf)
        self.lock = RedBottleLock(confirm=3, red_min=red_min, min_conf=min_conf,
                                  continue_conf=continue_conf, continue_iou=continue_iou,
                                  grace_frames=grace_frames, target_cls=sorted(self.target_cls),
                                  predict_window_ms=predict_window_ms,
                                  predict_max_frac=predict_max_frac,
                                  red_hold_enable=red_hold_enable,
                                  red_hold_max_ms=red_hold_max_ms,
                                  red_hold_max_frac=red_hold_max_frac,
                                  red_hold_min_n=red_hold_min_n)

        self.range_stop_mm = float(range_stop_mm)
        self.close_h = float(close_h)
        self.d_far = float(d_far)
        self.d_near = float(d_near)

        self.fwd_far = float(fwd_far)
        self.fwd_near = float(fwd_near)
        self.max_spd = float(max_spd)
        self.trim_cap = float(trim_cap)
        self.red_min_spd = float(red_min_spd)
        self.red_cue_spd = float(red_cue_spd)

        self.red_cue = bool(red_cue)
        self.red_min_n = float(red_min_n)
        self.red_min_w = float(red_min_w)

        self.unified_speed = bool(unified_speed)
        self.accel_pct_s = float(accel_pct_s)
        self.decel_pct_s = float(decel_pct_s)
        self.semantic_grace_ms = float(semantic_grace_ms)
        self.red_cue_after_ms = float(red_cue_after_ms)
        self.turn_enter_px = float(turn_enter_px)
        self.turn_exit_px = float(turn_exit_px)
        self.direction_confirm_frames = int(direction_confirm_frames)
        self.motion_search_allowed = bool(motion_search_allowed)
        self.red_follow = bool(red_follow)

        self.vision_max_age_ms = float(vision_max_age_ms)
        self.tlm_max_age_ms = float(tlm_max_age_ms)

        self.deg_per_px = float(deg_per_px)
        self.bearing_tol_deg = float(bearing_tol_deg)
        self.bearing_spd = float(bearing_spd)

        self.search_enable = bool(search_enable)
        self.sweep_spd = float(sweep_spd)
        self.search_spd = float(search_spd)
        self.sweep_ms = float(sweep_ms)
        self.search_switch_ms = float(search_switch_ms)

        self.trim_closed_loop = bool(trim_closed_loop)
        self.trim_progress_px = float(trim_progress_px)
        self.trim_stall_frames = int(trim_stall_frames)
        self.trim_max_ms = float(trim_max_ms)
        self.trim_overshoot_px = float(trim_overshoot_px)
        self.trim_settle_ms = float(trim_settle_ms)
        self.burst_start_ms = float(burst_start_ms)
        self.burst_min_ms = float(burst_min_ms)
        self.burst_max_ms = float(burst_max_ms)
        self.burst_pct = float(burst_pct)
        self.burst_settle = float(burst_settle)
        self.grace_ms = float(grace_ms)
        self.hold_ms = float(hold_ms)
        self.coast_ms = float(coast_ms)
        self.still_confirm = bool(still_confirm)
        self.still_vel_max = float(still_vel_max)
        self.still_frames = int(still_frames)


        self.ctrl = None
        self.last_cmd = Cmd(DIR_STOP, 0, 'INIT')
        self.arrived = False
        self._spd_prev = None
        self._spd_ms = None
        self._last_lock_ms = None
        self._last_target_ms = None
        self._lost_at = None
        self._red_dir = None
        self._turn_dir_last = None
        self._turn_ms = None
        self._dir_cand = None
        self._dir_cand_frames = []
        self._bearing_yaw = None
        self._need_still = False
        self._still_cnt = 0
        self._was_locked = False
        self._trim_start = None
        self._trim_err0 = None
        self._trim_best = None
        self._trim_stall = 0
        self.trim_deadline = None
        self.settle_until = 0

        self._last_rng = None
        self._rng_suspect = 0

        self.stats = dict(samples=0, stop_precheck=0, stop_guard=0,
                          coast=0, red_cue=0, track=0, shaped=0, weak_cont=0, qual_kept=0)


        self.debug = {}

        self.trim_test_cap = float(trim_test_cap)
        self.trim_speed_mode = str(trim_speed_mode)


        self.red_hold_cap = min(50.0, float(red_hold_cap))
        self.coast_cap = min(12.0, float(coast_cap))
        self.qualify_conf = float(qualify_conf)
        self.qualify_frames = int(qualify_frames)
        self.qualify_vision_age = float(qualify_vision_age)
        self.qualify_tlm_age = float(qualify_tlm_age)
        self.qualify_rng_min = float(qualify_rng_min)


        self.qual_hist_keep_ms = min(300.0, max(0.0, float(qual_hist_keep_ms)))
        self._qual_hist = []          # [(frame_id, cx, w, t_ms)] 最近 N 个新鲜匹配帧
        self._qualified = False


        self.qual_block = collections.Counter()
        self._in_red_hold = False
        self._in_weak = False
        self._cap_used = 'global'
        self._last_gated_rng = None
        self._last_raw_err = None
        self._target_seq = 0


    def effective_config(self):
        """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
        return dict(
            target_cls=sorted(self.target_cls),
            min_conf=self.min_conf,
            continue_conf=self.continue_conf,
            continue_iou=self.lock.continue_iou,
            range_stop_mm=self.range_stop_mm, close_h=self.close_h,
            d_far=self.d_far, d_near=self.d_near,
            fwd_far=self.fwd_far, fwd_near=self.fwd_near, max_spd=self.max_spd,
            trim_cap=self.trim_cap, red_min_spd=self.red_min_spd,
            red_cue_spd=self.red_cue_spd,
            unified_speed=self.unified_speed,
            accel_pct_s=self.accel_pct_s, decel_pct_s=self.decel_pct_s,
            semantic_grace_ms=self.semantic_grace_ms,
            red_cue_after_ms=self.red_cue_after_ms,
            turn_enter_px=self.turn_enter_px, turn_exit_px=self.turn_exit_px,
            direction_confirm_frames=self.direction_confirm_frames,
            motion_search_allowed=self.motion_search_allowed,
            vision_max_age_ms=self.vision_max_age_ms,
            tlm_max_age_ms=self.tlm_max_age_ms,
            search_enable=self.search_enable,
            continuous=True,
            trim_test_cap=self.trim_test_cap,
            trim_speed_mode=self.trim_speed_mode,
            red_hold_cap=self.red_hold_cap,
            coast_cap=self.coast_cap,
            qualify_conf=self.qualify_conf,
            qualify_frames=self.qualify_frames,
            qualify_vision_age=self.qualify_vision_age,
            qualify_tlm_age=self.qualify_tlm_age,
            qualify_rng_min=self.qualify_rng_min,
            qual_hist_keep_ms=self.qual_hist_keep_ms,
        )

    def _stop(self, reason):
        self._spd_prev = None
        self._spd_ms = None
        self.last_cmd = Cmd(DIR_STOP, 0, reason)
        return self.last_cmd

    @staticmethod
    def _iou(a, b):
        if not a or not b:
            return None
        x = max(0, min(a['cx'] + a['w'] / 2, b['cx'] + b['w'] / 2)
                - max(a['cx'] - a['w'] / 2, b['cx'] - b['w'] / 2))
        y = max(0, min(a['cy'] + a['h'] / 2, b['cy'] + b['h'] / 2)
                - max(a['cy'] - a['h'] / 2, b['cy'] - b['h'] / 2))
        union = a['w'] * a['h'] + b['w'] * b['h'] - x * y
        return (x * y / union) if union > 0 else 0.0

    def _fill_candidate_debug(self, dets, chosen=None, prev=None):
        """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
        if not isinstance(dets, (list, tuple)):
            dets = []
        reasons, eligible = [], []
        for d in dets:
            if not isinstance(d, dict):
                continue
            if d.get('cls') not in self.target_cls:
                reasons.append('类别不符')
                continue
            conf = float(d.get('conf') or 0)
            red = float(d.get('red_ratio') or 0)
            if conf < self.min_conf:
                reasons.append('低置信度%.2f' % conf)
                continue
            if red < self.lock.red_min:
                reasons.append('红色过滤%.2f' % red)
                continue
            eligible.append(d)


        _merged = collapse_same_object(eligible)
        if len(_merged) != len(eligible):
            reasons.append('跨类重复框')
            eligible = _merged
        if len(eligible) > 1:
            reasons.append('多目标歧义%d' % len(eligible))
        i_ = self._iou(chosen, prev) if (chosen and prev) else None
        wr = (chosen['w'] / prev['w']) if (chosen and prev and prev.get('w')) else None

        def bb(d):
            return ([round(float(d['cx']), 1), round(float(d['cy']), 1),
                     round(float(d['w']), 1), round(float(d['h']), 1)] if d else None)
        self.debug.update(
            eligible_count=len(eligible),
            rejected_reasons='|'.join(sorted(set(reasons))),
            selected_bbox=bb(chosen), previous_bbox=bb(prev),
            match_iou=(round(i_, 3) if i_ is not None else None),
            match_width_ratio=(round(wr, 3) if wr is not None else None),
        )

    # 提速资格只由不同的新鲜真实目标帧累积；失去可靠输入时降级，防止用旧帧持续加速。
    def _update_qualify(self, target, packet, status, now):
        """提速资格只由不同的新鲜真实目标帧累积；失去可靠输入时降级，防止用旧帧持续加速。"""
        if target is None or target.get('_red_hold'):
            self._qual_hist = []
            self._qualified = False
            self.debug['qualify_state'] = 'off'
            self.debug['qualify_block'] = ('red_hold' if target is not None else 'no_target')
            self.qual_block[('red_hold' if target is not None else 'no_target')] += 1
            return


        if target.get('_weak'):
            self._qual_hist = []
            self._qualified = False
            self.debug['qualify_state'] = 'blocked'
            self.debug['qualify_block'] = 'weak_cont'
            self.qual_block['weak_cont'] += 1
            return
        fid = (packet or {}).get('frame_id') if isinstance(packet, dict) else None
        vage = (packet or {}).get('age_ms') if isinstance(packet, dict) else None
        tage = (status or {}).get('tlm_age_ms') if isinstance(status, dict) else None
        rng = (status or {}).get('rng') if isinstance(status, dict) else None
        conf = float(target.get('conf') or 0)


        if self.qual_hist_keep_ms > 0:
            self._qual_hist = [h for h in self._qual_hist
                               if (now - h[3]) <= self.qual_hist_keep_ms]


        block = None
        if not isinstance(fid, int):
            block = 'no_frame_id'
        elif conf < self.qualify_conf:
            block = 'conf'
        elif not (isinstance(vage, (int, float)) and 0 <= vage <= self.qualify_vision_age):
            block = 'vision_age'
        elif not (isinstance(tage, (int, float)) and 0 <= tage <= self.qualify_tlm_age):
            block = 'tlm_age'
        elif not (isinstance(rng, (int, float)) and rng >= self.qualify_rng_min):
            block = 'rng'
        elif int(self.debug.get('eligible_count') or 0) != 1:
            block = 'ambiguous'
        if block is not None:
            if block == 'tlm_age' and self.qual_hist_keep_ms > 0:


                self.stats['qual_kept'] += 1
                self.debug['qual_hist_len'] = len(self._qual_hist)
            else:
                self._qual_hist = []
            self._qualified = False
            self.debug['qualify_state'] = 'blocked'
            self.debug['qualify_block'] = block
            self.qual_block[block] += 1
            return
        if not self._qual_hist or self._qual_hist[-1][0] != fid:
            self._qual_hist.append((fid, float(target['cx']), float(target.get('w') or 0), now))
            if len(self._qual_hist) > self.qualify_frames:
                self._qual_hist.pop(0)

        if len(self._qual_hist) >= 2:
            for (_, cx0, w0, _t0), (_, cx1, _w1, _t1) in zip(self._qual_hist, self._qual_hist[1:]):
                if w0 > 0 and abs(cx1 - cx0) > 0.8 * w0:
                    self._qual_hist = []
                    self.debug['qualify_state'] = 'blocked'
                    self.debug['qualify_block'] = 'discontinuous'
                    self.qual_block['discontinuous'] += 1
                    break
        self._qualified = (len(self._qual_hist) >= self.qualify_frames)
        if self._qualified:
            self.debug['qualify_state'] = 'ok'
            self.debug['qualify_block'] = None
        elif self.debug.get('qualify_state') != 'blocked':

            self.debug['qualify_state'] = 'warming'
            self.debug['qualify_block'] = 'warming'
            self.qual_block['warming'] += 1

    def _trim_cap_now(self):
        """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
        if self.trim_speed_mode == 'qualified-fixed':
            return self.trim_test_cap if self._qualified else self.trim_cap
        return self.trim_cap

    def _gated_rng(self, rng):
        """超声波离群门控（规则与 `PropFollow._fuse_distance` 一致）。"""
        if not isinstance(rng, (int, float)) or not math.isfinite(rng) or not (20 <= rng < 4000):
            return None
        if self._last_rng is not None:
            tol = max(400.0, 0.4 * self._last_rng)
            if abs(rng - self._last_rng) > tol:
                self._rng_suspect += 1
                if self._rng_suspect <= 2:
                    return None
        self._rng_suspect = 0
        self._last_rng = rng
        return rng

    def _dist_to_speed(self, rng):
        """距离 → 前进速度。APPROACH 与 APPROACH_RED 共用（S1）。"""
        if rng is None:
            return None
        if rng >= self.d_far:
            return self.fwd_far
        if rng <= self.d_near:
            return self.fwd_near
        f = (rng - self.d_near) / float(self.d_far - self.d_near)
        return self.fwd_near + f * (self.fwd_far - self.fwd_near)

    # 前进族共享时间连续速度斜坡；停车和最终安全限幅优先于平滑。
    def _shape(self, cmd, now):
        """前进族共享时间连续速度斜坡；停车和最终安全限幅优先于平滑。"""
        if cmd.dir not in FWD_FAMILY or cmd.spd <= 0:
            self._spd_prev = None
            self._spd_ms = None
            return cmd
        target = float(cmd.spd)


        if self.accel_pct_s <= 0 and self.decel_pct_s <= 0:
            self._spd_prev, self._spd_ms = target, now
            return cmd
        if self._spd_prev is None or self._spd_ms is None:


            self._spd_prev = 0.0
            self._spd_ms = now - 100.0
        dt = max(0.001, (now - self._spd_ms) / 1000.0)


        rate = self.accel_pct_s if target >= self._spd_prev else self.decel_pct_s
        if self.red_follow:
            rate = min(rate, 10.0)
        max_step = rate * dt
        d = target - self._spd_prev
        if d > max_step:
            target = self._spd_prev + max_step
            self.stats['shaped'] += 1
        elif d < -max_step:
            target = self._spd_prev - max_step
            self.stats['shaped'] += 1
        self._spd_prev, self._spd_ms = target, now
        return Cmd(cmd.dir, round(target, 2), cmd.state, cmd.reason,
                   cmd.err, cmd.w, cmd.smooth_cx)


    def _precheck(self, packet, status, now):
        """输入合法性 / 新鲜度 / 距离。返回 stop reason 或 None。"""
        if not isinstance(status, dict) or not status.get('tlm_ok'):
            return 'STALE_TELEMETRY'
        age = status.get('tlm_age_ms', 1e9)
        if not isinstance(age, (int, float)) or not (0 <= age <= self.tlm_max_age_ms):
            return 'STALE_TELEMETRY'
        if status.get('fpga_mode') != 0:
            return 'WRONG_MODE'
        rng = status.get('rng')
        rng_ok = (isinstance(rng, (int, float)) and math.isfinite(rng) and 20 <= rng < 4000)
        if not rng_ok:

            return 'INVALID_RANGE'
        if rng <= self.range_stop_mm:
            return 'RANGE_STOP'
        return None


    # 把可靠目标转换为控制请求；速度请求与后续最终限幅分开处理。
    def _req_track(self, target, packet, status, now):
        """把可靠目标转换为控制请求；速度请求与后续最终限幅分开处理。"""
        self.stats['track'] += 1
        rng = status.get('rng')
        if self.ctrl is None:
            self._build_ctrl()
        if not self.lock.new_frame:
            return self.last_cmd
        vis = (target_cls_code(self.target_cls), round(target['cx']), round(target['cy']),
               round(target['w']), round(target['h']), round(target['conf'] * 100),
               1 if target.get('_weak') else 0)
        aligned = abs(target['cx'] - self.cx_center) <= 170
        dets = (packet.get('detections') or []) if isinstance(packet, dict) else []
        self._fill_candidate_debug(dets, target, prev=getattr(self, '_prev_target', None))
        self._prev_target = target
        self.debug['raw_err'] = round(float(target['cx']) - self.cx_center, 1)
        self.debug['filtered_err'] = self.debug['raw_err']
        if target.get('_red_hold'):
            self.debug['red_hold'] = True
            self._in_red_hold = True
        else:
            self._in_red_hold = False

        self._in_weak = bool(target.get('_weak'))
        if self._in_weak:
            self.stats['weak_cont'] += 1
        self._update_qualify(target, packet, status, now)


        if self.ctrl is not None:
            self.ctrl.trim_spd_cap = self._trim_cap_now()
        cmd = self.ctrl.step(vis, now, range_mm=(rng if aligned else None))


        if (self.unified_speed and cmd.dir == DIR_FWD
                and cmd.state.startswith('APPROACH')):

            spd = self._dist_to_speed(self._gated_rng(rng))
            if spd is not None:
                cmd = Cmd(cmd.dir, round(spd, 2), cmd.state + '_U', cmd.reason,
                          cmd.err, cmd.w, cmd.smooth_cx)
        return cmd

    def _req_lost(self, packet, status, now):
        """无语义目标：语义宽容（S3）→ 红块引导（S4 门控）→ 停车。"""
        if self._lost_at is None:
            self._lost_at = now
        dets = (packet.get('detections') or []) if isinstance(packet, dict) else []
        self._fill_candidate_debug(dets, None, prev=getattr(self, '_prev_target', None))
        self._in_red_hold = False
        self._in_weak = False
        self._qual_hist = []
        self._qualified = False


        self.debug['qualify_state'] = 'off'
        self.debug['qualify_block'] = 'no_target'
        self.qual_block['no_target'] += 1

        if (self.semantic_grace_ms > 0 and self._last_target_ms is not None
                and (now - self._last_target_ms) <= self.semantic_grace_ms):
            prev = self.last_cmd
            if prev.dir in FWD_FAMILY and prev.spd > 0:
                self.stats['coast'] += 1
                spd = min(float(prev.spd), 12.0)      # 续行不加速，上限 12%
                return Cmd(prev.dir, spd, 'COAST', 'grace %dms' % (now - self._last_target_ms))

        if self.red_cue and self._red_cue_allowed(now):
            rcx = packet.get('red_cx') if isinstance(packet, dict) else None
            rn = packet.get('red_n') if isinstance(packet, dict) else None
            rw = packet.get('red_w') if isinstance(packet, dict) else None
            if (isinstance(rcx, (int, float)) and isinstance(rn, (int, float))
                    and rn >= self.red_min_n
                    and isinstance(rw, (int, float)) and rw >= self.red_min_w):
                return self._req_red(rcx, packet, status, now)

        return Cmd(DIR_STOP, 0, 'LOST', 'no semantic target')

    def _red_cue_allowed(self, now):
        if self.red_cue_after_ms <= 0:
            return True
        if self._last_lock_ms is None:
            return True
        return (now - self._last_lock_ms) >= self.red_cue_after_ms

    def _req_red(self, rcx, packet, status, now):
        """红块引导：只负责"把目标转到画面中间"，前进要语义确认。"""
        self.stats['red_cue'] += 1


        if not hasattr(self, '_red_cx_s'):
            self._red_cx_s = float(rcx)
        self._red_cx_s = 0.6 * self._red_cx_s + 0.4 * float(rcx)
        err_s = self._red_cx_s - self.cx_center
        err = rcx - self.cx_center
        self.debug['raw_err'] = round(float(err), 1)
        self.debug['filtered_err'] = round(float(err), 1)

        if self._red_dir is None:
            if abs(err) > self.turn_enter_px:
                self._red_dir = DIR_SPIN_L if err > 0 else DIR_SPIN_R
        elif abs(err) <= self.turn_exit_px:
            self._red_dir = None
        elif abs(err) > self.turn_enter_px and ((err > 0) != (self._red_dir == DIR_SPIN_L)):
            self._red_dir = DIR_SPIN_L if err > 0 else DIR_SPIN_R


        if self._red_dir is not None and self.red_follow:
            err_abs = abs(float(err))

            d = DIR_LEFT if err > 0 else DIR_RIGHT
            base = self._dist_to_speed(self._gated_rng(status.get('rng')))
            cap = float(self.red_hold_cap)
            base = cap if base is None else min(base, cap)


            err_abs_s = abs(float(err_s))          # 用平滑后的误差算比例档，抑制抖动
            frac = min(1.0, max(0.0, (err_abs_s - self.turn_enter_px) / 350.0))
            want = self.red_min_spd + (self.red_cue_spd - self.red_min_spd) * frac
            spd = min(want, cap, base)
            spd = max(spd, 0.0)
            return Cmd(d, round(spd, 2), 'RED_TRACK', 'err=%+d spd=%.0f f=%.2f' % (err, spd, frac))
        if self._red_dir is not None:
            d = self._red_dir

            if self.direction_confirm_frames > 1 and self._turn_dir_last is not None and d != self._turn_dir_last:
                fid = packet.get('frame_id')
                if fid not in self._dir_cand_frames:
                    self._dir_cand_frames.append(fid)
                if len(self._dir_cand_frames) < self.direction_confirm_frames:
                    d = self._turn_dir_last          # 确认不足，保持原方向
                else:
                    self._dir_cand_frames = []
                    self._turn_dir_last = d
                    self._turn_ms = now
            else:
                if d != self._turn_dir_last:
                    self._turn_dir_last = d
                    self._turn_ms = now
                    self._dir_cand_frames = []
            frac = min(1.0, max(0.0, (abs(err) - self.turn_enter_px) / 500.0))
            spd = self.red_min_spd + (self.red_cue_spd - self.red_min_spd) * frac
            return Cmd(d, round(spd, 2), 'SEEK_RED', 'err=%+d' % err)


        has_target = any((d.get('cls') in self.target_cls)
                         and float(d.get('conf') or 0) >= self.lock.min_conf
                         and float(d.get('red_ratio') or 0) >= self.lock.red_min
                         for d in (packet.get('detections') or []) if isinstance(d, dict))
        if has_target:
            rng = status.get('rng')
            spd = self._dist_to_speed(self._gated_rng(rng))
            if spd is None:
                return Cmd(DIR_STOP, 0, 'APPROACH_RED', 'no range')
            return Cmd(DIR_FWD, round(spd, 2), 'APPROACH_RED', 'red centered')


        if self.red_follow:
            spd = self._dist_to_speed(self._gated_rng(status.get('rng')))
            cap = float(self.red_hold_cap)
            spd = cap if spd is None else min(spd, cap)
            if spd > 0:
                return Cmd(DIR_FWD, round(spd, 2), 'RED_FOLLOW', 'red only, cap %g' % cap)
        return Cmd(DIR_STOP, 0, 'RED_CENTERED', 'no semantic confirm')


    # 所有运动请求在此统一复核输入时效、距离与状态限速；分支不得绕过最终保护。
    def _guard(self, cmd, packet, status, now):
        """所有运动请求在此统一复核输入时效、距离与状态限速；分支不得绕过最终保护。"""
        reason = self._precheck(packet, status, now)
        if reason is not None:
            self.stats['stop_guard'] += 1
            return self._stop(reason)

        if isinstance(packet, dict):
            vage = packet.get('age_ms', 0)
            if not isinstance(vage, (int, float)) or vage > self.vision_max_age_ms:
                self.stats['stop_guard'] += 1
                return self._stop('STALE_VISION')


        _seek_red_ok = (self.red_follow and cmd.state == 'SEEK_RED'
                        and self._last_lock_ms is not None)
        if (not self.motion_search_allowed) and cmd.state in SEARCH_STATES \
                and cmd.dir in SPIN_FAMILY and not _seek_red_ok:
            self.stats['stop_guard'] += 1
            return self._stop('SEARCH_DISABLED')


        cands = []
        if cmd.state == 'COAST':
            cands.append((self.coast_cap, 'coast'))
        elif self._in_weak:


            cands.append((self.red_hold_cap, 'weak_cont'))
        elif self._in_red_hold:
            cands.append((self.red_hold_cap, 'red_hold'))


        elif cmd.state == 'RED_TRACK':
            cands.append((self.red_hold_cap, 'red_track'))
        elif cmd.dir in (DIR_LEFT, DIR_RIGHT):      # 差速纠偏（TRIM）
            cap = self._trim_cap_now()


            if self.red_follow and isinstance(cmd.err, (int, float)):


                _f = min(1.0, max(0.45, abs(float(cmd.err)) / 300.0))
                cap = cap * _f


            use_qual = (self.trim_speed_mode == 'qualified-fixed' and self._qualified)
            cands.append((cap, ('trim_qualified:%g' if use_qual else 'trim_base:%g') % cap))
        elif cmd.dir in SPIN_FAMILY:
            cands.append((8.0, 'spin'))
        cands.append((self.max_spd, 'global'))

        rng_now = (status or {}).get('rng') if isinstance(status, dict) else None
        d_cap = None
        if isinstance(rng_now, (int, float)) and 20 <= rng_now < 4000:
            d_cap = self._dist_to_speed(rng_now)
            if d_cap is not None:
                cands.append((d_cap, 'range'))
        cap, cap_reason = min(cands, key=lambda x: x[0])
        spd = min(float(cmd.spd), cap)


        _hold_can_rise = (cap_reason == 'red_hold' and self.red_hold_cap > 12.0
                          and self.red_follow)
        if (cap_reason in ('coast', 'red_hold', 'weak_cont')
                and self._spd_prev is not None and not _hold_can_rise):
            spd = min(spd, float(self._spd_prev))
        spd = max(0.0, round(spd, 2))
        self._cap_used = cap_reason
        if cmd.dir in FWD_FAMILY:

            self._spd_prev, self._spd_ms = spd, now
            if self.ctrl is not None:
                self.ctrl.last_fwd_spd = spd
                self.ctrl.last_fwd_ms = now
        self.debug['final_cap'] = round(float(cap), 2)
        self.debug['cap_reason'] = cap_reason

        self.debug['limited_by'] = ('request' if float(cmd.spd) < float(cap) - 1e-9
                                    else cap_reason)
        return Cmd(cmd.dir, spd, cmd.state, cmd.reason, cmd.err, cmd.w, cmd.smooth_cx)


    def step(self, packet, status, now):
        self.stats['samples'] += 1


        if self.qual_hist_keep_ms > 0 and self._qual_hist:
            self._qual_hist = [h for h in self._qual_hist
                               if (now - h[3]) <= self.qual_hist_keep_ms]

        dets = (packet.get('detections') or []) if isinstance(packet, dict) else []
        self.debug = dict(
            mono_ms=round(now, 1),
            producer_id=(packet or {}).get('producer_id') if isinstance(packet, dict) else None,
            frame_id=(packet or {}).get('frame_id') if isinstance(packet, dict) else None,
            vision_age=(packet or {}).get('age_ms') if isinstance(packet, dict) else None,
            is_new_frame=bool(getattr(self.lock, 'new_frame', False)),
            detector_count=len(dets),
            rng_raw=(status or {}).get('rng') if isinstance(status, dict) else None,
            rng_gated=None,
            tlm_age=(status or {}).get('tlm_age_ms') if isinstance(status, dict) else None,
            wheelL=(status or {}).get('velL') if isinstance(status, dict) else None,
            wheelR=(status or {}).get('velR') if isinstance(status, dict) else None,
            lock_state='', eligible_count=0, rejected_reasons='',
            selected_bbox=None, previous_bbox=None, match_iou=None,
            match_width_ratio=None, target_id=None,
            last_fresh_match_ms=None, raw_err=None, filtered_err=None,
            request_state='', request_dir=None, request_spd=None, shaped_spd=None,
            final_state='', final_dir=None, final_spd=None, guard_reason='',
        )
        if self.arrived:
            return self._stop('ARRIVED')

        reason = self._precheck(packet, status, now)
        if reason is not None:
            self.stats['stop_precheck'] += 1

            self.lock.update(packet, now)
            return self._stop(reason)

        target = self.lock.update(packet, now)
        if target is not None:
            self._lost_at = None
            self._last_lock_ms = now
            self._last_target_ms = now
            yaw = status.get('yaw')
            if isinstance(yaw, (int, float)):
                self._bearing_yaw = yaw + self.deg_per_px * (self.cx_center - target['cx'])
            if not self._was_locked:
                self._need_still = bool(self.still_confirm)
                self._still_cnt = 0
            self._was_locked = True
        else:
            self._was_locked = False
            self._need_still = False
            self._still_cnt = 0

        if target is not None:
            req = self._req_track(target, packet, status, now)
        else:
            req = self._req_lost(packet, status, now)

        self.debug.update(request_state=req.state, request_dir=req.dir,
                          request_spd=round(float(req.spd), 2))
        req = self._shape(req, now)
        self.debug['shaped_spd'] = round(float(req.spd), 2)

        cmd = self._guard(req, packet, status, now)
        self.debug.update(final_state=cmd.state, final_dir=cmd.dir,
                          final_spd=round(float(cmd.spd), 2),
                          lock_state=getattr(self.lock, 'state', ''),
                          last_fresh_match_ms=self._last_target_ms)
        if cmd.dir == 0 and cmd.spd == 0 and cmd.reason:
            self.debug['guard_reason'] = cmd.reason
        self.last_cmd = cmd
        if cmd.spd > 0 and cmd.dir in FWD_FAMILY:
            self._sync_ctrl_speed(cmd.spd, now)
        return cmd


    def _build_ctrl(self):
        self.ctrl = PropFollow(
            cx_center=self.cx_center, near_w=9999, accept_n=1, follow=self.follow,
            conf_min=round(self.min_conf * 100),
            conf_min_cont=min(round(self.continue_conf * 100), round(self.min_conf * 100)),
            mirror=True, grace_ms=self.grace_ms, hold_ms=self.hold_ms,
            search_when_never_ms=0,             # 本轮禁用搜索
            sweep_spd=self.sweep_spd, search_spd=self.search_spd, burst=True,
            burst_pct=self.burst_pct, trim=True, trim_spd_cap=self.trim_cap,
            spin_at=9999,
            burst_start_ms=self.burst_start_ms, burst_min_ms=self.burst_min_ms,
            burst_max_ms=self.burst_max_ms, burst_settle_ms=self.burst_settle,
            spd_fwd_far=self.fwd_far, spd_fwd_near=self.fwd_near,
            spd_spin_max=8.0,                   # 原地转上限 8%
            d_far_mm=self.d_far, d_near_mm=self.d_near,
            range_stop_mm=self.range_stop_mm,
            close_h_frac=self.close_h, close_h_release=max(0.05, self.close_h - 0.10),
            db_in=self.turn_exit_px, db_out=self.turn_enter_px)

    def _sync_ctrl_speed(self, spd, now):
        if self.ctrl is not None and spd > 0:
            self.ctrl.last_fwd_spd = float(spd)
            self.ctrl.last_fwd_ms = now


def target_cls_code(cls_set):
    """PropFollow 只认单个 cls，这里用一个非 0 的占位值（0 会被判无效）。"""
    return min(cls_set) if cls_set else 44
