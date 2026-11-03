#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""视觉目标搜索与语义任务编排，协调方位、目标状态与运动命令。"""
import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from run_c import pull_frame, Sender, TlmReader
import run_c as rc
import red_nav_e2e as rd
from prop_ctrl import PropFollow, DIR_NAME
from nav_layer import (NavCtl, wrap, DEG_PER_PX as NAV_DEG_PER_PX,
                       SIDE_BLIND_MM, BODY_MARGIN_MM)


SPECS = {
    'bag':    dict(w_min=150, w_max=9999, aspect_min=0.55, aspect_max=2.20,
                   det=dict(), stop_mm=350, h_m=0.45),
    'bottle': dict(w_min=40,  w_max=340,  aspect_min=0.00, aspect_max=0.40,
                   det=dict(rmin=45, rdom=10, min_pixels=120, mode='bbox'),
                   stop_mm=450, h_m=0.22),
    'any':    dict(w_min=0,   w_max=9999, aspect_min=0.00, aspect_max=99.0,
                   det=dict(), stop_mm=350),
}


def match_spec(det, spec):
    """捕获阶段用：必须符合特征档"""
    if not det:
        return False
    w, h = det.get('w', 0), det.get('h', 0)
    if w <= 0 or h <= 0:
        return False
    a = w / float(h)
    return (spec['w_min'] <= w <= spec['w_max']
            and spec['aspect_min'] <= a <= spec['aspect_max'])


def look_for(spec, loose=False):
    """拉一帧 → 检测 → 按特征匹配。"""
    if not pull_frame(timeout=2.5):
        return None
    try:
        det = rd.detect_red(rc.LOCAL_FRAME, **(spec.get('det') or {}))
    except Exception:
        return None
    if not det:
        return None
    return det if (loose or match_spec(det, spec)) else None


def verify_sharp(sender, spec, settle=0.45, log=None, tag='锐利帧复验'):
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    sender.send(0, 0)
    time.sleep(settle)
    if not pull_frame(timeout=2.5):
        return None
    try:
        det = rd.detect_red(rc.LOCAL_FRAME, **(spec.get('det') or {}))
    except Exception:
        return None
    if det and match_spec(det, spec):
        if log:
            log('  %s 通过：cx=%d w=%d h=%d aspect=%.2f'
                % (tag, det['cx'], det['w'], det['h'], det['w'] / float(det['h'])))
        return det
    if log and det:
        log('  %s 未通过（aspect=%.2f 不是瓶子形状，疑似书包侧对/拖影），继续搜索'
            % (tag, det['w'] / float(det['h'])))
    return None


DEG_PER_PX = 1.0 / 19.7


def _wrap(a):
    while a > 180:
        a -= 360
    while a < -180:
        a += 360
    return a


def turn_to_yaw(sender, tlm, target, pct, tol=8.0, timeout=9.0):
    """转到指定 yaw（闭环）。yaw 需减小→左转(5)，需增大→右转(6)。"""
    t_end = time.time() + timeout
    while time.time() < t_end:
        y = tlm.yaw_deg if tlm else None
        if y is None:
            break
        d = _wrap(y - target)
        if abs(d) <= tol:
            sender.send(0, 0)
            return True
        sender.send(5 if d > 0 else 6, pct)     # 5=SPIN_L 6=SPIN_R
        time.sleep(0.05)
    sender.send(0, 0)
    return False


def scan_until_found(sender, spec, a, tlm, deadline, log):
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""


    loose = dict(spec.get('det') or {})
    loose['min_pixels'] = max(40, int(loose.get('min_pixels', 120) * 0.5))


    TRIED_COOL = 15.0        # 同一方位两次确认之间的冷却（≈一圈周期）：

    if a.scan_mode == 'cont':
        direction = a.scan_dir
        pct = a.scan_pct
        rate = max(10.0, 9.4 * pct)
        clusters = {}                # 15° 桶 → [最大像素数, 方位, 命中次数]
        tried = {}                   # 桶 → 上次确认时刻（冷却用）
        def _cool(k):
            return time.time() - tried.get(k, -1e9) > TRIED_COOL
        turned = 0.0
        last_yaw = tlm.yaw_deg if tlm else None
        t_cand = 0.0
        log('SCAN  慢转连续看：%s %d%%（约 %.0f°/s，一圈 %.1fs，拖影约 %.1f°）'
            % (DIR_NAME[direction], pct, rate, 360.0 / rate, rate * 0.05))
        try:
            while time.time() < deadline:
                sender.send(direction, pct)        # 10Hz+ 刷新，避死手开关
                ok = pull_frame(timeout=2.0)
                det = None
                if ok:
                    try:
                        det = rd.detect_red(rc.LOCAL_FRAME, **(spec.get('det') or {}))
                    except Exception:
                        det = None
                if det and match_spec(det, spec):


                    vdet = verify_sharp(sender, spec, log=log, tag='SCAN  转动中命中复验')
                    if vdet:
                        log('SCAN  锁定目标 cx=%d w=%d h=%d（复验通过）'
                            % (vdet['cx'], vdet['w'], vdet['h']))
                        return vdet

                    last_yaw = tlm.yaw_deg if tlm else None
                    turned = 0.0
                    continue
                y = tlm.yaw_deg if tlm else None
                if y is not None and last_yaw is not None:
                    turned += abs(_wrap(y - last_yaw))
                    last_yaw = y
                now = time.time()
                if ok and now - t_cand > 0.10 and y is not None:
                    t_cand = now
                    try:
                        d2 = rd.detect_red(rc.LOCAL_FRAME, **loose)
                    except Exception:
                        d2 = None
                    if d2 and match_spec(d2, spec):
                        bearing = _wrap(y - (d2['cx'] - a.cx_center) * DEG_PER_PX)
                        k = int(round(bearing / 15.0))
                        if k not in clusters or d2['pixels'] > clusters[k][0]:
                            clusters[k] = [d2['pixels'], bearing, 1]
                        else:
                            clusters[k][2] += 1        # 连续命中计数

                        wgt, b, hits = clusters[k]
                        if hits >= 3 and turned > 90.0 and _cool(k):
                            tried[k] = time.time()
                            sender.send(0, 0)
                            log('SCAN  提前认领：方位 %.0f°（%d 像素，第 %d 次'
                                '看到），不必等转满一圈' % (b, wgt, hits))
                            if turn_to_yaw(sender, tlm, b, max(8, pct)):
                                time.sleep(0.3)
                                for _ in range(4):
                                    if pull_frame(timeout=2.0):
                                        try:
                                            det = rd.detect_red(
                                                rc.LOCAL_FRAME,
                                                **(spec.get('det') or {}))
                                        except Exception:
                                            det = None
                                        if det and match_spec(det, spec):
                                            log('SCAN  提前认领锁定 cx=%d w=%d h=%d'
                                                % (det['cx'], det['w'], det['h']))
                                            return det
                                    time.sleep(0.08)
                                log('SCAN  提前认领方位 %.0f° 没严格命中，继续慢转' % b)
                            clusters[k][2] = 0            # 去过了，重新计数
                            last_yaw = tlm.yaw_deg if tlm else None
                            turned = 0.0
                if turned > 360.0:
                    turned = 0.0

                    fresh = {k: v for k, v in clusters.items() if _cool(k)}
                    if fresh:
                        k, (wgt, b, _h) = max(fresh.items(),
                                              key=lambda kv: kv[1][0])
                        log('SCAN  扫完一圈，按记忆朝最可信方位 %.0f° 转过去（%d 像素）'
                            % (b, wgt))
                        sender.send(0, 0)
                        if turn_to_yaw(sender, tlm, b, max(8, pct)):
                            time.sleep(0.3)
                            for _ in range(4):
                                if pull_frame(timeout=2.0):
                                    try:
                                        det = rd.detect_red(rc.LOCAL_FRAME,
                                                            **(spec.get('det') or {}))
                                    except Exception:
                                        det = None
                                    if det and match_spec(det, spec):
                                        log('SCAN  在记忆方位上锁定 cx=%d w=%d h=%d'
                                            % (det['cx'], det['w'], det['h']))
                                        return det
                                time.sleep(0.08)
                            log('SCAN  方位 %.0f° 上没严格命中，继续慢转' % b)
                        tried[k] = time.time()     # 记冷却时间，防反复认领
                        clusters[k][2] = 0         # 命中计数清零
                    else:

                        if clusters:
                            log('SCAN  一圈的候选都确认过仍没中（冷却中），换向')
                        else:
                            log('SCAN  一圈没有红色候选，换向')
                        direction = 6 if direction == 5 else 5
                        log('SCAN  换向 %s' % DIR_NAME[direction])
        finally:
            sender.send(0, 0)
        log('SCAN  扫描超时未找到目标')
        return None


    direction = a.scan_dir
    step = max(15.0, a.scan_step_deg)
    pct = a.scan_pct
    rate = max(15.0, 9.4 * pct)
    steps = 0
    per_lap = max(4, int(round(360.0 / step)))
    cands = []                      # [(weight, target_yaw, dist_mm)]


    def look_strict():
        return look_for(spec)

    def look_loose():
        """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
        if not pull_frame(timeout=2.5):
            return None
        try:
            d = rd.detect_red(rc.LOCAL_FRAME, **loose)
        except Exception:
            return None
        return d if match_spec(d, spec) else None

    log('SCAN  环顾搜索：%s %d%%（先看当前朝向→转 %.0f°→停稳→再看；一圈 %d 步）'
        % (DIR_NAME[direction], pct, step, per_lap))


    while time.time() < deadline:

        det = None
        for _ in range(3):
            det = look_strict()
            if det:
                break
            time.sleep(0.07)
        if det:
            log('SCAN  锁定目标 cx=%d w=%d h=%d（第 %d 步，直接过去）'
                % (det['cx'], det['w'], det['h'], steps))
            sender.send(0, 0)
            return det


        ynow = tlm.yaw_deg if tlm else None
        if ynow is not None:
            d2 = look_loose()
            if d2:
                bearing = _wrap(ynow - (d2['cx'] - a.cx_center) * DEG_PER_PX)
                rng = tlm.fresh_range() if tlm else None
                cands.append((d2['pixels'], bearing, rng))
                log('SCAN  记下红色候选：方位 %.0f°（yaw %.0f°, cx=%d, %d 像素）'
                    % (bearing, ynow, d2['cx'], d2['pixels']))


        y0 = tlm.yaw_deg if tlm else None
        t_lim = time.time() + max(1.0, step / rate + 1.2)
        if y0 is not None:
            while time.time() < t_lim:
                sender.send(direction, pct)
                y = tlm.yaw_deg
                if y is not None and abs(_wrap(y - y0)) >= step:
                    break
                time.sleep(0.03)
        else:
            t_lim = time.time() + (275 + step / rate * 1000.0) / 1000.0
            while time.time() < t_lim:
                sender.send(direction, pct)
                time.sleep(0.03)
        sender.send(0, 0)
        time.sleep(0.28)                 # 停稳 + 曝光刷新，下一轮 A 取清晰帧

        steps += 1

        if steps % per_lap == 0:
            lap = steps // per_lap
            if cands:
                cands.sort(reverse=True)
                wgt, b, rng = cands[0]
                log('SCAN  第 %d 圈扫完，按记忆朝最可信候选方位 %.0f° 转过去（%d 像素）'
                    % (lap, b, wgt))
                if turn_to_yaw(sender, tlm, b, pct):
                    time.sleep(0.3)
                    for _ in range(4):
                        det = look_strict()
                        if det:
                            log('SCAN  在记忆方位上锁定 cx=%d w=%d h=%d'
                                % (det['cx'], det['w'], det['h']))
                            return det
                        time.sleep(0.08)
                    log('SCAN  记忆方位 %.0f° 上没有严格命中，继续环顾' % b)
                cands[0] = (wgt // 4, b, rng)      # 去过的候选降权
            else:
                log('SCAN  第 %d 圈扫完，没有任何红色候选，换向 %s'
                    % (lap, DIR_NAME[6 if direction == 5 else 5]))
                direction = 6 if direction == 5 else 5
    log('SCAN  扫描超时未找到目标')
    return None


def reacquire(ctl, sender, spec, bearing, log, span=80.0, step=20.0):
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    if bearing is not None:
        log('  NAV  按方位记忆 %.0f° 重新捕获' % bearing)
        if ctl.turn_to(bearing, pct=12, tol=6.0):
            for _ in range(5):
                det = look_for(spec, loose=True)


                if det and not match_spec(det, spec):
                    log('  NAV  方位上有红物但不是瓶子形状 '
                        '(aspect=%.2f)，跳过' % (det['w'] / float(det['h'])))
                    break
                if det:
                    log('  NAV  在记忆方位上重新捕获 cx=%d h=%d'
                        % (det['cx'], det['h']))
                    return det
                time.sleep(0.1)
        off = step
        while off <= span:
            for sgn in (+1.0, -1.0):
                tgt = wrap(bearing + sgn * off)
                if not ctl.turn_to(tgt, pct=12, tol=6.0):
                    continue
                for _ in range(3):
                    det = look_for(spec, loose=True)
                    if det and not match_spec(det, spec):
                        log('  NAV  ±%.0f° 处有红物但不是瓶子形状，跳过'
                            % (sgn * off))
                        break
                    if det:
                        log('  NAV  扫到 ±%.0f° 处重新捕获 cx=%d h=%d'
                            % (sgn * off, det['cx'], det['h']))
                        return det
                    time.sleep(0.09)
            off += step
        log('  NAV  记忆方位 ±%.0f° 都没有' % span)


    log('  NAV  整圈慢转搜索')
    t_end = time.time() + 8.5
    while time.time() < t_end:
        sender.send(5, 5)
        det = look_for(spec, loose=True)
        if det:

            vdet = verify_sharp(sender, spec, log=log, tag='NAV  慢转命中复验')
            if vdet:
                log('  NAV  慢转扫到 cx=%d h=%d' % (vdet['cx'], vdet['h']))
                return vdet
        time.sleep(0.05)
    ctl.stop()
    return None


def approach_nav(sender, tlm, spec, a, deadline, log, now0):
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    ctl = NavCtl(sender, tlm, log, hdg_db=a.hdg_db)
    close_h = int(720 * a.close_h_frac)
    side = a.avoid_side
    attempts = 0
    lost = 0
    no_yaw = 0
    last_msg = ''
    last_bearing = None      # 目标最后出现过的绝对方位（绕行后按它重捕获）
    last_msg_obs = ''
    bad_heads = []           # 走不通的空档航向（绝对 yaw），下一轮排除


    last_r = None
    last_r_t = 0.0

    while time.time() < deadline:

        _r = tlm.fresh_range() if tlm else None
        if _r is not None and 60 <= _r <= 3000:
            last_r = _r
            last_r_t = time.time()

        det = look_for(spec, loose=True)


        if det and not match_spec(det, spec) and det['h'] < 500:
            det = None


        if not det:
            lost += 1
            if lost == 1:
                ctl.stop()
                log('  NAV  目标不可见 ⇒ 按方位记忆重新捕获')
                det = reacquire(ctl, sender, spec, last_bearing, log)
                if det:
                    lost = 0
                else:
                    log('  NAV  重新捕获失败，退出导航接近')
                    return 'LOST'
            else:
                time.sleep(0.08)
                continue
        lost = 0


        if det['h'] >= close_h:
            if abs(det['cx'] - a.cx_center) > a.arrive_err:
                yy = tlm.yaw_deg if tlm else None
                if yy is not None:
                    b2 = wrap(yy - (det['cx'] - a.cx_center) * NAV_DEG_PER_PX)
                    log('  NAV  已到跟前但目标偏 %+dpx（>%d），先转正到 %.0f°'
                        % (det['cx'] - a.cx_center, a.arrive_err, b2))


                    ctl.turn_to(b2, pct=10, tol=8.0, settle_ms=180)
                    continue


            r_use = last_r if (last_r is not None
                               and (time.time() - last_r_t) < 3.0) else None
            if r_use is not None and r_use > a.arrive_mm:


                log('  NAV  已接近 h=%d，前距 %dmm > %dmm ⇒ 连续低速贴近（%d%%）'
                    % (det['h'], r_use, a.arrive_mm, a.final_spd))
                sender.set_beep(False)          # 贴近期间确保软件蜂鸣不响
                t_end = time.time() + a.final_max_s
                while time.time() < t_end:
                    rr = tlm.fresh_range() if tlm else None
                    if rr is not None and rr <= a.arrive_mm:
                        break
                    sender.send(1, a.final_spd)  # 1 = FWD，连续发（不停）
                    time.sleep(0.08)
                sender.send(0, 0)
                time.sleep(0.12)
                r_after = tlm.fresh_range() if tlm else None
                log('  NAV  贴近结束：前距 %s（目标 %dmm）'
                    % (('%dmm' % r_after) if r_after is not None else '无效',
                       a.arrive_mm))
                continue

            ctl.stop()
            log('  NAV  ★到达：框高 h=%d ≥ %d，前距 %s，偏差 %+dpx'
                % (det['h'], close_h,
                   ('%dmm' % r_use) if r_use is not None else '无效(用图像判据)',
                   det['cx'] - a.cx_center))


            sender.mode(0)                  # 0 = MODE_MANUAL：关自动避障
            time.sleep(0.08)
            sender.set_beep(False)
            time.sleep(0.10)
            log('  NAV  ♪ 到达提示音：鸣两声（已先发 MANUAL 关避障）')
            sender.beep_pattern(n=2, on_ms=220, off_ms=280)
            sender.set_beep(False)
            if a.follow:
                log('  NAV  交给视觉伺服接管（跟随模式）')
                return approach(sender, tlm, spec, a, deadline, log, now0)
            return 'ARRIVED'


        y = tlm.yaw_deg if tlm else None
        if y is None:
            no_yaw += 1
            if no_yaw >= 20:            # 连续 20 拍没 yaw ⇒ 导航不可用
                log('  NAV  IMU yaw 无效，退回纯视觉伺服')
                return approach(sender, tlm, spec, a, deadline, log, now0)
        else:
            no_yaw = 0
        err = det['cx'] - a.cx_center
        bearing = wrap(y - err * NAV_DEG_PER_PX) if y is not None else None
        msg = 'cx=%d h=%d 方位=%s' % (det['cx'], det['h'],
                                     ('%.0f°' % bearing) if bearing is not None else '?')
        if bearing is not None:
            last_bearing = bearing
        if msg != last_msg:
            log('  NAV  %s' % msg)
            last_msg = msg


        if bearing is not None and y is not None:
            e = wrap(y - bearing)
            if abs(e) > a.nav_align_deg:
                log('  NAV  对准：yaw %.0f°→%.0f°（差 %+.0f°）' % (y, bearing, e))
                ctl.turn_to(bearing, pct=max(10, a.nav_spd), tol=5.0)
                continue


        eff_obs = a.obs_mm
        _d_img = None
        if spec.get('h_m') and det and det['h'] > 0:
            _d_img = 1091.0 * spec['h_m'] / det['h'] * 1000.0     # mm
        _r_now = last_r if (last_r is not None
                            and (time.time() - last_r_t) < 1.5) else None
        if _d_img is not None and _r_now is not None:
            if _r_now >= 0.75 * _d_img:
                eff_obs = 0        # 前向回波就是目标本身
                if last_msg_obs != 'off':
                    log('  NAV  US %dmm ≈ 目标推算距离 %dmm ⇒ 测到的是目标，'
                        '关闭前障判定' % (_r_now, int(_d_img)))
                    last_msg_obs = 'off'
            else:
                eff_obs = a.obs_mm  # US 明显比目标近 ⇒ 中间有真障碍
                if last_msg_obs != '':
                    log('  NAV  US %dmm ≪ 目标推算 %dmm ⇒ 目标前有真障碍，'
                        '保留遇障判定' % (_r_now, int(_d_img)))
                    last_msg_obs = ''
        else:
            eff_obs = a.obs_mm if det['h'] < a.obs_off_h else 0
            if eff_obs == 0 and last_msg_obs != 'off':
                log('  NAV  目标已贴近(h=%d)，关闭前障判定（超声波测到的就是目标）'
                    % det['h'])
                last_msg_obs = 'off'
            elif eff_obs and last_msg_obs == 'off':
                last_msg_obs = ''
        st, mv = ctl.fwd(a.nav_chunk, bearing, spd=a.nav_spd,
                         deadline=deadline, obs_mm=eff_obs)

        if st == 'OBSTACLE':
            attempts += 1
            if attempts > a.avoid_max:
                ctl.stop()
                log('  NAV  连续 %d 次都被挡，放弃' % attempts)
                return 'BLOCKED'


            span = min(90.0, a.scan_span + (attempts - 1) * 30)
            log('  NAV  ★前方障碍（< %dmm），第 %d 次：扫描 ±%.0f° 找空档'
                % (a.obs_mm, attempts, span))
            prof = ctl.polar_scan(y, span=span, step=a.scan_step,
                                  pct=max(14, a.nav_spd))
            log('  NAV  净空表 ' + '  '.join(
                '%+d°:%s' % (o, '?' if d is None else '%d' % d) for o, d in prof))
            t_off = (wrap(bearing - y)
                     if (bearing is not None and y is not None) else 0.0)
            off, clear = ctl.pick_gap(prof, t_off, min_clear=a.pass_min,
                                      bad_yaws=bad_heads,
                                      cur_yaw=(y if y is not None else None))
            if off is None:
                ctl.stop()
                log('  NAV  ★没有任何方向净空 ≥ %dmm'
                    '（已扣盲区 %.0f + 车身 %.0f）⇒ 过不去，停车不硬挤'
                    % (a.pass_min, SIDE_BLIND_MM, BODY_MARGIN_MM))
                return 'BLOCKED'
            y_go = wrap((y or 0) + off)


            step_mm = min(clear, a.gap_step)
            log('  NAV  走空档 %+.0f°（净空 %.0fmm），先走 %.0fmm 再重看目标'
                % (off, clear, step_mm))
            ctl.turn_to(y_go, pct=max(12, a.nav_spd), tol=8.0, settle_ms=150)


            st2, mv2 = ctl.fwd(step_mm, y_go, spd=a.nav_spd,
                               deadline=deadline, obs_mm=a.gap_obs)
            if st2 == 'OBSTACLE':

                bad_heads.append(y_go)
                log('  NAV  空档方向 %+.0f°（yaw %.0f°）走到 %.0fmm 就顶住了'
                    ' ⇒ 记入黑名单，下一轮换方向'
                    % (off, y_go, mv2))
            else:
                log('  NAV  空档航段 %s（走过 %.0fmm），回到主循环重看目标'
                    % (st2, mv2))
            continue

        if st in ('TIMEOUT', 'STALL'):
            ctl.stop()
            if st == 'STALL':
                log('  NAV  ★轮速≈0 且里程不涨 ⇒ 判定走不动'
                    '（电机没电/堵转/编码器断），提前退出，不再等任务级超时')
            return st

    ctl.stop()
    return 'FOLLOWED' if a.follow else 'TIMEOUT'


def approach(sender, tlm, spec, a, deadline, log, now0):
    """接近阶段：交给 PropFollow，只认颜色不卡特征档。"""
    pf = PropFollow(cx_center=a.cx_center, mirror=True, burst=True,
                    burst_pct=a.burst_pct, burst_settle_ms=a.burst_settle,
                    follow=a.follow,
                    range_stop_mm=(spec.get('stop_mm') or a.stop_mm),
                    db_in=a.db_in, db_out=a.db_out, spin_at=a.spin_at,
                    d_far_mm=a.d_far, d_near_mm=a.d_near,
                    spd_fwd_far=a.fwd_far, spd_fwd_near=a.fwd_near,
                    trim_spd_cap=a.trim_spd, slew_pct_s=a.slew,
                    smooth_n=5, jump_px=200)
    last_state = None


    stop_mm = spec.get('stop_mm') or a.stop_mm
    latched = False
    while time.time() < deadline:
        now = int((time.time() - now0) * 1000.0)
        det = look_for(spec, loose=True)
        vis = None
        if det:
            vis = (det.get('cls', 1), det['cx'], det['cy'],
                   det['w'], det['h'], det['conf'])
        rng = tlm.fresh_range() if tlm else None
        cmd = pf.step(vis, now, range_mm=rng)

        if cmd.state in ('HOLD', 'ARRIVED'):
            latched = True
        if latched:
            if rng is not None and rng > stop_mm + 150:
                latched = False
            elif cmd.dir in (1, 3, 4):            # FWD / LEFT / RIGHT ⇒ 一律停车
                cmd = pf._emit(0, 0, 'HOLD', 'latched %dmm' % (rng or -1),
                               cmd.err, cmd.w, cmd.smooth_cx)
        sender.send(cmd.dir, cmd.spd)
        if cmd.state != last_state:
            log('  %-10s %-8s %2d%%  %s' % (cmd.state, DIR_NAME[cmd.dir],
                                            cmd.spd, cmd.reason))
            last_state = cmd.state
        if cmd.state == 'ARRIVED' and not a.follow:
            return 'ARRIVED'
        time.sleep(0.01)
    if a.follow:
        return 'FOLLOWED'
    return 'TIMEOUT'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tasks', default='any',
                    help='逗号分隔的任务序列，如 bottle / bag,bottle / any')
    ap.add_argument('--live', action='store_true', help='真发运动命令（默认 dry）')
    ap.add_argument('--follow', action='store_true',
                    help='跟随模式：找到后一直跟着走（目标远离就追），而不是到位即停')
    ap.add_argument('--task-seconds', type=float, default=60.0,
                    help='每个任务的最长时间')
    ap.add_argument('--stop-mm', type=int, default=350,
                    help='到达判据：超声波前距 <= 此值判到达')
    ap.add_argument('--scan-mode', choices=('cont', 'step'), default='cont',
                    help='cont=慢转连续看（默认，一圈约13s）；step=停稳取清晰帧')
    ap.add_argument('--scan-pct', type=int, default=3,
                    help='扫描转速占空比%%（cont 模式默认 3%%≈28°/s，拖影约1.4°不糊）')
    ap.add_argument('--scan-step-deg', type=float, default=30.0,
                    help='每步转动角度（默认 30°，视野约 65° 重叠 35°，更灵敏；须停稳取清晰帧）')
    ap.add_argument('--scan-ms', type=int, default=400, help='扫描脉冲时长 ms（≈12°/步）')
    ap.add_argument('--scan-dir', type=int, default=5, help='起始扫描方向 5=左 6=右')
    ap.add_argument('--cx-center', type=int, default=664)
    ap.add_argument('--burst-pct', type=int, default=10)
    ap.add_argument('--burst-settle', type=int, default=420)
    ap.add_argument('--db-in', type=int, default=90)
    ap.add_argument('--db-out', type=int, default=170)
    ap.add_argument('--spin-at', type=int, default=300)
    ap.add_argument('--fwd-far', type=int, default=55)
    ap.add_argument('--fwd-near', type=int, default=20)
    ap.add_argument('--d-far', type=int, default=1800)
    ap.add_argument('--d-near', type=int, default=600)
    ap.add_argument('--trim-spd', type=int, default=12)
    ap.add_argument('--slew', type=float, default=80.0)
    ap.add_argument('--approach', choices=('nav', 'servo'), default='nav',
                    help='nav=任务四式航段+绕行（默认，可绕障碍）；servo=纯视觉伺服')
    ap.add_argument('--nav-chunk', type=int, default=250,
                    help='每航段直行距离 mm（走完重新看目标重规划）')
    ap.add_argument('--nav-spd', type=int, default=16, help='导航航段前进/转向占空比%%')
    ap.add_argument('--nav-align-deg', type=float, default=12.0,
                    help='方位偏差超过此角度就先原地对准再走')
    ap.add_argument('--hdg-db', type=float, default=6.0, help='航向保持死区（度）')
    ap.add_argument('--obs-mm', type=int, default=480,
                    help='前方超声波小于此值连续2拍 ⇒ 判为障碍，触发绕行'
                         '（2026-10-25 由 380 提到 480：留出转弯/横移的余量，'
                         '否则车头贴到障碍前才开始绕，车身侧边会蹭到）')
    ap.add_argument('--avoid-deg', type=float, default=90.0,
                    help='绕行转开角度。★ 用 90°＝纯横移：车头转成与障碍垂直，'
                         '侧移时车身不再斜着往前扫，横向间隙 = 侧移距离本身。'
                         '（55° 时前进分量 57%% 会在间隙还很小的时候就推进，'
                         '实测把障碍蹭倒——见 2026-10-25 16:00 试验）')
    ap.add_argument('--avoid-mm', type=int, default=500,
                    help='绕行横向让出距离 mm（基础值；第 n 次绕行再加 (n-1)×150）。'
                         '90° 纯横移 ⇒ 这个值就是横向间隙本身')
    ap.add_argument('--avoid-max', type=int, default=12,
                    help='最多避障轮次（每轮扫一次+走一小段）')
    ap.add_argument('--avoid-side', choices=('L', 'R'), default='L',
                    help='首选绕行侧')
    ap.add_argument('--scan-span', type=float, default=60.0,
                    help='被挡时扫描半角（度）。车头会依次转到 ±span 读距离')
    ap.add_argument('--scan-step', type=float, default=30.0,
                    help='扫描步长（度）；超声波波束约 15°，30° 够用')
    ap.add_argument('--gap-step', type=int, default=600,
                    help='沿空档方向一次走多远 mm。★ 不能太短：每走一小段就要'
                         '重新扫一次净空（10~17s），实测 250mm 时 3 轮就吃掉 60s')
    ap.add_argument('--arrive-err', type=int, default=300,
                    help='判定到达还要求 |偏差| <= 此 px（否则先转正，防止侧身挤进窄缝）')
    ap.add_argument('--pass-min', type=int, default=300,
                    help='判定"能过"所需净空 mm（实测值已扣盲区80+车身120）')
    ap.add_argument('--gap-obs', type=int, default=300,
                    help='空档航段内的硬安全门限 mm（防止把墙当成空档直冲）')
    ap.add_argument('--obs-off-h', type=int, default=450,
                    help='目标框高 ≥ 此值就关闭前障判定（超声波前方只剩目标本身）')


    ap.add_argument('--beep-mm', type=int, default=480,
                    help='（已弃用）曾用于软件侧遇障报警阈值')
    ap.add_argument('--beep-hold', type=float, default=1.5,
                    help='（已弃用）')
    ap.add_argument('--arrive-mm', type=int, default=300,
                    help='★ 最终停止前距 mm（超声波+图像联合判距；默认 300，'
                         '原来只靠框高会停在 450~600mm，用户反馈偏远）')
    ap.add_argument('--final-spd', type=int, default=10,
                    help='最后贴近阶段的占空比%%（低速避免撞上目标）')
    ap.add_argument('--final-max-s', type=float, default=2.5,
                    help='最后连续贴近的最长时间（秒）。默认 2.5s：10%% 占空比约 120mm，'
                         '够从 400mm 贴到 300mm，又不会因超声波丢读数冲过头')
    ap.add_argument('--close-h-frac', type=float, default=0.90,
                    help='框高/720 ≥ 此值判到达（0.90≈0.7m）')
    a = ap.parse_args()

    tasks = [t.strip() for t in a.tasks.split(',') if t.strip()]
    for t in tasks:
        if t not in SPECS:
            print('未知目标档: %s（可选 %s）' % (t, sorted(SPECS)))
            return 2

    tlm = TlmReader('/dev/ttyUSB0')
    sender = Sender(a.live)

    print('=' * 72)
    print('任务六 语义导航/自主任务执行  任务序列: %s  %s'
          % (' → '.join(tasks), '★真实运动' if a.live else 'DRY'))
    print('=' * 72)

    now0 = time.time()
    results = []
    try:
        for ti, task in enumerate(tasks):
            spec = SPECS[task]
            print('')
            print('── 任务 %d/%d：目标[%s]  特征 w∈[%d,%d] w/h∈[%.2f,%.2f] ──'
                  % (ti + 1, len(tasks), task, spec['w_min'], spec['w_max'],
                     spec['aspect_min'], spec['aspect_max']))
            deadline = time.time() + a.task_seconds

            def log(msg):
                print('  [%5.1fs] %s' % (time.time() - now0, msg))

            det = scan_until_found(sender, spec, a, tlm, deadline, log)
            if not det:
                log('✗ 超时未找到目标[%s]' % task)
                results.append((task, 'NOT_FOUND'))
                sender.send(0, 0)
                continue

            if a.approach == 'nav':
                r = approach_nav(sender, tlm, spec, a, deadline, log, now0)
            else:
                r = approach(sender, tlm, spec, a, deadline, log, now0)
            log('★ 任务[%s] 结果: %s' % (task, r))
            results.append((task, r))
            sender.send(0, 0)
            time.sleep(0.5)
    finally:
        sender.close()
        tlm.close()

    print('')
    print('──── 任务执行汇总 ────')
    ok = sum(1 for _, r in results if r == 'ARRIVED')
    for task, r in results:
        print('  %-8s %s' % (task, r))
    print('完成 %d / %d' % (ok, len(results)))
    return 0 if ok == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
