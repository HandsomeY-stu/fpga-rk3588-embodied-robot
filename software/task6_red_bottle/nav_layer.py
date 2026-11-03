#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""导航任务的策略封装，连接观测、阶段切换及运动输出。"""
import time

DIR_FWD, DIR_BACK = 1, 2
DIR_LEFT, DIR_RIGHT = 3, 4
DIR_SPIN_L, DIR_SPIN_R, DIR_STOP = 5, 6, 0

DEG_PER_PX = 1.0 / 19.7


SIDE_BLIND_MM = 80.0


BODY_MARGIN_MM = 120.0


def wrap(a):
    while a > 180:
        a -= 360
    while a < -180:
        a += 360
    return a


class NavCtl(object):
    """航段执行器：航向保持直行 / 定角转 / 绕行"""

    def __init__(self, sender, tlm, log=print, hdg_db=6.0, hdg_gain=1.0):
        self.sender = sender
        self.tlm = tlm
        self.log = log
        self.hdg_db = hdg_db          # 航向死区（度），小于它不纠
        self.hdg_gain = hdg_gain
        self.travelled_mm = 0.0       # 累计里程（调试用）


    def send(self, d, spd):
        try:
            self.sender.send(d, spd)
        except Exception:
            pass

    def stop(self):
        self.send(DIR_STOP, 0)

    def yaw(self):
        return self.tlm.yaw_deg if self.tlm else None

    def speed_mm_s(self):
        """(左轮+右轮)/2，取前进方向为正（encoder 单位 mm/s）"""
        if not self.tlm:
            return 0.0
        l, r = self.tlm.vell, self.tlm.velr
        if l is None or r is None:
            return 0.0
        return (l + r) / 2.0

    def rng(self):
        try:
            return self.tlm.fresh_range()
        except Exception:
            return None


    def turn_to(self, target_yaw, pct=12, tol=6.0, timeout=10.0, settle_ms=350):
        """转到指定 yaw。yaw 需减小→SPIN_L，需增大→SPIN_R。"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            y = self.yaw()
            if y is None:
                self.stop()
                return False
            d = wrap(y - target_yaw)
            if abs(d) <= tol:
                self.stop()
                time.sleep(settle_ms / 1000.0)
                return True

            spd = pct if abs(d) > 25 else max(8, int(pct * abs(d) / 25.0))
            self.send(DIR_SPIN_L if d > 0 else DIR_SPIN_R, spd)
            time.sleep(0.05)
        self.stop()
        return False


    def fwd(self, dist_mm, heading, spd=16, deadline=None,
            obs_mm=380, obs_confirm=2, stall_s=3.0, stall_mm=8.0):
        """沿 heading 直行 dist_mm（里程积分判结束）。"""
        if deadline is None:
            deadline = time.time() + 30.0
        t_prev = time.time()
        moved = 0.0
        obs = 0


        live = getattr(self.sender, 'live', True)
        prog_moved = 0.0
        prog_t = time.time()
        while time.time() < deadline:
            now = time.time()
            dt = now - t_prev
            t_prev = now

            v = self.speed_mm_s()
            if v <= 0 and (not self.tlm or self.tlm.vell is None
                           or self.tlm.velr is None):


                v = spd * 4.8
            if v > 0:
                moved += v * dt
                self.travelled_mm += v * dt
            if moved >= dist_mm:
                self.stop()
                return 'DONE', moved


            if live:
                if moved - prog_moved >= stall_mm:
                    prog_moved = moved
                    prog_t = now
                elif now - prog_t >= stall_s:
                    self.stop()
                    self.log('  FWD  ★%.1fs 内里程只涨 %.0fmm（轮速≈0）⇒ 判定走不动'
                             '（电机没电/堵转/编码器断），提前停车'
                             % (now - prog_t, moved - prog_moved))
                    return 'STALL', moved


            r = self.rng() if obs_mm and obs_mm > 0 else None
            if r is not None and r < obs_mm:
                obs += 1
                if obs >= obs_confirm:
                    self.stop()
                    return 'OBSTACLE', moved
            else:
                obs = 0


            y = self.yaw()
            if y is None or heading is None:
                self.send(DIR_FWD, spd)
            else:
                e = wrap(y - heading)      # e>0 ⇒ yaw 偏大 ⇒ 需减小 ⇒ 左转
                if e > self.hdg_db:
                    self.send(DIR_LEFT, spd)
                elif e < -self.hdg_db:
                    self.send(DIR_RIGHT, spd)
                else:
                    self.send(DIR_FWD, spd)
            time.sleep(0.05)
        self.stop()
        return 'TIMEOUT', moved

    def _back(self, dist_mm, spd=10, deadline=None):
        """倒车 dist_mm（按里程/名义速度积分）。超声波在车头，倒车是盲区，"""
        if deadline is None:
            deadline = time.time() + 6.0
        t_prev = time.time()
        moved = 0.0
        while time.time() < deadline and moved < dist_mm:
            now = time.time()
            dt = now - t_prev
            t_prev = now
            v = abs(self.speed_mm_s())
            if v <= 0:
                v = spd * 4.8
            moved += v * dt
            self.send(DIR_BACK, spd)
            time.sleep(0.05)
        self.stop()
        return moved

    def polar_scan(self, y0, span=90.0, step=30.0, settle=0.18, n=2, pct=10):
        """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""


        offs = []
        k = int(round(span / step))

        for i in range(k + 1):
            offs.append(-i * step)
            if i:
                offs.append(i * step)
        res = []
        for off in offs:
            b = wrap(y0 + off)
            if not self.turn_to(b, pct=pct, tol=10.0, timeout=6.0,
                                settle_ms=120):
                res.append((off, None))
                continue
            time.sleep(settle)
            vals = []
            for _ in range(n):
                r = self.rng()
                if r:
                    vals.append(r)
                time.sleep(0.11)
            res.append((off, max(vals) if vals else None))
        self.turn_to(y0, pct=pct, tol=10.0, settle_ms=120)
        return res

    @staticmethod
    def pick_gap(profile, target_off=0.0, min_clear=300.0, min_gain=250.0,
                 bad_yaws=None, cur_yaw=None, bad_tol=25.0, sector=True):
        """从极坐标净空表里挑一个"真能走过去"的空档"""
        dc = {}
        ahead = None
        for off, d in profile:
            if d is None:
                dc[off] = None
                continue
            dc[off] = d - SIDE_BLIND_MM - BODY_MARGIN_MM
            if off == 0:
                ahead = dc[off]

        offs = sorted(dc.keys())
        step = (offs[1] - offs[0]) if len(offs) > 1 else 30.0

        def _bad(o):
            if not (bad_yaws and cur_yaw is not None):
                return False
            absy = wrap(cur_yaw + o)
            return any(abs(wrap(absy - b)) < bad_tol for b in bad_yaws)


        sector_c = []
        loose_c = []
        for o, c in dc.items():
            if c is None or c < min_clear or o == 0 or _bad(o):
                continue
            if ahead is not None and c < ahead + min_gain:
                continue
            sgn = 1 if o > 0 else -1
            outward = dc.get(o + sgn * step)
            if outward is None:
                loose_c.append((o, c, 'outward_unknown'))
            elif outward < min_clear:
                continue                     # 朝外也被挡 ⇒ 这是"缝"，跳过
            else:
                sector_c.append((o, min(c, outward), 'sector'))
        cands = sector_c or [(o, c) for o, c, _ in loose_c]
        if not cands:
            return None, None
        best = min(cands, key=lambda x: abs(wrap(x[0] - target_off)))
        return best[0], best[1]

    def probe_sides(self, y0, pct=16, deg=90.0, settle=0.35, n=4):
        """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
        res = {}
        for s, sign in (('L', -1.0), ('R', +1.0)):
            if not self.turn_to(wrap(y0 + sign * deg), pct=max(14, pct), tol=8.0):
                res[s] = None
                continue
            time.sleep(settle)
            vals = []
            for _ in range(n):
                r = self.rng()
                if r:
                    vals.append(r)
                time.sleep(0.12)
            res[s] = max(vals) if vals else None
        self.turn_to(y0, pct=max(14, pct), tol=8.0)
        return res


    def detour(self, side, deg=90.0, dist_mm=400.0, spd=16, timeout=25.0,
               obs_mm=260, back_mm=0.0):
        """绕行一个航段：转开 deg 度 → 走 dist_mm → 转回原航向。"""
        t_end = time.time() + timeout
        y0 = self.yaw()
        if y0 is None:
            self.log('  DETOUR 无 yaw，放弃绕行')
            return False
        sign = +1.0 if side == 'R' else -1.0      # R: yaw 增大
        y1 = wrap(y0 + sign * deg)
        self.log('  DETOUR %s 侧让：yaw %.1f → %.1f（%.0f°），再走 %.0fmm'
                 % (side, y0, y1, deg, dist_mm))

        if back_mm > 0:
            self.log('  DETOUR 先倒 %.0fmm 留转弯余量' % back_mm)
            self._back(back_mm, spd=max(8, spd - 6))
        if not self.turn_to(y1, pct=max(14, spd), tol=5.0,
                            timeout=min(10.0, max(2.0, t_end - time.time()))):
            return False


        st, mv = self.fwd(dist_mm, y1, spd=spd,
                          deadline=min(t_end, time.time() + 10.0),
                          obs_mm=obs_mm)
        if st == 'OBSTACLE':
            self.log('  DETOUR %s 侧向也被挡住（走了 %.0fmm），放弃这一侧'
                     % (side, mv))
            self.turn_to(y0, pct=max(12, spd), tol=6.0, timeout=8.0)
            return False
        if st == 'STALL':
            self.log('  DETOUR %s 侧移走不动（只走了 %.0fmm），回正航向后放弃'
                     % (side, mv))
            self.turn_to(y0, pct=max(12, spd), tol=6.0, timeout=8.0)
            return False
        self.turn_to(y0, pct=max(12, spd), tol=6.0, timeout=8.0)
        self.log('  DETOUR %s 完成（侧移 %.0fmm），回到 yaw %.0f°'
                 % (side, mv, y0))
        return True
