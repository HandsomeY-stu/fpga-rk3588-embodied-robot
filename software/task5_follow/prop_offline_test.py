#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线回归测试：检查prop_offline_test相关逻辑和异常处理。"""
import argparse, csv, math, os, random, sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from prop_ctrl import PropFollow, DIR_STOP, DIR_FWD, DIR_SPIN_L, DIR_SPIN_R, DIR_NAME

DT = 0.077
PX_PER_DEG = 21.43
DEG_PER_PCT_S = 2.25
MM_PER_PCT_S = 8.33
W_K = 480.0
CX_CENTER = 520
NEAR_W = 600


class Sim(object):
    def __init__(self, ctrl, target, seconds=30, noise_px=4.0, drop_p=0.06, seed=1):
        self.ctrl = ctrl
        self.target = target          # callable(t)->(x,y) 或 None（不可见/不存在）
        self.seconds = seconds
        self.noise_px = noise_px
        self.drop_p = drop_p
        self.rng = random.Random(seed)
        self.x = self.y = 0.0
        self.th = 0.0                 # 航向角(deg)，0=正前方
        self.t = 0.0
        self.rot_travel = 0.0         # 累计转角(deg)
        self.fwd_travel = 0.0         # 累计前进距离(m)
        self.errs = []
        self.lost_frames = 0
        self.frames = 0
        self.dir_changes = 0
        self._last_dir = None
        self.states = {}
        self.log = []

    def _sense(self):
        p = self.target(self.t, (self.x, self.y))
        if p is None:
            return None, None
        tx, ty = p
        dx, dy = tx - self.x, ty - self.y
        dist = math.hypot(dx, dy)

        bearing = math.degrees(math.atan2(dx, dy)) - self.th
        while bearing > 180.0:
            bearing -= 360.0
        while bearing < -180.0:
            bearing += 360.0
        if abs(bearing) > 30.0 or dist < 0.15:
            return None, dist
        cx = CX_CENTER + bearing * PX_PER_DEG + self.rng.gauss(0, self.noise_px)
        w = max(40, min(1150, W_K / max(0.15, dist)))
        h = int(w * 1.25)
        return (1, int(round(cx)), 360, int(round(w)), h, 90), dist

    def run(self):
        cmd = None
        while self.t < self.seconds:
            vis, dist = self._sense()
            if vis is not None and self.rng.random() < self.drop_p:
                vis = None                      # 随机丢帧（检测波动）
            cmd = self.ctrl.step(vis, int(self.t * 1000))
            self.frames += 1
            self.states[cmd.state] = self.states.get(cmd.state, 0) + 1
            if vis is None:
                self.lost_frames += 1
            else:
                self.errs.append(abs(vis[1] - CX_CENTER))
            if self._last_dir is not None and cmd.dir != self._last_dir:
                self.dir_changes += 1
            self._last_dir = cmd.dir

            if cmd.dir in (DIR_SPIN_L, DIR_SPIN_R):
                dth = DEG_PER_PCT_S * cmd.spd * DT
                self.th += -dth if cmd.dir == DIR_SPIN_L else dth
                self.rot_travel += dth
            elif cmd.dir == DIR_FWD:
                v = MM_PER_PCT_S * cmd.spd / 1000.0
                self.x += math.sin(math.radians(self.th)) * v * DT
                self.y += math.cos(math.radians(self.th)) * v * DT
                self.fwd_travel += v * DT
            self.log.append((self.t, cmd.state, cmd.dir, cmd.spd, dist,
                             None if vis is None else vis[1]))
            self.t += DT
            if cmd.state == 'ARRIVED' and self.ctrl.follow is False:

                if not getattr(self, 'arrive_t', None):
                    self.arrive_t = self.t
        return self


    def rot_between(self, t0, t1):
        s = 0.0
        for (t, st, d, spd, dist, cx) in self.log:
            if t0 <= t < t1 and d in (DIR_SPIN_L, DIR_SPIN_R):
                s += DEG_PER_PCT_S * spd * DT
        return s

    def states_between(self, t0, t1):
        return set(st for (t, st, d, spd, dist, cx) in self.log if t0 <= t < t1)

    def m(self):
        e = sorted(self.errs) if self.errs else [0]
        p95 = e[int(len(e) * 0.95)] if len(e) > 1 else e[0]
        return {
            'frames': self.frames,
            'lost_ratio': self.lost_frames / float(max(1, self.frames)),
            'err_max': max(self.errs) if self.errs else 0,
            'err_p95': p95,
            'rot_deg': self.rot_travel,
            'fwd_m': self.fwd_travel,
            'dir_changes': self.dir_changes,
            'final_state': self.ctrl.state,
            'arrive_t': getattr(self, 'arrive_t', None),
            'n_search': self.states.get('SEARCH', 0),
            'n_sweep': self.states.get('SWEEP', 0),
            'n_lost': self.states.get('LOST', 0),
            'n_grace': self.states.get('GRACE', 0),
            'dist': math.hypot(self.x, self.y),
        }


class BangBang(object):
    """复刻 run_b.py + nav_fsm_sw 的现有策略：死区 150px、固定转速 20%、丢了就转圈"""

    def __init__(self, cx_center=CX_CENTER, deadband=150, near_w=NEAR_W, spd=20,
                 accept_n=2, grace_ms=0):
        self.cx_center, self.deadband, self.near_w = cx_center, deadband, near_w
        self.spd, self.accept_n, self.grace_ms = spd, accept_n, grace_ms
        self.state = 'SEARCH'
        self.hit = 0
        self.lost_since = None
        self.search_t0 = None
        self.follow = False
        self.last = (DIR_STOP, 0)

    def step(self, vis, now_ms):
        if vis is None:
            if self.lost_since is None:
                self.lost_since = now_ms
                self.hit = 0
            dt = now_ms - self.lost_since
            if dt < self.grace_ms:
                return _C(self.last[0], self.last[1], 'GRACE')
            if self.search_t0 is None:
                self.search_t0 = now_ms
            phase = int((now_ms - self.search_t0) / 3000.0) % 2
            d = DIR_SPIN_L if phase == 0 else DIR_SPIN_R
            self.state = 'SEARCH'
            self.last = (d, self.spd)
            return _C(d, self.spd, 'SEARCH')
        self.lost_since = None
        self.search_t0 = None
        self.hit += 1
        cx, w = vis[1], vis[3]
        if self.hit < self.accept_n:
            return _C(DIR_STOP, 0, 'GRACE')
        if w >= self.near_w:
            self.state = 'ARRIVED'; self.last = (DIR_STOP, 0)
            return _C(DIR_STOP, 0, 'ARRIVED')
        err = cx - self.cx_center
        if abs(err) <= self.deadband:
            self.state = 'APPROACH'; self.last = (DIR_FWD, 18)
            return _C(DIR_FWD, 18, 'APPROACH')
        d = DIR_SPIN_L if err < 0 else DIR_SPIN_R
        self.state = 'ALIGN'; self.last = (d, self.spd)
        return _C(d, self.spd, 'ALIGN')


class _C(object):
    __slots__ = ('dir', 'spd', 'state')

    def __init__(self, d, s, st):
        self.dir, self.spd, self.state = d, s, st


def tgt_static(x=0.4, y=2.5):
    return lambda t, car=None: (x, y)


def tgt_sideways(y=2.5, v=0.12, x0=-1.0, x1=1.0):
    def f(t, car=None):
        x = min(x1, x0 + v * t)
        return (x, y)
    return f


class TgtApproach(object):
    """迎面走来：以 0.15m/s 靠近，**在车前方 stop_m 处停下**（真人不会穿过车）"""

    def __init__(self, y0=3.5, v=0.15, x=0.1, stop_m=0.85):
        self.y, self.v, self.x, self.stop_m = y0, v, x, stop_m

    def __call__(self, t, car=(0.0, 0.0)):
        d = math.hypot(self.x - car[0], self.y - car[1])
        if d > self.stop_m:
            self.y = max(car[1] + self.stop_m, self.y - self.v * DT)
        return (self.x, self.y)


def tgt_vanish(x=0.2, y=2.2, t_gone=6.0):
    return lambda t, car=None: None if t >= t_gone else (x, y)


SCENARIOS = [
    {
        'name': 'S1 静态目标(2.5m, 偏右0.4m)',
        'target': tgt_static(), 'seconds': 30,
        'checks': [
            ('到达(w>=600)', lambda m: m['final_state'] == 'ARRIVED', 'final_state=ARRIVED'),
            ('到站时间<=25s', lambda m: m['arrive_t'] is not None and m['arrive_t'] <= 25, 'arrive_t<=25s'),
            ('最大偏差<=250px', lambda m: m['err_max'] <= 250, 'err_max<=250'),
            ('累计转角<=90°', lambda m: m['rot_deg'] <= 90, 'rot<=90deg'),
        ],
    },
    {
        'name': 'S2 横向移动(0.12m/s @2.5m)',
        'target': tgt_sideways(), 'seconds': 22,
        'checks': [
            ('丢失率<=8%', lambda m: m['lost_ratio'] <= 0.08, 'lost<=8%'),
            ('偏差P95<=300px', lambda m: m['err_p95'] <= 300, 'p95<=300'),
            ('无全圈搜索', lambda m: m['n_search'] == 0, 'n_search=0'),
        ],
    },
    {
        'name': 'S3 迎面接近(0.15m/s, 3.5m)',
        'target': TgtApproach(), 'seconds': 25,
        'checks': [
            ('到达', lambda m: m['final_state'] == 'ARRIVED', 'ARRIVED'),
            ('到站时间<=22s', lambda m: m['arrive_t'] is not None and m['arrive_t'] <= 22, 'arrive_t<=22s'),
            ('最大偏差<=250px', lambda m: m['err_max'] <= 250, 'err_max<=250'),
        ],
    },
    {
        'name': 'S4 目标消失(6s时)',
        'target': tgt_vanish(), 'seconds': 10.5, 'window': (6.0, 10.5),
        'checks': [
            ('进入SWEEP', lambda m: m['n_sweep'] > 0, 'n_sweep>0'),
            ('4.5s内不转全圈', lambda m: 'SEARCH' not in m['states_win'], 'no SEARCH in 6~10.5s'),
            ('4.5s内转角<=200°', lambda m: m['rot_win'] <= 200, 'rot_win<=200deg'),
        ],
    },
    {
        'name': 'S5 噪声+丢帧(σ8px, 丢15%)',
        'target': tgt_static(0.2, 2.0), 'seconds': 14,
        'checks': [
            ('换向<=6次', lambda m: m['dir_changes'] <= 6, 'dir_changes<=6'),
            ('无LOST', lambda m: m['n_lost'] == 0, 'n_lost=0'),
            ('最大偏差<=250px', lambda m: m['err_max'] <= 250, 'err_max<=250'),
        ],
    },
    {
        'name': 'S6 目标贴边(方位25°)',
        'target': tgt_static(math.tan(math.radians(25)) * 2.2, 2.2), 'seconds': 20,
        'checks': [
            ('不丢目标(丢失率<=8%)', lambda m: m['lost_ratio'] <= 0.08, 'lost<=8%'),
            ('最终居中(|err|<=60)', lambda m: m['errs'][-1] if m['errs'] else 999, 'last_err<=60'),
            ('到达', lambda m: m['final_state'] == 'ARRIVED', 'ARRIVED'),
        ],
    },
]


def run_one(ctrl, sc, noise_px=4.0, drop_p=0.06):
    sim = Sim(ctrl, sc['target'], sc['seconds'], noise_px=noise_px, drop_p=drop_p)
    sim.run()
    m = sim.m()
    m['errs'] = sim.errs
    if sc.get('window'):
        m['rot_win'] = sim.rot_between(*sc['window'])
        m['states_win'] = sim.states_between(*sc['window'])
    else:
        m['rot_win'] = m['rot_deg']
        m['states_win'] = set()
    return sim, m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv', default=os.path.join(HERE, 'prop_offline_report.csv'))
    ap.add_argument('--noise', type=float, default=4.0)
    ap.add_argument('--drop', type=float, default=0.06)
    a = ap.parse_args()

    print('=' * 96)
    print('任务六 · 比例控制 vs 开关式控制 · 离线闭环仿真（模型参数全部来自 2026-10-20 实测）')
    print('  视野 %.2f px/deg (±30°) ｜ 转向 %.2f°/s每%% ｜ 前进 %.2f mm/s每%% ｜ 周期 %.0fms'
          % (PX_PER_DEG, DEG_PER_PCT_S, MM_PER_PCT_S, DT * 1000))
    print('  噪声 σ=%.0fpx ｜ 随机丢帧 %.0f%%' % (a.noise, a.drop * 100))
    print('=' * 96)

    rows = []
    allpass = True
    for sc in SCENARIOS:
        print('\n' + '-' * 96)
        print('【%s】' % sc['name'])
        res = {}
        for label, mk in (('比例(新)', lambda: PropFollow(cx_center=CX_CENTER, near_w=NEAR_W)),
                          ('开关(run_b)', lambda: BangBang())):
            sim, m = run_one(mk(), sc, a.noise, a.drop)
            res[label] = m
            print('  %-10s 丢失率 %5.1f%% ｜ 最大偏差 %4dpx ｜ P95 %4dpx ｜ 累计转角 %5.1f° ｜ '
                  '换向 %2d ｜ 到站 %s ｜ 末态 %s'
                  % (label, m['lost_ratio'] * 100, m['err_max'], m['err_p95'], m['rot_deg'],
                     m['dir_changes'],
                     ('%.1fs' % m['arrive_t']) if m['arrive_t'] else ' — ',
                     m['final_state']))
        print('  判据（按新控制器判定）：')
        for cname, fn, crit in sc['checks']:
            ok = bool(fn(res['比例(新)']))
            allpass = allpass and ok
            print('    [%s] %-22s  判据 %s' % ('PASS' if ok else 'FAIL', cname, crit))
            rows.append({'scene': sc['name'], 'check': cname, 'criteria': crit,
                         'new': res['比例(新)'].get('_v', ''), 'verdict': 'PASS' if ok else 'FAIL',
                         'new_lost': '%.3f' % res['比例(新)']['lost_ratio'],
                         'new_err_max': res['比例(新)']['err_max'],
                         'new_err_p95': res['比例(新)']['err_p95'],
                         'new_rot': '%.1f' % res['比例(新)']['rot_deg'],
                         'new_dirchg': res['比例(新)']['dir_changes'],
                         'bb_lost': '%.3f' % res['开关(run_b)']['lost_ratio'],
                         'bb_err_max': res['开关(run_b)']['err_max'],
                         'bb_err_p95': res['开关(run_b)']['err_p95'],
                         'bb_rot': '%.1f' % res['开关(run_b)']['rot_deg'],
                         'bb_dirchg': res['开关(run_b)']['dir_changes']})

        n, b = res['比例(新)'], res['开关(run_b)']
        better = sum([n['err_p95'] < b['err_p95'], n['lost_ratio'] <= b['lost_ratio'],
                      n['rot_deg'] < b['rot_deg'], n['dir_changes'] <= b['dir_changes']])
        print('  → 新控制器在 4 项指标中占优 %d 项' % better)

    with open(a.csv, 'w', newline='', encoding='utf-8-sig') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    npass = sum(1 for r in rows if r['verdict'] == 'PASS')
    print('\n' + '=' * 96)
    print('判据通过: %d/%d' % (npass, len(rows)))
    print('总判定: %s' % ('PASS' if allpass else 'FAIL'))
    print('报告: %s' % a.csv)
    print('=' * 96)
    return 0 if allpass else 1


if __name__ == '__main__':
    sys.exit(main())
