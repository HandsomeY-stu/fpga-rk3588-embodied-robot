#!/usr/bin/env python3
"""对给定录制日志执行历史A/B策略重放；不连接设备。"""
import argparse, csv, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
_argv = sys.argv[:]
sys.argv = ['nav']
import nav_fsm_sw as nav
sys.argv = _argv
import red_nav_e2e as rd
from prop_ctrl import PropFollow, DIR_NAME
DT_MS = 77
SCAN_SWITCH_S = 3.0
SEQ = [(None, 'S0 无目标（启动）'), (None, 'S0 无目标（启动）'), (None, 'S0 无目标（启动）'), ('look.jpg', 'S1 目标在画面左下'), ('look.jpg', 'S1 目标在画面左下'), ('now.jpg', 'S2 目标偏右 cx=600'), ('now.jpg', 'S2 目标偏右 cx=600'), ('t.jpg', 'S3 目标接近居中 cx=524'), ('t.jpg', 'S3 目标接近居中 cx=524'), ('f0.jpg', 'S4 静止目标 8 帧'), ('f1.jpg', 'S4 静止目标 8 帧'), ('f2.jpg', 'S4 静止目标 8 帧'), ('f3.jpg', 'S4 静止目标 8 帧'), ('f4.jpg', 'S4 静止目标 8 帧'), ('f5.jpg', 'S4 静止目标 8 帧'), ('f6.jpg', 'S4 静止目标 8 帧'), ('f7.jpg', 'S4 静止目标 8 帧'), ('chk1.jpg', 'S5 目标右移 cx=691 w=600'), ('chk0.jpg', 'S5 目标右移 cx=808 w=560'), ('chk.jpg', 'S5 目标右移 cx=770 w=600'), ('live_frame.jpg', 'S6 目标回到左侧 cx=495 w=680'), ('diag.jpg', 'S6 目标居中 cx=637 w=640'), (None, 'S7 目标离开'), (None, 'S7 目标离开'), (None, 'S7 目标离开'), (None, 'S7 目标离开'), (None, 'S7 目标离开'), (None, 'S7 目标离开'), (None, 'S7 目标离开'), (None, 'S7 目标离开'), (None, 'S7 目标离开'), (None, 'S7 目标离开'), (None, 'S7 目标离开'), (None, 'S7 目标离开')]

class RunB(object):
    """run_b.py 的真实决策路径（逻辑原样复制自 run_b.py L98-167）"""

    def __init__(self, cx_center=520, deadband=0, near_w=600, max_spd=18, accept_n=2, follow=True):
        nav.ACCEPT_N = accept_n
        nav.CX_CENTER = cx_center
        nav.NEAR_W = near_w
        self.fsm = nav.NavFsm()
        self.deadband, self.max_spd, self.follow = (deadband, max_spd, follow)
        self.cx_hist = []
        self.search_t0 = 0.0

    def step(self, det, t_ms):
        vis = None
        if det:
            self.cx_hist.append(det['cx'])
            if len(self.cx_hist) > 3:
                self.cx_hist.pop(0)
            smooth_cx = sorted(self.cx_hist)[len(self.cx_hist) // 2]
            if self.deadband and abs(smooth_cx - nav.CX_CENTER) <= self.deadband:
                smooth_cx = nav.CX_CENTER
            vis = (det['cls'], smooth_cx, det['cy'], det['w'], det['h'], det['conf'])
            self.last_smooth = smooth_cx
        else:
            self.cx_hist = []
            self.last_smooth = None
        self.fsm.step(vis, 999, 0, t_ms)
        st = self.fsm.state
        if st == 'SEARCH' and (not det):
            pass
        if st == 'APPROACH':
            if self.follow and det:
                w = det['w']
                sp = min(self.max_spd, 25 if w < 250 else 18 if w < 350 else 12)
            else:
                sp = min(self.max_spd, nav.FWD_SPD)
            d = nav.DIR_FWD
        elif st == 'ALIGN_LEFT':
            d, sp = (nav.DIR_SPIN_L, min(self.max_spd, 20))
        elif st == 'ALIGN_RIGHT':
            d, sp = (nav.DIR_SPIN_R, min(self.max_spd, 20))
        elif st == 'SEARCH':
            if det:
                d, sp = (nav.DIR_STOP, 0)
            else:
                phase = int((t_ms / 1000.0 - self.search_t0) / SCAN_SWITCH_S) % 2
                d = nav.DIR_SPIN_L if phase == 0 else nav.DIR_SPIN_R
                sp = min(self.max_spd, 20)
        else:
            d, sp = (nav.DIR_STOP, 0)
        return (st, d, sp)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv', default=os.path.join(HERE, 'replay_ab.csv'))
    ap.add_argument('--cx-center', type=int, default=520)
    a = ap.parse_args()
    rb = RunB(cx_center=a.cx_center, deadband=0, near_w=600, max_spd=18)
    rc = PropFollow(cx_center=a.cx_center, near_w=600, db_in=30, db_out=60, spd_spin_max=16, spd_fwd_far=16, spd_fwd_near=10, follow=True)
    print('=' * 104)
    print('063 第一阶段 · 同一组帧回放 run_b vs run_c（cx_center=%d）' % a.cx_center)
    print('  周期 %dms ｜ run_b: deadband=0 max_spd=18 ｜ run_c: db 30/60 max_spd=16' % DT_MS)
    print('=' * 104)
    hdr = '%-5s %-34s %-22s | %-24s | %s' % ('t(s)', '检测', 'run_b', 'run_c', '差异')
    print(hdr)
    print('-' * 104)
    rows = []
    diff_frames = 0
    for i, (fn, label) in enumerate(SEQ):
        t_ms = i * DT_MS
        det = None
        if fn:
            det = rd.detect_red(os.path.join(HERE, fn))
        b_st, b_d, b_sp = rb.step(det, t_ms)
        c_cmd = rc.step((det['cls'], det['cx'], det['cy'], det['w'], det['h'], det['conf']) if det else None, t_ms)
        d_s = 'raw=%-4d bsm=%-4d w=%-4d err=%+4d' % (det['cx'], rb.last_smooth if rb.last_smooth is not None else -1, det['w'], det['cx'] - a.cx_center) if det else '无目标'
        b_s = '%-12s %-5s %2d%%' % (b_st, DIR_NAME[b_d], b_sp)
        c_s = '%-9s %-5s %2d%%' % (c_cmd.state, DIR_NAME[c_cmd.dir], c_cmd.spd)
        same = 'same' if b_d == c_cmd.dir and b_sp == c_cmd.spd else 'DIFF'
        if same == 'DIFF':
            diff_frames += 1
        print('%-5.2f %-34s %-22s | %-24s | %s' % (t_ms / 1000.0, d_s, b_s, c_s, same))
        rows.append({'t_ms': t_ms, 'seg': label, 'file': fn or '', 'cx': det['cx'] if det else '', 'w': det['w'] if det else '', 'err': det['cx'] - a.cx_center if det else '', 'b_state': b_st, 'b_dir': b_d, 'b_spd': b_sp, 'c_state': c_cmd.state, 'c_dir': c_cmd.dir, 'c_spd': c_cmd.spd, 'same': same})
    with open(a.csv, 'w', newline='', encoding='utf-8-sig') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print('-' * 104)
    print('帧数 %d ｜ 命令不同 %d 帧（%.0f%%）' % (len(rows), diff_frames, 100.0 * diff_frames / len(rows)))
    print('run_b 用到的方向码: %s' % sorted(set((r['b_dir'] for r in rows))))
    print('run_c 用到的方向码: %s' % sorted(set((r['c_dir'] for r in rows))))
    print('报告: %s' % a.csv)
    return 0
if __name__ == '__main__':
    sys.exit(main())
