#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""兼容跟随执行入口，保留既有控制器的命令行接口。"""
import argparse, os, socket, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
_real = sys.argv[:]
sys.argv = ['nav']
import nav_fsm_sw as nav
sys.argv = _real
import red_nav_e2e as rd

LOCAL_FRAME = os.path.join(HERE, 'live_frame.jpg')


RK_HOST = os.environ.get('RK_HOST', '10.36.218.39')
RELAY = (RK_HOST, 9100)
FRAME_URL = 'http://%s:9200/frame.jpg' % RK_HOST
SCAN_SWITCH_S = 3.0          # 搜索时每 3 秒换扫描方向


def pull_frame(timeout=3):
    import urllib.request
    try:
        with urllib.request.urlopen(FRAME_URL, timeout=timeout) as r:
            data = r.read()
        if len(data) < 2000:
            return False
        with open(LOCAL_FRAME, 'wb') as f:
            f.write(data)
        return True
    except Exception:
        return False


class Sender:
    def __init__(self, live):
        self.live = live
        self.s = None
        if live:
            self.s = socket.create_connection(RELAY, timeout=5)

    def send(self, d, sp):
        if not self.live:
            return
        try:
            self.s.sendall(('%d %d\n' % (d, sp)).encode())
        except Exception as e:
            print('  [WARN] TCP 发送失败: %s' % e)

    def close(self):
        if self.s:
            try:
                self.s.sendall(b'0 0\n')
                self.s.close()
            except Exception:
                pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--live', action='store_true', help='真发运动命令（默认 dry）')
    ap.add_argument('--follow', action='store_true', help='跟随模式：ARRIVED 不停，保持距离')
    ap.add_argument('--max-spd', type=int, default=20, help='最高速度%%（默认 20）')
    ap.add_argument('--seconds', type=int, default=25, help='最长运行秒数')
    ap.add_argument('--save-fail', action='store_true', help='保存无目标帧 fail_*.jpg')
    ap.add_argument('--deadband', type=int, default=0,
                    help='横向吸附死区 px：|cx-640|<=N 当居中直接前进（如 250）')
    ap.add_argument('--near-w', type=int, default=0, help='覆盖到达阈值（0=状态机默认 400）')
    ap.add_argument('--cx-center', type=int, default=640,
                    help='车头正前方在画面中的 x 像素（标定值；默认640=画面中心）')
    ap.add_argument('--accept-n', type=int, default=2,
                    help='连续确认帧数（默认2；转中检测不稳时降到2）')
    a = ap.parse_args()

    print('=' * 70)
    print('任务六 B 阶段：%s ｜ 限速 %d%% ｜ 最长 %d 秒 %s'
          % ('★ 真实运动' if a.live else 'DRY', a.max_spd, a.seconds,
             '｜跟随' if a.follow else ''))
    print('=' * 70)
    if a.live:
        print('⚠ 3 秒后开始；Ctrl+C 可停；板上 S1 为急停')
        time.sleep(3)

    nav.ACCEPT_N = max(1, a.accept_n)      # 覆盖连续确认帧数
    nav.CX_CENTER = a.cx_center
    print('正前方标定 cx_center=%d（画面中心 640，偏差 %+d px）' % (a.cx_center, a.cx_center - 640))
    fsm = nav.NavFsm()
    snd = Sender(a.live)
    t = 0
    t0 = time.time()
    cx_hist = []
    last_st = None
    search_t0 = time.time()

    try:
        while time.time() - t0 < a.seconds:
            cyc0 = time.time()
            if not pull_frame():
                print('  [WARN] 拉帧失败')
                time.sleep(0.2)
                continue

            det = rd.detect_red(LOCAL_FRAME)
            if det is None and a.save_fail:
                import shutil
                shutil.copy(LOCAL_FRAME, os.path.join(HERE, 'fail_%05d.jpg' % (t // 200)))

            vis = None
            if det:
                cx_hist.append(det['cx'])
                if len(cx_hist) > 3:
                    cx_hist.pop(0)
                smooth_cx = sorted(cx_hist)[len(cx_hist) // 2]      # 中位数平滑
                if a.deadband and abs(smooth_cx - nav.CX_CENTER) <= a.deadband:
                    smooth_cx = nav.CX_CENTER                       # 吸附 → 强制判居中
                vis = (det['cls'], smooth_cx, det['cy'], det['w'], det['h'], det['conf'])
                det['cx'] = smooth_cx
            else:
                cx_hist = []

            if a.near_w:
                nav.NEAR_W = a.near_w
            fsm.step(vis, 999, 0, t)
            t += 200

            st = fsm.state
            if st != last_st:
                if st == 'SEARCH':
                    search_t0 = time.time()         # 每次进入搜索重新计时
                last_st = st


            if st == 'APPROACH':
                if a.follow and det:
                    w = det['w']
                    sp = min(a.max_spd, 25 if w < 250 else (18 if w < 350 else 12))
                else:
                    sp = min(a.max_spd, nav.FWD_SPD)
                d = nav.DIR_FWD
            elif st == 'ALIGN_LEFT':
                d, sp = nav.DIR_SPIN_L, min(a.max_spd, 20)          # 原地旋转，不能差速
            elif st == 'ALIGN_RIGHT':
                d, sp = nav.DIR_SPIN_R, min(a.max_spd, 20)
            elif st == 'SEARCH':
                if det:

                    d, sp = nav.DIR_STOP, 0
                else:
                    phase = int((time.time() - search_t0) / SCAN_SWITCH_S) % 2
                    d = nav.DIR_SPIN_L if phase == 0 else nav.DIR_SPIN_R
                    sp = min(a.max_spd, 20)
            else:
                d, sp = nav.DIR_STOP, 0                             # ARRIVED/LOST/OBSTACLE/ESTOP

            snd.send(d, sp)

            cyc = (time.time() - cyc0) * 1000
            det_s = ('cx=%-4d w=%-4d conf=%d' % (det['cx'], det['w'], det['conf'])) if det else '无目标'
            print('[%5.1fs] %-13s %-26s → dir=%d spd=%-2d (%.0fms)'
                  % (time.time() - t0, st, det_s, d, sp, cyc))

            if st == 'ARRIVED' and not a.follow:
                print('  ★ 已到达目标附近')
                break
            if t >= 200 * 2000:
                break
    except KeyboardInterrupt:
        print('\n[人工中断]')
    finally:
        snd.close()
        print('已发停车并断开（中继 2s 后停发 → gate 超时自动停车）')


if __name__ == '__main__':
    main()
