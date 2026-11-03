#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""连续跟随执行入口，连接视觉、遥测和差速控制。"""
import argparse, csv, os, socket, sys, threading, time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import red_nav_e2e as rd
import telemetry_log as tl          # 复用 decode_tlm / parse_stream
from prop_ctrl import PropFollow, DIR_STOP, DIR_NAME

LOCAL_FRAME = os.path.join(HERE, 'live_frame_c.jpg')


RK_HOST = os.environ.get('RK_HOST', '10.36.218.39')
RELAY = (RK_HOST, 9100)
FRAME_URL = 'http://%s:9200/frame.jpg' % RK_HOST


def pull_frame(timeout=3.0):
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


class Sender(object):
    def __init__(self, live):
        self.live = live
        self.s = None
        self._beep_on = False      # 当前蜂鸣器请求状态
        self._beep_ms = 0          # 上次实发时刻
        if live:
            self.s = socket.create_connection(RELAY, timeout=5)

    def send(self, d, sp):
        if not self.live:
            return True
        try:
            self.s.sendall(('%d %d\n' % (d, sp)).encode())


            if self._beep_on:
                now = int(time.time() * 1000)
                if now - self._beep_ms > 1500:
                    self.s.sendall(b'beep 1\n')
                    self._beep_ms = now
            return True
        except Exception as e:
            print('  [WARN] TCP 发送失败: %s' % e)
            return False


    def mode(self, m):
        """请求切模式（经中继转发）。0 = MODE_MANUAL（关自动避障报警）。"""
        if not self.live:
            return True
        try:
            self.s.sendall(('mode %d\n' % (int(m) & 0xFF)).encode())
            return True
        except Exception as e:
            print('  [WARN] 模式命令发送失败: %s' % e)
            return False

    def beep(self, on):
        if not self.live:
            return True
        try:
            self.s.sendall(('beep %d\n' % (1 if on else 0)).encode())
            self._beep_on = bool(on)
            self._beep_ms = int(time.time() * 1000)
            return True
        except Exception as e:
            print('  [WARN] 蜂鸣器命令发送失败: %s' % e)
            return False

    def set_beep(self, on, now_ms=None):
        """请求蜂鸣器状态。返回是否真的发了。"""
        now = now_ms if now_ms is not None else int(time.time() * 1000)
        want = bool(on)
        if want != self._beep_on:
            return self.beep(want)                 # 状态变化 → 立即发
        if want and (now - self._beep_ms > 1500):
            return self.beep(True)                 # 保持响 → 保活
        return False

    def beep_pattern(self, n=2, on_ms=220, off_ms=260):
        """鸣 n 声（用于"到达到目标"提示音）。阻塞，n=2 约 0.96s。"""
        for i in range(n):
            self.beep(True)
            time.sleep(on_ms / 1000.0)
            self.beep(False)
            if i != n - 1:
                time.sleep(off_ms / 1000.0)

    def close(self):
        if self.s:
            try:
                self.s.sendall(b'mode 0\n')     # 归还 MANUAL：防 av_near 残留长响
                time.sleep(0.05)
                self.s.sendall(b'beep 0\n')
                self.s.sendall(b'0 0\n')
                time.sleep(0.05)
                self.s.close()
            except Exception:
                pass


class TlmReader(object):
    """后台线程读 FPGA 遥测（/dev/ttyUSB0，115200，5Hz），主循环非阻塞取最新值。"""

    def __init__(self, port=None, baud=115200):
        self.port = port
        self.ok = False
        self.range_mm = None
        self.yaw_deg = None
        self.state = None
        self.vell = self.velr = None
        self.last_ms = 0.0
        self.n = 0
        self.bad = 0
        self._buf = b''
        self._stop = False
        self._ser = None
        if not port or str(port).lower() in ('none', 'off', ''):
            print('  遥测: 已关闭（只靠画面判距）')
            return
        try:
            import serial
            self._ser = serial.Serial(port, baud, timeout=0.2)
            self.ok = True
            threading.Thread(target=self._loop, daemon=True).start()
            print('  遥测: %s @%d 已打开（板内直读）' % (port, baud))
        except Exception as e:
            print('  [WARN] 遥测口 %s 打不开: %s ⇒ 退回只用画面判距' % (port, e))

    def _loop(self):
        while not self._stop:
            try:
                chunk = self._ser.read(64)
            except Exception:
                time.sleep(0.05)
                continue
            if not chunk:
                continue
            self._buf += chunk
            try:
                frames, self._buf, bad = tl.parse_stream(self._buf)
            except Exception:
                self._buf = b''
                continue
            self.bad += bad
            if frames:
                f = frames[-1]
                self.range_mm = f['range_mm']
                self.yaw_deg = f['yaw_deg']
                self.state = f['state']
                self.vell, self.velr = f['vell'], f['velr']
                self.last_ms = time.time() * 1000.0
                self.n += 1

    def fresh_range(self, max_age_ms=400):
        """返回可信的前距 mm；无效/超量程/过期 → None"""
        if not self.ok or self.range_mm is None:
            return None
        if time.time() * 1000.0 - self.last_ms > max_age_ms:
            return None
        if self.range_mm < 20 or self.range_mm >= 4000:
            return None
        return self.range_mm

    def close(self):
        self._stop = True
        try:
            if self._ser:
                self._ser.close()
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--live', action='store_true', help='真发运动命令（默认 dry）')
    ap.add_argument('--follow', action='store_true', help='跟随模式：到站不停，保持距离')
    ap.add_argument('--seconds', type=float, default=30, help='最长运行秒数')
    ap.add_argument('--cx-center', type=int, default=664,
                    help='车头正前方在画面中的 x（2026-10-22 实车标定 664，20帧极差1px）')
    ap.add_argument('--no-mirror', action='store_true',
                    help='画面未左右镜像时使用。默认按【已镜像】处理（2026-10-22 实测确认）')
    ap.add_argument('--burst', action='store_true',
                    help='脉冲转向：短脉冲+停顿，避开电机死区与 0.4s 反转滞后（推荐）')
    ap.add_argument('--burst-pct', type=int, default=14, help='脉冲占空比%%（实测 >=10%% 才稳定起步）')
    ap.add_argument('--burst-settle', type=int, default=420, help='脉冲后停顿 ms（实测反转滞后约 400ms）')
    ap.add_argument('--no-trim', action='store_true',
                    help='关闭行进差速微调（默认开启：小偏差边走边纠，不用原地转）')
    ap.add_argument('--spin-at', type=int, default=300,
                    help='|err| 超过此 px 才改用原地转（默认 300 ≈ 14°）')
    ap.add_argument('--d-far', type=int, default=1500, help='距离 >= 此值用最高前进速度')
    ap.add_argument('--d-near', type=int, default=500, help='距离 <= 此值用最低前进速度')
    ap.add_argument('--trim-spd', type=int, default=12,
                    help='差速纠偏时的速度上限%%（限转向速率，防高速来回摆）')
    ap.add_argument('--slew', type=float, default=80.0,
                    help='前进速度变化率上限 %%/s（0=关闭），防速度硬跳变造成一顿一顿')
    ap.add_argument('--near-w', type=int, default=600, help='到达阈值：框宽 >= 此值判到达')
    ap.add_argument('--db-in', type=int, default=90, help='进入居中的偏差阈值 px（90≈4.3°）')
    ap.add_argument('--db-out', type=int, default=170, help='退出居中的偏差阈值 px（170≈8.2°）')
    ap.add_argument('--max-spd', type=int, default=24, help='转向速度上限%%（中继硬限速 25）')
    ap.add_argument('--fwd-far', type=int, default=22, help='远距离前进速度%%')
    ap.add_argument('--fwd-near', type=int, default=10, help='近距离前进速度%%')
    ap.add_argument('--sweep-deg', type=float, default=24.0, help='丢目标后扫视半幅角度')
    ap.add_argument('--deg-per-pct', type=float, default=2.25, help='标定：每 1%% 速度的转速 °/s')
    ap.add_argument('--grace-ms', type=int, default=300, help='丢帧保持窗 ms')
    ap.add_argument('--hold-ms', type=int, default=700, help='保持窗后停车等待 ms')
    ap.add_argument('--save-fail', action='store_true', help='保存无目标帧 fail_c_*.jpg')
    ap.add_argument('--csv', default=None, help='逐帧日志 CSV 路径')
    ap.add_argument('--tty', default=os.environ.get('TLM_TTY', '/dev/ttyUSB0'),
                    help='遥测串口（默认 /dev/ttyUSB0）；传 none 关闭')
    ap.add_argument('--stop-mm', type=int, default=300, help='到达距离 mm（超声波前距）')
    ap.add_argument('--range-max-mm', type=int, default=4000, help='超声波有效上限 mm')
    a = ap.parse_args()

    sweep_spd = 12
    sweep_ms = int(a.sweep_deg / (a.deg_per_pct * sweep_spd) * 1000.0)

    ctrl = PropFollow(cx_center=a.cx_center, near_w=a.near_w,
                      db_in=a.db_in, db_out=a.db_out,
                      spd_fwd_far=a.fwd_far, spd_fwd_near=a.fwd_near,
                      spd_spin_max=a.max_spd, sweep_ms=sweep_ms,
                      grace_ms=a.grace_ms, hold_ms=a.hold_ms, follow=a.follow,
                      mirror=not a.no_mirror,
                      burst=a.burst, burst_pct=a.burst_pct,
                      burst_settle_ms=a.burst_settle,
                      trim=not a.no_trim, spin_at=a.spin_at,
                      d_far_mm=a.d_far, d_near_mm=a.d_near, trim_spd_cap=a.trim_spd,
                      slew_pct_s=a.slew,
                      range_stop_mm=a.stop_mm)

    print('=' * 78)
    print('任务六 C 阶段（比例控制）：%s ｜ 最长 %.0fs ｜ %s'
          % ('★ 真实运动' if a.live else 'DRY（只算不发）', a.seconds,
             '跟随模式' if a.follow else '到达即停'))
    print('=' * 78)
    print('  正前方标定 cx_center=%d（画面中心 640，偏差 %+d px）' % (a.cx_center, a.cx_center - 640))
    print('  画面镜像=%s ⇒ err>0(%s侧) 时转向 %s'
          % ('是（默认）' if not a.no_mirror else '否',
             '画面右' if not a.no_mirror else '画面右',
             'SPIN_L（左转）' if not a.no_mirror else 'SPIN_R（右转）'))
    print('  居中迟滞 %d→%d px ｜ 转向速度 %.0f~%d%%（偏差 %dpx 到顶）'
          % (a.db_in, a.db_out, ctrl.spd_spin_min, ctrl.spd_spin_max, ctrl.err_span))
    print('  前进速度 %d%%(w<=%d) → %d%%(w>=%d) ｜ 到达 w>=%d'
          % (a.fwd_far, ctrl.far_w, a.fwd_near, a.near_w, a.near_w))
    print('  丢帧保持 %dms ｜ 停车等待 %dms ｜ 扫视 ±%.0f° (=%dms @%d%%) ｜ 扫 2 轮后全圈'
          % (a.grace_ms, a.hold_ms, a.sweep_deg, sweep_ms, sweep_spd))
    if a.live:
        print('⚠ 3 秒后开始；Ctrl+C 可停（比按 S1 温和，不用重烧）')
        time.sleep(3)

    snd = Sender(a.live)
    tlm = TlmReader(a.tty)


    if tlm.ok:
        _tw = time.time()
        while time.time() - _tw < 1.5 and tlm.n == 0:
            time.sleep(0.05)
        if tlm.n:
            print('  遥测首帧已收到，前距 %s' % (('%dmm' % tlm.range_mm) if tlm.range_mm else '--'))
        else:
            print('  [WARN] 1.5s 内未收到遥测帧 ⇒ 本次仅靠画面框宽判距')
    rows = []
    t0 = time.time()
    last_st = None
    cyc_hist = []
    try:
        while time.time() - t0 < a.seconds:
            cyc0 = time.time()
            now_ms = int((time.time() - t0) * 1000)
            ok = pull_frame()
            det = None
            if ok:
                det = rd.detect_red(LOCAL_FRAME)
            elif a.save_fail:
                pass
            if det is None and a.save_fail and ok:
                import shutil
                shutil.copy(LOCAL_FRAME, os.path.join(HERE, 'fail_c_%05d.jpg' % (now_ms // 200)))

            vis = (det['cls'], det['cx'], det['cy'], det['w'], det['h'], det['conf']) if det else None
            rng = tlm.fresh_range()
            cmd = ctrl.step(vis, now_ms, range_mm=rng)
            snd.send(cmd.dir, cmd.spd)

            cyc = (time.time() - cyc0) * 1000
            cyc_hist.append(cyc)
            if len(cyc_hist) > 20:
                cyc_hist.pop(0)
            if cmd.state != last_st:
                print('  [%5.1fs] ▶ %-8s' % (now_ms / 1000.0, cmd.state))
                last_st = cmd.state
            det_s = ('cx=%-4d w=%-4d err=%+4d conf=%d' % (det['cx'], det['w'], det['cx'] - a.cx_center, det['conf'])) if det else '无目标'
            rng_s = ('%4dmm' % rng) if rng else '  --  '
            print('[%5.1fs] %-8s %-34s %s → %-6s %2d%%  %s'
                  % (now_ms / 1000.0, cmd.state, det_s, rng_s,
                     DIR_NAME.get(cmd.dir, '?'), cmd.spd, cmd.reason))
            rows.append((now_ms, cmd.state, cmd.dir, cmd.spd,
                         det['cx'] if det else '', det['w'] if det else '',
                         cmd.err if cmd.err is not None else '', round(cyc, 1),
                         rng if rng else ''))

            if cmd.state == 'ARRIVED' and not a.follow:
                print('  ★ 已到达目标附近（%s）' % cmd.reason)
                break
    except KeyboardInterrupt:
        print('\n[人工中断]')
    finally:
        snd.close()
        tlm.close()
        if tlm.ok:
            print('遥测: 共 %d 帧 ｜ 坏帧 %d ｜ 最后前距 %s'
                  % (tlm.n, tlm.bad, ('%dmm' % tlm.range_mm) if tlm.range_mm else '--'))
        if cyc_hist:
            print('闭环周期：中位 %.0fms（最小 %.0f / 最大 %.0f），共 %d 帧'
                  % (sorted(cyc_hist)[len(cyc_hist) // 2], min(cyc_hist), max(cyc_hist), len(rows)))
        print('控制器统计: %s' % ctrl.stats)
        if a.csv and rows:
            with open(a.csv, 'w', newline='', encoding='utf-8-sig') as f:
                w = csv.writer(f)
                w.writerow(['t_ms', 'state', 'dir', 'spd', 'cx', 'w', 'err', 'cycle_ms', 'range_mm'])
                w.writerows(rows)
            print('逐帧日志: %s' % a.csv)
        print('已发停车并断开（中继 2s 后停发 → gate 超时自动停车）')


if __name__ == '__main__':
    main()
