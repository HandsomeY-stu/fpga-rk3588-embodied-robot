#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""诊断辅助程序：采集dir相关接口响应。"""
import argparse
import os
import statistics
import sys
import threading
import time

sys.path.insert(0, '/root/task6')

FRAME = '/tmp/frame.jpg'
RELAY = ('127.0.0.1', 9100)
YAW = {'v': None}


def serial_reader(stop):
    """后台读遥测，只关心 yaw"""
    try:
        import serial
        s = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.3)
    except Exception as e:
        print('  串口打开失败: %s' % e)
        return
    buf = b''
    while not stop.is_set():
        try:
            buf += s.read(64)
        except Exception:
            break
        while b'\n' in buf:
            line, buf = buf.split(b'\n', 1)
            line = line.strip()
            if line.startswith(b'#') and len(line) >= 41:
                try:
                    d = bytes.fromhex(line[1:41].decode('ascii', 'ignore'))
                except ValueError:
                    continue
                v = d[12] | (d[13] << 8)
                if v >= 32768:
                    v -= 65536
                YAW['v'] = v / 100.0
    try:
        s.close()
    except Exception:
        pass


def grab_cx(sec):
    """在 sec 秒内反复检测，返回 cx 列表"""
    import red_nav_e2e as rd
    out = []
    t0 = time.time()
    while time.time() - t0 < sec:
        try:
            d = rd.detect_red(FRAME)
        except Exception:
            d = None
        if d:
            out.append(d['cx'])
        time.sleep(0.02)
    return out


def wrap(d):
    while d > 180:
        d -= 360
    while d < -180:
        d += 360
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', type=int, default=5, help='5=SPIN_L 6=SPIN_R')
    ap.add_argument('--pct', type=int, default=10, help='速度 %%')
    ap.add_argument('--ms', type=int, default=250, help='脉冲时长 ms')
    a = ap.parse_args()

    stop = threading.Event()
    th = threading.Thread(target=serial_reader, args=(stop,), daemon=True)
    th.start()
    time.sleep(0.9)

    import socket
    sock = socket.create_connection(RELAY, timeout=5)

    c0 = grab_cx(0.8)
    y0 = YAW['v']
    if not c0:
        print('✗ 脉冲前检测不到目标，先让书包出现在画面里')
        sock.close()
        return 2
    cx0 = statistics.median(c0)

    name = {5: 'SPIN_L(左转)', 6: 'SPIN_R(右转)'}.get(a.dir, str(a.dir))
    print('脉冲前: cx=%d (极差%d)  yaw=%s' % (cx0, max(c0) - min(c0), y0))
    print('发 %s %d%% 持续 %dms ...' % (name, a.pct, a.ms))

    sock.sendall(('%d %d\n' % (a.dir, a.pct)).encode())
    time.sleep(a.ms / 1000.0)
    sock.sendall(b'0 0\n')
    sock.close()

    time.sleep(0.9)
    c1 = grab_cx(0.8)
    y1 = YAW['v']
    stop.set()

    if not c1:
        print('✗ 脉冲后检测不到目标（可能已转出画面），请缩短 --ms 或减小 --pct')
        return 3
    cx1 = statistics.median(c1)

    print('脉冲后: cx=%d (极差%d)  yaw=%s' % (cx1, max(c1) - min(c1), y1))
    print('')

    dcx = cx1 - cx0
    print('Δcx = %+d px' % dcx)
    if y0 is not None and y1 is not None:
        dy = wrap(y1 - y0)
        print('Δyaw = %+.2f°  （负=左转/逆时针，正是先前实测的约定）' % dy)
    else:
        dy = None
        print('Δyaw = 未知（遥测没读到）')

    print('')
    if abs(dcx) < 40:
        print('⚠ Δcx 太小（%+d），判不了。把 --ms 加大到 400 再试' % dcx)
        return 4

    print('结论：')
    if dy is not None and abs(dy) < 3:
        print('  Δyaw 太小，车基本没转，本次无效，请加大 --pct')
        return 4

    if a.dir == 5:      # 左转
        if dcx < 0:
            print('  左转 ⇒ cx 减小  ⇒ 画面【已镜像】⇒ err>0 应发 SPIN_L  (mirror=True)')
        else:
            print('  左转 ⇒ cx 增大  ⇒ 画面【未镜像】⇒ err>0 应发 SPIN_R  (mirror=False)')
    else:               # 右转
        if dcx > 0:
            print('  右转 ⇒ cx 增大  ⇒ 画面【已镜像】⇒ err>0 应发 SPIN_L  (mirror=True)')
        else:
            print('  右转 ⇒ cx 减小  ⇒ 画面【未镜像】⇒ err>0 应发 SPIN_R  (mirror=False)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
