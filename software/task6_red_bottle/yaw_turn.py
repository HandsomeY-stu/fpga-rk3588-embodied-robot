#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""基于航向误差的有界转向辅助程序。"""
import argparse
import sys
import threading
import time

sys.path.insert(0, '/root/task6')
RELAY = ('127.0.0.1', 9100)
YAW = {'v': None, 'ts': 0.0}
STOP = threading.Event()


def serial_reader():
    try:
        import serial
        s = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.3)
    except Exception as e:
        print('  串口打开失败: %s' % e)
        return
    buf = b''
    while not STOP.is_set():
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
                YAW['ts'] = time.time()
    try:
        s.close()
    except Exception:
        pass


def wrap(a):
    while a > 180:
        a -= 360
    while a < -180:
        a += 360
    return a


def fresh_yaw(timeout=1.5):
    """等一个比上次更新的 yaw（遥测 5Hz）"""
    t0 = YAW['ts']
    deadline = time.time() + timeout
    while time.time() < deadline:
        if YAW['ts'] > t0:
            return YAW['v']
        time.sleep(0.02)
    return YAW['v']


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--delta', type=float, required=True, help='要转的角度°（正=右转，负=左转）')
    ap.add_argument('--pct', type=int, default=10, help='转向占空比 %%')
    ap.add_argument('--tol', type=float, default=6.0, help='到位容差 °')
    ap.add_argument('--max-iter', type=int, default=24)
    a = ap.parse_args()

    threading.Thread(target=serial_reader, daemon=True).start()
    time.sleep(1.0)
    if YAW['v'] is None:
        print('✗ 读不到 yaw（遥测没通）')
        return 2

    import socket
    sock = socket.create_connection(RELAY, timeout=5)

    y0 = fresh_yaw()
    target = y0 + a.delta
    print('起始 yaw=%.2f°  目标 yaw=%.2f°（%+.1f°）  占空比 %d%%'
          % (y0, target, a.delta, a.pct))

    d = a.delta
    pct_spin = 6 if d > 0 else 5        # 正=右转 SPIN_R，负=左转 SPIN_L

    for it in range(a.max_iter):
        cur = fresh_yaw()
        if cur is None:
            break
        err = wrap(target - cur)
        if abs(err) <= a.tol:
            print('✓ 到位：yaw=%.2f° 误差 %+.2f°（%d 轮）' % (cur, err, it))
            break

        ms = 400 if abs(err) > 12 else 320
        sock.sendall(('%d %d\n' % (pct_spin, a.pct)).encode())
        time.sleep(ms / 1000.0)
        sock.sendall(b'0 0\n')
        time.sleep(0.45)                 # 等停稳（含 ~275ms 启动/停止滞后）
        after = fresh_yaw()
        print('  第%2d轮: 转 %3dms  yaw %.2f → %.2f°  剩余%+.1f°'
              % (it + 1, ms, cur, after if after is not None else float('nan'),
                 wrap(target - after) if after is not None else float('nan')))
    else:
        print('⚠ 达到最大轮数仍未收敛')

    sock.sendall(b'0 0\n')
    sock.close()
    STOP.set()
    final = YAW['v']
    print('结束 yaw=%.2f°  实际转过 %+.2f°' % (final, wrap(final - y0)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
