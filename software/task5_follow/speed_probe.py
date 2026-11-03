#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""诊断辅助程序：采集speed相关接口响应。"""
import argparse
import statistics
import sys
import threading
import time

sys.path.insert(0, '/root/task6')
RELAY = ('127.0.0.1', 9100)
TLM = []          # (t, velL, velR, rng)
STOP = threading.Event()


def serial_reader():
    try:
        import serial
        s = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.3)
    except Exception as e:
        print('  串口打开失败: %s' % e)
        return
    buf = b''
    t0 = time.time()
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
                s16 = lambda i: (lambda v: v - 65536 if v >= 32768 else v)(d[i] | (d[i + 1] << 8))
                u16 = lambda i: d[i] | (d[i + 1] << 8)
                TLM.append((time.time() - t0, s16(6), s16(8), u16(10)))
    try:
        s.close()
    except Exception:
        pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pcts', default='10,20,30,40,50')
    ap.add_argument('--on-ms', type=int, default=1200)
    ap.add_argument('--off-ms', type=int, default=700)
    a = ap.parse_args()
    pcts = [int(x) for x in a.pcts.split(',')]

    threading.Thread(target=serial_reader, daemon=True).start()
    time.sleep(1.2)
    if not TLM:
        print('✗ 读不到遥测')
        return 2

    import socket
    sock = socket.create_connection(RELAY, timeout=5)
    print('档位   稳态 velL   velR   平均   合速度估计')
    print('-' * 52)
    best = 0.0
    for pct in pcts:
        t_mark = TLM[-1][0]
        sock.sendall(('1 %d\n' % pct).encode())      # DIR_FWD=1
        time.sleep(a.on_ms / 1000.0)
        sock.sendall(b'0 0\n')

        seg = [r for r in TLM if r[0] - t_mark > 0.6 and r[0] - t_mark < a.on_ms / 1000.0]
        time.sleep(a.off_ms / 1000.0)
        if not seg:
            print('  %2d%%   (无遥测样本)' % pct)
            continue
        vl = statistics.median([abs(r[1]) for r in seg])
        vr = statistics.median([abs(r[2]) for r in seg])
        avg = (vl + vr) / 2.0
        best = max(best, avg)
        print('  %2d%%   %6.0f    %6.0f  %6.0f   %5.0f mm/s = %.2f m/s'
              % (pct, vl, vr, avg, avg, avg / 1000.0))
    sock.close()
    STOP.set()
    print('-' * 52)
    print('本次实测最高: %.0f mm/s (%.2f m/s)' % (best, best / 1000.0))
    print('人正常步行约 800~1000 mm/s ⇒ 需要至少 %.0f%% 以上才可能跟上'
          % (800.0 / max(1.0, best / max(pcts)) if best else 0))
    return 0


if __name__ == '__main__':
    sys.exit(main())
