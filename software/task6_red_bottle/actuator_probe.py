#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""诊断辅助程序：采集actuator相关接口响应。"""
import argparse
import statistics
import sys
import threading
import time

sys.path.insert(0, '/root/task6')

FRAME = '/tmp/frame.jpg'
RELAY = ('127.0.0.1', 9100)
YAW = {'v': None}


def serial_reader(stop):
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', type=int, default=5)
    ap.add_argument('--pct', type=int, default=14)
    ap.add_argument('--ms', type=int, default=1500)
    ap.add_argument('--pre', type=int, default=700, help='发命令前的静置观察 ms')
    ap.add_argument('--post', type=int, default=2000, help='停止后观察 ms')
    a = ap.parse_args()

    import socket
    import red_nav_e2e as rd

    stop = threading.Event()
    threading.Thread(target=serial_reader, args=(stop,), daemon=True).start()
    time.sleep(0.9)

    sock = socket.create_connection(RELAY, timeout=5)
    samples = []          # (t_ms, cx, yaw)
    t0 = time.time()

    def sample_until(until_s):
        while time.time() - t0 < until_s:
            try:
                d = rd.detect_red(FRAME)
            except Exception:
                d = None
            t = (time.time() - t0) * 1000.0
            if d:
                samples.append((t, d['cx'], YAW['v']))
            time.sleep(0.015)

    sample_until(a.pre / 1000.0)
    t_cmd = (time.time() - t0) * 1000.0
    name = {5: 'SPIN_L', 6: 'SPIN_R'}.get(a.dir, str(a.dir))
    sock.sendall(('%d %d\n' % (a.dir, a.pct)).encode())
    print('t=%.0fms 发 %s %d%% 持续 %dms' % (t_cmd, name, a.pct, a.ms))

    sample_until((a.pre + a.ms) / 1000.0)
    t_stop = (time.time() - t0) * 1000.0
    sock.sendall(b'0 0\n')
    sock.close()
    print('t=%.0fms 发 STOP' % t_stop)

    sample_until((a.pre + a.ms + a.post) / 1000.0)
    stop.set()

    if len(samples) < 20:
        print('样本太少(%d)，检测可能不稳' % len(samples))
        return 2


    print('')
    print('%-8s %-6s %-8s' % ('t_ms', 'cx', 'yaw'))
    lastprint = -1e9
    for t, cx, y in samples:
        if t - lastprint >= 80:
            mark = ''
            if abs(t - t_cmd) < 40:
                mark = '  <== 命令开始'
            if abs(t - t_stop) < 40:
                mark = '  <== 命令停止'
            print('%-8.0f %-6d %-8s%s' % (t, cx, ('%.2f' % y) if y is not None else '-', mark))
            lastprint = t


    base = [cx for t, cx, _ in samples if t < t_cmd]
    cx0 = statistics.median(base) if base else None
    pre_dev = (max(base) - min(base)) if base else 0


    TH = 15.0     # 认为"动了"的最小变化 px
    series = [(t, cx) for t, cx, _ in samples if t >= t_cmd - 50]
    t_move = None
    for t, cx in series:
        if t < t_cmd:
            continue
        if abs(cx - cx0) > TH:
            t_move = t
            break


    best_rate = 0.0
    for i, (t_i, cx_i) in enumerate(series):
        for j in range(i + 1, len(series)):
            t_j, cx_j = series[j]
            dt = (t_j - t_i) / 1000.0
            if dt > 0.25:
                break
            if dt >= 0.1:
                r = abs(cx_j - cx_i) / dt
                if r > best_rate:
                    best_rate = r


    after = [(t, cx) for t, cx in series if t >= t_stop]
    t_last_move = t_stop
    glide = 0.0
    if after:
        cx_at_stop = after[0][1]
        for t, cx in after:
            if abs(cx - cx_at_stop) > TH:
                t_last_move = t
        glide = abs(after[-1][1] - cx_at_stop)
        cx_at_stop_v = cx_at_stop
    else:
        cx_at_stop_v = None

    print('')
    print('================ 结论 ================')
    print('静止基线   : cx=%.0f（静置期波动 %+.0f px，可作噪声基准）' % (cx0, pre_dev))
    if t_move is not None:
        print('① 启动延迟 : %.0f ms （命令→cx 明显变化）' % (t_move - t_cmd))
    else:
        print('① 启动延迟 : >%d ms，车身没动起来' % a.ms)
    print('② 稳态转速 : %.0f px/s  （≈%.1f °/s，按 19.7px/°）' % (best_rate, best_rate / 19.7))
    if cx_at_stop_v is not None:
        print('③ 停止滞后 : %.0f ms 仍在变化，滑行 %.0f px （≈%.1f°）'
              % (t_last_move - t_stop, glide, glide / 19.7))
    print('')
    print('⇒ 单次最小可控转角 ≈ 滑行 %.0f px ≈ %.1f°' % (glide, glide / 19.7))
    print('   （想更精细就必须降低 %d%% 占空比，或加反向刹车脉冲）' % a.pct)
    return 0


if __name__ == '__main__':
    sys.exit(main())
