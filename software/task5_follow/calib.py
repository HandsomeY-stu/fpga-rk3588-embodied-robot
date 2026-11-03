#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""摄像头中心与运动响应的标定辅助工具。"""

import argparse
import os
import socket
import statistics
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import telemetry_log as tlm      # 复用遥测解析（不改它）


DIR_STOP, DIR_FWD, DIR_BACK, DIR_LEFT, DIR_RIGHT, DIR_SPIN_L, DIR_SPIN_R = range(7)
DIR_NAME = {0: 'STOP', 1: 'FWD', 2: 'BACK', 3: 'LEFT', 4: 'RIGHT', 5: 'SPIN_L', 6: 'SPIN_R'}

SPEED_CAP = 24          # 中继硬限速 25%，留 1 余量


class Telem(object):
    """非阻塞读遥测；内部维护最近一帧"""

    def __init__(self, port):
        import serial
        self.ser = serial.Serial(port, 115200, timeout=0.05)
        self.buf = b''
        self.last = None
        self.bad = 0

    def pump(self):
        try:
            chunk = self.ser.read(256)
        except Exception:
            chunk = b''
        if chunk:
            self.buf += chunk
        frames, self.buf, bad = tlm.parse_stream(self.buf)
        self.bad += bad
        if frames:
            self.last = frames[-1]
        return frames

    def drain(self, seconds):
        """在 seconds 内持续读，返回所有帧（带本地时间戳）"""
        out = []
        t0 = time.time()
        while time.time() - t0 < seconds:
            for f in self.pump():
                f['t'] = time.time()
                out.append(f)
            time.sleep(0.01)
        return out

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


class Relay(object):
    def __init__(self, addr):
        host, port = addr.split(':')
        self.sock = socket.create_connection((host, int(port)), timeout=5)
        self.sock.settimeout(2)
        print('  中继已连接 %s' % addr)

    def send(self, d, sp):
        sp = max(0, min(SPEED_CAP, int(sp)))
        self.sock.sendall(('%d %d\n' % (d, sp)).encode())
        return sp

    def stop(self):
        try:
            self.send(DIR_STOP, 0)
            time.sleep(0.35)          # 让保活至少发一拍零速
        except Exception:
            pass

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass


def yaw_delta_cdeg(a, b):
    """yaw 为 ±18000 (0.01°)，求 b-a 的最短角差（含跨 ±180° 处理）"""
    d = (b - a) % 36000
    if d > 18000:
        d -= 36000
    return d


def yaw_rate_deg_s(frames):
    """用一个脉冲窗口内的帧做最小二乘拟合，返回 (°/s, 累计转角°, 有效样本数)"""
    if len(frames) < 3:
        return None, None, len(frames)
    t0 = frames[0]['t']
    ts = [f['t'] - t0 for f in frames]
    yaws = [f['yaw_cdeg'] for f in frames]

    un = [yaws[0]]
    for y in yaws[1:]:
        un.append(un[-1] + yaw_delta_cdeg(un[-1] % 36000, y))
    n = len(ts)
    mt = sum(ts) / n
    my = sum(un) / n
    num = sum((ts[i] - mt) * (un[i] - my) for i in range(n))
    den = sum((ts[i] - mt) ** 2 for i in range(n))
    if den <= 1e-9:
        return None, None, n
    slope = num / den / 100.0            # cdeg/s → °/s
    return slope, (un[-1] - un[0]) / 100.0, n


def mode_center(a):
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    import urllib.request
    import red_nav_e2e as rd

    print('=' * 78)
    print('§五.1 车头正前方标定（红书包固定在车头几何正前方，车体与摄像头姿态不动）')
    print('=' * 78)
    print('采 %d 帧，取 cx 中位数。取不到就报错，不会退回默认值。\n' % a.frames)

    cxs, dets = [], []
    tries = 0
    while len(cxs) < a.frames and tries < a.frames * 4:
        tries += 1
        try:
            data = urllib.request.urlopen(a.frame_url, timeout=5).read()
        except Exception as e:
            print('  拉帧失败: %s' % e)
            time.sleep(0.2)
            continue
        p = os.path.join(HERE, 'calib_center.jpg')
        open(p, 'wb').write(data)
        d = rd.detect_red(p)
        if d:
            cxs.append(d['cx'])
            dets.append((d['cx'], d['w'], d['conf']))
            print('  [%2d] cx=%-4d w=%-4d conf=%d' % (len(cxs), d['cx'], d['w'], d['conf']))
        else:
            print('  [--] 无目标（跳过）')
        time.sleep(0.15)

    if len(cxs) < 10:
        print('\n失败：有效帧只有 %d 帧（要求 >=10）。请确认红书包在画面中且占画面 1/4 以上。' % len(cxs))
        return 1

    med = int(statistics.median(cxs))
    print('\n' + '-' * 78)
    print('有效帧 %d  |  cx 中位数 = %d  （min %d / max %d / 极差 %d）'
          % (len(cxs), med, min(cxs), max(cxs), max(cxs) - min(cxs)))
    print('建议参数： --cx-center %d' % med)
    if max(cxs) - min(cxs) > 20:
        print('警告：极差 >20px，说明目标或车体在动，标定不可信，请重做。')
    print('-' * 78)
    return 0


def mode_spin(a):
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    print('=' * 78)
    print('§五.2 原地转速标定（轮子必须离地）')
    print('=' * 78)
    print('方向码：5=SPIN_L(原地左) 6=SPIN_R(原地右)；每档脉冲 %dms，间隔 %dms，脉冲后发零速\n'
          % (a.pulse_ms, a.gap_ms))

    t = Telem(a.port)
    r = Relay(a.relay)
    rows = []
    try:
        t.drain(0.6)          # 丢弃旧帧
        for d, label in ((DIR_SPIN_L, 'SPIN_L'), (DIR_SPIN_R, 'SPIN_R')):
            for pct in a.pcts:
                if pct > SPEED_CAP:
                    print('  跳过 %s %d%%（超上限 %d）' % (label, pct, SPEED_CAP))
                    continue
                print('  → %-6s %2d%%  脉冲 %dms' % (label, pct, a.pulse_ms))
                r.stop()
                t.drain(0.3)
                base = t.last

                r.send(d, pct)
                win = t.drain(a.pulse_ms / 1000.0)
                r.stop()
                time.sleep(a.gap_ms / 1000.0)


                if base is not None:
                    win = [f for f in win if f['t'] >= base['t']]
                rate, total, n = yaw_rate_deg_s(win)
                if rate is None:
                    print('     采样不足（%d 帧），判无效' % n)
                    rows.append(dict(mode='spin', target=label, pct=pct,
                                     rate=None, total=None, n=n, verdict='NO_DATA'))
                    continue
                if abs(total) < 1.0:
                    verdict = 'NO_ROTATION'      # 转了不到 1°，可能扭矩不足
                else:
                    verdict = 'OK'
                sign = '+' if rate >= 0 else '-'
                print('     yaw %s%.2f°/s  累计 %+.2f°  %d帧  → %s'
                      % (sign, rate, total, n, verdict))
                rows.append(dict(mode='spin', target=label, pct=pct,
                                 rate=rate, total=total, n=n, verdict=verdict))
    finally:
        r.stop()
        r.close()
        t.close()


    print('\n' + '-' * 78)
    print('%-8s %5s %10s %10s %8s' % ('目标', '速度', '角速度°/s', '累计转角°', '判定'))
    for x in rows:
        if x['rate'] is None:
            print('%-8s %4d%% %10s %10s %8s' % (x['target'], x['pct'], '-', '-', x['verdict']))
        else:
            print('%-8s %4d%% %+10.2f %+10.2f %8s'
                  % (x['target'], x['pct'], x['rate'], x['total'], x['verdict']))


    ok = [x for x in rows if x['rate'] is not None and x['verdict'] == 'OK']
    if ok:
        per = [abs(x['rate']) / x['pct'] for x in ok]
        med = statistics.median(per)
        print('\n角速度/速度比 中位数 = %.3f °/s 每 1%%' % med)
        print('建议参数： --deg-per-pct %.2f' % med)

        signs_l = set(x['rate'] > 0 for x in ok if x['target'] == 'SPIN_L')
        signs_r = set(x['rate'] > 0 for x in ok if x['target'] == 'SPIN_R')
        if len(signs_l) == 1 and len(signs_r) == 1 and signs_l != signs_r:
            print('方向：SPIN_L 与 SPIN_R 的 yaw 变化方向相反 ✅（左右定义正确）')
        elif signs_l and signs_r:
            print('警告：SPIN_L / SPIN_R 的 yaw 变化方向未呈现相反关系，需查电机接线或 yaw 符号。')

        low = [x for x in rows if x['pct'] < 8]
        for x in low:
            if x['verdict'] == 'NO_ROTATION':
                print('提示：%d%% 时未产生可测转动 → 低于此速度无法起转。' % x['pct'])
    else:
        print('\n无有效档位：请确认轮子离地可自由转动、中继与遥测都在线。')

    _dump_csv(a, rows, 'calib_spin')
    print('-' * 78)
    return 0


def mode_fwd(a):
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    print('=' * 78)
    print('§五.3 前进速度标定（轮子必须离地）')
    print('=' * 78)
    print('方向码 1=FWD；每档脉冲 %dms，间隔 %dms\n' % (a.pulse_ms, a.gap_ms))

    t = Telem(a.port)
    r = Relay(a.relay)
    rows = []
    try:
        t.drain(0.6)
        for pct in a.pcts:
            if pct > SPEED_CAP:
                continue
            print('  → FWD %2d%%  脉冲 %dms' % (pct, a.pulse_ms))
            r.stop()
            t.drain(0.3)
            r.send(DIR_FWD, pct)
            win = t.drain(a.pulse_ms / 1000.0)
            r.stop()
            time.sleep(a.gap_ms / 1000.0)

            if len(win) < 2:
                print('     采样不足（%d 帧）' % len(win))
                rows.append(dict(mode='fwd', target='FWD', pct=pct,
                                 vell=None, velr=None, verdict='NO_DATA'))
                continue

            core = win[1:-1] if len(win) >= 4 else win
            vell = statistics.median([f['vell'] for f in core])
            velr = statistics.median([f['velr'] for f in core])
            mean = (vell + velr) / 2.0
            if abs(mean) < 5:
                verdict = 'NO_ROTATION'
            elif abs(vell - velr) > max(30, abs(mean) * 0.3):
                verdict = 'L_R_IMBALANCE'
            else:
                verdict = 'OK'
            print('     左 %+4d 右 %+4d mm/s  均值 %+5.0f  %d帧  → %s'
                  % (vell, velr, mean, len(core), verdict))
            rows.append(dict(mode='fwd', target='FWD', pct=pct,
                             vell=vell, velr=velr, verdict=verdict))
    finally:
        r.stop()
        r.close()
        t.close()

    print('\n' + '-' * 78)
    print('%-6s %5s %10s %10s %10s %10s' % ('目标', '速度', '左轮mm/s', '右轮mm/s', '均值', '判定'))
    for x in rows:
        if x['vell'] is None:
            print('%-6s %4d%% %10s %10s %10s %10s' % (x['target'], x['pct'], '-', '-', '-', x['verdict']))
        else:
            print('%-6s %4d%% %+10d %+10d %+10.0f %10s'
                  % (x['target'], x['pct'], x['vell'], x['velr'], (x['vell'] + x['velr']) / 2.0, x['verdict']))

    ok = [x for x in rows if x['vell'] is not None and x['verdict'] == 'OK']
    if ok:
        rates = [(x['vell'] + x['velr']) / 2.0 / x['pct'] for x in ok]
        print('\n线速度/速度比 中位数 = %.1f mm/s 每 1%%' % statistics.median(rates))
    low = [x for x in rows if x['pct'] < 8 and x['verdict'] == 'NO_ROTATION']
    if low:
        print('提示：%d%% 无轮速 → 确认电机低于 8%% 是否无法起转。' % low[0]['pct'])

    _dump_csv(a, rows, 'calib_fwd')
    print('-' * 78)
    return 0


def _dump_csv(a, rows, prefix):
    if not rows:
        return
    p = a.csv or os.path.join(HERE, '%s_%s.csv' % (prefix, time.strftime('%m%d_%H%M%S')))
    keys = ['mode', 'target', 'pct', 'rate', 'total', 'vell', 'velr', 'n', 'verdict']
    keys = [k for k in keys if any(k in r for r in rows)]
    with open(p, 'w', encoding='utf-8', newline='') as f:
        f.write(','.join(keys) + '\n')
        for r in rows:
            f.write(','.join(('' if r.get(k) is None else str(r.get(k))) for k in keys) + '\n')
    print('CSV: %s' % p)


def main():
    ap = argparse.ArgumentParser(description='063 §五 标定工具')
    ap.add_argument('--mode', required=True, choices=['center', 'spin', 'fwd'])
    ap.add_argument('--port', default=None, help='CH340 遥测串口（spin/fwd 必需）')
    ap.add_argument('--relay', default=os.environ.get('RK_HOST', '10.36.218.39') + ':9100', help='中继地址 host:port')
    ap.add_argument('--frame-url', default='http://%s:9200/frame.jpg' % os.environ.get('RK_HOST', '10.36.218.39'))
    ap.add_argument('--frames', type=int, default=12, help='center 模式采样帧数（>=10）')
    ap.add_argument('--pcts', default=None, help='逗号分隔的速度档，如 9,12,16,20,24')
    ap.add_argument('--pulse-ms', type=int, default=1500)
    ap.add_argument('--gap-ms', type=int, default=1500)
    ap.add_argument('--wheels-off', action='store_true',
                    help='声明已把轮子离地（spin/fwd 必需，否则拒绝执行）')
    ap.add_argument('--csv', default=None)
    a = ap.parse_args()

    if a.mode == 'center':
        return mode_center(a)

    if not a.wheels_off:
        print('拒绝执行：spin / fwd 会让车轮转动。请先把轮子离地，再加 --wheels-off 重新运行。')
        return 2
    if not a.port:
        print('缺少 --port：spin / fwd 需要 CH340 遥测串口（先插线，再用 --list 确认编号）。')
        return 2
    if a.pcts:
        a.pcts = [int(x) for x in a.pcts.split(',') if x.strip()]
    else:
        a.pcts = [9, 12, 16, 20, 24] if a.mode == 'spin' else [10, 14, 18]

    if a.mode == 'spin':
        return mode_spin(a)
    return mode_fwd(a)


if __name__ == '__main__':
    sys.exit(main())
