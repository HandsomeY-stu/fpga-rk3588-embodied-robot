#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""诊断辅助程序：采集scan相关接口响应。"""
import argparse
import json
import socket
import sys
import time
import urllib.request

sys.path.insert(0, '/root/task6')
import red_nav_e2e as rd            # noqa: E402

DEG_PER_PX = 1.0 / 19.7
DIR_SPIN_L, DIR_SPIN_R, DIR_STOP = 5, 6, 0

SPECS = {
    'bag':    dict(w_min=330, w_max=9999, aspect_min=0.70, aspect_max=1.90,
                   det=dict()),
    'bottle': dict(w_min=50, w_max=300, aspect_min=0.00, aspect_max=0.85,
                   det=dict(rmin=45, rdom=10, min_pixels=120, mode='bbox')),
    'any':    dict(w_min=0, w_max=9999, aspect_min=0.00, aspect_max=99.0,
                   det=dict()),
}


def get_status(host, port):
    return json.load(urllib.request.urlopen(
        'http://%s:%d/api/status' % (host, port), timeout=3))


def wrap(a):
    while a > 180:
        a -= 360
    while a < -180:
        a += 360
    return a


def match_spec(d, spec):
    if not d or not d.get('h'):
        return False
    w, a = d['w'], d['w'] / float(d['h'])
    return spec['w_min'] <= w <= spec['w_max'] and spec['aspect_min'] <= a <= spec['aspect_max']


def pull_frame(host, port):
    try:
        b = urllib.request.urlopen('http://%s:%d/api/frame.jpg' % (host, port),
                                   timeout=2).read()
        if not b:
            return False
        open('/tmp/_probe.jpg', 'wb').write(b)
        return True
    except Exception:
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--spec', choices=list(SPECS), default='bag')
    ap.add_argument('--pct', type=int, default=3, help='旋转占空比%%（3≈28°/s，拖影约1.4°）')
    ap.add_argument('--laps', type=float, default=1.0)
    ap.add_argument('--dir', type=int, default=DIR_SPIN_L, choices=(5, 6))
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--http', type=int, default=8080)
    ap.add_argument('--relay', type=int, default=9100)
    a = ap.parse_args()

    spec = SPECS[a.spec]
    det_kw = spec.get('det') or {}
    socks = socket.create_connection(('127.0.0.1', a.relay), timeout=5)

    def send(d, sp):
        socks.sendall(('%d %d\n' % (d, sp)).encode())

    try:
        y0 = get_status(a.host, a.http)['yaw']
    except Exception as e:
        print('读状态失败: %s' % e)
        return 1
    if y0 is None:
        print('⚠ yaw 无效（IMU 没数据）⇒ 无法按时长估算转角，仍继续但圈数不准')

    rate = max(10.0, 9.4 * a.pct)
    target = 360.0 * a.laps
    print('环顾开始：%s %d%%（约 %.0f°/s，拖影约 %.1f°），目标累计转角 %.0f°'
          % ('左转' if a.dir == DIR_SPIN_L else '右转', a.pct, rate,
             rate * 0.05, target))
    print('%-7s %-9s %-6s %-9s %s' % ('累计°', 'yaw', '红像素', '框(w x h)', '判定'))

    buckets = {}
    turned = 0.0
    last_yaw = y0
    t0 = time.time()
    last_send = 0.0
    last_report = 0.0
    frames = 0
    while turned < target and time.time() - t0 < target / rate * 1.6 + 12:
        now = time.time()
        if now - last_send >= 0.08:
            send(a.dir, a.pct)
            last_send = now

        if pull_frame(a.host, a.http):
            frames += 1
            try:
                d = rd.detect_red('/tmp/_probe.jpg', **det_kw)
            except Exception:
                d = None
            if d:
                ok = match_spec(d, spec)

                mx = max(60, (det_kw.get('min_pixels') or 120) // 2)
                if d['pixels'] >= mx and now - last_report > 0.25:
                    last_report = now
                    y = get_status(a.host, a.http).get('yaw')
                    bear = wrap((y or 0) - (d['cx'] - 664) * DEG_PER_PX)
                    print('%-7.0f %-9s %-6d %-9s %s'
                          % (turned, ('%.1f' % y) if y is not None else '-',
                             d['pixels'], '%dx%d' % (d['w'], d['h']),
                             '✅匹配 ' + a.spec if ok else '✗形状不符'))
                    if ok:
                        k = int(round(bear / 15.0))
                        if k not in buckets or d['pixels'] > buckets[k][0]:
                            buckets[k] = (d['pixels'], bear, '%dx%d' % (d['w'], d['h']))

        y = get_status(a.host, a.http).get('yaw')
        if y is not None and last_yaw is not None:
            turned += abs(wrap(y - last_yaw))
        last_yaw = y
        time.sleep(0.05)
    send(DIR_STOP, 0)

    print()
    print('共 %d 帧，累计转了 %.0f°' % (frames, turned))
    if buckets:
        print('匹配到的候选方位（15° 桶，按像素数排序）:')
        for k, (px, bear, wh) in sorted(buckets.items(), key=lambda kv: -kv[1][0])[:8]:
            print('   方位 %+7.1f°   红像素 %4d   框 %s' % (bear, px, wh))
        best = max(buckets.values())
        print()
        print('★ 最可信方位 %+.1f°（%d 像素，%s）' % (best[1], best[0], best[2]))
    else:
        print('★ 没有匹配到 %s 档的目标' % a.spec)
    return 0


if __name__ == '__main__':
    sys.exit(main())
