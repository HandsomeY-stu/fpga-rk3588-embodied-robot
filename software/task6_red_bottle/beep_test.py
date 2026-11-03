#!/usr/bin/env python3
"""beep_test模块的实现或辅助入口，所属功能见代码索引。"""
import argparse
import socket
import time

RELAY = ('127.0.0.1', 9100)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--hold', type=float, default=1.5,
                    help='持续响的秒数（默认 1.5）')
    ap.add_argument('--pairs', type=int, default=2, help='最后鸣几声（默认 2）')
    ap.add_argument('--on-ms', type=int, default=220, help='每声时长 ms')
    ap.add_argument('--off-ms', type=int, default=280, help='两声间隔 ms')
    a = ap.parse_args()

    s = socket.create_connection(RELAY, timeout=5)

    def beep(on):
        s.sendall(('beep %d\n' % (1 if on else 0)).encode())

    print('① 持续响 %.1f 秒（每 1.5s 重发保活，对抗 FPGA 3s 超时）' % a.hold)
    t0 = time.time()
    last = 0.0
    while time.time() - t0 < a.hold:
        if time.time() - last > 1.5:
            beep(True)
            last = time.time()
        time.sleep(0.05)
    beep(False)
    print('② 静音 1.2 秒')
    time.sleep(1.2)

    print('③ 鸣 %d 声（%dms 响 / %dms 停）' % (a.pairs, a.on_ms, a.off_ms))
    for i in range(a.pairs):
        beep(True)
        time.sleep(a.on_ms / 1000.0)
        beep(False)
        if i != a.pairs - 1:
            time.sleep(a.off_ms / 1000.0)

    print('④ 全部关闭，静音')
    beep(False)
    time.sleep(0.2)
    s.close()
    print('完成。若①听不到而③听不到 ⇒ 检查：')
    print('   1) 位流是否含 CMD_BEEP（0x06）：grep -a UserCode 只能验位流身份，')
    print('      要看是不是新烧的那份（UserCode 会变）')
    print('   2) 跳线/蜂鸣器是否接好（引脚 N19）')
    print('   3) 中继是否在跑：ss -ltn | grep 9100')


if __name__ == '__main__':
    main()
