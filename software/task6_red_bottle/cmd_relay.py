#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TCP到串口运动命令中继，处理序号与命令时效。"""
import os, socket, sys, termios, threading, time

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 9100
DEV = sys.argv[2] if len(sys.argv) > 2 else '/dev/ttyS0'
KEEPALIVE_MS = 300
STALE_S = 2.0
SPEED_MAX = int(os.environ.get('SPEED_MAX', '25'))


# 协议使用CRC-8多项式0x07；字段顺序需与FPGA解析器一致。
def crc8(d):
    c = 0
    for b in d:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def frame(seq, d, sp):
    body = bytes([3, seq & 0xFF, 0x01, d & 0xFF, sp & 0xFF])
    return b'\xA5\x5A' + body + bytes([crc8(body)])


class Relay:
    def __init__(self):
        self.fd = os.open(DEV, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        a = termios.tcgetattr(self.fd)
        a[0] = 0; a[1] = 0
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL; a[3] = 0
        a[4] = termios.B115200; a[5] = termios.B115200
        termios.tcsetattr(self.fd, termios.TCSANOW, a)
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        self.seq = 1
        self.dir = 0
        self.sp = 0
        self.last_cmd_t = 0.0
        self.lock = threading.Lock()
        self.sent = 0
        self.stale_stops = 0

    def set(self, d, sp):
        sp = min(sp, SPEED_MAX)          # 硬限速
        with self.lock:
            self.dir, self.sp = d, sp
            self.last_cmd_t = time.time()
        print('[CMD] dir=%d spd=%d' % (d, sp), flush=True)

    def loop(self):
        while True:
            time.sleep(KEEPALIVE_MS / 1000.0)
            with self.lock:
                if time.time() - self.last_cmd_t > STALE_S:
                    d, sp = 0, 0
                    self.stale_stops += 1
                else:
                    d, sp = self.dir, self.sp
            try:
                os.write(self.fd, frame(self.seq, d, sp))
                self.seq = self.seq % 250 + 1
                self.sent += 1
            except Exception as e:
                print('[ERR] write: %s' % e, flush=True)


def main():
    r = Relay()
    print('中继启动: 端口 %d, 串口 %s, 保活 %dms, 断流 %ss 后停车, 硬限速 %d%%'
          % (PORT, DEV, KEEPALIVE_MS, STALE_S, SPEED_MAX), flush=True)
    threading.Thread(target=r.loop, daemon=True).start()
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('0.0.0.0', PORT)); srv.listen(2)
    print('等待 PC 连接...', flush=True)
    while True:
        c, addr = srv.accept()
        print('[PC] 连接 %s' % (addr,), flush=True)
        buf = b''
        try:
            while True:
                data = c.recv(256)
                if not data:
                    break
                buf += data
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    t = line.decode('ascii', 'ignore').strip()
                    if not t:
                        continue
                    if t == 'STATUS':
                        c.sendall(('sent=%d stale_stops=%d dir=%d spd=%d\n'
                                   % (r.sent, r.stale_stops, r.dir, r.sp)).encode())
                        continue
                    p = t.split()
                    if len(p) == 2:
                        r.set(int(p[0]), int(p[1]))
        except Exception as e:
            print('[PC] 断开: %s' % e, flush=True)
        finally:
            r.set(0, 0)
            c.close()
            print('[PC] 已断开 → 命令归零（车会停）', flush=True)


if __name__ == '__main__':
    main()
