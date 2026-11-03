"""有界视野搜索辅助程序，按阶段调整观察方向。"""
import sys, time
sys.path.insert(0, '/root/task6')
import red_nav_e2e as rd, run_c as rc
from PIL import Image, ImageChops
import socket

RELAY = ('127.0.0.1', 9100)
KW = dict(rmin=45, rdom=10, min_pixels=120, mode='bbox')
KW_LOOSE = dict(rmin=45, rdom=10, min_pixels=50, mode='bbox')

def send(s, d, sp):
    s.sendall(('%d %d\n' % (d, sp)).encode())

s = socket.create_connection(RELAY, timeout=5)
print('%-8s %-8s %-6s %-9s %s' % ('yaw', '红像素', '严格', '形状匹配', '框(w x h)'))
for i in range(8):
    send(s, 5, 15)          # SPIN_L 15%
    time.sleep(0.62)        # ≈50°
    send(s, 0, 0)
    time.sleep(0.30)
    rc.pull_frame(2.0)
    f = rc.LOCAL_FRAME
    strict = rd.detect_red(f, **KW)
    loose = rd.detect_red(f, **KW_LOOSE)
    im = Image.open(f).convert('RGB'); r, g, b = im.split()
    m = ImageChops.multiply(ImageChops.multiply(
        r.point(lambda v: 255 if v >= 45 else 0),
        ImageChops.subtract(r, g).point(lambda v: 255 if v >= 10 else 0)),
        ImageChops.subtract(r, b).point(lambda v: 255 if v >= 10 else 0))
    cnt = m.histogram()[255]
    shape = ''
    if loose:
        a = loose['w'] / float(loose['h']) if loose['h'] else 0
        ok = (50 <= loose['w'] <= 300) and (a <= 0.85)
        shape = '是' if ok else '否(宽高比%.2f)' % a
    print('%-8s %-8d %-6s %-9s %s' % ('#%d' % (i + 1), cnt,
          '命中' if strict else '-', shape or '-',
          ('%dx%d' % (loose['w'], loose['h'])) if loose else '-'))
send(s, 0, 0)
s.close()
