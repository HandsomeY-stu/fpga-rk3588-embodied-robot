#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""检测框录制工具，用于离线检查目标连续性。"""
import sys, os, time, shutil
from PIL import Image

SRC = '/tmp/person.bmp'
PIDFILE = '/tmp/rec_boxes.pid'


def start(outdir, mode='bmp'):
    if os.path.exists(PIDFILE):
        pid = open(PIDFILE).read().strip()
        if os.path.exists('/proc/%s' % pid):
            print('已在录制中 (pid %s)，先 stop' % pid)
            return
    os.makedirs(outdir, exist_ok=True)
    with open(PIDFILE, 'w') as f:
        f.write(str(os.getpid()))

    ext = 'bmp' if mode != 'jpg' else 'jpg'
    last_key = None
    idx = 0
    t0 = time.time()
    while True:
        try:
            st = os.stat(SRC)
            key = (st.st_mtime_ns, st.st_size)
        except OSError:
            time.sleep(0.02)
            continue
        if key != last_key:
            last_key = key
            dst = os.path.join(outdir, 'f_%06d.%s' % (idx, ext))
            try:
                if ext == 'bmp':

                    shutil.copyfile(SRC, dst)
                else:
                    im = Image.open(SRC).convert('RGB')
                    im.save(dst, 'JPEG', quality=95)
                idx += 1
            except Exception:
                pass  # 半张图/瞬时 I/O 冲突，跳过
        time.sleep(0.02)

        try:
            dur = time.time() - t0
            fps = idx / dur if dur > 0 else 0
            with open(os.path.join(outdir, 'manifest.txt'), 'w') as f:
                f.write('frames=%d\nduration=%.1f\nfps=%.2f\nfmt=%s\n' % (idx, dur, fps, ext))
        except Exception:
            pass


def stop():
    if os.path.exists(PIDFILE):
        try:
            pid = int(open(PIDFILE).read().strip())
            os.kill(pid, 9)
        except Exception:
            pass
        os.remove(PIDFILE)
        print('已停止')
    else:
        print('没有在录制')


if __name__ == '__main__':
    cmd = sys.argv[1] if len(sys.argv) > 1 else ''
    if cmd == 'start':
        out = sys.argv[2] if len(sys.argv) > 2 else '/root/rec_frames'
        mode = sys.argv[3] if len(sys.argv) > 3 else 'bmp'
        start(out, mode)
    elif cmd == 'stop':
        stop()
    else:
        print(__doc__)
