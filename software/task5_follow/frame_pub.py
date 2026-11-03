#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""将采集环形缓冲中的完整图像原子发布为网页预览帧。"""
import os
import shutil
import time

SRC_DIR = '/tmp/fr'
DST = '/tmp/frame.jpg'
TMP = '/tmp/frame.jpg.tmp'
STATS_S = 60.0          # 每 60 秒打一行统计，便于确认它真的在跑


def ring(limit):
    """按 mtime 倒序返回最新的 limit 个环形帧"""
    try:
        names = [f for f in os.listdir(SRC_DIR) if f.endswith('.jpg')]
    except OSError:
        return []
    items = []
    for f in names:
        p = os.path.join(SRC_DIR, f)
        try:
            items.append((os.stat(p).st_mtime, p))
        except OSError:
            pass
    items.sort(reverse=True)
    return [p for _, p in items[:limit]]


def main():
    last = None
    pub = 0
    t_stat = time.time()
    while True:
        cand = ring(2)
        if len(cand) < 2:
            time.sleep(0.01)
            continue
        src = cand[1]        # 第二新 = 上一轮写的，已完整
        try:
            mt = os.stat(src).st_mtime
            if last == (src, mt):
                time.sleep(0.005)
                continue
            shutil.copyfile(src, TMP)
            os.replace(TMP, DST)     # 原子替换
            last = (src, mt)
            pub += 1
        except OSError:
            pass
        if time.time() - t_stat >= STATS_S:
            print('frame_pub: 近 %.0fs 发布 %d 帧（约 %.1f fps）'
                  % (STATS_S, pub, pub / STATS_S), flush=True)
            pub = 0
            t_stat = time.time()
        time.sleep(0.005)


if __name__ == '__main__':
    main()
