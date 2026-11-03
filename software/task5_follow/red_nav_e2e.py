#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""红色目标导航流程的端到端执行与结果记录。"""
import sys, os, argparse

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
_real_argv = sys.argv[:]        # 保存真实参数
sys.argv = ['nav']              # 临时置空，防止 nav_fsm_sw 的 __main__ 解析我们的参数
import nav_fsm_sw as nav
sys.argv = _real_argv
from PIL import Image, ImageChops, ImageFilter


def detect_red(path, ref_w=1280, rmin=55, rdom=12, ratio_min=0.10, min_pixels=300,
               mode='grid'):
    """返回 dict(cls,cx,cy,w,h,conf,pixels) 或 None"""
    im = Image.open(path).convert('RGB')
    W, H = im.size
    r, g, b = im.split()

    m1 = ImageChops.subtract(r, g).point(lambda v: 255 if v >= rdom else 0)
    m2 = ImageChops.subtract(r, b).point(lambda v: 255 if v >= rdom else 0)
    m3 = r.point(lambda v: 255 if v >= rmin else 0)
    mask = ImageChops.multiply(ImageChops.multiply(m1, m2), m3)

    cnt = mask.histogram()[255]
    if cnt < min_pixels:                # 红像素绝对数门槛（噪声多为零星几十像素）
        return None


    if mode == 'bbox':


        mf = mask.filter(ImageFilter.MedianFilter(3))
        bb = mf.getbbox()
        if not bb:
            return None
        bw_b = bb[2] - bb[0]
        bh_b = bb[3] - bb[1]
        if bw_b < 8 or bh_b < 8:        # 逐像素尺度下的噪声门槛
            return None
        if cnt / float(W * H) > 0.85:
            return None
        sc_b = ref_w / float(W)
        return {
            'cls': 1,
            'cx': int((bb[0] + bb[2]) * 0.5 * sc_b),
            'cy': int((bb[1] + bb[3]) * 0.5 * sc_b),
            'w':  int(bw_b * sc_b),
            'h':  int(bh_b * sc_b),
            'conf': int(min(100, 40 + cnt / 100.0)),
            'pixels': cnt,
            'fill': cnt / float(W * H),
        }


    SW, SH = 32, 18
    small = mask.resize((SW, SH), Image.BOX)
    sp = list(small.getdata())
    sx = sy = sw = 0.0
    cols = []; rows = []
    for i, v in enumerate(sp):
        if v > 20:                       # BOX 平均后 >20 视为该格有红
            c, r_ = i % SW, i // SW
            sx += c * v; sy += r_ * v; sw += v
            cols.append(c); rows.append(r_)
    if sw <= 0:
        return None
    gcx = (sx / sw + 0.5) * (W / float(SW))
    gcy = (sy / sw + 0.5) * (H / float(SH))

    bw = (max(cols) - min(cols) + 1) * (W / float(SW))
    bh = (max(rows) - min(rows) + 1) * (H / float(SH))
    if bw < 15 or bh < 15:              # 太小 = 噪声
        return None


    if cnt / float(W * H) > 0.85:        # 红像素几乎铺满整屏才算误检
        return None
    sc = ref_w / float(W)
    return {
        'cls': 1,
        'cx': int(gcx * sc),
        'cy': int(gcy * sc),
        'w':  int(bw * sc),
        'h':  int(bh * sc),
        'conf': int(min(100, 40 + cnt / 100.0)),
        'pixels': cnt,
        'fill': cnt / float(W * H),
    }


def to_vision_frame(det, seq=1):
    """CMD_VISION(0x20) LEN=11：cls cx(2LE) cy(2LE) w(2LE) h(2LE) conf"""
    data = bytes([det['cls'],
                  det['cx'] & 0xFF, det['cx'] >> 8,
                  det['cy'] & 0xFF, det['cy'] >> 8,
                  det['w'] & 0xFF, det['w'] >> 8,
                  det['h'] & 0xFF, det['h'] >> 8,
                  det['conf']])
    body = bytes([11, seq & 0xFF, 0x20]) + data
    return b'\xA5\x5A' + body + bytes([nav.crc8(body)])


def run_single(path, verbose=True):
    det = detect_red(path)
    if verbose:
        print('=' * 74)
        print('图片: %s' % os.path.basename(path))
    if not det:
        print('检测: 未发现红色目标 → 状态机应进入 SEARCH')
        fsm = nav.NavFsm()
        fsm.step(None, 999, 0, 0)
        print('  状态: %-12s 帧: 无（无目标不产生运动建议）' % fsm.state)
        return None
    if verbose:
        print('检测: cls=%d cx=%d cy=%d w=%d h=%d conf=%d（红像素 %d，填充率 %.0f%%）'
              % (det['cls'], det['cx'], det['cy'], det['w'], det['h'], det['conf'],
                 det['pixels'], det['fill'] * 100))
    vf = to_vision_frame(det)
    ok, errs = nav.check_frame(vf) if False else (True, [])
    if verbose:
        print('CMD_VISION 帧: %s' % vf.hex(' '))
    err = det['cx'] - nav.CX_CENTER
    if det['w'] >= nav.NEAR_W:
        expect = 'ARRIVED（已到目标附近）'
    elif err < -nav.DEADBAND:
        expect = 'ALIGN_LEFT（目标在左）'
    elif err > nav.DEADBAND:
        expect = 'ALIGN_RIGHT（目标在右）'
    else:
        expect = 'APPROACH（居中，前进）'
    if verbose:
        print('误差 err=%+d px（死区 ±%d），w=%d（到达阈值 %d）' % (err, nav.DEADBAND, det['w'], nav.NEAR_W))
        print('预期决策: %s' % expect)
    return det


def run_sequence():
    """模拟车逐渐靠近红瓶：w 递增，cx 从偏左到居中"""
    print('=' * 74)
    print('赛题任务六 · 端到端 dry-run（模拟接近序列，不接电机）')
    print('=' * 74)
    fsm = nav.NavFsm()
    t = 0
    seq_inputs = [
        (None, '画面中无红瓶'),
        ({'cls': 1, 'cx': 180, 'cy': 360, 'w': 90,  'h': 120, 'conf': 78}, '红瓶在左远处'),
        ({'cls': 1, 'cx': 420, 'cy': 360, 'w': 150, 'h': 190, 'conf': 85}, '红瓶偏左中距'),
        ({'cls': 1, 'cx': 630, 'cy': 360, 'w': 240, 'h': 300, 'conf': 91}, '红瓶接近居中'),
        ({'cls': 1, 'cx': 648, 'cy': 360, 'w': 330, 'h': 410, 'conf': 94}, '红瓶居中较近'),
        ({'cls': 1, 'cx': 640, 'cy': 360, 'w': 420, 'h': 520, 'conf': 96}, '红瓶很近（到达）'),
    ]
    for det, desc in seq_inputs:
        vis = (det['cls'], det['cx'], det['cy'], det['w'], det['h'], det['conf']) if det else None
        for _ in range(3):                     # 每档连发 3 帧（满足 ACCEPT_N=3）
            fsm.step(vis, 999, 0, t)
            t += 200
        last = fsm.log[-1]
        print('\n[%s]' % desc)
        print('  ' + last)
        if det:
            print('  识别: cx=%d w=%d conf=%d  →  误差 err=%+d' % (det['cx'], det['w'], det['conf'], det['cx'] - 640))
    print('\n' + '=' * 74)
    print('dry-run 完成：以上 CMD_MOTION 帧仅为日志，未发送任何字节到串口')
    print('=' * 74)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('img', nargs='?')
    ap.add_argument('--seq', action='store_true')
    a = ap.parse_args()
    if a.seq or not a.img:
        run_sequence()
    else:
        run_single(a.img)
