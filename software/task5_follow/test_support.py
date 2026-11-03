"""离线回归测试：检查support相关逻辑和异常处理。"""
CX = 664


def det(cls, cx, h, conf=0.60, w=None, red=0.55):
    w = w or max(24, int(h * 0.55))
    return dict(cls=cls, conf=conf, cx=float(cx), cy=360.0, w=float(w), h=float(h),
                red_ratio=red)


def pkt(fid, dets, red_cx=None, red_n=0, red_w=0, red_h=0, age=30, producer=1):
    return dict(color_schema=1, age_ms=age, producer_id=producer, frame_id=fid,
                detections=list(dets or []), red_cx=red_cx, red_n=red_n,
                red_w=red_w, red_h=red_h)


def st(rng, yaw=90.0, mode=0, tlm_ok=True, tlm_age=40, velL=0, velR=0):
    return dict(tlm_ok=tlm_ok, tlm_age_ms=tlm_age, fpga_mode=mode, rng=rng,
                yaw=yaw, velL=velL, velR=velR)


def red_pkt(fid, cx, h=300, n=800, w=110, dets=None, age=30):
    return pkt(fid, dets or [], red_cx=cx, red_n=n, red_w=w, red_h=h, age=age)
