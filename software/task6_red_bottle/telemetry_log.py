#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""FPGA遥测解码与记录工具，提取轮速、距离和航向等字段。"""
import argparse, csv, os, struct, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
_RK_HOST = os.environ.get('RK_HOST', '10.36.218.39')
FRAME_URL = 'http://%s:9200/frame.jpg' % _RK_HOST
LOCAL_FRAME = os.path.join(HERE, 'tlm_frame.jpg')

TLM_SYNC = b'\xA5\x5A'
TLM_CAL = b'\x11\x22'


def decode_tlm(payload):
    """payload = 20 字节；返回 dict 或 None（同步/定标不对即 None）"""
    if len(payload) != 20:
        return None
    if payload[0:2] != TLM_SYNC or payload[2:4] != TLM_CAL:
        return None

    vell, velr, rng, yaw, leg2, d1, d2 = struct.unpack('<hhHhHHH', payload[6:20])
    return {
        'state': payload[4], 'err': payload[5],
        'vell': vell, 'velr': velr, 'range_mm': rng,
        'yaw_cdeg': yaw, 'yaw_deg': yaw / 100.0,
        'leg2_mm': leg2, 'diag1': d1, 'diag2': d2,
    }


def parse_stream(buf):
    """从字节流里切出完整帧；返回 (frames, 剩余 buf, 坏帧数)"""
    frames, bad = [], 0
    while True:
        i = buf.find(b'#')
        if i < 0:
            buf = b'' if len(buf) > 200 else buf
            break
        buf = buf[i:]
        if len(buf) < 43:
            break
        body = buf[1:41]
        tail = buf[41:43]
        if tail not in (b'\r\n', b'\n', b'\r'):
            bad += 1
            buf = buf[1:]
            continue
        try:
            payload = bytes.fromhex(body.decode('ascii'))
        except Exception:
            bad += 1
            buf = buf[1:]
            continue
        d = decode_tlm(payload)
        if d is None:
            bad += 1
        else:
            d['raw'] = buf[:43]
            frames.append(d)
        buf = buf[43:]
    return frames, buf, bad


def selftest():
    print('=' * 78)
    print('遥测解析自检（无硬件，构造已知值的帧）')
    print('=' * 78)
    cases = [
        dict(state=3, err=0x40 | 7, vell=152, velr=148, rng=754, yaw=1234, leg2=300, d1=0x0102, d2=0x0304),
        dict(state=0, err=0x40, vell=0, velr=0, rng=65535, yaw=-560, leg2=0, d1=0, d2=0),
        dict(state=9, err=0x40 | 0x2A, vell=-300, velr=299, rng=300, yaw=-17999, leg2=65535, d1=0xFFFF, d2=0xAAAA),
    ]
    ok = 0
    for c in cases:
        payload = (TLM_SYNC + TLM_CAL
                   + bytes([c['state'], c['err']])
                   + struct.pack('<hhHhHHH', c['vell'], c['velr'], c['rng'],
                                 c['yaw'], c['leg2'], c['d1'], c['d2']))
        assert len(payload) == 20, len(payload)
        line = b'#' + payload.hex().upper().encode() + b'\r\n'
        fr, rest, bad = parse_stream(b'garbage' + line + line[:10])
        d = fr[0] if fr else None
        good = (d is not None and d['vell'] == c['vell'] and d['velr'] == c['velr']
                and d['range_mm'] == c['rng'] and d['yaw_cdeg'] == c['yaw']
                and d['leg2_mm'] == c['leg2'] and d['state'] == c['state'])
        ok += 1 if good else 0
        print('  [%s] state=%-2d velL=%+5d velR=%+5d rng=%-5d yaw=%+7.2f° leg2=%-5d'
              % ('PASS' if good else 'FAIL', c['state'], c['vell'], c['velr'],
                 c['rng'], c['yaw'] / 100.0, c['leg2']))
        if not good:
            print('        解析结果: %s' % d)

    bad_payload = TLM_SYNC + b'\x33\x44' + bytes(16)
    fr, _, bad = parse_stream(b'#' + bad_payload.hex().encode() + b'\r\n')
    good = (len(fr) == 0 and bad == 1)
    ok += 1 if good else 0
    print('  [%s] 反证：定标常量错 → 整帧丢弃（收到 %d 帧，坏帧 %d）'
          % ('PASS' if good else 'FAIL', len(fr), bad))
    print('\n自检: %d/%d PASS ｜ 总判定: %s' % (ok, len(cases) + 1,
                                            'PASS' if ok == len(cases) + 1 else 'FAIL'))
    return 0 if ok == len(cases) + 1 else 1


def list_ports():
    import serial.tools.list_ports as lp
    ports = list(lp.comports())
    if not ports:
        print('没找到任何串口。若设备管理器里有 CH340 但这里为空，说明是残留的幽灵条目（线没插）。')
        return ports
    for p in ports:
        tag = 'CH340' if 'CH340' in (p.description or '') else ''
        print('  %-6s %-30s %s %s' % (p.device, p.description, p.hwid, tag))
    return ports


def guess_port(prefer=None):
    if prefer:
        return prefer
    import serial.tools.list_ports as lp
    cands = [p.device for p in lp.comports() if 'CH340' in (p.description or '')]
    return cands[0] if cands else None


def pull_frame():
    import urllib.request
    try:
        with urllib.request.urlopen(FRAME_URL, timeout=3) as r:
            data = r.read()
        if len(data) < 2000:
            return False
        with open(LOCAL_FRAME, 'wb') as f:
            f.write(data)
        return True
    except Exception:
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--selftest', action='store_true')
    ap.add_argument('--port', default=None)
    ap.add_argument('--seconds', type=float, default=30)
    ap.add_argument('--csv', default=os.path.join(HERE, 'merged_telem_vision.csv'))
    ap.add_argument('--frame', action='store_true', help='同时记录视觉帧（需 9200 在跑）')
    a = ap.parse_args()

    if a.list:
        print('串口列表：')
        list_ports()
        return 0
    if a.selftest:
        return selftest()

    port = guess_port(a.port)
    if not port:
        print('没有可用串口：CH340 未连接（设备管理器里的 COM6/7/8 是残留条目，实际打不开）。')
        print('请插上 FPGA 板载 CH340 的 USB 线后重试，或显式指定 --port COMx。')
        return 2

    import serial
    try:
        ser = serial.Serial(port, 115200, timeout=0.1)
    except Exception as e:
        print('打开 %s 失败：%s' % (port, e))
        return 2
    print('=' * 78)
    print('遥测记录：%s 115200 8N1 ｜ %.0f 秒 ｜ 视觉帧 %s ｜ 只读，不发运动命令'
          % (port, a.seconds, '开' if a.frame else '关'))
    print('=' * 78)

    t0 = time.time()
    buf = b''
    rows, n_tlm, n_vis, n_bad = [], 0, 0, 0
    det_fn = None
    if a.frame:
        import red_nav_e2e as rd
        det_fn = rd.detect_red
    last_vis = 0.0

    try:
        while time.time() - t0 < a.seconds:
            chunk = ser.read(256)
            if chunk:
                buf += chunk
                frames, buf, bad = parse_stream(buf)
                n_bad += bad
                for d in frames:
                    now = int((time.time() - t0) * 1000)
                    n_tlm += 1
                    rows.append([now, 'TELEM', d['state'], d['err'], d['vell'], d['velr'],
                                 d['range_mm'], '%.2f' % d['yaw_deg'], d['leg2_mm'],
                                 d['diag1'], d['diag2'], '', '', '', ''])
                    print('[%6dms] TELEM state=%-2d err=%02X velL=%+5d velR=%+5d rng=%-5d yaw=%+8.2f° leg2=%d'
                          % (now, d['state'], d['err'], d['vell'], d['velr'],
                             d['range_mm'], d['yaw_deg'], d['leg2_mm']))
            if a.frame and time.time() - last_vis > 0.07:
                last_vis = time.time()
                if pull_frame():
                    det = det_fn(LOCAL_FRAME)
                    now = int((time.time() - t0) * 1000)
                    n_vis += 1
                    if det:
                        rows.append([now, 'VISION', '', '', '', '', '', '', '',
                                     '', '', det['cx'], det['cy'], det['w'], det['conf']])
                    else:
                        rows.append([now, 'VISION', '', '', '', '', '', '', '',
                                     '', '', '', '', '', ''])
    except KeyboardInterrupt:
        print('\n[人工中断]')
    finally:
        ser.close()

    with open(a.csv, 'w', newline='', encoding='utf-8-sig') as f:
        w = csv.writer(f)
        w.writerow(['t_ms', 'type', 'state', 'err', 'velL_mms', 'velR_mms', 'range_mm',
                    'yaw_deg', 'leg2_mm', 'diag1', 'diag2', 'cx', 'cy', 'w', 'conf'])
        w.writerows(rows)
    dur = time.time() - t0
    print('-' * 78)
    print('遥测帧 %d（%.1f Hz）｜ 视觉帧 %d ｜ 坏帧 %d ｜ 合并行 %d ｜ 时长 %.1fs'
          % (n_tlm, n_tlm / max(0.1, dur), n_vis, n_bad, len(rows), dur))
    print('合并 CSV: %s' % a.csv)
    return 0 if n_tlm > 0 else 3


if __name__ == '__main__':
    sys.exit(main())
