#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RK侧网页控制与TCP中继服务，管理遥测、检测进程及任务启动/停止。"""
import json
import os
import signal
import subprocess
import sys
import termios
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from fpga_web_modes import request_mode, mode_status

HERE = os.path.dirname(os.path.abspath(__file__))

CMD_DEV = os.environ.get('CMD_DEV', '/dev/ttyS0')        # RK3588 → FPGA 命令口
TLM_DEV = os.environ.get('TLM_DEV', '/dev/ttyUSB0')      # FPGA → RK3588 遥测（CH340）
HTTP_PORT = int(os.environ.get('HTTP_PORT', '8080'))
RELAY_PORT = int(os.environ.get('RELAY_PORT', '9100'))
SPEED_MAX = int(os.environ.get('SPEED_MAX', '60'))
CMD_TTL_MS = 350          # 命令有效期（死手开关）
SEND_HZ = 20

DIR_STOP, DIR_FWD, DIR_BACK, DIR_LEFT, DIR_RIGHT, DIR_SPIN_L, DIR_SPIN_R = 0, 1, 2, 3, 4, 5, 6
DIR_NAME = {0: 'STOP', 1: 'FWD', 2: 'BACK', 3: 'LEFT', 4: 'RIGHT', 5: 'SPIN_L', 6: 'SPIN_R'}
CMD_STOP_CODE = 0x04
CMD_BEEP_CODE = 0x06
CMD_OSD_MODE_CODE = 0x22
CMD_MODE_CODE = 0x03     # FB38: MODE 0=manual, 1/4=start request through FPGA sequencer
MODE_MANUAL   = 0x00


# 协议使用CRC-8多项式0x07；字段顺序需与FPGA解析器一致。
def crc8(d):
    c = 0
    for b in d:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def frame_motion(seq, d, sp):
    body = bytes([3, seq & 0xFF, 0x01, d & 0xFF, sp & 0xFF])
    return b'\xA5\x5A' + body + bytes([crc8(body)])


def frame_mode(seq, mode):
    """CMD_MODE (0x03) LEN=2 DATA: mode(1)"""
    body = bytes([2, seq & 0xFF, CMD_MODE_CODE, mode & 0xFF])
    return b'\xA5\x5A' + body + bytes([crc8(body)])


def frame_beep(seq, en):
    """CMD_BEEP (0x06) LEN=2 DATA: en(1)"""
    body = bytes([2, seq & 0xFF, CMD_BEEP_CODE, 1 if en else 0])
    return b'\xA5\x5A' + body + bytes([crc8(body)])


def frame_osd(seq, mode):
    """CMD_OSD_MODE (0x22) LEN=2 DATA: mode(1) —— 0=纯净 1=只姿态角 2=姿态角+视觉框"""
    body = bytes([2, seq & 0xFF, CMD_OSD_MODE_CODE, mode & 0xFF])
    return b'\xA5\x5A' + body + bytes([crc8(body)])


def frame_stop(seq):
    body = bytes([1, seq & 0xFF, CMD_STOP_CODE])
    return b'\xA5\x5A' + body + bytes([crc8(body)])


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.fpga_switch_lock = threading.Lock()
        self.fpga_auto_requested = False
        self.fpga_generation = 0
        self.person_uart = False
        self.tlm_updated_at = 0.0
        self.cmd_dir = 0
        self.cmd_spd = 0
        self.cmd_until = 0.0        # 命令有效期（monotonic）
        self.last_src = '-'         # 'web' / 'tcp9100'
        self.sent = 0
        self.idle_since = None
        self.beep_on = False        # 最近一次 beep 请求
        self.beep_until = 0.0

        self.tlm = dict(velL=0, velR=0, rng=None, yaw=None, fpga_state=None, age_ms=None)
        self.tlm_ok = False

        self.guard_on = False       # 软件避障守护
        self.guard_mm = 300
        self.guard_blocked = False

        self.preview_proc = None
        self.preview_on = False
        self.task6_start_lock = threading.Lock()
        self.task6_generation = 0
        self.tcp_inhibit = False


        self.trial_owner = False
        self.trial_run_id = None


        self.stop_seq = 0
        self.osd_mode = 1

        self.tgt_key = 'bottle'
        self.tgt_cls = '44'
        self.tgt_red = 0.18
        self.task6_proc = None
        self.person_proc = None      # 人员识别（SSD）进程
        self.task6_mode = None
        self.task6_tasks = 'any'
        self.task6_follow = False
        self.tlm_pause = False      # 任务六运行时暂停本进程读遥测（避免两个读者抢串口）

        self.msg = ''


S = State()


class Uart:
    def __init__(self, dev):
        self.ok = False
        self.fd = None
        self.seq = 1
        try:
            self.fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            a = termios.tcgetattr(self.fd)
            a[0] = 0
            a[1] = 0
            a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            a[3] = 0
            a[4] = termios.B115200
            a[5] = termios.B115200
            termios.tcsetattr(self.fd, termios.TCSANOW, a)
            termios.tcflush(self.fd, termios.TCIOFLUSH)
            self.ok = True
            print('[UART] %s 已打开（命令口）' % dev, flush=True)
        except Exception as e:
            print('[UART] %s 打不开: %s ⇒ 遥控不可用' % (dev, e), flush=True)

    def _wr(self, data):
        if not self.ok:
            return False
        try:
            os.write(self.fd, data)
            return True
        except Exception:
            return False

    def motion(self, d, sp):
        ok = self._wr(frame_motion(self.seq, d, sp))
        self.seq = self.seq % 250 + 1
        if ok:
            S.sent += 1
        return ok

    def stop(self):
        ok = self._wr(frame_stop(self.seq))
        self.seq = self.seq % 250 + 1
        return ok

    def beep(self, en):
        ok = self._wr(frame_beep(self.seq, en))
        self.seq = self.seq % 250 + 1
        return ok

    def mode(self, m):
        ok = self._wr(frame_mode(self.seq, m))
        self.seq = self.seq % 250 + 1
        return ok

    def osd(self, m):
        ok = self._wr(frame_osd(self.seq, m))
        self.seq = self.seq % 250 + 1
        return ok


U = Uart(CMD_DEV)


def cmd_set(d, sp, src):
    """设置活跃命令（带有效期）。避障守护在这里生效。"""
    if src == 'web':
        with S.lock:


            if S.trial_owner:
                return 0, 0
            S.tcp_inhibit=True
            S.task6_generation+=1
        if S.task6_proc is not None:
            cmd_stop('manual-takeover')
    sp = max(0, min(int(sp), SPEED_MAX))
    d = int(d)
    if S.guard_on and not S.guard_blocked and d in (DIR_FWD, DIR_LEFT, DIR_RIGHT):
        rng = S.tlm.get('rng')
        if rng is not None and rng < S.guard_mm:
            S.guard_blocked = True
            S.msg = '避障守护：前方 %dmm < %dmm，已拦停' % (rng, S.guard_mm)
            d, sp = DIR_STOP, 0
    elif S.guard_blocked:
        rng = S.tlm.get('rng')
        if rng is None or rng > S.guard_mm + 80:
            S.guard_blocked = False
            S.msg = '避障守护：前方已清空，解除拦停'
    with S.lock:
        if src == 'tcp9100' and S.tcp_inhibit:
            return 0, 0
        S.fpga_generation += 1
        S.fpga_auto_requested = False
        if S.tlm.get('mode') in (1, 4):
            U.mode(MODE_MANUAL)
        S.cmd_dir, S.cmd_spd = d, sp
        S.cmd_until = time.monotonic() + CMD_TTL_MS / 1000.0
        S.last_src = src
        S.idle_since = None
    return d, sp


def cmd_stop(src='web'):
    with S.lock:
        S.tcp_inhibit = True


        S.stop_seq += 1
        if S.trial_owner:
            S.msg = '人为停车（已收回试验所有权 %s）' % (S.trial_run_id or '?')
        S.trial_owner = False
        S.trial_run_id = None
        S.task6_generation += 1
        S.fpga_generation += 1
        S.fpga_auto_requested = False
        S.cmd_dir, S.cmd_spd = 0, 0
        S.cmd_until = 0.0
        S.idle_since = time.monotonic()
        S.last_src = src
    U.stop()
    _stop_proc('task6_proc')
    S.task6_mode = None


def send_loop():
    """20Hz：有活跃命令就发；空闲就发一次停车然后停发（把链路还给本地按键）"""
    period = 1.0 / SEND_HZ
    stopped_sent = True
    while True:
        time.sleep(period)
        now = time.monotonic()
        with S.lock:
            if S.fpga_auto_requested:
                stopped_sent = True
                continue
            alive = now < S.cmd_until
            d, sp = S.cmd_dir, S.cmd_spd
            if not alive and S.idle_since is None:
                S.idle_since = now
            if alive:
                U.motion(d, sp)
                stopped_sent = False
            elif not stopped_sent:
                U.motion(0, 0)
                stopped_sent = True


def tlm_loop():
    try:
        import serial
    except Exception as e:
        print('[TLM] 无法导入 pyserial: %s' % e, flush=True)
        return
    buf = b''
    while True:
        try:
            ser = serial.Serial(TLM_DEV, 115200, timeout=0.3)
        except Exception as e:
            print('[TLM] %s 打不开: %s（5s 后重试）' % (TLM_DEV, e), flush=True)
            time.sleep(5)
            continue
        print('[TLM] %s 已打开（遥测口）' % TLM_DEV, flush=True)
        buf = b''
        while True:
            try:
                chunk = ser.read(64)
            except Exception:
                break
            if S.tlm_pause:          # 任务六要用这个串口 ⇒ 本进程让出来
                break
            if not chunk:
                continue
            buf += chunk
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                line = line.strip()
                if not (line.startswith(b'#') and len(line) >= 41):
                    continue
                try:
                    d = bytes.fromhex(line[1:41].decode('ascii', 'ignore'))
                except ValueError:
                    continue
                u16 = lambda i: d[i] | (d[i + 1] << 8)
                s16 = lambda i: (lambda v: v - 65536 if v >= 32768 else v)(d[i] | (d[i + 1] << 8))
                rng = u16(10)
                S.tlm = dict(velL=s16(6), velR=s16(8),
                             rng=(None if rng == 0xFFFF or rng >= 4000 else rng),
                             yaw=s16(12) / 100.0, fpga_state=d[4], age_ms=0,
                             mode=d[17], force_stop=bool(d[16] & 8),
                             start_count=(u16(18) >> 12) & 15)
                S.tlm_updated_at = time.monotonic()
                S.tlm_ok = True
        try:
            ser.close()
        except Exception:
            pass
        if S.tlm_pause:
            time.sleep(0.3)
        else:
            time.sleep(1)


def _kill_by_cmdline(pat):
    n = 0
    out = subprocess.run(['ps', '-eo', 'pid,args'], capture_output=True, text=True).stdout
    me = os.getpid()
    for ln in out.splitlines():
        parts = ln.strip().split(None, 1)
        if len(parts) < 2:
            continue
        pid, args = parts
        if pat in args and 'webctl' not in args and int(pid) != me:
            try:
                os.kill(int(pid), signal.SIGTERM)
                n += 1
            except Exception:
                pass
    return n


def _stop_proc(attr):
    p = getattr(S, attr)
    if p is None:
        return False
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except Exception:
        try:
            p.terminate()
        except Exception:
            pass
    try:
        p.wait(timeout=3)
    except Exception:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except Exception:
            pass
    setattr(S, attr, None)
    return True


def _spawn(attr, argv, env=None):
    _stop_proc(attr)
    e = dict(os.environ)
    e['PYTHONUNBUFFERED'] = '1'      # 关键：否则子进程 stdout 块缓冲，日志要等退出才落盘
    if env:
        e.update(env)

    log = open('/tmp/task6.log', 'ab', buffering=0)
    log.write(('\n=== %s 启动: %s ===\n' % (time.strftime('%H:%M:%S'), ' '.join(argv))).encode())
    p = subprocess.Popen(argv, cwd=HERE, env=e, stdout=log,
                         stderr=subprocess.STDOUT, start_new_session=True)
    setattr(S, attr, p)
    return p


def set_preview(on):


    if on:
        return set_person(True, uart=False, fb=True)
    else:
        return set_person(False)


PERSON_BIN = '/root/task6live_20261029/realtime_yolo/realtime_yolo'
PERSON_DEMO = '/root/task6live_20261029/models'
PERSON_ARGV = [
    PERSON_BIN,
    '-m', PERSON_DEMO + '/yolov5s-640-640.rknn',
    '-a', PERSON_DEMO + '/box_priors.txt',
    '-l', PERSON_DEMO + '/coco_labels_list.txt',
    '--person-watch',          # 只画人，持续写 /tmp/person.bmp + json
    '--boxes-all',
    '--label-scale', '2',      # 类别名字号（2 ⇒ 10x14 像素）
    '--secs', '0',             # 0 = 一直跑（由 /api/person off 停）
    '--conf', '0.30',
    '--other-conf', '0.15',
    '--sharpen', '0.6',
    '--bmp-period', '0.05',
    '--bmp-scale', '2',
    '--mask-rect', '32,32,472,144',
    '--verbose',
]

def set_person(on, uart=False, fb=False):
    """开/关目标识别看板。"""
    if on:
        if S.task6_proc is not None and S.task6_proc.poll() is None:
            return False, '任务六正在跑，先停任务六（两者都要占摄像头）'
        if not os.path.exists(PERSON_BIN):
            return False, '板上没找到 %s（先部署 SSD 程序）' % PERSON_BIN


        for pat in ('/root/grab_loop2.sh', '/root/frame_pub.py',
                    'gst-launch-1.0'):
            _kill_by_cmdline(pat)
        subprocess.run(['pkill', '-9', '-x', 'gst-launch-1.0'],
                       capture_output=True)
        time.sleep(1.0)
        if S.person_proc is not None and S.person_proc.poll() is None:
            return True, '目标识别已在运行'
        try:
            argv = list(PERSON_ARGV)
            if not uart:
                argv.append('--no-uart')      # 默认不发 CMD_VISION（避免与运动命令抢串口）
            if fb:
                argv += ['--fb', '/dev/fb0']
            S.person_proc = subprocess.Popen(
                argv, stdout=open('/tmp/person.log', 'wb'),
                stderr=subprocess.STDOUT)
        except Exception as e:
            return False, '启动失败: %s' % e
        S.person_uart = bool(uart)
        S.preview_on = bool(fb)
        S.msg = ('目标识别已开启（人员+物体，黄色方框 + 类别名）'
                 + ('，并同步 FPGA OSD' if uart else '')
                 + ('，已输出到显示屏' if fb else ''))
        return True, S.msg
    else:
        if S.person_proc is not None and S.person_proc.poll() is None:
            try:
                S.person_proc.terminate()
            except Exception:
                pass
            time.sleep(0.5)
        S.person_proc = None
        S.person_uart = False
        S.preview_on = False

        threading.Thread(target=start_vision, daemon=True).start()
        S.msg = '目标识别已关闭，画面已交回抓帧链路'
        return True, S.msg


def set_task6(on, mode='follow', tasks=None, follow_after=False, seconds=600):
    if not on:return _set_red_task6(on,mode,tasks,follow_after,seconds)
    if not S.task6_start_lock.acquire(blocking=False):return False,'任务六正在启动'
    try:return _set_red_task6(on,mode,tasks,follow_after,seconds)
    finally:S.task6_start_lock.release()

def _set_red_task6(on, mode='follow', tasks=None, follow_after=False, seconds=600):
    if not on:
        cmd_stop('task6-stop')
        S.task6_mode = None
        S.tlm_pause = False
        S.msg = '红瓶任务已停止，目标识别保留'
        return True, S.msg
    if mode not in ('follow','semantic') or tasks not in (None,'bottle','red_bottle'):
        return False, '本版任务六固定选择红色瓶子'
    if S.task6_proc is not None and S.task6_proc.poll() is None:
        return True, '红瓶任务正在运行'
    if mode_status(S).get('fpga_mode') != 0:
        return False, '请先退出 FPGA 自动任务并确认遥测正常'
    if S.preview_on:
        return False, '请先关闭独占摄像头的预览框'
    with S.lock:token=S.task6_generation


    if S.person_proc is not None and S.person_proc.poll() is None and S.person_uart:
        set_person(False)
    ok,msg=set_person(True,uart=False)
    if not ok:return False,msg
    until=time.monotonic()+3
    while time.monotonic()<until:
        try:
            with open('/tmp/person.json') as f:packet=json.load(f)
            if packet.get('color_schema')==1 and time.time()-os.stat('/tmp/person.json').st_mtime<.3:break
        except (OSError,ValueError):pass
        time.sleep(.05)
    else:return False,'红瓶同帧颜色检测未就绪，未启动运动程序'
    follow = mode=='follow' or bool(follow_after)


    argv=[sys.executable,os.path.join(HERE,'run_red_bottle.py'),'--live','--cx-center','664','--seconds',str(seconds),
          '--range-stop','300','--close-h','0.96',
          '--target-cls',S.tgt_cls,'--red-min',str(S.tgt_red),
          '--trim-closed-loop','--trim-progress-px','3','--trim-stall-frames','8']
    if follow:argv.append('--follow')
    with S.lock:
        if token != S.task6_generation:return False,'启动已被停车或手动接管取消'
        if mode_status(S).get('fpga_mode')!=0:return False,'模式或遥测已改变，取消启动'
        S.tcp_inhibit=False
        _spawn('task6_proc',argv)
        S.task6_mode=mode;S.task6_tasks='red_bottle';S.task6_follow=follow
    S.msg='红瓶跟随已启动' if follow else '红瓶靠近已启动（到位即停）'
    return True,S.msg


HTML = r'''<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>小车远程控制台</title>
<style>
:root{--bg:#12151a;--card:#1b2029;--line:#2b3340;--txt:#e8edf4;--dim:#8b98a8;
      --go:#2ea86b;--back:#c08b2a;--spin:#2b6fd6;--estop:#d63b3b;--ok:#3ddc84}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{margin:0;background:var(--bg);color:var(--txt);
     font-family:-apple-system,"PingFang SC","Microsoft YaHei",sans-serif;padding:14px;
     -webkit-touch-callout:none;overscroll-behavior-y:contain}
.hdr{position:sticky;top:0;z-index:10;background:var(--bg);
     padding:2px 0 10px;display:flex;align-items:center;gap:10px;flex-wrap:wrap}
.badge{display:inline-flex;align-items:center;gap:7px;font-size:13px;font-weight:700;
       padding:7px 13px;border-radius:999px;background:#222834;border:1px solid var(--line);
       color:var(--dim);white-space:nowrap}
.badge.ok{background:rgba(61,220,132,.16);border-color:rgba(61,220,132,.5);color:#7df0ae}
.badge.bad{background:rgba(214,59,59,.16);border-color:rgba(214,59,59,.5);color:#ff9b9b}
.badge .dot{width:9px;height:9px;margin:0}
h1{font-size:16px;margin:0;font-weight:600;flex:1 1 auto}
.sub{color:var(--dim);font-size:12px;margin:0 0 12px}
.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;margin-bottom:14px}
button{border:none;border-radius:14px;color:#fff;font-size:16px;font-weight:600;
       padding:0;height:78px;background:var(--card);border:1px solid var(--line);
       transition:transform .06s,filter .12s;touch-action:none;user-select:none}
button:active{transform:scale(.96);filter:brightness(1.25)}
button.fwd{background:var(--go)} button.back{background:var(--back)}
button.spin{background:var(--spin)} button.estop{background:var(--estop);height:78px}
button.arc{background:#3a4a63}
.card{background:var(--card);border:1px solid var(--line);border-radius:14px;
      padding:12px;margin-bottom:12px}
.row{display:flex;align-items:center;justify-content:space-between;gap:10px}
.row+.row{margin-top:10px}
.lbl{font-size:13px;color:var(--dim)}
.val{font-variant-numeric:tabular-nums;font-weight:600}
input[type=range]{width:100%;accent-color:var(--go)}
.sw{width:52px;height:30px;border-radius:15px;background:#333c4a;position:relative;
    flex:0 0 auto;transition:background .15s;cursor:pointer}
.sw::after{content:"";position:absolute;top:3px;left:3px;width:24px;height:24px;
           border-radius:50%;background:#fff;transition:left .15s}
.sw.on{background:var(--ok)} .sw.on::after{left:25px}
.sw.dis{opacity:.35;cursor:not-allowed}
.stat{display:grid;grid-template-columns:repeat(2,1fr);gap:8px 14px}
.stat div{display:flex;justify-content:space-between;font-size:13px}
.msg{color:#ffcf6b;font-size:12px;min-height:16px;margin-top:6px}
select{background:#222834;color:var(--txt);border:1px solid var(--line);border-radius:9px;
       padding:8px 10px;font-size:13px;max-width:52%}
.tip{color:var(--dim);font-size:11px;margin-top:4px}
.dot{display:inline-block;width:8px;height:8px;border-radius:50%;background:#666;margin-right:6px}
.dot.ok{background:var(--ok)} .dot.bad{background:var(--estop)}
</style></head><body>
<h1>小车远程控制台</h1>
<div class="hdr">
  <div class="badge" id="badge"><span class="dot"></span><span id="badgeTxt">连接中…</span></div>
  <span class="lbl" id="hdrInfo" style="font-size:11px"></span>
</div>
<div class="sub" id="fpgaStat">FPGA 遥测：—</div>
<div class="tip" style="font-size:11px">版本 <b>task6live-20261029</b> · 本页只代表任务六（红瓶跟随）局部验证，
<b>不代表任务三/四已全面验收</b>；任一路点导航未启用。位流 0xFB38。</div>

<div class="grid">
  <button class="arc" data-dir="5">原地左转</button>
  <button class="fwd" data-dir="1">前　进</button>
  <button class="arc" data-dir="6">原地右转</button>
  <button class="arc" data-dir="3">左　转</button>
  <button class="back" data-dir="2">后　退</button>
  <button class="arc" data-dir="4">右　转</button>
</div>
<button class="estop" id="estop" style="width:100%;margin-bottom:14px">急 停</button>

<div class="card">
  <div class="row"><span class="lbl">实时画面</span><span class="val" id="camTip">连接中…</span></div>
  <img id="cam" alt="画面" style="width:100%;border-radius:8px;display:block;
       background:#000;margin-top:6px;aspect-ratio:16/9;object-fit:contain">
  <div class="tip" id="camTip2">未开启目标识别时显示原始画面；开启后<strong>自动叠加黄色识别框</strong>（人员 + 所有物体 + 类别名）</div>
</div>

<!-- ★ 2026-10-26 人员识别看板（任务书"人员识别"演示用）
     ★ 2026-10-27 升级为「目标识别看板」：SSD 的**所有类别**都画黄色方框，
       方框下方写类别名（--boxes-all），横幅显示类别 + 个数 + 最高置信度。
     开启后画面切到 SSD 检测结果。演示录制时把手机对着实车 + 这个页面即可同框。 -->
<div class="card">
  <div class="row"><span class="lbl">目标识别（人员 + 物体，SSD）</span>
    <div class="sw" id="sw-person"></div>
  </div>
  <div id="pBanner" style="margin:8px 0;padding:10px;border-radius:8px;
       text-align:center;font-weight:700;font-size:16px;
       background:#eef2f7;color:#5a6b80">未开启</div>
  <div class="tip">YOLOv5 640 检测：支持瓶子、人等 COCO 类别。BOTTLE 表示瓶类，不能判断瓶内是否是水；完整检测列表用于自动验收。识别框已直接叠加在上方「实时画面」。</div>
  <label style="display:flex;align-items:center;gap:8px;margin:6px 0;font-size:14px;
         color:var(--fg2,#5a6b80);cursor:pointer">
    <input type="checkbox" id="pUart" style="width:16px;height:16px">
    同步 FPGA OSD（把识别结果回传 FPGA 叠加显示 C/X/Y/Q/S/N）
  </label>
  <div class="tip" id="pTip">开启后独占摄像头（任务六会暂停），关掉自动恢复实时画面。
    勾选上面的选项后<strong>不要同时用网页驱动小车</strong>（两者共用 /dev/ttyS0）。</div>
</div>

<div class="card">
  <div class="row"><span class="lbl">速度</span><span class="val"><span id="spdv">30</span>%</span></div>
  <div class="row"><input type="range" id="spd" min="5" max="60" value="30" step="1"></div>
  <div class="tip">按住按钮才动，松手立即停（死手开关 350ms）</div>
</div>

<div class="card">
  <div class="row"><span class="lbl">手动遥控保护（RK3588 软件，&lt; 300mm 拦停）</span>
      <div class="sw" id="sw-guard"></div></div>
  <div class="row"><span class="lbl">任务三 · FPGA 自主避障</span>
      <div class="sw" id="sw-avoid"></div></div>
  <div class="row"><span class="lbl">任务四 · 自主导航（段落一 300mm）</span>
      <div class="sw" id="sw-nav"></div></div>
  <div class="tip" id="mode-readback">读取 FPGA 模式…</div>
  <div class="tip">开启后小车可能立即动作；关闭则停车并回手动。任意路点导航尚未启用。切换前请关闭任务六，并取消“同步 FPGA OSD”。</div>
  <div class="tip" id="red-target">红瓶候选：等待检测</div><div class="row"><span class="lbl">任务六 · 红色瓶子跟随</span><div class="sw" id="sw-t6"></div></div>
  <div class="tip">固定锁定红瓶；连续确认后动作，丢失或数据过期停车等待。新控制尚未做实车运动验收；开始前保持可停车空间。</div>
  <div class="sub" id="t6Stat" style="font-size:12px">任务六状态：未运行</div>
  <div class="row" style="margin-top:6px"><span class="lbl">画面 OSD</span>
    <select id="osdSel" style="font-size:13px;padding:4px">
      <option value="1">只姿态角（默认）</option>
      <option value="0">纯净画面（锁定目标时用）</option>
    </select></div>
  <div class="tip">纯净画面用于任务五/六锁定目标：关掉左上角文字框，避免近距遮挡目标。</div><div class="row"><span class="lbl">指定目标</span>
      <select id="tgt">
        <option value="bottle">红色瓶子（cls 44）</option>
        <option value="bag">红色书包（27/31/33）</option>
        <option value="both">红瓶 + 书包（都跟）</option>
        <option value="person">人（cls 1）</option>
      </select></div>
  <div class="row"><span class="lbl">找到后持续跟随<small class="tip">（关=到位即停）</small></span>
      <div class="sw on" id="sw-follow"></div></div>
  <div class="row"><span class="lbl">任务六 · 靠近红瓶</span><div class="sw" id="sw-t6s"></div></div>
  <div class="row"><span class="lbl">屏幕预览（带识别框）</span><div class="sw" id="sw-prev"></div></div>
  <div class="msg" id="msg"></div>
</div>

<div class="card">
  <div class="stat">
    <div><span class="lbl">前距</span><span class="val" id="s-rng">–</span></div>
    <div><span class="lbl">yaw</span><span class="val" id="s-yaw">–</span></div>
    <div><span class="lbl">左轮</span><span class="val" id="s-vl">–</span></div>
    <div><span class="lbl">右轮</span><span class="val" id="s-vr">–</span></div>
    <div><span class="lbl">任务四状态</span><span class="val" id="s-st">–</span></div>
    <div><span class="lbl">已发帧</span><span class="val" id="s-sent">–</span></div>
  </div>
</div>

<script>
/* ★ 2026-10-26 人员识别看板状态：必须用 var 在最前面声明。
   poll() 在脚本中段就被调用一次，若用 let 声明在后面会命中 TDZ
   （ReferenceError: Cannot access 'pOn' before initialization）⇒ 整页 JS 挂掉。 */
var pOn = false;
const $=s=>document.querySelector(s);
// 速度记忆：手机上不用每次都调
let spd = +(localStorage.getItem('spd') || 30);
let holding=null, timer=null;

function post(p,d){return fetch(p,{method:'POST',headers:{'Content-Type':'application/json'},
  body:JSON.stringify(d||{})}).then(r=>r.json()).catch(e=>({ok:false,err:String(e)}));}

// ---- 按住移动 ----
document.querySelectorAll('button[data-dir]').forEach(b=>{
  const dir=+b.dataset.dir;
  const start=e=>{e.preventDefault(); holding=dir; pulse();};
  const stop =e=>{e.preventDefault(); if(holding===dir){holding=null; post('/api/stop');}};
  b.addEventListener('pointerdown',start);
  b.addEventListener('pointerup',stop);
  b.addEventListener('pointerleave',stop);
  b.addEventListener('pointercancel',stop);
});
function pulse(){ if(holding===null) return; post('/api/motion',{dir:holding,spd:spd}); }

// 150ms 发一次（服务端有效期 350ms，够冗余）
timer=setInterval(pulse,150);

$('#estop').addEventListener('pointerdown',e=>{e.preventDefault();
  holding=null; post('/api/estop').then(r=>flash('急停已发送'));});

$('#spd').addEventListener('input',e=>{spd=+e.target.value;
  $('#spdv').textContent=spd; localStorage.setItem('spd',spd);});
$('#spd').value = spd; $('#spdv').textContent = spd;

function flash(t){ $('#msg').textContent=t; }

// ---- 开关 ----
function bindSw(id, path, body){
  const el=$(id);
  el.addEventListener('click',()=>{
    if(el.classList.contains('dis')) { flash('该功能暂不可用'); return; }
    const on=!el.classList.contains('on');
    post(path, Object.assign({on:on}, body||{})).then(r=>{
      if(r.ok) el.classList.toggle('on', on); flash(r.msg||'');
    });
  });
}
bindSw('#sw-guard','/api/guard');
function bindFpgaMode(id, mode) {
  const el=$(id); let pending=false;
  el.addEventListener('click',()=>{
    if(pending) return;
    pending=true; holding=null;
    post('/api/fpga_mode',{mode:el.classList.contains('on')?0:mode})
      .then(r=>flash(r.msg||''))
      .catch(()=>flash('模式请求失败，请检查连接'))
      .finally(()=>{pending=false; poll();});
  });
}
bindFpgaMode('#sw-avoid',1);
bindFpgaMode('#sw-nav',4);
bindSw('#sw-t6','/api/task6',{mode:'follow'});
bindSw('#sw-prev','/api/preview');
/* 目标识别看板：把"同步 FPGA OSD"勾选状态一起发过去 */
(function(){
  const sw = $('#sw-person');
  if (sw) sw.addEventListener('click', ()=>{
    if (sw.classList.contains('dis')) return;
    const on = !sw.classList.contains('on');
    const uart = !!($('#pUart') && $('#pUart').checked);
    post('/api/person', {on:on, uart:uart}).then(r=>{
      if (r.ok) sw.classList.toggle('on', on); flash(r.msg||'');
    });
  });
  const cb = $('#pUart');
  if (cb) cb.addEventListener('change', ()=>{
    /* 已经在跑 ⇒ 用新设置重启一次，让 OSD 同步立刻生效 */
    if (sw && sw.classList.contains('on')) {
      const uart = cb.checked;
      post('/api/person', {on:false}).then(()=>{
        setTimeout(()=>post('/api/person', {on:true, uart:uart})
          .then(r=>{ if(!r.ok) flash(r.msg||''); }), 900);
      });
    }
  });
})();

// 目标类型 / 跟随模式：本地设置，启动语义导航时带上
const tgt=$('#tgt');
tgt.value = 'bottle';
tgt.addEventListener('change',()=>localStorage.setItem('tgt', tgt.value));
$('#sw-follow').addEventListener('click',()=>{
  $('#sw-follow').classList.toggle('on');
  localStorage.setItem('fol', $('#sw-follow').classList.contains('on')?'1':'0');
});
if(localStorage.getItem('fol')==='0') $('#sw-follow').classList.remove('on');

// 语义导航开关：把"目标 + 是否跟随"一起发过去
$('#sw-t6s').addEventListener('click',()=>{
  const el=$('#sw-t6s'), on=!el.classList.contains('on');
  post('/api/task6',{on:on, mode:'semantic', tasks:tgt.value,
                     follow:$('#sw-follow').classList.contains('on')})
    .then(r=>{ if(r.ok) el.classList.toggle('on',on); flash(r.msg||''); });
});

// ---- 状态轮询 ----
let lastOk = 0, connOk = null, connSince = 0;
function setConn(ok){
  if(connOk === ok) return;
  connOk = ok;
  const b=$('#badge'), t=$('#badgeTxt');
  b.classList.toggle('ok', ok);
  b.classList.toggle('bad', !ok);
  t.textContent = ok ? '已连接' : '已断开';
  if(ok) connSince = Date.now();
  if(!ok) flash('与板子的连接断开，请检查 WiFi');
}
function setSw(id,on){ const el=$(id); if(el) el.classList.toggle('on',!!on); }
function poll(){
  fetch('/api/status',{cache:'no-store'}).then(r=>r.json()).then(s=>{
    lastOk = Date.now();
    setConn(true);
    $('#hdrInfo').textContent = '板子 '+location.host+' · 已连接 '+
      Math.floor((Date.now()-connSince)/1000)+'s';
    $('#s-rng').textContent = s.rng==null?'–':s.rng+' mm';
    $('#s-yaw').textContent = s.yaw==null?'–':s.yaw.toFixed(2)+'°';
    $('#s-vl').textContent  = s.velL+' mm/s';
    $('#s-vr').textContent  = s.velR+' mm/s';
    $('#s-st').textContent  = s.fpga_state==null?'–':s.fpga_state;
    $('#s-sent').textContent= s.sent;
    /* ★ P3.2：三态。tlm_ok 只代表"曾经收到"，必须看 age 才知当前是否有效 */
    if (s.tlm_valid) {
      $('#fpgaStat').innerHTML = '<span class="dot ok"></span>FPGA 遥测正常 · 命令源 '+(s.src||'-')
        + ' · '+(s.tlm_age_ms|0)+'ms';
    } else if (s.tlm_ever) {
      $('#fpgaStat').innerHTML = '<span class="dot bad"></span>FPGA 遥测<b>已过期</b>（'
        + (s.tlm_age_ms|0) + 'ms 未更新）· 数据不可用于控制';
    } else {
      $('#fpgaStat').innerHTML = '<span class="dot bad"></span>FPGA 遥测<b>从未收到</b>';
    }
    (function(){
      const sel = $('#osdSel');
      if (!sel) return;
      if (s.osd_mode != null && document.activeElement !== sel) sel.value = String(s.osd_mode);
      if (!sel._bound) { sel._bound = true;
        sel.addEventListener('change', ()=>{ post('/api/osd', {mode: parseInt(sel.value,10)}); }); }
    })();
    /* 跟随目标开关：切 红瓶/红书包/人（下次启动任务六生效） */
    (function(){
      const sel = $('#tgt');
      if (!sel) return;
      if (s.tgt_key && document.activeElement !== sel) sel.value = s.tgt_key;
      if (!sel._bound) { sel._bound = true;
        sel.addEventListener('change', ()=>{ post('/api/target', {key: sel.value}); }); }
    })();
    /* ★ P3.4：任务六控制器状态（state/dir/speed/lock/DRY-LIVE） */
    fetch('/api/task6_status',{cache:'no-store'}).then(r=>r.json()).then(j=>{
      const el = $('#t6Stat'); if(!el) return;
      if (j.state == null) { el.textContent = '任务六状态：未运行'; return; }
      const mode = j.live ? 'LIVE' : 'DRY';
      const fresh = (j.age_ms != null && j.age_ms < 1000);
      el.textContent = '任务六状态：' + mode + ' · ' + j.state
        + ' · dir=' + j.dir + ' spd=' + j.speed
        + ' · lock=' + (j.lock_state||'-')
        + ' · frame=' + (j.frame_id==null?'-':j.frame_id)
        + (fresh ? '' : ' · ⚠状态已过期(' + j.age_ms + 'ms)');
      el.style.color = (j.speed>0 && fresh) ? '#b3541e' : '';
    }).catch(()=>{});
    setSw('#sw-guard', s.guard_on);
    setSw('#sw-avoid', s.fpga_mode===1);
    setSw('#sw-nav', s.fpga_mode===4);
    $('#mode-readback').textContent='当前模式：'+s.fpga_mode_name+
      (s.fpga_force_stop?' · 已停车':'')+' · 网页整合版 2026-10-28';
    if($('#pUart') && s.person) $('#pUart').checked=!!s.person_uart;
    setSw('#sw-t6',  s.task6===true && s.task6_mode==='follow');
    setSw('#sw-t6s', s.task6===true && s.task6_mode==='semantic');
    setSw('#sw-prev', s.preview);
    setSw('#sw-person', s.person);
    pOn = !!s.person;
    if(s.msg) flash(s.msg);
  }).catch(()=>{ setConn(false); });
}
setInterval(poll,500); poll();

// ---- 网页内实时画面（2026-10-25）----
// 走 /api/frame.jpg 读抓帧链路的 /tmp/frame.jpg，5fps 刷新（够看，省流量）。
// 页面在后台时停抓，回来立刻恢复；连续失败就提示，不刷屏。
// ★★ 2026-10-30：改成 MJPEG 连续流（/stream.mjpg），不再 4fps 轮询抓帧。
//   浏览器把 multipart/x-mixed-replace 当视频流连续渲染；断开自动重连。
(function(){
  const im=$('#cam'); if(!im) return;
  const tip=$('#camTip');
  const src='/stream.mjpg';
  function play(){
    if(document.hidden) return;                    // 后台不占带宽
    im.src = src + '?t=' + Date.now();
    if(tip) tip.textContent='连续画面（MJPEG）';
  }
  im.onerror=function(){
    if(tip) tip.textContent='画面断开，重连中…';
    setTimeout(play, 1500);
  };
  document.addEventListener('visibilitychange', function(){
    if(document.hidden){ im.src=''; }               // 切后台就断开
    else { play(); }
  });
  play();
})();

// ---- 人员识别看板（2026-10-26）----
// ★ 2026-10-31：识别框已合并进主画面「实时画面」（cam 走 /stream.mjpg，后端在
//   检测器开启时自动切到带框的 person.bmp）。这里只保留**状态横幅**（personStat），
//   不再单独拉一张 pimg 图（原 personTick 的重复图已移除）。
function personStat(){
  if(!pOn) return;
  fetch('/api/person.json',{cache:'no-store'}).then(r=>r.json()).then(j=>{
    const b=$('#pBanner');
    /* ★ 2026-10-27 修：原来用 `Date.now()/1000 - j.ts < 3` 判新鲜度，
     *   但 j.ts 是**检测程序自己的运行秒数**（如 15841.8），不是 epoch 时间戳
     *   ⇒ 差值恒为 ~1.79e9 ⇒ 永远判"不新鲜" ⇒ 明明画面里已经框住了人，
     *   横幅却一直显示"画面中未检测到目标"。
     *   改成读服务端按文件 mtime 算出的 age_ms。 */
    const fresh = Number.isFinite(j.age_ms) && j.age_ms < 300;
    const reds=(j.detections||[]).filter(d=>d.cls===44 && d.conf>=0.4 && d.red_ratio>=0.18);
    const rt=$('#red-target'); if(rt)rt.textContent=fresh?('红瓶候选：'+reds.length+'；任务六仅选择红瓶，透明瓶不参与'):'红瓶候选：检测数据过期';
    /* ★ 2026-10-27：检测是**间歇**的（SSD 逐帧抖动，实测约一半帧有框），
     *   若逐帧显示，横幅会一直在"检测到/未检测到"之间闪。
     *   这里做 2.5 秒保持：命中过就继续保持，超过 2.5s 没命中才显示"未检测到"。 */
    const nowMs = Date.now();
    if (fresh && j.n > 0) { window.__lastHit = {t: nowMs, j: j}; }
    const hold = window.__lastHit && (nowMs - window.__lastHit.t) < 2500;
    if(hold){
      const jj = (fresh && j.n > 0) ? j : window.__lastHit.j;
      const nm=(jj.name||'').toUpperCase();
      if(fresh && (j.bottle_count||0)>0){b.textContent='检测到瓶子 BOTTLE ×'+j.bottle_count+'  最高 '+Math.round(j.bottle_conf*100)+'%'; b.style.background='#14532d'; b.style.color='#ffffff'; return;}
      b.textContent=(nm?('检测到 '+nm):'检测到目标')+'  ×'+jj.n
                    +'  最高 '+Math.round((jj.conf||0)*100)+'%';
      b.style.background='#e6f7ec'; b.style.color='#137a3f';
    }else{
      b.textContent='画面中未检测到目标';
      b.style.background='#eef2f7'; b.style.color='#5a6b80';
    }
  }).catch(()=>{});
}
function personLoop(){
  const on = pOn;               // 状态由 /api/status 同步（见 poll()）
  const sw = $('#sw-person');
  if (sw) { if (on) sw.classList.add('on'); else sw.classList.remove('on'); }
  // ★ 主画面 cam 走 /stream.mjpg，后端在检测器开启时自动切带框源；这里只负责
  //   在开/关切换时刷新一次主画面，让前端立刻换源，不等 MJPEG 自动重连。
  const cam=$('#cam');
  if(cam && on !== window.__camBoxed){
    window.__camBoxed = on;
    cam.src = '/stream.mjpg?t=' + Date.now();
  }
  if(!on){
    const b=$('#pBanner');
    if(b){ b.textContent='未开启'; b.style.background='#eef2f7'; b.style.color='#5a6b80'; }
  }
  if(on){ personStat(); }
  setTimeout(personLoop, 400);
}
personLoop();
/* 2026-10-30: camTick 已删除，画面改用 MJPEG 流，这里不再需要 */
// 看门狗：2 秒没有任何成功响应 ⇒ 判为断开（即使浏览器没报错）
setInterval(()=>{ if(lastOk && Date.now()-lastOk > 2000) setConn(false); }, 500);
</script></body></html>'''


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _json(self, obj, code=200):
        b = json.dumps(obj, ensure_ascii=False).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            b = HTML.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(b)))
            self.end_headers()
            self.wfile.write(b)
        elif self.path.startswith('/stream.mjpg'):


            try:
                self.send_response(200)
                self.send_header('Content-Type',
                                 'multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
                self.send_header('Pragma', 'no-cache')
                self.send_header('Connection', 'close')
                self.end_headers()
                sent = None
                bmp_key = None
                bmp_jpg = b''
                t_end = time.monotonic() + 3600.0      # 单连接最长 1 小时，之后让前端重连
                while time.monotonic() < t_end:
                    data = b''
                    try:
                        person_on = (S.person_proc is not None
                                     and S.person_proc.poll() is None)
                        if person_on:
                            stt = os.stat('/tmp/person.bmp')
                            key = ('bmp', stt.st_mtime_ns, stt.st_size)
                            if key != bmp_key:
                                try:
                                    from PIL import Image
                                    import io as _io
                                    im = Image.open('/tmp/person.bmp').convert('RGB')
                                    buf = _io.BytesIO()
                                    im.save(buf, 'JPEG', quality=70)
                                    bmp_jpg = buf.getvalue()
                                    bmp_key = key
                                except Exception:
                                    pass          # 正在写盘/半张图 ⇒ 用上一张
                            data = bmp_jpg
                        else:
                            stt = os.stat('/tmp/frame.jpg')
                            key = ('jpg', stt.st_mtime_ns, stt.st_size)
                            if key != sent:
                                with open('/tmp/frame.jpg', 'rb') as f:
                                    data = f.read()
                    except (OSError, ValueError):
                        data = b''
                    if data and key != sent:
                        try:
                            self.wfile.write(
                                b'--frame\r\nContent-Type: image/jpeg\r\n'
                                b'Content-Length: ' + str(len(data)).encode() + b'\r\n\r\n')
                            self.wfile.write(data)
                            self.wfile.write(b'\r\n')
                            self.wfile.flush()
                        except (BrokenPipeError, ConnectionResetError):
                            return
                        sent = key
                    time.sleep(0.03)                   # ~30fps 检查，重复帧不推
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass                                   # 客户端关页面了，正常退出
            return
        elif self.path.startswith('/api/frame.jpg'):


            try:
                with open('/tmp/frame.jpg', 'rb') as f:
                    b = f.read()
                if not b:
                    raise IOError('empty')
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
                self.send_header('Content-Length', str(len(b)))
                self.end_headers()
                self.wfile.write(b)
            except Exception:
                self.send_response(503)
                self.send_header('Content-Type', 'text/plain')
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(b'no frame')
        elif self.path.startswith('/api/person.bmp'):


            try:
                with open('/tmp/person.bmp', 'rb') as f:
                    b = f.read()
                if not b:
                    raise IOError('empty')
                self.send_response(200)
                self.send_header('Content-Type', 'image/bmp')
                self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
                self.send_header('Content-Length', str(len(b)))
                self.end_headers()
                self.wfile.write(b)
            except Exception:
                self.send_response(503)
                self.send_header('Content-Type', 'text/plain')
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(b'no person frame')
        elif self.path.startswith('/api/person.json'):
            try:
                with open('/tmp/person.json', 'rb') as f:
                    b = f.read()


                try:
                    _st = os.stat('/tmp/person.json')
                    _j = json.loads(b.decode('utf-8', 'ignore') or '{}')
                    _j['age_ms'] = int((time.time() - _st.st_mtime) * 1000)
                    b = json.dumps(_j).encode('utf-8')
                except Exception:
                    pass
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(b)))
                self.end_headers()
                self.wfile.write(b)
            except Exception:
                self._json({'n': 0, 'conf': 0.0, 'ts': 0})
        elif self.path == '/api/task6_status':


            try:
                with open('/tmp/task6_red_status.json', 'rb') as f:
                    b = f.read()
                _st = os.stat('/tmp/task6_red_status.json')
                _j = json.loads(b.decode('utf-8', 'ignore') or '{}')
                _j['age_ms'] = int((time.time() - _st.st_mtime) * 1000)
                b = json.dumps(_j).encode('utf-8')
            except Exception:
                b = json.dumps({'state': None, 'age_ms': None,
                                'note': '无任务六状态文件（未启动或已退出）'}).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(b)))
            self.end_headers()
            self.wfile.write(b)
        elif self.path == '/api/status':
            rng = S.tlm.get('rng')


            _age = max(0, int((time.monotonic() - S.tlm_updated_at) * 1000))
            self._json(dict(
                tlm_ok=S.tlm_ok, tlm_ever=S.tlm_ok, tlm_age_ms=_age,
                tlm_valid=(bool(S.tlm_ok) and _age <= 500),
                rng=rng, yaw=S.tlm.get('yaw'),
                velL=S.tlm.get('velL'), velR=S.tlm.get('velR'),
                fpga_state=S.tlm.get('fpga_state'),
                sent=S.sent, src=S.last_src,
                guard_on=S.guard_on, guard_blocked=S.guard_blocked,
                task6=S.task6_proc is not None and S.task6_proc.poll() is None,
                task6_mode=S.task6_mode,
                task6_tasks=S.task6_tasks,
                task6_follow=S.task6_follow,
                osd_mode=S.osd_mode, tgt_key=S.tgt_key, tgt_cls=S.tgt_cls,
                preview=S.preview_on,
                person=S.person_proc is not None and S.person_proc.poll() is None,
                person_uart=S.person_uart, web_version='task6live-20261029',
                tcp_inhibit=bool(S.tcp_inhibit),
                trial_owner=bool(S.trial_owner), trial_run_id=S.trial_run_id,
                stop_seq=int(S.stop_seq),
                msg=S.msg, **mode_status(S)))
        else:
            self._json({'ok': False, 'err': 'not found'}, 404)

    def _api_trial(self, body):
        """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
        on = bool(body.get('on'))
        rid = body.get('run_id')
        with S.lock:
            if on:
                if S.task6_proc is not None and S.task6_proc.poll() is None:
                    self._json({'ok': False, 'err': '任务六在跑，拒绝交出控制权'}, 409)
                    return
                if S.cmd_dir != 0 or S.cmd_spd > 0:
                    self._json({'ok': False, 'err': '当前有运动命令，先停车'}, 409)
                    return
                exp = body.get('expect_stop_seq')
                if exp is not None:
                    try:
                        exp = int(exp)
                    except (TypeError, ValueError):
                        self._json({'ok': False, 'err': 'expect_stop_seq 非法'}, 400)
                        return
                    if exp != S.stop_seq:

                        S.trial_owner = False
                        S.trial_run_id = None
                        S.tcp_inhibit = True
                        self._json({'ok': False, 'err': 'stopped_since_grant',
                                    'stop_seq': S.stop_seq,
                                    'stop_seq_expected': exp}, 409)
                        return
                S.trial_owner = True
                S.trial_run_id = rid
                S.tcp_inhibit = False
                self._json({'ok': True, 'trial_owner': True, 'run_id': rid,
                            'stop_seq': S.stop_seq})
            else:
                S.trial_owner = False
                S.trial_run_id = None
                S.tcp_inhibit = True
                S.cmd_dir, S.cmd_spd = 0, 0
                self._json({'ok': True, 'trial_owner': False, 'stop_seq': S.stop_seq})

    def do_POST(self):
        try:
            n = int(self.headers.get('Content-Length') or 0)
            body = json.loads(self.rfile.read(n) or b'{}')
        except Exception:
            body = {}
        p = self.path
        if p == '/api/trial':
            self._api_trial(body)
        elif p == '/api/motion':
            d, sp = cmd_set(body.get('dir', 0), body.get('spd', 30), 'web')
            self._json({'ok': True, 'dir': d, 'spd': sp})
        elif p == '/api/stop':
            cmd_stop('web')
            S.msg = ''
            self._json({'ok': True, 'msg': '已停车'})
        elif p == '/api/estop':
            cmd_stop('web')
            for _ in range(5):
                U.stop()
            S.msg = '急停已执行'
            self._json({'ok': True, 'msg': S.msg})
        elif p == '/api/fpga_mode':
            if body.get('mode') == 0:cmd_stop('fpga-manual-takeover')
            ok, msg = request_mode(S, U, body.get('mode'))
            S.msg = msg
            self._json({'ok': ok, 'msg': msg, **mode_status(S)})
        elif p == '/api/guard':
            S.guard_on = bool(body.get('on'))
            S.guard_blocked = False
            S.msg = '软件避障守护：%s' % ('开' if S.guard_on else '关')
            self._json({'ok': True, 'msg': S.msg})
        elif p == '/api/preview':
            ok, m = set_preview(bool(body.get('on')))
            self._json({'ok': ok, 'msg': m})
        elif p == '/api/target':
            k = str(body.get('key','bottle'))
            TGT = {
              'bottle': ('44',          0.18, '红瓶（cls 44）'),
              'bag':    ('27,31,33',    0.18, '红书包（backpack/handbag/suitcase）'),
              'both':   ('27,31,33,44', 0.18, '红瓶 + 书包'),
              'person': ('1',           0.0,  '人（cls 1，关闭红色门槛）'),
            }
            if k not in TGT:
                self._json({'ok': False, 'msg': 'key must be bottle/bag/both/person'}, 400)
                return
            cls, red, name = TGT[k]
            S.tgt_key, S.tgt_cls, S.tgt_red = k, cls, red
            S.msg = '跟随目标已切换：%s（下次启动任务六生效）' % name
            self._json({'ok': True, 'key': k, 'cls': cls, 'red_min': red, 'msg': S.msg})
        elif p == '/api/osd':
            try:
                m = int(body.get('mode', 1))
            except Exception:
                m = -1
            if m not in (0, 1):
                self._json({'ok': False, 'msg': 'mode must be 0 or 1'}, 400)
                return
            ok = U.osd(m)
            S.osd_mode = m
            names = {0: '纯净画面（无 OSD）', 1: '只姿态角'}
            S.msg = 'OSD 已切换：%s' % names[m]
            self._json({'ok': ok, 'mode': m, 'msg': S.msg})
        elif p == '/api/person':
            ok, m = set_person(bool(body.get('on')), bool(body.get('uart')))
            self._json({'ok': ok, 'msg': m})
        elif p == '/api/task6_trial':
            seconds=body.get('seconds',2)
            if type(seconds) not in (int,float) or not 0.5<=seconds<=5:
                self._json({'ok':False,'msg':'trial duration must be 0.5..5 seconds'},400)
                return
            ok,m=set_task6(True,'follow','bottle',True,seconds)
            self._json({'ok':ok,'msg':m})
        elif p == '/api/task6':
            ok, m = set_task6(bool(body.get('on')),
                              body.get('mode', 'follow'),
                              tasks=body.get('tasks'),
                              follow_after=bool(body.get('follow')))
            self._json({'ok': ok, 'msg': m})
        else:
            self._json({'ok': False, 'err': 'not found'}, 404)


def relay_loop():
    import socket
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('0.0.0.0', RELAY_PORT))
    srv.listen(4)
    print('[TCP] 中继端口 %d 已监听（兼容 run_c.py / semantic_nav.py）' % RELAY_PORT, flush=True)

    def handle(conn):
        buf = b''
        try:
            while True:
                data = conn.recv(256)
                if not data:
                    break
                buf += data
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    t = line.strip().split()
                    if len(t) >= 2 and t[0] == b'beep':


                        try:
                            U.beep(int(t[1]) != 0)
                            S.beep_on = (int(t[1]) != 0)
                            S.beep_until = time.monotonic() + 2.5
                        except ValueError:
                            pass
                    elif len(t) >= 2 and t[0] == b'mode':


                        try:
                            if int(t[1]) == MODE_MANUAL:
                                U.mode(MODE_MANUAL)
                        except ValueError:
                            pass
                    elif len(t) >= 2:
                        try:
                            cmd_set(int(t[0]), int(t[1]), 'tcp9100')
                        except ValueError:
                            pass
        except Exception:
            pass
        finally:
            try:
                conn.close()
            except Exception:
                pass

    while True:
        try:
            c, _ = srv.accept()
            threading.Thread(target=handle, args=(c,), daemon=True).start()
        except Exception:
            time.sleep(0.2)


def _cmdline_running(*tokens, exclude_self=True):
    """判断是否有进程的 **argv 元素** 精确包含全部 tokens（读 /proc）"""
    try:
        me = os.getpid()
        for pid in os.listdir('/proc'):
            if not pid.isdigit():
                continue
            if exclude_self and int(pid) == me:
                continue
            try:
                raw = open('/proc/%s/cmdline' % pid, 'rb').read()
            except Exception:
                continue
            if not raw:
                continue
            argv = [x.decode('utf-8', 'ignore') for x in raw.split(b'\0') if x]
            if not argv:
                continue

            if all(any(a == tk or a.endswith(tk) for a in argv) for tk in tokens):

                try:
                    st = open('/proc/%s/stat' % pid).read().split()[2]
                except Exception:
                    st = 'S'
                if st == 'Z':
                    continue
                return True
    except Exception:
        pass
    return False


def start_vision():
    """确保抓帧链路在跑（grab_loop2 + frame_pub + http.server 9200）"""
    started = []
    if not _cmdline_running('/root/grab_loop2.sh'):
        subprocess.Popen(['/root/grab_loop2.sh'], stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL, start_new_session=True)
        started.append('grab_loop2')
    time.sleep(1.0)
    if not _cmdline_running('/root/frame_pub.py'):
        subprocess.Popen([sys.executable, '/root/frame_pub.py'],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                         start_new_session=True)
        started.append('frame_pub')
    if not _cmdline_running('http.server', '9200'):
        subprocess.Popen([sys.executable, '-m', 'http.server', '9200'],
                         cwd='/tmp', stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL, start_new_session=True)
        started.append('http9200')
    print('[VIS] 抓帧链路: %s' % ('、'.join(started) + ' 已启动' if started
                                  else '本来就在跑'), flush=True)


def vision_watchdog():
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    while True:
        time.sleep(5)
        try:


            if S.task6_proc is not None and S.task6_proc.poll() is not None:
                S.task6_proc = None
                S.tlm_pause = False
                print('[T6] 任务六进程已自行结束，遥测口已归还', flush=True)
            if S.preview_on:
                continue                     # 预览占着摄像头，不能起抓帧
            start_vision()
        except Exception:
            pass


def main():
    print('=' * 62, flush=True)
    print('小车远程控制台  HTTP :%d  中继 :%d  SPEED_MAX=%d%%'
          % (HTTP_PORT, RELAY_PORT, SPEED_MAX), flush=True)
    print('=' * 62, flush=True)
    threading.Thread(target=send_loop, daemon=True).start()
    threading.Thread(target=tlm_loop, daemon=True).start()
    threading.Thread(target=relay_loop, daemon=True).start()
    threading.Thread(target=start_vision, daemon=True).start()
    threading.Thread(target=vision_watchdog, daemon=True).start()
    srv = ThreadingHTTPServer(('0.0.0.0', HTTP_PORT), Handler)
    print('浏览器打开  http://<板子IP>:%d' % HTTP_PORT, flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        cmd_stop('shutdown')
        _stop_proc('task6_proc')
        _stop_proc('preview_proc')


if __name__ == '__main__':
    main()
