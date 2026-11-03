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
from pathlib import Path
from assets import load_index, serve_static
from fpga_web_modes import request_mode, mode_status

REPO_ROOT = Path(__file__).resolve().parents[1]
HERE = os.environ.get('ROBOT_TASK_DIR', str(REPO_ROOT / 'software' / 'task6_red_bottle'))
GRAB_SCRIPT = os.environ.get('ROBOT_GRAB_SCRIPT', str(REPO_ROOT / 'software/runtime/grab_loop2.sh'))
FRAME_SCRIPT = os.environ.get('ROBOT_FRAME_SCRIPT', str(REPO_ROOT / 'software/task5_follow/frame_pub.py'))

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
        self.task6_start_lock = threading.Lock()
        self.task6_generation = 0
        self.tcp_inhibit = False
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
        _kill_by_cmdline('grab_loop2.sh')
        _kill_by_cmdline('frame_pub.py')
        _kill_by_cmdline('gst-launch-1.0')
        time.sleep(1.5)
        e = dict(os.environ)
        e['DISPLAY'] = ':0'
        p = subprocess.Popen(
            ['gst-launch-1.0', '-q', 'v4l2src', 'device=/dev/video0',
             '!', 'videoconvert', '!', 'video/x-raw,format=I420,colorimetry=bt601',
             '!', 'xvimagesink', 'sync=false'],
            env=e, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True)
        S.preview_proc = p
        S.msg = '预览已开启（抓帧暂停）'
        return True, '预览已开启'
    else:
        _stop_proc('preview_proc')
        _kill_by_cmdline('gst-launch-1.0')
        time.sleep(1.5)
        subprocess.Popen([GRAB_SCRIPT], stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL, start_new_session=True)
        time.sleep(1.0)
        subprocess.Popen(['python3', FRAME_SCRIPT], stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL, start_new_session=True)
        S.msg = '预览已关闭（抓帧恢复）'
        return True, '预览已关闭'


PERSON_BIN = os.environ.get('ROBOT_DETECTOR_BIN', str(REPO_ROOT / 'software/detector/realtime_yolo'))
PERSON_DEMO = os.environ.get('ROBOT_MODEL_DIR', str(REPO_ROOT / 'models'))
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
    '--other-conf', '0.30',
    '--sharpen', '0.6',
    '--bmp-period', '0.05',
    '--bmp-scale', '2',
    '--mask-rect', '32,32,472,144',
    '--verbose',
]

def set_person(on, uart=False):
    """开/关目标识别看板。"""
    if on:
        if S.task6_proc is not None and S.task6_proc.poll() is None:
            return False, '任务六正在跑，先停任务六（两者都要占摄像头）'
        if not os.path.exists(PERSON_BIN):
            return False, '板上没找到 %s（先部署检测器程序）' % PERSON_BIN

        for pat in (GRAB_SCRIPT, FRAME_SCRIPT):
            _kill_by_cmdline(pat)
        subprocess.run(['pkill', '-x', 'gst-launch-1.0'],
                       capture_output=True)
        time.sleep(0.6)
        if S.person_proc is not None and S.person_proc.poll() is None:
            return True, '目标识别已在运行'
        try:
            argv = list(PERSON_ARGV)
            if not uart:
                argv.append('--no-uart')      # 默认不发 CMD_VISION（避免与运动命令抢串口）
            S.person_proc = subprocess.Popen(
                argv, stdout=open('/tmp/person.log', 'wb'),
                stderr=subprocess.STDOUT)
        except Exception as e:
            return False, '启动失败: %s' % e
        S.person_uart = bool(uart)
        S.msg = ('目标识别已开启（人员+物体，黄色方框 + 类别名）'
                 + ('，并同步 FPGA OSD' if uart else ''))
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
    if S.preview_proc is not None:
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
          '--range-stop','350','--close-h','0.82',
          '--target-cls',S.tgt_cls,'--red-min',str(S.tgt_red),
          '--min-conf','0.10']
    if follow:argv.append('--follow')
    with S.lock:
        if token != S.task6_generation:return False,'启动已被停车或手动接管取消'
        if mode_status(S).get('fpga_mode')!=0:return False,'模式或遥测已改变，取消启动'
        S.tcp_inhibit=False
        _spawn('task6_proc',argv)
        S.task6_mode=mode;S.task6_tasks='red_bottle';S.task6_follow=follow
    S.msg='红瓶跟随已启动' if follow else '红瓶靠近已启动（到位即停）'
    return True,S.msg


HTML = load_index()


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
        # Static resources use a fixed whitelist; no filesystem path is accepted from the URL.
        if self.path.startswith('/static/'):
            serve_static(self, self.path)
            return
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
                preview=S.preview_proc is not None and S.preview_proc.poll() is None,
                person=S.person_proc is not None and S.person_proc.poll() is None,
                person_uart=S.person_uart, web_version='integrated-v48-20261103-r03',
                msg=S.msg, **mode_status(S)))
        else:
            self._json({'ok': False, 'err': 'not found'}, 404)

    def do_POST(self):
        try:
            n = int(self.headers.get('Content-Length') or 0)
            body = json.loads(self.rfile.read(n) or b'{}')
        except Exception:
            body = {}
        p = self.path
        if p == '/api/motion':
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
    # V4L2 is exclusive: leave the camera with an active detector or preview.
    for proc in (S.person_proc, S.preview_proc):
        if proc is not None and proc.poll() is None:
            return
    started = []
    if not _cmdline_running(GRAB_SCRIPT):
        subprocess.Popen([GRAB_SCRIPT], stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL, start_new_session=True)
        started.append('grab_loop2')
    time.sleep(1.0)
    if not _cmdline_running(FRAME_SCRIPT):
        subprocess.Popen([sys.executable, FRAME_SCRIPT],
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
            if S.preview_proc is not None and S.preview_proc.poll() is None:
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
