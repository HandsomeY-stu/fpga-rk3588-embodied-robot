#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线回归测试：检查runner相关逻辑和异常处理。"""
import json
import os
import pathlib
import socket
import socketserver
import subprocess
import sys
import tempfile
import threading
import time
import http.server

HERE = pathlib.Path(__file__).resolve().parent
PY = sys.executable


STATE = {
    'status': {},
    'vision': {},
    'trial_ok': True,
    'trial_calls': [],
    'relay_data': [],
    'relay_conns': 0,
    'fid': 100,
    'vision_raw': None,
    'stop_seq': 0,
    'bad_vision_reads': 0,
}


class FakeWeb(http.server.BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, obj, code=200):
        b = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        if self.path == '/api/status':


            s = dict(STATE['status'])
            s['stop_seq'] = STATE['stop_seq']
            self._send(s)
        elif self.path == '/api/person.json':
            if STATE.get('vision_raw') is not None:

                b = STATE['vision_raw'].encode()
                self.send_response(200)
                self.send_header('Content-Type', 'text/html')
                self.send_header('Content-Length', str(len(b)))
                self.end_headers()
                self.wfile.write(b)
                return
            if STATE.get('bad_vision_reads', 0) > 0:


                STATE['bad_vision_reads'] -= 1
                b = b'<html>gateway hiccup</html>'
                self.send_response(200)
                self.send_header('Content-Type', 'text/html')
                self.send_header('Content-Length', str(len(b)))
                self.end_headers()
                self.wfile.write(b)
                return


            STATE['fid'] += 1
            v = dict(STATE['vision'])
            v['frame_id'] = STATE['fid']
            self._send(v)
        else:
            self._send({'ok': False}, 404)

    def do_POST(self):
        n = int(self.headers.get('Content-Length') or 0)
        try:
            body = json.loads(self.rfile.read(n) or b'{}')
        except Exception:
            body = {}
        if self.path == '/api/trial':
            STATE['trial_calls'].append(body)
            exp = body.get('expect_stop_seq')
            if STATE['trial_ok'] and body.get('on') and exp is not None \
                    and int(exp) != STATE['stop_seq']:

                STATE['status']['tcp_inhibit'] = True
                STATE['status']['trial_owner'] = False
                self._send({'ok': False, 'err': 'stopped_since_grant',
                            'stop_seq': STATE['stop_seq']}, 409)
                return
            if STATE['trial_ok']:
                STATE['status']['tcp_inhibit'] = False
                STATE['status']['trial_owner'] = bool(body.get('on'))
                self._send({'ok': True, 'trial_owner': bool(body.get('on')),
                            'stop_seq': STATE['stop_seq']})
            else:
                self._send({'ok': False, 'err': 'mock refuse'}, 409)
        else:
            self._send({'ok': False}, 404)


class FakeRelay(socketserver.BaseRequestHandler):
    def handle(self):
        STATE['relay_conns'] += 1
        self.request.settimeout(0.3)
        buf = b''
        t_end = time.time() + 60
        while time.time() < t_end:
            try:
                d = self.request.recv(64)
            except socket.timeout:
                continue
            except OSError:
                break
            if not d:
                break
            buf += d
            STATE['relay_data'].append(d.decode('utf-8', 'replace').strip())


class ThreadedTCP(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def base_status(**kw):
    s = dict(tlm_ok=True, tlm_age_ms=40, fpga_mode=0, rng=1200, yaw=90.0,
             velL=0, velR=0, task6=False, task6_tasks='', person_uart=False,
             tcp_inhibit=False, trial_owner=False, stop_seq=0)
    s.update(kw)
    return s


def base_vision(fid=100, **kw):
    v = dict(color_schema=1, age_ms=40, producer_id=1, frame_id=fid,
             red_cx=664, red_n=800, red_w=110, red_h=300,
             detections=[dict(cls=27, conf=0.62, cx=664.0, cy=360.0, w=165.0,
                              h=300.0, red_ratio=0.55)])
    v.update(kw)
    return v


def run_trial(args, outdir, timeout=40):
    cmd = [PY, str(HERE / 'run_follow_trial.py'), '--outdir', str(outdir),
           '--url', 'http://127.0.0.1:%d' % WEB_PORT,
           '--relay-host', '127.0.0.1', '--relay-port', str(RELAY_PORT)] + args
    p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    return p


def reset(**over):
    STATE['status'] = base_status(**over.pop('status', {}))
    STATE['vision'] = base_vision(**over.pop('vision', {}))
    STATE['trial_ok'] = over.pop('trial_ok', True)
    STATE['trial_calls'] = []
    STATE['relay_data'] = []
    STATE['vision_raw'] = None
    STATE['relay_conns'] = 0
    STATE['stop_seq'] = 0
    STATE['bad_vision_reads'] = 0


CASES = []


def case(fn):
    CASES.append(fn)
    return fn


@case
def r01_preflight_fail_no_socket(tmp):
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    reset(status=dict(fpga_mode=1))          # 非手动模式
    rid = 'r01'
    p = run_trial(['--mode', 'live', '--profile', 'BAG', '--variant', 'G0',
                   '--seconds', '3', '--run-id', rid], tmp)
    assert p.returncode == 2, '返回码应为 2，实际 %d\n%s' % (p.returncode, p.stderr)
    d = pathlib.Path(tmp) / rid
    assert (d / 'failed_preflight.json').exists(), '未写 failed_preflight.json'
    assert STATE['relay_conns'] == 0, '预检查失败却连了运动 socket（%d 次）' % STATE['relay_conns']


@case
def r02_timeout_sends_stop(tmp):
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    reset()
    rid = 'r02'
    p = run_trial(['--mode', 'live', '--profile', 'BAG', '--variant', 'G0',
                   '--seconds', '1.2', '--run-id', rid], tmp)
    assert p.returncode == 0, '应正常结束，实际 %d\n%s\n%s' % (p.returncode, p.stdout, p.stderr)
    joined = '|'.join(STATE['relay_data'])
    assert '0 0' in joined, '收尾没有发 STOP，实际收到: %s' % joined
    res = json.loads((pathlib.Path(tmp) / rid / 'result.json').read_text(encoding='utf-8'))
    assert res['samples'] > 0, '没有控制样本'
    assert (pathlib.Path(tmp) / rid / 'after.json').exists(), '缺少 after.json'


@case
def r03_inhibit_not_bypassed(tmp):
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    reset(status=dict(tcp_inhibit=True), trial_ok=False)
    rid = 'r03'
    p = run_trial(['--mode', 'live', '--profile', 'BAG', '--variant', 'G0',
                   '--seconds', '2', '--run-id', rid], tmp)
    assert p.returncode == 2, '抑制未被拦，返回 %d' % p.returncode
    assert STATE['relay_conns'] == 0, '抑制状态下仍连了运动 socket'
    assert STATE['trial_calls'], '没有尝试申请所有权'


@case
def r04_inhibit_acquired_via_api(tmp):
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    reset(status=dict(tcp_inhibit=True), trial_ok=True)
    rid = 'r04'
    p = run_trial(['--mode', 'live', '--profile', 'BAG', '--variant', 'G0',
                   '--seconds', '1.0', '--run-id', rid], tmp)
    assert p.returncode == 0, '拿到所有权后仍失败 %d\n%s' % (p.returncode, p.stderr)
    ons = [c for c in STATE['trial_calls'] if c.get('on')]
    offs = [c for c in STATE['trial_calls'] if c.get('on') is False]
    assert ons, '没有申请所有权'
    assert offs, '收尾没有交还所有权（§6.4）'


@case
def r05_runid_exists(tmp):
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    reset()
    rid = 'r05'
    (pathlib.Path(tmp) / rid).mkdir(parents=True, exist_ok=True)
    (pathlib.Path(tmp) / rid / 'keep.txt').write_text('keep', encoding='utf-8')
    p = run_trial(['--mode', 'dry', '--profile', 'BAG', '--variant', 'G0',
                   '--seconds', '1', '--run-id', rid], tmp)
    assert p.returncode != 0, '已存在的 run-id 居然成功'
    assert (pathlib.Path(tmp) / rid / 'keep.txt').exists(), '覆盖了已有 run-id 目录'


@case
def r06_far_speed_guard(tmp):
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    reset()
    p = run_trial(['--mode', 'dry', '--profile', 'BAG', '--variant', 'G0',
                   '--seconds', '1', '--far-speed', '25', '--run-id', 'r06'], tmp)
    assert p.returncode != 0, '非法 --far-speed 未被拒'


@case
def r07_live_seconds_cap(tmp):
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    reset()


    p = run_trial(['--mode', 'live', '--profile', 'BAG', '--variant', 'G0',
                   '--seconds', '130', '--run-id', 'r07'], tmp)
    assert p.returncode != 0, 'live 130 秒未被拒'


@case
def r08_vision_stale_stops(tmp):
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    reset()
    rid = 'r08'

    th = threading.Timer(1.8, lambda: STATE['vision'].__setitem__('age_ms', 999))
    th.start()
    p = run_trial(['--mode', 'live', '--profile', 'BAG', '--variant', 'G0',
                   '--seconds', '6', '--run-id', rid], tmp)
    th.cancel()
    assert p.returncode == 1, '视觉过期未终止，返回 %d' % p.returncode
    res = json.loads((pathlib.Path(tmp) / rid / 'result.json').read_text(encoding='utf-8'))
    assert res['reason'] == 'VISION_STALE', '终止原因应为 VISION_STALE，实际 %s' % res['reason']
    assert '0 0' in '|'.join(STATE['relay_data']), '终止后没有发 STOP'


@case
def r09_wheel_over_limit(tmp):
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    reset()
    rid = 'r09'
    th = threading.Timer(1.8, lambda: STATE['status'].__setitem__('velL', 250))
    th.start()
    p = run_trial(['--mode', 'live', '--profile', 'BAG', '--variant', 'G0',
                   '--seconds', '6', '--run-id', rid], tmp)
    th.cancel()
    res = json.loads((pathlib.Path(tmp) / rid / 'result.json').read_text(encoding='utf-8'))
    assert res['reason'] == 'WHEEL_OVER_200MMPS', '应为 WHEEL_OVER_200MMPS，实际 %s' % res['reason']


@case
def r10_parse_error_stops(tmp):
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    reset()
    rid = 'r10'
    th = threading.Timer(1.8, lambda: STATE.__setitem__('vision_raw', '<html>not json</html>'))
    th.start()
    p = run_trial(['--mode', 'live', '--profile', 'BAG', '--variant', 'G0',
                   '--seconds', '6', '--run-id', rid], tmp)
    th.cancel()
    res = json.loads((pathlib.Path(tmp) / rid / 'result.json').read_text(encoding='utf-8'))
    assert res['reason'] in ('PARSE_ERROR', 'VISION_STALE'), '实际 %s' % res['reason']
    assert '0 0' in '|'.join(STATE['relay_data']), '异常后没有发 STOP'


@case
def r11_dry_never_opens_socket(tmp):
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    reset()
    p = run_trial(['--mode', 'dry', '--profile', 'BAG', '--variant', 'G0',
                   '--seconds', '1.0', '--run-id', 'r11'], tmp)
    assert p.returncode == 0, 'DRY 应正常结束 %d\n%s' % (p.returncode, p.stderr)
    assert STATE['relay_conns'] == 0, 'DRY 模式连了运动 socket'


@case
def r12_outputs_complete(tmp):
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    reset()
    rid = 'r12'
    p = run_trial(['--mode', 'dry', '--profile', 'BAG', '--variant', 'G0',
                   '--seconds', '1.2', '--run-id', rid], tmp)
    d = pathlib.Path(tmp) / rid
    need = ['manifest.json', 'before.json', 'after.json', 'control.jsonl',
            'vision.jsonl', 'telemetry.jsonl', 'result.json']
    miss = [n for n in need if not (d / n).exists()]
    assert not miss, '缺少输出文件: %s' % miss
    assert (d / 'frames').is_dir(), '缺少 frames/ 目录'
    man = json.loads((d / 'manifest.json').read_text(encoding='utf-8'))
    assert man['effective_config']['fwd_far'] == 22.0, 'manifest 未记录有效参数'
    assert man['source_sha256']['follow_policy.py'], 'manifest 缺少源码 SHA'


@case
def r13_user_stop_no_autoreclaim(tmp):
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    reset()
    rid = 'r13'

    def human_stop():

        STATE['stop_seq'] += 1
        STATE['status']['stop_seq'] = STATE['stop_seq']
        STATE['status']['tcp_inhibit'] = True
        STATE['status']['trial_owner'] = False
        STATE['calls_at_stop'] = len(STATE['trial_calls'])

    th = threading.Timer(1.2, human_stop)
    th.start()
    p = run_trial(['--mode', 'live', '--profile', 'BAG', '--variant', 'G0',
                   '--seconds', '8', '--run-id', rid], tmp)
    th.cancel()
    res = json.loads((pathlib.Path(tmp) / rid / 'result.json').read_text(encoding='utf-8'))
    assert res['reason'] == 'STOPPED_BY_USER', '人为停车后未终止，实际 %s' % res['reason']

    ons = [c for c in STATE['trial_calls'] if c.get('on')]
    assert len(ons) <= STATE.get('calls_at_stop', 0), \
        '停车后仍重新申请所有权（共 %d 次，停车时 %d 次）' % (
            len(ons), STATE.get('calls_at_stop', 0))
    assert '0 0' in '|'.join(STATE['relay_data']), '停车后没有发零速'


@case
def r14_ownership_revoked_aborts(tmp):
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    reset()
    rid = 'r14'

    def revoke():
        STATE['status']['tcp_inhibit'] = True
        STATE['status']['trial_owner'] = False
        STATE['calls_at_stop'] = len(STATE['trial_calls'])

    th = threading.Timer(1.2, revoke)
    th.start()
    p = run_trial(['--mode', 'live', '--profile', 'BAG', '--variant', 'G0',
                   '--seconds', '8', '--run-id', rid], tmp)
    th.cancel()
    res = json.loads((pathlib.Path(tmp) / rid / 'result.json').read_text(encoding='utf-8'))
    assert res['reason'] == 'OWNERSHIP_REVOKED', '撤权后未终止，实际 %s' % res['reason']
    ons = [c for c in STATE['trial_calls'] if c.get('on')]
    assert len(ons) <= STATE.get('calls_at_stop', 0), '撤权后仍重新申请所有权 %d 次' % len(ons)


@case
def r15_transient_hiccup_recovers(tmp):
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    reset()
    rid = 'r15'

    def hiccup():
        STATE['bad_vision_reads'] = 2       # 连续 2 拍坏 JSON，第 3 拍恢复

    th = threading.Timer(1.5, hiccup)
    th.start()
    p = run_trial(['--mode', 'live', '--profile', 'BAG', '--variant', 'G0',
                   '--seconds', '4', '--run-id', rid], tmp)
    th.cancel()
    res = json.loads((pathlib.Path(tmp) / rid / 'result.json').read_text(encoding='utf-8'))
    assert res['reason'] == 'COMPLETED', '2 拍抖动就把本轮判死，实际 %s' % res['reason']
    assert '0 0' in '|'.join(STATE['relay_data']), '抖动期间没有立即发零速'


@case
def r16_owner_taken_by_other(tmp):
    """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
    reset()
    rid = 'r16'

    def steal():
        STATE['status']['trial_owner'] = True
        STATE['status']['trial_run_id'] = 'someone_else'

    th = threading.Timer(1.2, steal)
    th.start()
    p = run_trial(['--mode', 'live', '--profile', 'BAG', '--variant', 'G0',
                   '--seconds', '8', '--run-id', rid], tmp)
    th.cancel()
    res = json.loads((pathlib.Path(tmp) / rid / 'result.json').read_text(encoding='utf-8'))
    assert str(res['reason']).startswith('OWNERSHIP_TAKEN'), \
        '所有权被抢后未终止，实际 %s' % res['reason']


def main():
    global WEB_PORT, RELAY_PORT
    web = http.server.ThreadingHTTPServer(('127.0.0.1', 0), FakeWeb)
    WEB_PORT = web.server_address[1]
    relay = ThreadedTCP(('127.0.0.1', 0), FakeRelay)
    RELAY_PORT = relay.server_address[1]
    threading.Thread(target=web.serve_forever, daemon=True).start()
    threading.Thread(target=relay.serve_forever, daemon=True).start()
    print('假 Web :%d  假 relay :%d' % (WEB_PORT, RELAY_PORT))

    fails = 0
    with tempfile.TemporaryDirectory() as tmp:
        for fn in CASES:
            try:
                fn(tmp)
                print('  PASS  %s' % fn.__name__)
            except AssertionError as e:
                fails += 1
                print('  FAIL  %s: %s' % (fn.__name__, e))
            except Exception as e:      # noqa: BLE001
                fails += 1
                print('  ERR   %s: %s: %s' % (fn.__name__, type(e).__name__, e))
    web.shutdown()
    relay.shutdown()
    print('-' * 70)
    print('%d/%d 通过' % (len(CASES) - fails, len(CASES)))
    return 0 if fails == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
