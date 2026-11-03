#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务五执行入口，提供DRY/LIVE模式、预检、控制权管理和结束停车。"""
import argparse
import json
import math
import os
import pathlib
import socket
import sys
import time
import urllib.request

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from follow_policy import FollowPolicy   # noqa: E402


PROFILES = {
    'BAG':    dict(target_cls=(27, 31, 33)),
    'BOTTLE': dict(target_cls=(44,)),
}


FIXED = dict(range_stop_mm=300, close_h=0.96, min_conf=0.30, red_min=0.18,
             fwd_near=10, trim_cap=12, red_min_spd=4, red_cue_spd=8,
             d_far=1000, d_near=400)


VARIANTS = {
    'G0': [],
    'S1': ['unified_speed'],
    'S2': ['unified_speed', 'shape'],
    'S3': ['unified_speed', 'shape', 'grace'],
    'S4': ['unified_speed', 'shape', 'grace', 'red_delay'],
    'S5': ['unified_speed', 'shape', 'grace', 'turn_hyst'],
    'S6': ['unified_speed', 'shape', 'grace', 'turn_hyst', 'dir_confirm'],
}


BASE_ARGS = dict(accel_pct_s=0, decel_pct_s=0)

MECH_ARGS = {
    'unified_speed': dict(unified_speed=True),
    'shape':         dict(accel_pct_s=20, decel_pct_s=40),
    'grace':         dict(semantic_grace_ms=200),
    'red_delay':     dict(red_cue_after_ms=900),
    'turn_hyst':     dict(turn_enter_px=130, turn_exit_px=70),
    'dir_confirm':   dict(direction_confirm_frames=2),
}


LIMITS = dict(vision_age_stop_ms=300, tlm_age_stop_ms=400,
              wheel_speed_max_mmps=200, preflight_rng_min=800,
              preflight_rng_max=2500, still_mmps=10, still_samples=3)


class TrialAbort(Exception):
    pass


def now_ms():
    return time.monotonic() * 1000.0


class Trial(object):
    def __init__(self, a):
        self.a = a
        self.outdir = pathlib.Path(a.outdir) / a.run_id
        self.t0 = now_ms()
        self.sock = None
        self.policy = None
        self.stops = []
        self.last_rng = None
        self.last_status = {}
        self.wheel_max = 0.0


        self.grant_stop_seq = None
        self._parse_err = 0
        self.frames_dir = self.outdir / 'frames'
        self.files = {}
        self._last_vid = None
        self._vision_dedup = 0


    def prepare(self):
        if self.outdir.exists():
            raise TrialAbort('run-id 已存在，拒绝覆盖: %s' % self.outdir)
        self.outdir.mkdir(parents=True)
        self.frames_dir.mkdir()
        for name in ('control', 'vision', 'telemetry', 'events'):
            self.files[name] = (self.outdir / (name + '.jsonl')).open('w', encoding='utf-8')
        mech = VARIANTS[self.a.variant]
        if self.a.only:
            mech = [m for m in self.a.only.split(',') if m]
        kw = dict(PROFILES[self.a.profile])
        kw.update(BASE_ARGS)          # 先落 G0 基线
        kw.update(FIXED)
        kw['trim_cap'] = float(getattr(self.a, 'trim_cap', 12.0))

        kw['red_follow'] = bool(getattr(self.a, 'red_follow', False))


        kw['turn_enter_px'] = float(getattr(self.a, 'turn_enter', 130.0))
        kw['turn_exit_px'] = float(getattr(self.a, 'turn_exit', 70.0))
        kw['red_min_spd'] = float(getattr(self.a, 'red_min_spd', 4.0))
        kw['red_cue_spd'] = float(getattr(self.a, 'red_cue_spd', 8.0))
        kw['fwd_far'] = float(self.a.far_speed)
        kw['max_spd'] = max(24.0, float(self.a.far_speed))
        kw['motion_search_allowed'] = False

        kw['trim_test_cap'] = float(self.a.trim_test_cap)
        kw['trim_speed_mode'] = self.a.trim_speed_mode
        kw['red_hold_cap'] = float(self.a.red_hold_cap)
        kw['coast_cap'] = float(self.a.coast_cap)


        kw['qualify_tlm_age'] = float(self.a.qualify_tlm_age)
        for m in mech:
            if m not in MECH_ARGS:
                raise TrialAbort('未知机制 %r（可选 %s）' % (m, sorted(MECH_ARGS)))
            kw.update(MECH_ARGS[m])
        self.mech = mech
        self.policy = FollowPolicy(**kw)
        return self.policy.effective_config()

    def api(self, path):
        url = self.a.url + path
        try:
            with urllib.request.urlopen(url, timeout=0.25) as r:
                return json.load(r)
        except Exception as e:
            return {'_error': '%s: %s' % (type(e).__name__, e)}

    def api_post(self, path, body):
        """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
        data = json.dumps(body).encode()
        req = urllib.request.Request(self.a.url + path, data=data,
                                     headers={'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(req, timeout=0.5) as r:
                return json.load(r)
        except Exception as e:
            return {'_error': '%s: %s' % (type(e).__name__, e)}

    def log(self, name, row):
        f = self.files.get(name)
        if f:
            f.write(json.dumps(row, ensure_ascii=False, default=str) + '\n')
            f.flush()


    def preflight(self):
        """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
        checks = {}
        status = self.api('/api/status')
        vision = self.api('/api/person.json')
        self.last_status = status if isinstance(status, dict) else {}


        checks['manual_mode'] = (status.get('fpga_mode') == 0)


        t6 = bool(status.get('task6'))
        nav = bool(status.get('nav_running'))
        checks['no_task6_or_nav'] = (not t6) and (not nav)

        try:
            vl, vr = float(status.get('velL')), float(status.get('velR'))
            checks['wheels_still'] = (abs(vl) <= LIMITS['still_mmps']
                                      and abs(vr) <= LIMITS['still_mmps'])
        except (TypeError, ValueError):
            checks['wheels_still'] = False

        age = status.get('tlm_age_ms')
        checks['tlm_fresh'] = isinstance(age, (int, float)) and 0 <= age <= 300

        vage = vision.get('age_ms')
        checks['vision_fresh'] = isinstance(vage, (int, float)) and 0 <= vage <= 200

        rng = status.get('rng')
        checks['range_in_band'] = (isinstance(rng, (int, float)) and math.isfinite(rng)
                                   and LIMITS['preflight_rng_min'] <= rng <= LIMITS['preflight_rng_max'])

        checks['three_distinct_frames'] = self._three_frames()

        checks['detector_uart_off'] = (status.get('person_uart') in (None, False))


        if status.get('tcp_inhibit') and not status.get('trial_owner'):
            r = self.api_post('/api/trial', {'on': True, 'run_id': self.a.run_id,
                                             'expect_stop_seq': getattr(self, 'grant_stop_seq', None)})
            if r.get('ok'):
                self.owns_trial = True


                if r.get('stop_seq') is not None:
                    self.grant_stop_seq = r.get('stop_seq')
                s2 = self.api('/api/status')
                checks['trial_owner'] = (not s2.get('tcp_inhibit'))
                if s2.get('stop_seq') is not None:
                    self.grant_stop_seq = s2.get('stop_seq')
            else:
                checks['trial_owner'] = False
                info_err = r.get('err') or r.get('_error')
                checks['trial_owner_err'] = bool(info_err)
        else:
            checks['trial_owner'] = (not status.get('tcp_inhibit')) or bool(status.get('trial_owner'))
            if status.get('stop_seq') is not None:
                self.grant_stop_seq = status.get('stop_seq')

        ok = all(checks.values())
        info = dict(checks=checks, ok=ok, status=status, vision_age=vage,
                    rng=rng, ts=time.time())
        (self.outdir / 'before.json').write_text(
            json.dumps(info, ensure_ascii=False, indent=2, default=str), encoding='utf-8')
        if not ok:
            (self.outdir / 'failed_preflight.json').write_text(
                json.dumps(info, ensure_ascii=False, indent=2, default=str), encoding='utf-8')
        return ok, info

    def _three_frames(self):
        seen = []
        t_end = time.monotonic() + 1.5
        while time.monotonic() < t_end and len(seen) < 3:
            v = self.api('/api/person.json')
            fid = v.get('frame_id')
            if fid is None or fid in seen:
                time.sleep(0.05)
                continue
            seen.append(fid)
            if not self._has_valid_target(v):
                return False
            time.sleep(0.05)
        return len(seen) >= 3

    def _has_valid_target(self, v):
        cls = set(PROFILES[self.a.profile]['target_cls'])
        for d in (v.get('detections') or []):
            if not isinstance(d, dict):
                continue
            try:
                if (d.get('cls') in cls and float(d.get('conf') or 0) >= FIXED['min_conf']
                        and float(d.get('red_ratio') or 0) >= FIXED['red_min']):
                    return True
            except (TypeError, ValueError):
                continue
        return False


    def run(self):
        cfg = self.prepare()
        manifest = self.manifest(cfg)
        (self.outdir / 'manifest.json').write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2, default=str), encoding='utf-8')

        ok, info = self.preflight()
        if not ok:


            if getattr(self, 'owns_trial', False):
                self.api_post('/api/trial', {'on': False, 'run_id': self.a.run_id})
                self.owns_trial = False

            (self.outdir / 'after.json').write_text(
                json.dumps({'status': info.get('status'), 'stop_confirmed': None,
                            'note': 'preflight failed, no motion issued', 'ts': time.time()},
                           ensure_ascii=False, indent=2, default=str), encoding='utf-8')
            self.result('FAILED_PREFLIGHT', info)
            return 2

        if self.a.mode == 'live':
            try:
                self.sock = socket.create_connection((self.a.relay_host, self.a.relay_port),
                                                     timeout=0.5)
                self.sock.settimeout(0.2)
            except OSError as e:
                self.result('NO_RELAY', {'error': str(e)})
                return 3

        t_end = time.monotonic() + float(self.a.seconds)
        reason = None
        try:
            while time.monotonic() < t_end:
                tick = time.monotonic()
                reason = self.tick()
                if reason:
                    break
                time.sleep(max(0.0, 0.1 - (time.monotonic() - tick)))   # 10Hz 控制
        except TrialAbort as e:
            reason = 'ABORT:%s' % e
        except Exception as e:                                    # noqa: BLE001
            reason = 'EXCEPTION:%s:%s' % (type(e).__name__, e)
        finally:
            self.shutdown(reason)
        return 0 if reason is None else 1

    def _transient_or_abort(self, reason):
        """接口与参数保持当前实现；关键判据见对应代码与模块索引。"""
        self._parse_err = getattr(self, '_parse_err', 0) + 1
        self._hard_zero()
        if self._parse_err >= LIMITS.get('parse_err_abort', 3):
            return reason
        return None        # 本拍跳过，不下发命令

    def _hard_zero(self):
        """不受平滑/策略影响，直接给电机发零速并清掉发送结果。"""
        if self.a.mode == 'live' and self.sock is not None:
            try:
                self.sock.sendall(b'0 0\n')
                self._send_result = 'ok_zero(transient)'
            except OSError as e:
                self._send_result = 'fail:%s' % e

    def tick(self):
        t = now_ms()
        status = self.api('/api/status')
        vision = self.api('/api/person.json')
        if not isinstance(status, dict) or not isinstance(vision, dict):
            return self._transient_or_abort('PARSE_ERROR')
        if '_error' in status or '_error' in vision:
            return self._transient_or_abort('PARSE_ERROR')
        self._parse_err = 0
        self.last_status = status


        ss = status.get('stop_seq')
        if (self.grant_stop_seq is not None and isinstance(ss, int)
                and ss != self.grant_stop_seq):
            return 'STOPPED_BY_USER'

        if not status.get('trial_owner') and status.get('tcp_inhibit'):
            if self.a.mode == 'dry':
                pass        # DRY 模式不开所有权，也不检查
            else:
                return 'OWNERSHIP_REVOKED'

        if (self.a.mode == 'live' and status.get('trial_owner')
                and status.get('trial_run_id') not in (None, '', self.a.run_id)):
            return 'OWNERSHIP_TAKEN:%s' % status.get('trial_run_id')
        vage = vision.get('age_ms', 1e9)
        if isinstance(vage, (int, float)) and vage > LIMITS['vision_age_stop_ms']:
            return 'VISION_STALE'
        tage = status.get('tlm_age_ms', 1e9)
        if isinstance(tage, (int, float)) and tage > LIMITS['tlm_age_stop_ms']:
            return 'TELEMETRY_STALE'
        if status.get('fpga_mode') != 0:
            return 'MODE_CHANGED'
        for k in ('velL', 'velR'):
            try:
                self.wheel_max = max(self.wheel_max, abs(float(status.get(k))))
            except (TypeError, ValueError):
                pass


        if self.wheel_max > float(getattr(self.a, 'wheel_max', LIMITS['wheel_speed_max_mmps'])):
            return 'WHEEL_OVER_%gMMPS' % float(getattr(self.a, 'wheel_max',
                                                       LIMITS['wheel_speed_max_mmps']))

        cmd = self.policy.step(vision, status, t)


        row = dict(t_ms=t, frame_id=vision.get('frame_id'),
                   state=cmd.state, dir=cmd.dir, spd=cmd.spd,
                   lock=self.policy.lock.state, rng=status.get('rng'),
                   velL=status.get('velL'), velR=status.get('velR'))
        try:
            row.update(self.policy.debug or {})
        except Exception:      # noqa: BLE001
            pass
        row['actual_send_ts'] = round(now_ms(), 1) if self.a.mode == 'live' else None


        row['send_result'] = getattr(self, '_send_result', 'none')
        row['command_owner'] = ('trial:%s' % self.a.run_id) if self.a.mode == 'live' else 'dry'
        self.log('control', row)

        fid = vision.get('frame_id')
        if fid != self._last_vid:
            self._last_vid = fid


            self.log('vision', dict(t_ms=t, frame_id=fid, age_ms=vage,
                                    red_cx=vision.get('red_cx'), red_cy=vision.get('red_cy'),
                                    red_n=vision.get('red_n'), red_w=vision.get('red_w'),
                                    red_h=vision.get('red_h'),
                                    dets=vision.get('detections')))
        self.log('telemetry', dict(t_ms=t, tlm_age_ms=tage, rng=status.get('rng'),
                                   yaw=status.get('yaw'), velL=status.get('velL'),
                                   velR=status.get('velR')))

        if self.a.mode == 'live' and self.sock is not None:
            try:
                self.sock.sendall(('%d %d\n' % (cmd.dir, cmd.spd)).encode())
                self._send_result = 'ok'
            except OSError as e:
                self._send_result = 'fail:%s' % e
                return 'SEND_FAIL:%s' % e


        if now_ms() - self.t0 > (float(self.a.seconds) + 30.0) * 1000.0:
            return 'HARD_TIMEOUT'
        return None


    # 结束与异常路径均先停车，再确认遥测轮速，最后释放控制权。
    def shutdown(self, reason):


        if self.sock is not None:
            try:
                self.sock.sendall(b'0 0\n')
            except OSError:
                pass

        still = 0
        prev_age = None
        t_end = time.monotonic() + 2.5
        while time.monotonic() < t_end and still < LIMITS['still_samples']:
            s = self.api('/api/status')
            if isinstance(s, dict) and '_error' not in s:
                age = s.get('tlm_age_ms')
                distinct = (age != prev_age)
                prev_age = age
                try:
                    ok = (abs(float(s.get('velL'))) <= LIMITS['still_mmps']
                          and abs(float(s.get('velR'))) <= LIMITS['still_mmps'])
                except (TypeError, ValueError):
                    ok = False
                if ok and distinct:
                    still += 1
                self.log('telemetry', dict(t_ms=now_ms(), phase='shutdown',
                                           tlm_age_ms=age, rng=s.get('rng'),
                                           velL=s.get('velL'), velR=s.get('velR')))
            time.sleep(0.1)
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass

        if getattr(self, 'owns_trial', False):
            self.api_post('/api/trial', {'on': False, 'run_id': self.a.run_id})
            self.owns_trial = False
        after = self.api('/api/status')
        (self.outdir / 'after.json').write_text(
            json.dumps({'status': after, 'still_samples': still,
                        'stop_confirmed': still >= LIMITS['still_samples'],
                        'ts': time.time()}, ensure_ascii=False, indent=2, default=str),
            encoding='utf-8')
        self.result(reason or 'COMPLETED', {'still_samples': still,
                                            'stop_confirmed': still >= LIMITS['still_samples']})

    def manifest(self, cfg):
        import hashlib
        def h(p):
            p = pathlib.Path(p)
            return hashlib.sha256(p.read_bytes()).hexdigest() if p.exists() else None
        return dict(
            run_id=self.a.run_id, mode=self.a.mode, profile=self.a.profile,
            variant=self.a.variant, mechanisms=self.mech,
            seconds=self.a.seconds, far_speed=self.a.far_speed,
            effective_config=cfg,
            source_sha256={n: h(HERE / n) for n in
                           ('follow_policy.py', 'prop_ctrl.py', 'red_bottle_target.py',
                            'run_follow_trial.py')},
            limits=LIMITS, started=time.time(),
        )

    def result(self, reason, extra):
        cnt = {}
        rows = []
        p = self.outdir / 'control.jsonl'
        if p.exists():
            for line in p.read_text(encoding='utf-8').splitlines():
                try:
                    r = json.loads(line)
                    rows.append(r)
                    s = r['state']
                    cnt[s] = cnt.get(s, 0) + 1
                except Exception:      # noqa: BLE001
                    pass
        total = sum(cnt.values()) or 1


        PROTECT = ('RANGE_STOP', 'INVALID_RANGE', 'STALE_VISION',
                   'STALE_TELEMETRY', 'WRONG_MODE', 'SEARCH_DISABLED',
                   'WHEEL_OVER', 'HARD_TIMEOUT')
        valid = [r for r in rows
                 if not any(k in str(r.get('state', '')) for k in PROTECT)]
        fwd = sum(1 for r in valid
                  if r.get('dir') in (1, 3, 4) and float(r.get('spd') or 0) > 0)
        fwd_pct = round(100.0 * fwd / len(valid), 2) if valid else 0.0
        fwd_all = sum(1 for r in rows
                      if r.get('dir') in (1, 3, 4) and float(r.get('spd') or 0) > 0)
        fwd_pct_all = round(100.0 * fwd_all / total, 2)

        unneeded = sum(1 for r in valid
                       if (r.get('dir') == 0 or float(r.get('spd') or 0) <= 0)
                       and str(r.get('state', '')) not in ('LOST',))
        res = dict(run_id=self.a.run_id, reason=reason,
                   samples=total, state_counts=cnt,
                   forward_pct=fwd_pct,
                   forward_pct_all=fwd_pct_all,
                   valid_samples=len(valid),
                   unneeded_stops=unneeded,
                   wheel_max_mmps=self.wheel_max,
                   stop_confirmed=extra.get('stop_confirmed'),
                   stats=self.policy.stats if self.policy else {},
                   qualify_block=dict(getattr(self.policy, 'qual_block', {}) or {}),
                   ts=time.time())
        (self.outdir / 'result.json').write_text(
            json.dumps(res, ensure_ascii=False, indent=2, default=str), encoding='utf-8')
        for f in self.files.values():
            try:
                f.close()
            except Exception:      # noqa: BLE001
                pass
        print(json.dumps(res, ensure_ascii=False, indent=2, default=str))


def main():
    ap = argparse.ArgumentParser(
        description='任务五/六跟随试验执行器（默认 DRY，默认 5 秒）')
    ap.add_argument('--mode', choices=('dry', 'live'), default='dry')
    ap.add_argument('--profile', choices=tuple(PROFILES), default='BAG')
    ap.add_argument('--variant', choices=tuple(VARIANTS), default='G0')
    ap.add_argument('--seconds', type=float, default=5.0)
    ap.add_argument('--turn-enter', type=float, default=130.0,
                    help='进入转向的横向死区（px，默认 130＝原值）。调大 ⇒ 少了小幅来回，代价是偏得多才纠')
    ap.add_argument('--turn-exit', type=float, default=70.0,
                    help='退出转向的横向死区（px，默认 70＝原值），须小于 --turn-enter 形成迟滞')
    ap.add_argument('--red-min-spd', type=float, default=4.0,
                    help='SEEK_RED 原地转头时的小差速（默认 4%%，转得很慢像"停着"）')
    ap.add_argument('--red-cue-spd', type=float, default=8.0,
                    help='SEEK_RED 误差最大时的差速（默认 8%%，实测 4.6s 才转过来）')
    ap.add_argument('--red-follow', action='store_true',
                    help='★任务五专用★ 锁定过一次后丢失时"跟红不停车"（RED_FOLLOW/放行 SEEK_RED）。'
                         '任务六**不要开**：任务六需要"停下来找目标再重新锁定"的流程。默认关闭。')
    ap.add_argument('--wheel-max', type=float, default=float(LIMITS['wheel_speed_max_mmps']),
                    help='轮速保护上限 mm/s（默认 200；跑 far>35 才需要显式抬高，'
                         '实测 45%%≈217、60%%≈325）')
    ap.add_argument('--far-speed', type=float, default=22.0,
                    help='fwd_far，本轮搜索上界 30（计划书 §11）')
    ap.add_argument('--run-id', required=True)

    ap.add_argument('--trim-speed-mode', choices=('off', 'qualified-fixed'), default='off',
                    help='off=基线(恒用 trim_cap) / qualified-fixed=仅可靠目标用 trim_test_cap')
    ap.add_argument('--trim-cap', type=float, default=12.0,
                    help='基础纠偏档（默认 12%%）。实测 45%% 前进时目标 46%% 时间超死区、'
                         'TRIM 全程贴 12%% 上限 ⇒ 横向跟不上，可显式抬到 14/16/18 试验')
    ap.add_argument('--trim-test-cap', type=float, default=12.0,
                    help='本轮候选纠偏上限，只允许 12/14/16/18/20（>20 拒绝）')
    ap.add_argument('--red-hold-cap', type=float, default=12.0,
                    help='RED_HOLD 上限，恒 ≤12（不因提速获得加速权限）')
    ap.add_argument('--coast-cap', type=float, default=12.0,
                    help='COAST 上限，恒 ≤12')


    ap.add_argument('--qualify-tlm-age', type=float, default=200.0,
                    help='提速资格的遥测 age 门槛(ms)，默认 200；实车验证后可传 300')
    ap.add_argument('--only', default='',
                    help='只开指定机制（逗号分隔），用于单项复测')
    ap.add_argument('--outdir', default=str(HERE.parent / 'experiment'),
                    help='默认 <工程根>/experiment（HERE 是 software/task6，父目录即工程根）')
    ap.add_argument('--url', default='http://127.0.0.1:8080')
    ap.add_argument('--relay-host', default='127.0.0.1')
    ap.add_argument('--relay-port', type=int, default=9100)
    a = ap.parse_args()


    if a.trim_test_cap not in (12.0, 14.0, 16.0, 18.0, 20.0):
        ap.error('--trim-test-cap 只允许 12/14/16/18/20（本轮纠偏档上限 20）')


    if a.red_hold_cap > 50:
        ap.error('--red-hold-cap 不得超过 50')
    if a.coast_cap > 12:
        ap.error('--coast-cap 不得超过 12（滑行无视觉确认，不加速）')
    if a.mode == 'live' and a.seconds > 120:
        ap.error('单次实车不超过 120 秒（原计划书 30 秒，用户要求放宽）')


    if a.far_speed not in (22, 24, 26, 28, 30, 35, 40, 45, 50, 55, 60):
        ap.error('--far-speed 只允许 22/24/26/28/30/35/40/45/50/55/60（≤60，实测依据见注释）')
    return Trial(a).run()


if __name__ == '__main__':
    sys.exit(main())
