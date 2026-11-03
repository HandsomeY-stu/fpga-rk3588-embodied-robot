#!/usr/bin/env python3
"""对同一视觉/遥测录制比较策略输出；不连接设备。"""
import argparse
import collections
import json
import pathlib
import sys
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from follow_policy import FollowPolicy, FWD_FAMILY
PROFILES = {'BAG': (27, 31, 33), 'BOTTLE': (44,)}
PROTECT = ('RANGE_STOP', 'INVALID_RANGE', 'STALE_VISION', 'STALE_TELEMETRY', 'WRONG_MODE', 'SEARCH_DISABLED')

def load(p):
    if not p.exists():
        return []
    return [json.loads(x) for x in p.read_text(encoding='utf-8').splitlines() if x.strip()]

def replay(run_dir, profile, conf_fix, seconds_limit=None):
    """conf_fix=True 用修复后的 conf_min（=min_conf*100），False 模拟修复前（40）。"""
    vis = load(run_dir / 'vision.jsonl')
    tel = load(run_dir / 'telemetry.jsonl')
    if not vis or not tel:
        return None
    t0 = vis[0]['t_ms']
    pol = FollowPolicy(target_cls=PROFILES[profile], accel_pct_s=20, decel_pct_s=40, semantic_grace_ms=200, turn_enter_px=130, turn_exit_px=70, motion_search_allowed=False)
    if not conf_fix:
        pol._force_conf_min = True
        orig = pol._build_ctrl

        def patched():
            orig()
            pol.ctrl.conf_min = 40
        pol._build_ctrl = patched
    rows = []
    ti = 0
    for v in vis:
        t = v['t_ms'] - t0
        if seconds_limit and t > seconds_limit * 1000:
            break
        while ti + 1 < len(tel) and abs(tel[ti + 1]['t_ms'] - v['t_ms']) < abs(tel[ti]['t_ms'] - v['t_ms']):
            ti += 1
        tt = tel[ti]
        status = dict(tlm_ok=True, tlm_age_ms=tt.get('tlm_age_ms', 40), fpga_mode=0, rng=tt.get('rng'), yaw=tt.get('yaw'), velL=tt.get('velL', 0), velR=tt.get('velR', 0))
        pkt = dict(color_schema=1, age_ms=v.get('age_ms', 30), producer_id=1, frame_id=v.get('frame_id'), detections=v.get('dets') or [], red_cx=v.get('red_cx'), red_n=v.get('red_n'), red_w=v.get('red_w'), red_h=v.get('red_h'))
        c = pol.step(pkt, status, t)
        rows.append(dict(t=t, dir=c.dir, spd=c.spd, state=c.state, lock=pol.lock.state))
    return rows

def stats(rows):
    n = len(rows) or 1
    valid = [r for r in rows if not any((k in str(r['state']) for k in PROTECT))]
    fwd = sum((1 for r in valid if r['dir'] in FWD_FAMILY and r['spd'] > 0))
    stop = sum((1 for r in rows if r['dir'] == 0 or r['spd'] <= 0))
    ev = 0
    prev_stop = False
    for r in rows:
        s = r['dir'] == 0 or r['spd'] <= 0
        if s and (not prev_stop):
            ev += 1
        prev_stop = s
    return dict(samples=n, stop_samples=stop, stop_pct=round(100.0 * stop / n, 2), stop_events=ev, valid=len(valid), forward_pct=round(100.0 * fwd / len(valid), 2) if valid else 0.0, states=dict(collections.Counter((r['state'] for r in rows)).most_common(10)))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir')
    ap.add_argument('--profile', choices=tuple(PROFILES), default='BAG')
    a = ap.parse_args()
    d = pathlib.Path(a.run_dir)
    before = replay(d, a.profile, conf_fix=False)
    after = replay(d, a.profile, conf_fix=True)
    if before is None:
        print('缺 vision.jsonl / telemetry.jsonl:', d)
        return 2
    sb, sa = (stats(before), stats(after))
    print('=== %s（%s）离线重放对比 ===' % (d.name, a.profile))
    print('%-22s %-14s %-14s' % ('指标', '修复前(conf_min=40)', '修复后(=min_conf)'))
    for k in ('samples', 'stop_samples', 'stop_pct', 'stop_events', 'valid', 'forward_pct'):
        print('%-22s %-18s %-18s' % (k, sb[k], sa[k]))
    print()
    print('修复前状态:', sb['states'])
    print('修复后状态:', sa['states'])
    return 0
if __name__ == '__main__':
    sys.exit(main())
