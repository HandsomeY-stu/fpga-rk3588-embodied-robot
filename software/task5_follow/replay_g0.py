#!/usr/bin/env python3
"""重放录制视觉包并统计目标保持与输出状态；不连接设备。"""
import argparse
import json
import math
import pathlib
import sys
import collections
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from follow_policy import FollowPolicy, FWD_FAMILY, SPIN_FAMILY, SEARCH_STATES
PROFILES = {'BAG': (27, 31, 33), 'BOTTLE': (44,)}

def build_packet(row, fid, profile, assume_cx=664.0, use_confirming=True):
    """历史行 → 视觉包。

    ⚠ 历史 jsonl 只在 `lock_state == 'LOCKED'` 时记录 `target`，直接按"有没有 target"
      还原会把 60%+ 的帧误判成"没有候选"。而 `lock_state` 里 `CONFIRMING*` 明确表示
      "这一帧检到了候选但还没凑够 3 帧确认"，所以用 `--assume-cx` 补一个占位框，
      才能近似真实情况（cx 未知，默认取画面中心 664）。
    """
    t = row.get('target')
    ls = str(row.get('lock_state') or '')
    dets = []
    red_cx = None
    if isinstance(t, dict):
        cls = PROFILES[profile][0]
        dets = [dict(cls=cls, conf=float(t.get('conf') or 60) / 100.0, cx=float(t['cx']), cy=float(t.get('cy') or 360), w=float(t.get('w') or 150), h=float(t.get('h') or 300), red_ratio=0.55)]
        red_cx = float(t['cx'])
    elif use_confirming and ls.startswith('CONFIRMING'):
        cls = PROFILES[profile][0]
        dets = [dict(cls=cls, conf=0.45, cx=float(assume_cx), cy=360.0, w=150.0, h=300.0, red_ratio=0.55)]
        red_cx = float(assume_cx)
    return dict(color_schema=1, age_ms=30, producer_id=1, frame_id=fid, detections=dets, red_cx=red_cx, red_n=800 if red_cx is not None else 0, red_w=110 if red_cx is not None else 0, red_h=300 if red_cx is not None else 0)

def replay(path, rng, profile, variant_mech, assume_cx=664.0):
    rows = [json.loads(x) for x in pathlib.Path(path).read_text(encoding='utf-8').splitlines() if x.strip().startswith('{')]
    kw = dict(target_cls=PROFILES[profile], fwd_far=22, fwd_near=10, max_spd=24, trim_cap=12, red_min_spd=4, red_cue_spd=8, range_stop_mm=300, close_h=0.96, min_conf=0.3, red_min=0.18, motion_search_allowed=False)
    kw.update(variant_mech)
    pol = FollowPolicy(**kw)
    out = []
    for i, r in enumerate(rows):
        fid = r.get('frame_id') if isinstance(r.get('frame_id'), int) else i // 3
        p = build_packet(r, fid, profile, assume_cx=assume_cx)
        s = dict(tlm_ok=True, tlm_age_ms=40, fpga_mode=0, rng=rng, yaw=r.get('yaw') or 90.0, velL=0, velR=0)
        c = pol.step(p, s, i * 100.0)
        out.append(dict(t=i * 100.0, dir=c.dir, spd=c.spd, state=c.state, lock=pol.lock.state, visible=bool(p['detections'])))
    return (out, pol)

def metrics(out):
    n = len(out) or 1
    cnt = collections.Counter((x['state'] for x in out))
    fwd = sum((1 for x in out if x['dir'] in FWD_FAMILY and x['spd'] > 0))
    spin = sum((1 for x in out if x['dir'] in SPIN_FAMILY))
    stop = sum((1 for x in out if x['dir'] == 0 or x['spd'] <= 0))
    protect = ('RANGE_STOP', 'INVALID_RANGE', 'STALE_VISION', 'STALE_TELEMETRY', 'WRONG_MODE', 'WHEEL_OVER', 'SEARCH_DISABLED')
    unneeded = sum((1 for x in out if x['visible'] and (x['dir'] == 0 or x['spd'] <= 0) and (not any((k in x['state'] for k in protect))) and (x['state'] not in ('LOST',))))
    jumps = sum((1 for a, b in zip(out, out[1:]) if a['dir'] in FWD_FAMILY and b['dir'] in FWD_FAMILY and (abs(b['spd'] - a['spd']) >= 8)))
    rev = sum((1 for a, b in zip(out, out[1:]) if a['dir'] in SPIN_FAMILY and b['dir'] in SPIN_FAMILY and (a['dir'] != b['dir'])))
    search = sum((1 for x in out if x['state'] in SEARCH_STATES))
    return dict(samples=n, forward_pct=round(100.0 * fwd / n, 2), spin_pct=round(100.0 * spin / n, 2), stop_pct=round(100.0 * stop / n, 2), search_states=search, direction_reversals=rev, unneeded_stops=unneeded, fwd_jumps_ge8=jumps, state_counts=dict(cnt.most_common(10)))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log')
    ap.add_argument('--rng', type=float, default=1200.0)
    ap.add_argument('--profile', choices=tuple(PROFILES), default='BAG')
    ap.add_argument('--variant', default='G0', help='G0/S1..S6（累积）；默认 G0')
    ap.add_argument('--assume-cx', type=float, default=664.0, help='CONFIRMING 帧的占位目标中心（历史未记录 cx）')
    a = ap.parse_args()
    from run_follow_trial import VARIANTS, MECH_ARGS
    mech = {}
    for m in VARIANTS.get(a.variant, []):
        mech.update(MECH_ARGS[m])
    out, pol = replay(a.log, a.rng, a.profile, mech, assume_cx=a.assume_cx)
    m = metrics(out)
    m['log'] = pathlib.Path(a.log).name
    m['assumed_rng_mm'] = a.rng
    m['variant'] = a.variant
    m['policy_stats'] = pol.stats
    print(json.dumps(m, ensure_ascii=False, indent=2))
    return 0
if __name__ == '__main__':
    sys.exit(main())
