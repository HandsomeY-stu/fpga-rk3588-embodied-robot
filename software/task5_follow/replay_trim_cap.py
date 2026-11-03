#!/usr/bin/env python3
"""离线扫描速度上限和资格参数；收益判读优先用严格重放自证。"""
import argparse
import collections
import json
import pathlib
import sys
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from follow_policy import FollowPolicy, FWD_FAMILY, DIR_LEFT, DIR_RIGHT
PROFILES = {'BAG': (27, 31, 33), 'BOTTLE': (44,)}
PROTECT = ('RANGE_STOP', 'INVALID_RANGE', 'STALE_VISION', 'STALE_TELEMETRY', 'WRONG_MODE', 'SEARCH_DISABLED')

def load(p):
    if not p.exists():
        return []
    return [json.loads(x) for x in p.read_text(encoding='utf-8').splitlines() if x.strip()]

def to_packet(v):
    """把历史 vision.jsonl 一行还原成策略吃的视觉包。"""
    return dict(color_schema=1, age_ms=v.get('age_ms', 0), producer_id=v.get('producer_id', 1), frame_id=v.get('frame_id'), detections=v.get('dets') or v.get('detections') or [], red_cx=v.get('red_cx'), red_cy=v.get('red_cy'), red_n=v.get('red_n'), red_w=v.get('red_w'), red_h=v.get('red_h'))

def to_status(tl):
    return dict(tlm_ok=True, tlm_age_ms=tl.get('tlm_age_ms', 0), fpga_mode=0, rng=tl.get('rng'), yaw=tl.get('yaw'), velL=tl.get('velL', 0), velR=tl.get('velR', 0))

def nearest(tel, t, idx):
    """遥测 4Hz、控制 10Hz：取 t 之前最近的一条（保持因果）。"""
    while idx + 1 < len(tel) and tel[idx + 1]['t_ms'] <= t:
        idx += 1
    return (tel[idx], idx)

def replay(vis, tel, profile, cap, mode, q=None):
    q = q or {}
    pol = FollowPolicy(target_cls=PROFILES[profile], min_conf=0.3, red_min=0.18, range_stop_mm=300, close_h=0.96, d_far=1000, d_near=400, fwd_far=22.0, fwd_near=10.0, max_spd=24.0, trim_cap=12.0, trim_test_cap=float(cap), trim_speed_mode=mode, red_hold_cap=12.0, coast_cap=12.0, accel_pct_s=20.0, decel_pct_s=40.0, semantic_grace_ms=200, turn_enter_px=130, turn_exit_px=70, motion_search_allowed=False, qualify_conf=q.get('conf', 0.4), qualify_tlm_age=q.get('tlm_age', 200), qualify_rng_min=q.get('rng', 1000), qualify_frames=q.get('frames', 3), qualify_vision_age=q.get('vision_age', 120))
    idx = 0
    rows = []
    for v in vis:
        t = v['t_ms']
        if tel:
            tl, idx = nearest(tel, t, idx)
        else:
            tl = dict(t_ms=t, rng=None, tlm_age_ms=0, yaw=None, velL=0, velR=0)
        cmd = pol.step(to_packet(v), to_status(tl), t)
        d = pol.debug
        rows.append(dict(t=t, dir=cmd.dir, spd=cmd.spd, state=cmd.state, final_cap=d.get('final_cap'), cap_reason=d.get('cap_reason'), limited_by=d.get('limited_by'), request_spd=d.get('request_spd'), raw_err=d.get('raw_err'), rng=tl.get('rng'), velL=tl.get('velL'), velR=tl.get('velR')))
    return (pol, rows)

def summarize(rows, cap, pol=None):
    trim = [r for r in rows if r['dir'] in (DIR_LEFT, DIR_RIGHT)]
    prot = [r for r in rows if any((k in str(r['state']) for k in PROTECT))]
    valid = [r for r in rows if r not in prot]
    hit_cap = [r for r in trim if r['spd'] >= float(cap) - 0.01]
    qual = [r for r in trim if str(r['cap_reason'] or '').startswith('trim_qualified')]
    lb = collections.Counter((r['limited_by'] for r in trim))
    qlb = collections.Counter((r['limited_by'] for r in qual))
    qspd = [r['spd'] for r in qual]
    reasons = collections.Counter((r['cap_reason'] for r in trim))
    mean = sum((r['spd'] for r in trim)) / len(trim) if trim else 0.0
    vels = [(abs(float(r['velL'] or 0)) + abs(float(r['velR'] or 0))) / 2.0 for r in trim if r['velL'] is not None]
    return dict(cap=cap, samples=len(rows), trim_ticks=len(trim), trim_pct=round(100.0 * len(trim) / max(1, len(valid)), 2), trim_spd_mean=round(mean, 2), trim_spd_max=max([r['spd'] for r in trim], default=0.0), at_old_cap_12=len([r for r in trim if r['spd'] >= 11.99]), at_this_cap=len(hit_cap), at_this_cap_pct=round(100.0 * len(hit_cap) / max(1, len(trim)), 2), qualified_ticks=len(qual), qualified_pct=round(100.0 * len(qual) / max(1, len(trim)), 2), limited_by=dict(lb), cap_reason=dict(reasons), qualify_block=dict(pol.qual_block if pol is not None else {}), qualified_limited_by=dict(qlb), qualified_spd_mean=round(sum(qspd) / len(qspd), 2) if qspd else None, qualified_spd_max=max(qspd) if qspd else None, wheel_mean_mmps=round(sum(vels) / len(vels), 1) if vels else None)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('path')
    ap.add_argument('--profile', default='BAG', choices=tuple(PROFILES))
    ap.add_argument('--caps', default='12,14,16,18')
    ap.add_argument('--out', default=None)
    ap.add_argument('--q-conf', type=float, default=0.4)
    ap.add_argument('--q-tlm-age', type=float, default=200)
    ap.add_argument('--q-rng', type=float, default=1000)
    ap.add_argument('--q-frames', type=int, default=3)
    ap.add_argument('--sweep', action='store_true', help='对关键门槛做敏感性扫描')
    a = ap.parse_args()
    root = pathlib.Path(a.path)
    runs = [root] if (root / 'vision.jsonl').exists() else sorted((p for p in root.iterdir() if (p / 'vision.jsonl').exists()))
    caps = [float(x) for x in a.caps.split(',') if x.strip()]
    report = []
    for run in runs:
        vis, tel = (load(run / 'vision.jsonl'), load(run / 'telemetry.jsonl'))
        if not vis:
            print('%s: 无 vision.jsonl，跳过' % run.name)
            continue
        print('\n=== %s  (vision %d 条 / telemetry %d 条)' % (run.name, len(vis), len(tel)))
        line = dict(run=run.name, vision=len(vis), telemetry=len(tel), variants=[])
        for cap in caps:
            mode = 'off' if abs(cap - 12.0) < 1e-09 else 'qualified-fixed'
            pol, rows = replay(vis, tel, a.profile, cap, mode, dict(conf=a.q_conf, tlm_age=a.q_tlm_age, rng=a.q_rng, frames=a.q_frames))
            s = summarize(rows, cap, pol)
            s['mode'] = mode
            line['variants'].append(s)
            print('  cap=%-4g mode=%-15s TRIM %4d 拍(%5.1f%%) 均速 %5.2f  到本档 %4d 拍(%5.1f%%)  qualified %4d 拍(%5.1f%%)  实测轮均 %s mm/s' % (cap, mode, s['trim_ticks'], s['trim_pct'], s['trim_spd_mean'], s['at_this_cap'], s['at_this_cap_pct'], s['qualified_ticks'], s['qualified_pct'], s['wheel_mean_mmps']))
            if cap == caps[0]:
                print('        限速原因: %s' % s['cap_reason'])
                print('        limited_by: %s' % s['limited_by'])
                print('        资格拦截: %s' % s['qualify_block'])
        report.append(line)
    if a.sweep:
        print('\n=== 资格门槛敏感性（cap=18，仅看 qualified 占比）===')
        for run in runs:
            vis, tel = (load(run / 'vision.jsonl'), load(run / 'telemetry.jsonl'))
            if not vis:
                continue
            print('  %s' % run.name)
            for label, q in (('基准 tlm200/rng1000/conf.40', {}), ('tlm300', dict(tlm_age=300)), ('tlm300+rng800', dict(tlm_age=300, rng=800)), ('tlm300+conf.35', dict(tlm_age=300, conf=0.35)), ('tlm300+rng800+conf.35', dict(tlm_age=300, rng=800, conf=0.35)), ('以上+frames2', dict(tlm_age=300, rng=800, conf=0.35, frames=2))):
                pol, rows = replay(vis, tel, a.profile, 18.0, 'qualified-fixed', q)
                s = summarize(rows, 18.0, pol)
                print('     %-28s qualified %4d 拍 (%5.1f%%)  均速 %5.2f' % (label, s['qualified_ticks'], s['qualified_pct'], s['trim_spd_mean']))
    if report:
        print('\n=== 汇总（同一输入，仅策略输出）===')
        for line in report:
            base = line['variants'][0]
            print('%-16s B0 均速 %.2f' % (line['run'], base['trim_spd_mean']), end='')
            for v in line['variants'][1:]:
                d = v['trim_spd_mean'] - base['trim_spd_mean']
                print('  |%g: %.2f (%+.0f%%)' % (v['cap'], v['trim_spd_mean'], 100.0 * d / max(1e-09, base['trim_spd_mean'])), end='')
            share = 100.0 * base['at_old_cap_12'] / max(1, base['trim_ticks'])
            verdict = '可扫档' if share >= 10.0 else '先解释其他限速原因'
            print('  → 旧cap命中率 %.1f%%（%s）' % (share, verdict))
        if a.out:
            pathlib.Path(a.out).write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
            print('\n报告已写: %s' % a.out)
    return 0
if __name__ == '__main__':
    sys.exit(main())
