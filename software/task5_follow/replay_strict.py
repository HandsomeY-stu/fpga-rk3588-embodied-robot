#!/usr/bin/env python3
"""按控制拍原始字段严格重放，先校验基线再比较候选；不连接设备。"""
import argparse
import csv
import json
import pathlib
import sys
from collections import Counter
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from follow_policy import FollowPolicy, DIR_LEFT, DIR_RIGHT
PROFILES = {'BAG': (27, 31, 33), 'BOTTLE': (44,)}
PROTECT = ('RANGE_STOP', 'INVALID_RANGE', 'STALE_VISION', 'STALE_TELEMETRY', 'WRONG_MODE', 'SEARCH_DISABLED')
VARIANTS = {'G0': [], 'S1': ['unified_speed'], 'S2': ['unified_speed', 'shape'], 'S3': ['unified_speed', 'shape', 'grace'], 'S5': ['unified_speed', 'shape', 'grace', 'turn_hyst'], 'S6': ['unified_speed', 'shape', 'grace', 'turn_hyst', 'dir_confirm']}
MECH_ARGS = {'unified_speed': dict(unified_speed=True), 'shape': dict(accel_pct_s=20, decel_pct_s=40), 'grace': dict(semantic_grace_ms=200), 'red_delay': dict(red_cue_after_ms=900), 'turn_hyst': dict(turn_enter_px=130, turn_exit_px=70), 'dir_confirm': dict(direction_confirm_frames=2)}

def load(p):
    if not p.exists():
        return []
    return [json.loads(x) for x in p.read_text(encoding='utf-8').splitlines() if x.strip()]

def build_policy(profile, cap, mode, q, variant='S5'):
    q = q or {}
    kw = dict(accel_pct_s=0, decel_pct_s=0)
    for m in VARIANTS[variant]:
        kw.update(MECH_ARGS[m])
    return FollowPolicy(target_cls=PROFILES[profile], min_conf=0.3, red_min=0.18, range_stop_mm=300, close_h=0.96, d_far=1000, d_near=400, fwd_far=22.0, fwd_near=10.0, max_spd=24.0, trim_cap=12.0, trim_test_cap=float(cap), trim_speed_mode=mode, red_hold_cap=12.0, coast_cap=12.0, semantic_grace_ms=kw.pop('semantic_grace_ms', 0), turn_enter_px=kw.pop('turn_enter_px', 170), turn_exit_px=kw.pop('turn_exit_px', 90), motion_search_allowed=False, qualify_conf=q.get('conf', 0.4), qualify_tlm_age=q.get('tlm_age', 200), qualify_rng_min=q.get('rng', 1000), qualify_frames=q.get('frames', 3), qualify_vision_age=q.get('vision_age', 120), qual_hist_keep_ms=q.get('keep_ms', 0), **kw)

def run_replay(ctrl, vis_by_fid, profile, cap, mode, q, variant='S5'):
    pol = build_policy(profile, cap, mode, q, variant)
    rows = []
    for r in ctrl:
        fid = r.get('frame_id')
        v = vis_by_fid.get(fid) or {}
        packet = dict(color_schema=1, age_ms=v.get('age_ms', 0), producer_id=r.get('producer_id'), frame_id=fid, detections=v.get('dets') or [], red_cx=v.get('red_cx'), red_cy=v.get('red_cy'), red_n=v.get('red_n'), red_w=v.get('red_w'), red_h=v.get('red_h'))
        status = dict(tlm_ok=True, fpga_mode=0, tlm_age_ms=r.get('tlm_age'), rng=r.get('rng'), velL=r.get('velL', 0), velR=r.get('velR', 0))
        now = r.get('mono_ms') or r.get('t_ms')
        cmd = pol.step(packet, status, now)
        d = pol.debug
        rows.append(dict(idx=len(rows), frame_id=fid, t_s=round((r.get('t_ms') or 0) / 1000.0, 2), dir=cmd.dir, spd=cmd.spd, state=cmd.state, request_spd=d.get('request_spd'), request_dir=d.get('request_dir'), final_cap=d.get('final_cap'), cap_reason=d.get('cap_reason'), limited_by=d.get('limited_by'), qualify_state=d.get('qualify_state'), qualify_block=d.get('qualify_block'), orig_dir=r.get('dir'), orig_spd=r.get('spd'), orig_state=r.get('state'), orig_cap_reason=r.get('cap_reason'), orig_limited_by=r.get('limited_by')))
    return (pol, rows)

def compare(rows):
    n = len(rows)
    dir_ok = sum((1 for r in rows if r['dir'] == r['orig_dir']))
    state_ok = sum((1 for r in rows if r['state'] == r['orig_state']))
    spd_ok = sum((1 for r in rows if abs(float(r['spd'] or 0) - float(r['orig_spd'] or 0)) < 1e-06))
    cap_ok = sum((1 for r in rows if (r['cap_reason'] or '') == (r['orig_cap_reason'] or '')))
    diffs = [r for r in rows if r['dir'] != r['orig_dir'] or abs(float(r['spd'] or 0) - float(r['orig_spd'] or 0)) >= 1e-06]
    trim_idx = [r['idx'] for r in rows if r['orig_dir'] in (3, 4)]
    return dict(n=n, dir_match_pct=round(100.0 * dir_ok / n, 2), state_match_pct=round(100.0 * state_ok / n, 2), spd_exact_pct=round(100.0 * spd_ok / n, 2), cap_reason_match_pct=round(100.0 * cap_ok / n, 2), diff_ticks=len(diffs), diff_examples=[dict(idx=r['idx'], fid=r['frame_id'], replay=(r['dir'], r['spd'], r['state']), orig=(r['orig_dir'], r['orig_spd'], r['orig_state'])) for r in diffs[:10]], trim_ticks=len(trim_idx))

def summarize(rows, cap, mode):
    trim = [r for r in rows if r['dir'] in (DIR_LEFT, DIR_RIGHT)]
    orig_trim = [r for r in rows if r['orig_dir'] in (3, 4)]
    stop = [r for r in rows if r['dir'] in (0, None) or float(r['spd'] or 0) <= 0]
    qual = [r for r in rows if str(r['cap_reason'] or '').startswith('trim_qualified')]

    def mean(xs):
        return round(sum(xs) / len(xs), 3) if xs else None
    return dict(cap=cap, mode=mode, samples=len(rows), all_spd_mean=mean([float(r['spd'] or 0) for r in rows]), all_spd_mean_orig=mean([float(r['orig_spd'] or 0) for r in rows]), trim_ticks=len(trim), trim_ticks_orig=len(orig_trim), trim_spd_mean=mean([float(r['spd'] or 0) for r in trim]), trim_spd_mean_orig=mean([float(r['orig_spd'] or 0) for r in orig_trim]), qualified_ticks=len(qual), qualified_trim_ticks=len([r for r in qual if r['dir'] in (DIR_LEFT, DIR_RIGHT)]), qualified_at_cap=len([r for r in qual if float(r['spd'] or 0) >= float(cap) - 0.01]), qualified_spd_mean=mean([float(r['spd'] or 0) for r in qual]), qualified_limited_by=dict(Counter((str(r['limited_by']) for r in qual))), cap_reason=dict(Counter((str(r['cap_reason']) for r in rows))), limited_by=dict(Counter((str(r['limited_by']) for r in rows))), stop_ticks=len(stop), spd_hist=dict(Counter(('0' if (r['spd'] or 0) == 0 else '0-5' if r['spd'] < 5 else '5-10' if r['spd'] < 10 else '10-12' if r['spd'] <= 12 else '12-15' if r['spd'] <= 15 else '15-20' if r['spd'] <= 20 else '>=20' for r in rows))))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', required=True)
    ap.add_argument('--profile', default='BAG', choices=tuple(PROFILES))
    ap.add_argument('--mode', default='off', choices=('off', 'qualified-fixed'))
    ap.add_argument('--cap', type=float, default=12.0)
    ap.add_argument('--q-conf', type=float, default=0.4)
    ap.add_argument('--q-tlm-age', type=float, default=200.0)
    ap.add_argument('--q-rng', type=float, default=1000.0)
    ap.add_argument('--q-frames', type=int, default=3)
    ap.add_argument('--q-keep-ms', type=float, default=0.0, help='视觉确认历史保留窗（0=旧行为，>300 会被策略截断）')
    ap.add_argument('--variant', default='S5', choices=tuple(VARIANTS))
    ap.add_argument('--compare-to-control', action='store_true')
    ap.add_argument('--out', default=None, help='输出 JSON')
    ap.add_argument('--csv', default=None, help='输出逐拍 CSV')
    a = ap.parse_args()
    run = pathlib.Path(a.run)
    ctrl = load(run / 'control.jsonl')
    vis = {v.get('frame_id'): v for v in load(run / 'vision.jsonl')}
    if not ctrl:
        sys.exit('control.jsonl 缺失或为空')
    pol, rows = run_replay(ctrl, vis, a.profile, a.cap, a.mode, dict(conf=a.q_conf, tlm_age=a.q_tlm_age, rng=a.q_rng, frames=a.q_frames, keep_ms=a.q_keep_ms), a.variant)
    s = summarize(rows, a.cap, a.mode)
    print('=== %s  variant=%s mode=%s cap=%g  q_tlm_age=%g  q_keep_ms=%g ===' % (run.name, a.variant, a.mode, a.cap, a.q_tlm_age, a.q_keep_ms))
    print('  全部拍均速       重放 %.3f  / 原始 %.3f' % (s['all_spd_mean'], s['all_spd_mean_orig']))
    print('  TRIM 拍数        重放 %4d  / 原始 %4d' % (s['trim_ticks'], s['trim_ticks_orig']))
    print('  TRIM 均速        重放 %.3f  / 原始 %.3f' % (s['trim_spd_mean'], s['trim_spd_mean_orig']))
    print('  qualified 拍     %d（其中 TRIM %d）' % (s['qualified_ticks'], s['qualified_trim_ticks']))
    print('  cap_reason       %s' % s['cap_reason'])
    print('  limited_by       %s' % s['limited_by'])
    out = dict(run=str(run), **s)
    if a.compare_to_control:
        cmp_ = compare(rows)
        out['compare'] = cmp_
        print('\n--- 与原始控制日志逐拍比对 ---')
        print('  dir 一致 %.2f%% | state 一致 %.2f%% | spd 完全一致 %.2f%% | cap_reason 一致 %.2f%%' % (cmp_['dir_match_pct'], cmp_['state_match_pct'], cmp_['spd_exact_pct'], cmp_['cap_reason_match_pct']))
        print('  不一致拍数 %d' % cmp_['diff_ticks'])
        for e in cmp_['diff_examples'][:5]:
            print('    idx %-4d fid %-6s 重放 %s  原始 %s' % (e['idx'], e['fid'], e['replay'], e['orig']))
    if a.csv:
        with open(a.csv, 'w', newline='', encoding='utf-8') as f:
            wr = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            wr.writeheader()
            wr.writerows(rows)
        print('  逐拍 CSV: %s' % a.csv)
    if a.out:
        pathlib.Path(a.out).write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding='utf-8')
        print('  JSON: %s' % a.out)
    return 0
if __name__ == '__main__':
    sys.exit(main())
