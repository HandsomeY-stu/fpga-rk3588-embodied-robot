#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""静态视觉门槛检查，统计目标命中率、帧率与数据新鲜度。"""
import argparse
import json
import math
import pathlib
import sys
import time
import urllib.request

PROFILES = {'BAG': (27, 31, 33), 'BOTTLE': (44,)}
MIN_CONF = 0.30
MIN_RED = 0.18
OSD_RECT = (32, 32, 472, 144)      # x0,y0,x1,y1（与检测器 --mask-rect 一致）

GATES = dict(center_hit=95.0, side_hit=80.0, min_fps=10.0, age_p95_ms=200.0)


def api(url, path, body=None, timeout=2.0):
    try:
        if body is None:
            with urllib.request.urlopen(url + path, timeout=timeout) as r:
                return json.load(r)
        data = json.dumps(body).encode()
        req = urllib.request.Request(url + path, data=data,
                                     headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.load(r)
    except Exception as e:      # noqa: BLE001
        return {'_error': '%s: %s' % (type(e).__name__, e)}


def overlaps_osd(d):
    try:
        x0 = float(d['cx']) - float(d['w']) / 2
        x1 = float(d['cx']) + float(d['w']) / 2
        y0 = float(d['cy']) - float(d['h']) / 2
        y1 = float(d['cy']) + float(d['h']) / 2
    except (KeyError, TypeError, ValueError):
        return False
    ox0, oy0, ox1, oy1 = OSD_RECT
    return not (x1 < ox0 or x0 > ox1 or y1 < oy0 or y0 > oy1)


def p95(vals):
    if not vals:
        return None
    s = sorted(vals)
    k = min(len(s) - 1, int(math.ceil(0.95 * len(s))) - 1)
    return s[max(0, k)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--profile', choices=tuple(PROFILES), default='BAG')
    ap.add_argument('--pos', choices=('center', 'left', 'right', 'none'), required=True)
    ap.add_argument('--seconds', type=float, default=10.0)
    ap.add_argument('--run-id', required=True)
    ap.add_argument('--url', default='http://127.0.0.1:8080')
    ap.add_argument('--outdir', default='/root/task_follow_smooth_exp_20261030/experiment')
    ap.add_argument('--keep-detector', action='store_true',
                    help='跑完不停检测器')
    a = ap.parse_args()
    cls = set(PROFILES[a.profile])
    out = pathlib.Path(a.outdir) / a.run_id
    if out.exists():
        print('run-id 已存在，拒绝覆盖: %s' % out)
        return 2
    out.mkdir(parents=True)


    v0 = api(a.url, '/api/person.json')


    st0 = api(a.url, '/api/status')
    osd_mode = st0.get('osd_mode')
    already = isinstance(v0.get('age_ms'), (int, float)) and v0.get('age_ms') < 500
    if not already:
        api(a.url, '/api/person', {'on': False})
        time.sleep(1.0)
        api(a.url, '/api/person', {'on': True})
    last = None
    t_wait = time.monotonic() + 25
    ok = False
    while time.monotonic() < t_wait:
        v = api(a.url, '/api/person.json')
        fid = v.get('frame_id')
        if fid is not None and fid != last:
            if last is not None:
                ok = True
                break
            last = fid
        time.sleep(0.3)
    if not ok:
        (out / 'result.json').write_text(json.dumps(
            {'run_id': a.run_id, 'error': 'detector not producing frames'},
            ensure_ascii=False, indent=2), encoding='utf-8')
        print('检测器没出帧，退出')
        return 3


    seen = set()
    rows = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < a.seconds:
        v = api(a.url, '/api/person.json')
        if '_error' in v:
            time.sleep(0.05)
            continue
        fid = v.get('frame_id')
        pid = v.get('producer_id')
        key = (pid, fid)
        if fid is None or key in seen:
            time.sleep(0.05)
            continue
        seen.add(key)
        hit = False
        best = None
        osd_hit = False
        for d in (v.get('detections') or []):
            if not isinstance(d, dict):
                continue
            try:
                if (d.get('cls') in cls and float(d.get('conf') or 0) >= MIN_CONF
                        and float(d.get('red_ratio') or 0) >= MIN_RED):
                    hit = True
                    if overlaps_osd(d):
                        osd_hit = True
                    if best is None or float(d.get('conf') or 0) > float(best.get('conf') or 0):
                        best = d
            except (TypeError, ValueError):
                continue
        rows.append(dict(t=round(time.monotonic() - t0, 3), frame_id=fid,
                         age_ms=v.get('age_ms'), hit=hit, osd_overlap=osd_hit,
                         red_cx=v.get('red_cx'), red_n=v.get('red_n'),
                         best=best))
        time.sleep(0.03)

    dur = time.monotonic() - t0
    n = len(rows)
    hits = sum(1 for r in rows if r['hit'])
    ages = [r['age_ms'] for r in rows if isinstance(r['age_ms'], (int, float))]
    cxs = [r['best']['cx'] for r in rows if r['best'] and r['best'].get('cx') is not None]
    res = dict(
        run_id=a.run_id, profile=a.profile, pos=a.pos, seconds=round(dur, 2),
        unique_frames=n, hit_frames=hits,
        hit_pct=round(100.0 * hits / n, 2) if n else 0.0,
        effective_fps=round(n / dur, 2) if dur > 0 else 0.0,
        age_p95_ms=p95(ages), age_max_ms=max(ages) if ages else None,
        target_cx_median=round(sorted(cxs)[len(cxs) // 2], 1) if cxs else None,
        target_cx_min=round(min(cxs), 1) if cxs else None,
        target_cx_max=round(max(cxs), 1) if cxs else None,
        osd_overlap_frames=sum(1 for r in rows if r['osd_overlap']),
        osd_mode=osd_mode,
        gates=GATES,
    )

    if a.pos == 'none':

        bad = [r for r in rows if r['hit']]
        run3 = 0
        max_run = 0
        for r in rows:
            run3 = run3 + 1 if r['hit'] else 0
            max_run = max(max_run, run3)
        res.update(false_hits=len(bad), max_consecutive_hits=max_run,
                   pass_=bool(len(bad) <= 1 and max_run < 3))
    else:
        need = GATES['center_hit'] if a.pos == 'center' else GATES['side_hit']
        res['hit_gate'] = need
        res['pass_hit'] = bool(res['hit_pct'] >= need)
        res['pass_fps'] = bool(res['effective_fps'] >= GATES['min_fps'])
        res['pass_age'] = bool(res['age_p95_ms'] is not None
                               and res['age_p95_ms'] <= GATES['age_p95_ms'])

        if osd_mode == 0:
            res['pass_no_osd'] = True
            res['osd_check'] = 'skipped (osd_mode=0, 纯净画面)'
        else:
            res['pass_no_osd'] = bool(res['osd_overlap_frames'] == 0)
            res['osd_check'] = 'enforced (osd_mode=%s)' % osd_mode
        res['pass_'] = bool(res['pass_hit'] and res['pass_fps']
                            and res['pass_age'] and res['pass_no_osd'])
    (out / 'result.json').write_text(json.dumps(res, ensure_ascii=False, indent=2),
                                     encoding='utf-8')
    (out / 'samples.jsonl').write_text(
        '\n'.join(json.dumps(r, ensure_ascii=False, default=str) for r in rows),
        encoding='utf-8')
    if not a.keep_detector:
        api(a.url, '/api/person', {'on': False})
    print(json.dumps(res, ensure_ascii=False, indent=2))
    return 0 if res.get('pass_') else 1


if __name__ == '__main__':
    sys.exit(main())
