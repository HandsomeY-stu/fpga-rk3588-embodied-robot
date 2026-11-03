#!/usr/bin/env python3
"""统计录制日志的停车时长和原因；不连接设备。"""
import csv
import json
import math
import pathlib
import sys
import collections
TARGET_CLS = {27, 31, 33}
MIN_CONF = 0.3
MIN_RED = 0.18
IOU_MATCH = 0.12
WIDTH_RATIO = (0.45, 2.2)
PROTECT_STATES = ('RANGE_STOP', 'INVALID_RANGE', 'STALE_VISION', 'STALE_TELEMETRY', 'WRONG_MODE', 'SEARCH_DISABLED')

def iou(a, b):
    if not a or not b:
        return None
    x = max(0, min(a['cx'] + a['w'] / 2, b['cx'] + b['w'] / 2) - max(a['cx'] - a['w'] / 2, b['cx'] - b['w'] / 2))
    y = max(0, min(a['cy'] + a['h'] / 2, b['cy'] + b['h'] / 2) - max(a['cy'] - a['h'] / 2, b['cy'] - b['h'] / 2))
    union = a['w'] * a['h'] + b['w'] * b['h'] - x * y
    return x * y / union if union > 0 else 0.0

def load(p):
    if not p.exists():
        return []
    out = []
    for line in p.read_text(encoding='utf-8').splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            out.append(json.loads(line))
        except Exception:
            pass
    return out

def pick_candidate(dets):
    """返回 (选中候选, 拒绝原因, 合格候选数)。

    选择规则尽量贴近 `RedBottleLock.update`：cls 命中 + conf + red_ratio 全过才算合格；
    多个合格时按 (red_ratio, conf, 面积) 降序取第一个（首次获取规则）。
    """
    if not dets:
        return (None, '未检测', 0)
    cls_ok = [d for d in dets if d.get('cls') in TARGET_CLS]
    if not cls_ok:
        return (None, '类别不符', 0)
    conf_ok = [d for d in cls_ok if float(d.get('conf') or 0) >= MIN_CONF]
    if not conf_ok:
        return (None, '低置信度', 0)
    red_ok = [d for d in conf_ok if float(d.get('red_ratio') or 0) >= MIN_RED]
    if not red_ok:
        return (None, '红色过滤', 0)
    red_ok.sort(key=lambda d: (round(float(d.get('red_ratio') or 0), 2), float(d.get('conf') or 0), float(d.get('w') or 0) * float(d.get('h') or 0)), reverse=True)
    reason = '多目标歧义' if len(red_ok) > 1 else ''
    return (red_ok[0], reason, len(red_ok))

def analyze(run_dir):
    run_id = run_dir.name
    ctrl = load(run_dir / 'control.jsonl')
    vis = load(run_dir / 'vision.jsonl')
    tel = load(run_dir / 'telemetry.jsonl')
    if not ctrl:
        return (None, None)
    vmap = {}
    for v in vis:
        fid = v.get('frame_id')
        if fid is not None:
            vmap[fid] = v
    tmap = {t['t_ms']: t for t in tel}
    events = []
    cur = None
    prev_cand = None
    for i, c in enumerate(ctrl):
        is_stop = c.get('dir') == 0 or float(c.get('spd') or 0) <= 0
        v = vmap.get(c.get('frame_id'), {})
        cand, reason, n_ok = pick_candidate(v.get('dets') or [])
        rec = dict(i=i, c=c, v=v, cand=cand, reason=reason, n_ok=n_ok, prev=prev_cand)
        if is_stop:
            if cur is None:
                cur = dict(start=i, recs=[rec])
            else:
                cur['recs'].append(rec)
        elif cur is not None:
            events.append(cur)
            cur = None
        if cand is not None:
            prev_cand = cand
    if cur is not None:
        events.append(cur)
    rows = []
    for k, ev in enumerate(events):
        recs = ev['recs']
        c0, c1 = (recs[0]['c'], recs[-1]['c'])
        r0 = recs[0]
        v = r0['v']
        t = tmap.get(c0['t_ms'], {})
        cand, prev = (r0['cand'], r0['prev'])
        i_ = iou(cand, prev) if cand and prev else None
        wr = cand['w'] / prev['w'] if cand and prev and prev.get('w') else None
        state = c0.get('state', '')
        if state in ('RANGE_STOP', 'INVALID_RANGE'):
            cause = state
            pending = '否（距离保护，正常）'
        elif state == 'SEARCH_DISABLED':
            cause = '保护拦截：红块请求转向被禁'
            pending = '否'
        elif state == 'LOST':
            cause = '无合格目标（%s）' % r0['reason']
            pending = '否'
        elif state == 'RED_CENTERED':
            cause = '红块居中但无语义确认（%s）' % r0['reason']
            pending = '否'
        elif state == 'GRACE':
            cause = '短丢失保持期但速度为 0'
            pending = '待确认'
        elif state in ('STALE_VISION', 'STALE_TELEMETRY', 'WRONG_MODE'):
            cause = state
            pending = '否'
        else:
            cause = state or '未知'
            pending = '待确认'
        if cand and prev and (i_ is not None) and (i_ < IOU_MATCH):
            cause += '｜关联失败(IoU=%.2f)' % i_
        if wr is not None and (not WIDTH_RATIO[0] <= wr <= WIDTH_RATIO[1]):
            cause += '｜宽比越界(%.2f)' % wr
        rows.append(dict(run_id=run_id, event_idx=k, t_start_ms=round(c0['t_ms'] - ctrl[0]['t_ms'], 1), t_end_ms=round(c1['t_ms'] - ctrl[0]['t_ms'], 1), dur_ms=round(c1['t_ms'] - c0['t_ms'], 1), samples=len(recs), state=state, lock=c0.get('lock', ''), det_count=len(v.get('dets') or []), eligible_count=r0['n_ok'], reject_reason=r0['reason'], sel_cls=(cand or {}).get('cls', ''), sel_conf=round(float((cand or {}).get('conf') or 0), 3), sel_red=round(float((cand or {}).get('red_ratio') or 0), 3), sel_cx=round(float((cand or {}).get('cx') or 0), 1), prev_cx=round(float((prev or {}).get('cx') or 0), 1) if prev else '', match_iou=round(i_, 3) if i_ is not None else '', width_ratio=round(wr, 3) if wr is not None else '', vision_age=v.get('age_ms', ''), rng=c0.get('rng', ''), tlm_age=t.get('tlm_age_ms', ''), req_dir='未记录', req_spd='未记录', final_dir=c0.get('dir', ''), final_spd=c0.get('spd', ''), velL=c0.get('velL', ''), velR=c0.get('velR', ''), guard_reason=cause, pending=pending))
    n = len(ctrl)
    stop_n = sum((1 for c in ctrl if c.get('dir') == 0 or float(c.get('spd') or 0) <= 0))
    reasons = collections.Counter((r['guard_reason'].split('｜')[0] for r in rows))
    dur_by_reason = collections.defaultdict(float)
    for r in rows:
        dur_by_reason[r['guard_reason'].split('｜')[0]] += r['dur_ms']
    summary = dict(run_id=run_id, control_samples=n, stop_samples=stop_n, stop_events=len(rows), stop_time_ms=round(sum((r['dur_ms'] for r in rows)), 1), stop_pct=round(100.0 * stop_n / n, 2) if n else 0, events_by_cause=dict(reasons.most_common()), time_ms_by_cause={k: round(v, 1) for k, v in sorted(dur_by_reason.items(), key=lambda x: -x[1])}, pending_events=sum((1 for r in rows if r['pending'] != '否')))
    return (rows, summary)

def main():
    base = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else pathlib.Path(__file__).resolve().parent.parent.parent / 'evidence' / 'history'
    runs = [d for d in sorted(base.iterdir()) if d.is_dir() and (d / 'control.jsonl').exists()]
    if not runs:
        print('没找到含 control.jsonl 的实验目录:', base)
        return 2
    all_rows, summaries = ([], [])
    fields = None
    for d in runs:
        rows, summ = analyze(d)
        if not rows:
            continue
        fields = fields or list(rows[0].keys())
        all_rows.extend(rows)
        summaries.append(summ)
        with (d / 'stop_events.csv').open('w', newline='', encoding='utf-8') as f:
            w = csv.DictWriter(f, fieldnames=fields)
            w.writeheader()
            for r in rows:
                w.writerow(r)
        (d / 'stop_summary.json').write_text(json.dumps(summ, ensure_ascii=False, indent=2), encoding='utf-8')
        print('%-18s 控制 %3d 拍  停车 %3d 拍(%5.1f%%)  %2d 个事件  待确认 %d' % (summ['run_id'], summ['control_samples'], summ['stop_samples'], summ['stop_pct'], summ['stop_events'], summ['pending_events']))
        for k, v in list(summ['events_by_cause'].items())[:5]:
            print('      %-40s %2d 次  %.0f ms' % (k, v, summ['time_ms_by_cause'].get(k, 0)))
    out = base / 'stop_events_all.csv'
    if all_rows:
        with out.open('w', newline='', encoding='utf-8') as f:
            w = csv.DictWriter(f, fieldnames=fields)
            w.writeheader()
            for r in all_rows:
                w.writerow(r)
    (base / 'stop_summary_all.json').write_text(json.dumps(summaries, ensure_ascii=False, indent=2), encoding='utf-8')
    print('\n合计 %d 个事件 → %s' % (len(all_rows), out))
    return 0
if __name__ == '__main__':
    sys.exit(main())
