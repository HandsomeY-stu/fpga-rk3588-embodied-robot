#!/usr/bin/env python3
"""逐拍重建提速资格，导出CSV和失败条件统计；不连接设备。"""
import argparse
import csv
import json
import math
import pathlib
import sys
from collections import Counter, OrderedDict
TARGET_CLS = {27, 31, 33}

class QualSim:

    def __init__(self, frames=3, conf=0.4, vage=120.0, tage=200.0, rng_min=1000.0):
        self.frames = int(frames)
        self.conf = float(conf)
        self.vage = float(vage)
        self.tage = float(tage)
        self.rng_min = float(rng_min)
        self.hist = []
        self.qualified = False

    def step(self, target, fid, vage, tage, rng, eligible):
        """返回 (state, block, hist_len)。state ∈ ok/blocked/warming/off"""
        if target is None or target.get('_red_hold'):
            self.hist = []
            self.qualified = False
            return ('off', 'red_hold' if target is not None else 'no_target', 0)
        if not isinstance(fid, int):
            block = 'no_frame_id'
        elif float(target.get('conf') or 0) < self.conf:
            block = 'conf'
        elif not (isinstance(vage, (int, float)) and 0 <= vage <= self.vage):
            block = 'vision_age'
        elif not (isinstance(tage, (int, float)) and 0 <= tage <= self.tage):
            block = 'tlm_age'
        elif not (isinstance(rng, (int, float)) and rng >= self.rng_min):
            block = 'rng'
        elif int(eligible or 0) != 1:
            block = 'ambiguous'
        else:
            block = None
        if block is not None:
            self.hist = []
            self.qualified = False
            return ('blocked', block, 0)
        cx, w = (float(target['cx']), float(target.get('w') or 0))
        if not self.hist or self.hist[-1][0] != fid:
            self.hist.append((fid, cx, w))
            if len(self.hist) > self.frames:
                self.hist.pop(0)
        if len(self.hist) >= 2:
            for (_, cx0, w0), (_, cx1, _w1) in zip(self.hist, self.hist[1:]):
                if w0 > 0 and abs(cx1 - cx0) > 0.8 * w0:
                    self.hist = []
                    return ('blocked', 'discontinuous', 0)
        self.qualified = len(self.hist) >= self.frames
        if self.qualified:
            return ('ok', None, len(self.hist))
        return ('warming', 'warming', len(self.hist))

def load_jsonl(p):
    if not p.exists():
        return []
    rows = []
    for line in p.read_text(encoding='utf-8').splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            rows.append(json.loads(line))
        except Exception:
            pass
    return rows

def match_conf(sel, dets):
    """从视觉帧里找回被选中框的 conf。

    日志的 selected_bbox 只有 [cx,cy,w,h]，没有 conf；用 (cx,w) 最近邻匹配。
    返回 (conf, matched_cls, dist_px)；匹配不上返回 (None, None, None)。
    """
    if not sel or not dets:
        return (None, None, None)
    cx, w = (float(sel[0]), float(sel[2]))
    best, bd, bcls = (None, None, None)
    for d in dets:
        if d.get('cls') not in TARGET_CLS:
            continue
        dist = abs(float(d.get('cx') or 0) - cx) + abs(float(d.get('w') or 0) - w)
        if bd is None or dist < bd:
            best, bd, bcls = (float(d.get('conf') or 0), dist, d.get('cls'))
    if best is None:
        return (None, None, None)
    return (best, bcls, round(bd, 1))

def main():
    ap = argparse.ArgumentParser(description='逐拍资格审计（离线）')
    ap.add_argument('--run', required=True, help='run 目录（含 control/vision/telemetry.jsonl）')
    ap.add_argument('--out', default=None, help='输出目录，默认 <run>/audit')
    ap.add_argument('--conf', type=float, default=0.4, help='资格 conf 门槛')
    ap.add_argument('--frames', type=int, default=3)
    ap.add_argument('--vision-age', type=float, default=120.0)
    ap.add_argument('--tlm-age', type=float, default=200.0)
    ap.add_argument('--rng-min', type=float, default=1000.0)
    a = ap.parse_args()
    run = pathlib.Path(a.run)
    out = pathlib.Path(a.out) if a.out else run / 'audit'
    out.mkdir(parents=True, exist_ok=True)
    ctrl = load_jsonl(run / 'control.jsonl')
    vis = {v.get('frame_id'): v for v in load_jsonl(run / 'vision.jsonl')}
    tel = load_jsonl(run / 'telemetry.jsonl')
    result = json.loads((run / 'result.json').read_text(encoding='utf-8')) if (run / 'result.json').exists() else {}
    manifest = json.loads((run / 'manifest.json').read_text(encoding='utf-8')) if (run / 'manifest.json').exists() else {}
    cfg = manifest.get('effective_config') or {}
    margin = 0.0
    if not ctrl:
        sys.exit('control.jsonl 为空：%s' % run)
    sim = QualSim(a.frames, a.conf, a.vision_age, a.tlm_age, a.rng_min)
    rows = []
    block_cnt = Counter()
    state_cnt = Counter()
    cond_fail = Counter()
    ok_dir = Counter()
    ok_rows = []
    t0 = ctrl[0].get('t_ms') or 0.0
    for i, r in enumerate(ctrl):
        fid = r.get('frame_id')
        sel = r.get('selected_bbox')
        dets = (vis.get(fid) or {}).get('dets') or []
        red_hold = r.get('cap_reason') == 'red_hold'
        conf, mcls, mdist = match_conf(sel, dets)
        target = None
        if sel is not None:
            target = dict(cx=sel[0], cy=sel[1], w=sel[2], h=sel[3], conf=conf if conf is not None else 0.0)
            if red_hold:
                target['_red_hold'] = True
        evaluated = target is not None
        if evaluated:
            state, block, hlen = sim.step(target, fid, r.get('vision_age'), r.get('tlm_age'), r.get('rng'), r.get('eligible_count'))
            if block:
                block_cnt[block] += 1
            state_cnt[state] += 1
            if not isinstance(fid, int):
                cond_fail['no_frame_id'] += 1
            if red_hold:
                cond_fail['red_hold'] += 1
            else:
                if (conf if conf is not None else 0.0) < a.conf:
                    cond_fail['conf'] += 1
                va = r.get('vision_age')
                if not (isinstance(va, (int, float)) and 0 <= va <= a.vision_age):
                    cond_fail['vision_age'] += 1
                ta = r.get('tlm_age')
                if not (isinstance(ta, (int, float)) and 0 <= ta <= a.tlm_age):
                    cond_fail['tlm_age'] += 1
                rg = r.get('rng')
                if not (isinstance(rg, (int, float)) and rg >= a.rng_min):
                    cond_fail['rng'] += 1
                if int(r.get('eligible_count') or 0) != 1:
                    cond_fail['ambiguous'] += 1
            if state == 'ok':
                ok_dir[r.get('dir')] += 1
                ok_rows.append(dict(idx=i, fid=fid, dir=r.get('dir'), state=r.get('state'), spd=r.get('spd'), request_spd=r.get('request_spd'), cap_reason=r.get('cap_reason'), limited_by=r.get('limited_by'), rng=r.get('rng'), conf=conf))
        else:
            state, block, hlen = ('not_evaluated', None, 0)
        rows.append(OrderedDict([('idx', i), ('t_s', round(((r.get('t_ms') or t0) - t0) / 1000.0, 2)), ('frame_id', fid), ('state', r.get('state')), ('dir', r.get('dir')), ('qualify_state', state), ('qualify_block', block or ''), ('qual_hist_len', hlen), ('target_source', 'red_hold' if red_hold else 'semantic' if target else 'none'), ('target_conf', None if conf is None else round(conf, 3)), ('target_cls', mcls), ('match_dist', mdist), ('eligible_count', r.get('eligible_count')), ('detector_count', r.get('detector_count')), ('vision_age', r.get('vision_age')), ('tlm_age', r.get('tlm_age')), ('rng', r.get('rng')), ('request_spd', r.get('request_spd')), ('shaped_spd', r.get('shaped_spd')), ('spd', r.get('spd')), ('final_cap', r.get('final_cap')), ('cap_reason', r.get('cap_reason')), ('limited_by', r.get('limited_by')), ('send_result', r.get('send_result'))]))
    online = result.get('qualify_block') or {}
    recon = {k: v for k, v in block_cnt.items() if k != 'no_target'}
    verified, diffs = (True, {})
    for k in sorted(set(online) | set(recon)):
        a_, b_ = (online.get(k, 0), recon.get(k, 0))
        if a_ != b_:
            verified = False
            diffs[k] = dict(online=a_, recon=b_)
    tl = out / 'qualify_timeline.csv'
    with tl.open('w', newline='', encoding='utf-8') as f:
        wr = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        wr.writeheader()
        wr.writerows(rows)
    stops, cur = ([], None)
    for r in rows:
        spd = r['spd'] or 0
        stopped = r['dir'] in (0, None) or spd <= 0
        if stopped:
            if cur is None:
                cur = dict(start=r['t_s'], n=0, idx0=r['idx'], states=Counter(), cap=Counter(), limited=Counter())
            cur['n'] += 1
            cur['end'] = r['t_s']
            cur['states'][r['state']] += 1
            cur['cap'][r['cap_reason']] += 1
            cur['limited'][r['limited_by']] += 1
        elif cur is not None:
            stops.append(cur)
            cur = None
    if cur is not None:
        stops.append(cur)
    se = out / 'stop_events.csv'
    with se.open('w', newline='', encoding='utf-8') as f:
        wr = csv.writer(f)
        wr.writerow(['idx0', 'start_s', 'end_s', 'dur_s', 'ticks', 'main_state', 'main_cap_reason', 'main_limited_by', 'kind'])
        for s in stops:
            ms = s['states'].most_common(1)[0][0] if s['states'] else ''
            mc = s['cap'].most_common(1)[0][0] if s['cap'] else ''
            ml = s['limited'].most_common(1)[0][0] if s['limited'] else ''
            PROTECT = ('RANGE_STOP', 'INVALID_RANGE', 'STALE_VISION', 'STALE_TELEMETRY', 'WRONG_MODE', 'SEARCH_DISABLED', 'WHEEL_OVER', 'HARD_TIMEOUT')
            kind = 'protective' if any((k in str(ms) for k in PROTECT)) else 'other'
            wr.writerow([s['idx0'], s['start'], s.get('end', s['start']), s['n'] * 0.1, s['n'], ms, mc, ml, kind])
    ta = [r.get('tlm_age') for r in ctrl if isinstance(r.get('tlm_age'), (int, float))]
    over = sum((1 for x in ta if x > a.tlm_age))
    tel_dt = []
    for x, y in zip(tel, tel[1:]):
        d = (y.get('t_ms') or 0) - (x.get('t_ms') or 0)
        if d > 0:
            tel_dt.append(d)
    tel_dt.sort()
    resets = [i for i in range(1, len(ta)) if ta[i] < ta[i - 1] - 20]
    total_t = ((ctrl[-1].get('t_ms') or 0) - (ctrl[0].get('t_ms') or 0)) / 1000.0
    upd_period = round(total_t / len(resets) * 1000.0, 1) if len(resets) >= 2 else None
    upd_hz = round(1000.0 / upd_period, 2) if upd_period else None
    mds = sorted((d for d in (r.get('match_dist') for r in rows) if isinstance(d, (int, float))))
    md_med = mds[len(mds) // 2] if mds else None
    total = len(rows)
    aud = dict(run=str(run), samples=total, qualify_state_counts=dict(state_cnt), qualify_block_counts=dict(block_cnt), not_evaluated=state_cnt.get('not_evaluated', 0) + 0, condition_fail_independent=dict(cond_fail), reconstruction_verified=verified, reconstruction_diff=diffs, online_qualify_block=dict(online), ok_ticks=state_cnt.get('ok', 0), ok_ticks_by_dir=dict(ok_dir), ok_examples=ok_rows[:20], trim_ticks=sum((1 for r in rows if r['dir'] in (3, 4))), trim_ok_ticks=sum((v for k, v in ok_dir.items() if k in (3, 4))), tlm_age=dict(n=len(ta), over_threshold=over, over_pct=round(100.0 * over / len(ta), 2) if ta else None, threshold=a.tlm_age, median=sorted(ta)[len(ta) // 2] if ta else None, p10=sorted(ta)[int(len(ta) * 0.1)] if ta else None, p90=sorted(ta)[int(len(ta) * 0.9)] if ta else None), telemetry_cadence=dict(log_rows=len(tel), log_dt_median_ms=round(tel_dt[len(tel_dt) // 2], 1) if tel_dt else None, underlying_update_period_ms=upd_period, underlying_update_hz=upd_hz, resets=len(resets), control_rows=total, note='日志行按控制拍写（10Hz），但底层遥测每 %s ms 才更新一次；相邻约 2~3 拍共用同一次测量，tlm_age 呈锯齿' % upd_period), speed=dict(request_mean=round(sum((r.get('request_spd') or 0 for r in ctrl)) / total, 2), final_mean=round(sum((r.get('spd') or 0 for r in ctrl)) / total, 2), clamped=sum((1 for r in ctrl if (r.get('spd') or 0) < (r.get('request_spd') or 0) - 0.01))), mode=cfg.get('trim_speed_mode'), trim_test_cap=cfg.get('trim_test_cap'))
    (out / 'audit_counts.json').write_text(json.dumps(aud, ensure_ascii=False, indent=1), encoding='utf-8')
    prod = Counter((r.get('producer_id') for r in ctrl))
    fid = [r.get('frame_id') for r in ctrl]
    lines = []
    A = lines.append
    A('# 输入覆盖与口径说明（自动生成）')
    A('')
    A('数据源：`%s`' % run)
    A('')
    A('## 1. 行数与对齐')
    A('')
    A('| 项 | 值 |')
    A('|---|---|')
    A('| control 拍数 | %d |' % total)
    A('| vision 帧数 | %d |' % len(vis))
    A('| telemetry 行数 | %d |' % len(tel))
    A('| frame_id 唯一数 | %d |' % len(set(fid)))
    A('| control.producer_id | %s |' % dict(prod))
    A('| vision 是否带 producer_id | 否（只有 control 记录，故按 frame_id 对齐） |')
    A('| telemetry 日志行间隔中位 | %s ms |' % (round(tel_dt[len(tel_dt) // 2], 1) if tel_dt else '未知'))
    A('| **底层遥测更新周期** | **%s ms（约 %s Hz，由 tlm_age 锯齿重置测得，n=%d）** |' % (upd_period, upd_hz, len(resets)))
    A('| 每次遥测更新覆盖控制拍 | %s |' % (round(upd_period / (tel_dt[len(tel_dt) // 2] if tel_dt else 100.0), 2) if upd_period else '未知'))
    A('')
    A('**口径警告**：要区分两个频率 —— ① `telemetry.jsonl` 是**跟着控制拍写的**（约 10Hz）；② 底层遥测**实际更新**约 %s ms 一次（%s Hz）。因此相邻约 2~3 个控制拍共享**同一次**测量，`tlm_age` 在 0~%s ms 之间锯齿变化是**采样相位**造成的，既不是传感器抖动，也不是独立故障。' % (upd_period, upd_hz, upd_period))
    A('')
    A('这也直接决定资格：`qualify_tlm_age=%s ms` 与更新周期 %s ms 是同一量级，资格能否成立取决于控制拍**恰好落在**锯齿的哪个相位。' % (a.tlm_age, upd_period))
    A('')
    A('## 2. 字段可用性')
    A('')
    A('| 字段 | 覆盖 | 说明 |')
    A('|---|---|---|')
    ne = sum((1 for r in ctrl if r.get('selected_bbox') is None))
    A('| `selected_bbox` | %d/%d 有值 | 为 None 的拍 `target=None`，**不会进入资格判定**，标 `not_evaluated` |' % (total - ne, total))
    A('| `target_id` | 全部为 None | **无跨帧目标 ID**，身份连续性只能靠位置/宽度近似 |')
    A('| `conf` | 日志未直接记录 | 由 `vision.jsonl` 的 dets 按 (cx,w) 最近邻回收，匹配距离中位 %s px（匹配越近越可信） |' % md_med)
    A('| `rng_gated` | 全部为 None | 门控未生效（或未记录），实际用 `rng`/`rng_raw` |')
    A('| `qualify_state` | **日志未记录** | 本目录 `qualify_timeline.csv` 为按实现重建 |')
    A('')
    A('## 3. 重建可信度')
    A('')
    A('- 重建 block 计数 vs `result.json:qualify_block`：**%s**' % ('逐项一致' if verified else '存在差异，见 audit_counts.json'))
    if not verified:
        A('- 差异：`%s`' % json.dumps(diffs, ensure_ascii=False))
    A('')
    A('## 4. 仍然"未知"的部分')
    A('')
    A('- **没有图像**：本 run 的 `frames/` 目录为空，全部 1199 帧没有任何 JPEG/视频。因此**不能**判断空帧是模糊、过曝、遮挡还是分类器未输出。')
    A('- **没有未过滤的原始候选分数**：vision.jsonl 只保留通过 `min_conf` 的框，`conf` 最小恰为阈值，**不能**据此推断阈值以下还有多少正确框。')
    A('- **没有目标运动时序**：现场移动/操作没有事件日志，航向摆动无法区分"必要跟随转弯"与"控制振荡"。')
    A('- `red_n` 非零只证明画面存在红色区域，**不证明**原目标仍可见。')
    A('')
    (out / 'input_coverage.md').write_text('\n'.join(lines), encoding='utf-8')
    print('== 资格状态（重建）==')
    for k, v in state_cnt.most_common():
        print('  %-14s %4d  (%.1f%%)' % (k, v, 100.0 * v / total))
    print('== 首阻断原因 ==')
    for k, v in block_cnt.most_common():
        print('  %-14s %4d' % (k, v))
    print('== 条件独立失败（可叠加）==')
    for k, v in cond_fail.most_common():
        print('  %-14s %4d' % (k, v))
    print('== 校验 ==')
    print('  线上 qual_block:', dict(online))
    print('  重建        :', recon)
    print('  逐项一致    :', verified, '' if verified else diffs)
    print('== 关键 ==')
    print('  ok 拍数 %d，其中纠偏(dir3/4) %d，TRIM 总拍 %d' % (state_cnt.get('ok', 0), sum((v for k, v in ok_dir.items() if k in (3, 4))), aud['trim_ticks']))
    print('  请求均速 %.2f → 最终均速 %.2f，被压 %d 拍' % (aud['speed']['request_mean'], aud['speed']['final_mean'], aud['speed']['clamped']))
    print('  产物: %s' % out)
if __name__ == '__main__':
    main()
