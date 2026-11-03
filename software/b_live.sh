#!/bin/bash
# 任务六短时实车辅助入口，负责准备检测器和调用执行器。
set -u
SECS=${1:-5}
shift || true
EXTRA="$@"
M=http://127.0.0.1:8080
OUT=/root/b_live_out
mkdir -p "$OUT"

# ★★ 关键：webctl 的 cmd_stop() 会置 tcp_inhibit=True，之后**所有 TCP 9100 命令被静默拦截**
#   （cmd_set 里 `if src=='tcp9100' and S.tcp_inhibit: return 0,0`），
#   只有 _set_red_task6(on=True) 或重启服务才会清。本脚本是"绕过 webctl 直接跑 runner"，
#   所以必须先重启 carweb，否则车一步都不动（cx 完全不变，看起来像"参数没用"）。
echo "[b] 重启 carweb（清 tcp_inhibit）…"
systemctl restart carweb
sleep 10
systemctl is-active carweb

echo "[b] 开看板（最多 3 次）…"
READY=FAIL
for try in 1 2 3; do
    curl -s --max-time 8 -X POST -H 'Content-Type: application/json' -d '{"on":false}' $M/api/person >/dev/null
    sleep 2
    curl -s --max-time 8 -X POST -H 'Content-Type: application/json' -d '{"on":true}' $M/api/person >/dev/null
    READY=$(python3 -c "
import json,time,urllib.request
last=None;ok=False
for _ in range(24):
    try:
        with urllib.request.urlopen('$M/api/person.json',timeout=0.5) as r: j=json.load(r)
        f=j.get('frame_id')
        if f is not None and f!=last:
            if last is not None: ok=True;break
            last=f
    except Exception: pass
    time.sleep(0.5)
print('OK' if ok else 'FAIL')")
    echo "[b] 第 $try 次：帧号推进 = $READY"
    [ "$READY" = "OK" ] && break
    sleep 2
done
[ "$READY" != "OK" ] && { echo "[b] 看板没起来，退出"; exit 3; }

sleep 2
echo "[b] 前置检查："
curl -s --max-time 4 $M/api/status | python3 -c "
import json,sys
s=json.load(sys.stdin)
print('    tlm_valid=%s mode=%s rng=%s vel=%s/%s' % (s.get('tlm_valid'),s.get('fpga_mode_name'),s.get('rng'),s.get('velL'),s.get('velR')))
"
python3 -c "
import json
v=json.load(open('/tmp/person.json',encoding='utf-8'))
d=[x for x in (v.get('detections') or []) if x.get('cls')==44]
print('    红瓶候选 %d 个' % len(d), end='')
if d:
    b=d[0]; print(' conf=%.3f cx=%.0f h=%.0f red=%.2f' % (b['conf'],b['cx'],b['h'],b.get('red_ratio') or 0))
else: print()
"

echo "[b] 跑 ${SECS}s（live，参数：$EXTRA）…"
cd /root/task6live_20261029/task6 || exit 1
timeout $((SECS + 25)) python3 run_red_bottle.py --live --follow --seconds "$SECS" \
    --log "$OUT/live.jsonl" $EXTRA 2>&1 | tail -2

python3 -c "
import json,collections,os
p='$OUT/live.jsonl'
rows=[json.loads(l) for l in open(p,encoding='utf-8') if l.strip()]
print('[b] 控制拍数 %d live=%s 时长 %.2fs' % (len(rows), rows[0].get('live'), rows[-1]['elapsed']))
print('    state:', dict(collections.Counter(r['state'] for r in rows)))
print('    非零:', dict(collections.Counter((r['dir'],r['speed'],r['state']) for r in rows if r['speed']>0)))
cx=[(r.get('target') or {}).get('cx') for r in rows if (r.get('target') or {}).get('cx') is not None]
if cx: print('    cx: 首 %.0f → 末 %.0f（目标 664）' % (cx[0],cx[-1]))
"
curl -s --max-time 4 $M/api/status | python3 -c "
import json,sys
s=json.load(sys.stdin)
print('[b] 结束: vel=%s/%s rng=%s mode=%s' % (s.get('velL'),s.get('velR'),s.get('rng'),s.get('fpga_mode_name')))
"
