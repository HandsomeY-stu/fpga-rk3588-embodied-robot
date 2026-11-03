#!/usr/bin/env python3
"""任务六执行入口，读取红瓶识别与遥测，计算并按需发送靠近命令。"""
import argparse,json,pathlib,time,urllib.request,socket,signal
from red_bottle_target import RedBottleFollow

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--live',action='store_true');ap.add_argument('--follow',action='store_true');ap.add_argument('--seconds',type=float,default=30);ap.add_argument('--cx-center',type=int,default=664);ap.add_argument('--url',default='http://127.0.0.1:8080');ap.add_argument('--relay-host',default='127.0.0.1');ap.add_argument('--log',default='/tmp/task6_red_bottle.jsonl')

    ap.add_argument('--no-trim-closed-loop',action='store_true',help='关闭闭环微调（回到固定 300/800）')
    ap.add_argument('--trim-progress-px',type=float,default=3)
    ap.add_argument('--trim-stall-frames',type=int,default=8)
    ap.add_argument('--trim-max-ms',type=float,default=1500)
    ap.add_argument('--trim-overshoot-px',type=float,default=60)
    ap.add_argument('--trim-settle-ms',type=float,default=400)

    ap.add_argument('--burst-start-ms',type=float,default=0)
    ap.add_argument('--burst-min-ms',type=float,default=100)
    ap.add_argument('--burst-max-ms',type=float,default=100)

    ap.add_argument('--range-stop',type=float,default=350)   # 超声波距离阈值 mm
    ap.add_argument('--close-h',type=float,default=0.82)     # 瓶框高度占画面比例

    ap.add_argument('--min-conf',type=float,default=0.30)    # 允许开始跟踪的最低置信度
    ap.add_argument('--grace-frames',type=int,default=3)     # 连续丢帧 ≤N 不清零确认计数
    ap.add_argument('--coast-ms',type=float,default=0)     # 短丢失滑行时长（0=关闭）

    ap.add_argument('--max-spd',type=float,default=24)
    ap.add_argument('--fwd-far',type=float,default=22)
    ap.add_argument('--fwd-near',type=float,default=10)
    ap.add_argument('--spin-max',type=float,default=24)
    ap.add_argument('--trim-cap',type=float,default=12)


    ap.add_argument('--no-search',action='store_true',help='关闭搜索（丢失后只停不找）')
    ap.add_argument('--search-max-ms',type=float,default=20000,help='总搜索时长上限')
    ap.add_argument('--search-after-ms',type=float,default=3000,help='从未见过目标时，多久后开始搜索')

    ap.add_argument('--deg-per-px',type=float,default=0.0413,help='像素→角度换算（10-22 标定）')
    ap.add_argument('--bearing-tol-deg',type=float,default=5.0)
    ap.add_argument('--bearing-spd',type=float,default=10)

    ap.add_argument('--sweep-spd',type=float,default=8)
    ap.add_argument('--search-spd',type=float,default=8)

    ap.add_argument('--no-red-cue',action='store_true')
    ap.add_argument('--red-min-n',type=float,default=60,help='红块像素数下限（4px 抽样计数）')
    ap.add_argument('--red-min-w',type=float,default=24,help='红块宽度下限 px')
    ap.add_argument('--red-cue-spd',type=float,default=8)
    ap.add_argument('--grace-ms',type=float,default=300)   # 丢帧后保持上一动作的时长
    ap.add_argument('--hold-ms',type=float,default=700)    # 保持窗结束后停车等检测恢复
    ap.add_argument('--burst-pct',type=float,default=14)
    ap.add_argument('--burst-settle',type=float,default=420)  # 脉冲后停顿 ms


    ap.add_argument('--red-min',type=float,default=0.18)
    ap.add_argument('--target-cls',type=str,default='44',
                    help='目标类别，可逗号分隔多个：27=backpack 31=handbag 33=suitcase 44=bottle')
    a=ap.parse_args()
    policy=RedBottleFollow(cx_center=a.cx_center,follow=a.follow,
        trim_closed_loop=not a.no_trim_closed_loop,trim_progress_px=a.trim_progress_px,
        trim_stall_frames=a.trim_stall_frames,trim_max_ms=a.trim_max_ms,
        trim_overshoot_px=a.trim_overshoot_px,trim_settle_ms=a.trim_settle_ms,
        burst_start_ms=a.burst_start_ms,burst_min_ms=a.burst_min_ms,burst_max_ms=a.burst_max_ms,
        range_stop_mm=a.range_stop,close_h=a.close_h,
        min_conf=a.min_conf,grace_frames=a.grace_frames,coast_ms=a.coast_ms,
        target_cls=[int(x) for x in str(a.target_cls).replace(' ','').split(',') if x!=''],
        red_min=a.red_min,
        max_spd=a.max_spd,fwd_far=a.fwd_far,fwd_near=a.fwd_near,
        spin_max=a.spin_max,trim_cap=a.trim_cap,
        grace_ms=a.grace_ms,hold_ms=a.hold_ms,
        search_enable=not a.no_search,search_max_ms=a.search_max_ms,
        search_after_ms=a.search_after_ms,
        deg_per_px=a.deg_per_px,bearing_tol_deg=a.bearing_tol_deg,bearing_spd=a.bearing_spd,
        sweep_spd=a.sweep_spd,search_spd=a.search_spd,
        red_cue=not a.no_red_cue,red_min_n=a.red_min_n,red_min_w=a.red_min_w,
        red_cue_spd=a.red_cue_spd,
        burst_pct=a.burst_pct,burst_settle=a.burst_settle)
    sock=None;running=True
    def stop_signal(*_):
        nonlocal running
        running=False
    signal.signal(signal.SIGTERM,stop_signal);signal.signal(signal.SIGINT,stop_signal)
    def get(path):
        with urllib.request.urlopen(a.url+path,timeout=.25) as r:return json.load(r)
    log=pathlib.Path(a.log);log.parent.mkdir(parents=True,exist_ok=True)
    start=time.monotonic()
    try:
        if a.live:
            st=get('/api/status')
            if not st.get('tlm_ok') or st.get('fpga_mode')!=0 or st.get('person_uart'):raise RuntimeError('Need fresh manual telemetry and detector UART disabled')
            sock=socket.create_connection((a.relay_host,9100),timeout=.3);sock.settimeout(.2)
        with log.open('w',encoding='utf-8') as f:
            while running and time.monotonic()-start<a.seconds:
                tick=time.monotonic();packet={};status={};error=None
                try:packet=get('/api/person.json');status=get('/api/status');packet['age_ms']=packet.get('age_ms',1e9)+(time.monotonic()-tick)*1000
                except Exception as e:error=str(e)
                now=time.monotonic()*1000;cmd=policy.step(packet,status,now)
                if sock:sock.sendall(('%d %d\n'%(cmd.dir,cmd.spd)).encode())
                row=dict(elapsed=time.monotonic()-start,live=a.live,frame_id=packet.get('frame_id'),state=cmd.state,dir=cmd.dir,speed=cmd.spd,lock_state=policy.lock.state,new_frame=policy.lock.new_frame,target=policy.lock.target if policy.lock.locked else None,error=error)
                f.write(json.dumps(row,ensure_ascii=False)+'\n');f.flush()

                out=pathlib.Path('/tmp/task6_red_status.json');tmp=out.with_suffix('.tmp');tmp.write_text(json.dumps(row),encoding='utf-8');tmp.replace(out)
                if policy.arrived and not a.follow:break
                time.sleep(max(0,.1-(time.monotonic()-tick)))
    finally:
        if sock:
            try:sock.sendall(b'0 0\n')
            except OSError:pass
            sock.close()
    print(json.dumps({'log':str(log),'live':a.live,'seconds':time.monotonic()-start}))
if __name__=='__main__':main()
