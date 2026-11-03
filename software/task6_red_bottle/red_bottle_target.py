"""目标关联与红色目标策略，综合语义类别、颜色比例、空间连续性和丢失计时。"""
import math
from prop_ctrl import PropFollow, Cmd

def iou(a,b):
    x=max(0,min(a['cx']+a['w']/2,b['cx']+b['w']/2)-max(a['cx']-a['w']/2,b['cx']-b['w']/2))
    y=max(0,min(a['cy']+a['h']/2,b['cy']+b['h']/2)-max(a['cy']-a['h']/2,b['cy']-b['h']/2))
    union=a['w']*a['h']+b['w']*b['h']-x*y
    return x*y/union if union>0 else 0

class RedBottleLock:
    def __init__(self, confirm=3, red_min=.18, max_age_ms=300, retain_ms=1000,
                 min_conf=.10, grace_frames=3, target_cls=44):


        self.confirm=confirm;self.red_min=red_min;self.max_age_ms=max_age_ms;self.retain_ms=retain_ms
        self.min_conf=min_conf;self.grace_frames=grace_frames;self.miss=0


        if isinstance(target_cls,(list,tuple,set)):
            self.target_cls=set(int(x) for x in target_cls)
        else:
            self.target_cls={int(target_cls)}
        self.producer=None;self.frame=-1;self.last_new=-1e9;self.last_seen=-1e9
        self.target=None;self.hits=0;self.locked=False;self.new_frame=False;self.state='WAIT_TARGET'
    def update(self, packet, now):
        self.new_frame=False
        if not isinstance(packet,dict) or packet.get('color_schema')!=1 or not 0<=packet.get('age_ms',1e9)<=self.max_age_ms:
            self.locked=False;self.hits=0;self.state='STALE_VISION';return None
        producer=packet.get('producer_id');frame=packet.get('frame_id')
        if type(producer)!=int or type(frame)!=int:self.state='BAD_PACKET';self.locked=False;return None
        if producer!=self.producer:
            self.producer=producer;self.frame=-1;self.target=None;self.hits=0;self.locked=False
        if frame<=self.frame:
            if frame==self.frame and now-self.last_new<=self.max_age_ms:
                self.state='LOCKED_DUPLICATE' if self.locked else 'CONFIRMING_DUPLICATE'
                return self.target if self.locked else None
            self.state='DUPLICATE_OR_RESTART';self.locked=False;self.hits=0;return None
        self.frame=frame;self.last_new=now;self.new_frame=True
        candidates=[]
        for d in packet.get('detections',[]):
            if not isinstance(d,dict):continue
            vals=[d.get(k) for k in ('conf','cx','cy','w','h','red_ratio')]
            if any(not isinstance(v,(int,float)) or not math.isfinite(v) for v in vals):continue
            if d.get('cls') in self.target_cls and self.min_conf<=d['conf']<=1 and self.red_min<=d['red_ratio']<=1 and 0<=d['cx']<=1280 and 0<=d['cy']<=720 and 8<=d['w']<=1280 and 12<=d['h']<=720:
                candidates.append(d)
        if self.target and now-self.last_seen<=self.retain_ms:
            matches=[d for d in candidates if iou(d,self.target)>=.12 and .45<=d['w']/self.target['w']<=2.2]
            if len(matches)==1:candidates=matches
            else:candidates=[]
        elif len(candidates)>1:


            candidates=sorted(candidates,
                key=lambda d:(round(float(d.get('red_ratio') or 0),2), float(d.get('conf') or 0),
                               float(d.get('w') or 0)*float(d.get('h') or 0)),
                reverse=True)[:1]
        if len(candidates)!=1:
            self.locked=False
            self.miss += 1
            if self.miss > self.grace_frames:
                self.hits = 0        # 真丢（连续超过 grace 帧）才清零确认计数
            self.state='AMBIGUOUS' if len(candidates)>1 else 'TARGET_LOST'
            if now-self.last_seen>self.retain_ms:self.target=None
            return None
        chosen=dict(candidates[0]);self.target=chosen;self.last_seen=now;self.hits+=1;self.miss=0
        self.locked=self.hits>=self.confirm;self.state='LOCKED' if self.locked else 'CONFIRMING'
        return chosen if self.locked else None

class RedBottleFollow:
    def __init__(self,cx_center=664,follow=True,still_confirm=False,still_vel_max=15,still_frames=3,
                 trim_closed_loop=True,trim_progress_px=3,trim_stall_frames=8,
                 trim_max_ms=1500,trim_overshoot_px=60,trim_settle_ms=400,
                 burst_start_ms=275,burst_min_ms=250,burst_max_ms=250,
                 range_stop_mm=350,close_h=0.82,min_conf=.10,grace_frames=3,coast_ms=0,
                 target_cls=44,red_min=.18,
                 max_spd=40,fwd_far=38,fwd_near=24,spin_max=24,trim_cap=16,
                 d_far=1000,d_near=400,continuous=True,
                 grace_ms=300,hold_ms=700,burst_pct=14,burst_settle=420,
                 search_enable=True,search_max_ms=20000,search_after_ms=3000,
                 deg_per_px=0.0413,bearing_tol_deg=5.0,bearing_spd=10,
                 sweep_spd=8,search_spd=8,
                 red_cue=True,red_min_n=60,red_min_w=24,red_cue_spd=6,red_center_px=130,
                 red_release_px=45,red_min_spd=4):
        self.lock=RedBottleLock();self.cx_center=cx_center;self.follow=follow;self.ctrl=None;self.last_cmd=Cmd(0,0,'WAIT_TARGET');self.arrived=False;self.trim_deadline=None;self.settle_until=0


        self._cx_ema=None; self._cx_ema_alpha=0.4


        self.still_confirm=still_confirm;self.still_vel_max=still_vel_max;self.still_frames=still_frames
        self._was_locked=False;self._still_cnt=0;self._need_still=False


        self.trim_closed_loop=trim_closed_loop
        self.trim_progress_px=trim_progress_px
        self.trim_stall_frames=trim_stall_frames
        self.trim_max_ms=trim_max_ms
        self.trim_overshoot_px=trim_overshoot_px
        self.trim_settle_ms=trim_settle_ms
        self._trim_err0=None;self._trim_best=None;self._trim_start=None;self._trim_stall=0


        self.burst_start_ms=burst_start_ms
        self.burst_min_ms=burst_min_ms
        self.burst_max_ms=burst_max_ms


        self.range_stop_mm=range_stop_mm
        self.close_h=close_h

        self.max_spd=max_spd; self.fwd_far=fwd_far; self.fwd_near=fwd_near
        self.spin_max=spin_max; self.trim_cap=trim_cap
        self.d_far=d_far; self.d_near=d_near


        self.continuous=continuous


        self.grace_ms=grace_ms; self.hold_ms=hold_ms
        self.burst_pct=burst_pct; self.burst_settle=burst_settle


        self.search_enable=search_enable; self.search_max_ms=search_max_ms
        self.search_after_ms=search_after_ms


        self.deg_per_px=deg_per_px; self.bearing_tol_deg=bearing_tol_deg
        self.bearing_spd=bearing_spd

        self.sweep_spd=sweep_spd; self.search_spd=search_spd


        self.red_cue=red_cue; self.red_min_n=red_min_n; self.red_min_w=red_min_w
        self.red_cue_spd=red_cue_spd; self.red_center_px=red_center_px


        self.red_release_px=red_release_px; self.red_min_spd=red_min_spd
        self._red_dir=None
        self._align_dir=None
        self._bearing_yaw=None
        self._bag_side=None


        self.lock=RedBottleLock(confirm=1,red_min=red_min,min_conf=min_conf,grace_frames=grace_frames,
                                target_cls=target_cls)


        self.coast_ms=coast_ms
        self._lost_at=None
        self.last_move=None
    def stop(self,reason):
        self.ctrl=None;self.last_cmd=Cmd(0,0,reason);return self.last_cmd
    def _build_ctrl(self):


        self.ctrl=PropFollow(cx_center=self.cx_center,near_w=9999,accept_n=1,follow=self.follow,
            mirror=True,grace_ms=self.grace_ms,hold_ms=self.hold_ms,
            search_when_never_ms=(self.search_after_ms if self.search_enable else 0),
            sweep_spd=self.sweep_spd,search_spd=self.search_spd,burst=True,
            burst_pct=self.burst_pct,trim=True,trim_spd_cap=self.trim_cap,spin_at=(9999 if self.continuous else 300),
            burst_start_ms=self.burst_start_ms,burst_min_ms=self.burst_min_ms,
            burst_max_ms=self.burst_max_ms,burst_settle_ms=self.burst_settle,
            spd_fwd_far=self.fwd_far,spd_fwd_near=self.fwd_near,spd_spin_max=self.spin_max,
            d_far_mm=self.d_far,d_near_mm=self.d_near,
            range_stop_mm=self.range_stop_mm,
            close_h_frac=self.close_h,close_h_release=max(0.05,self.close_h-0.10),
            db_in=140,db_out=200)

    def step(self,packet,status,now):
        target=self.lock.update(packet,now)
        if not isinstance(status,dict) or not status.get('tlm_ok') or not 0<=status.get('tlm_age_ms',1e9)<=500:return self.stop('STALE_TELEMETRY')
        if status.get('fpga_mode')!=0:return self.stop('WRONG_MODE')
        rng=status.get('rng')


        rng_ok = (isinstance(rng,(int,float)) and math.isfinite(rng) and 20<=rng<4000)
        if self.arrived:return self.stop('ARRIVED')
        if target is not None:
            self._lost_at=None
            self._red_dir=None
            self._bag_side=None

            yaw=status.get('yaw')
            if isinstance(yaw,(int,float)):
                self._bearing_yaw = yaw + self.deg_per_px*(self.cx_center-target['cx'])

        if target:
            if not self._was_locked:
                self._need_still = bool(self.still_confirm)
                self._still_cnt = 0


                self._cx_ema = None
            self._was_locked = True
        else:
            self._was_locked = False
            self._need_still = False
            self._still_cnt = 0
        if not target:
            if self._lost_at is None: self._lost_at = now

            if self.red_cue:
                rcx=packet.get('red_cx') if isinstance(packet,dict) else None
                rn =packet.get('red_n')  if isinstance(packet,dict) else None
                rw =packet.get('red_w')  if isinstance(packet,dict) else None
                if (isinstance(rcx,(int,float)) and isinstance(rn,(int,float))
                        and rn>=self.red_min_n and isinstance(rw,(int,float)) and rw>=self.red_min_w):
                    err=rcx-self.cx_center


                    left_n  = packet.get('red_left_n')  if isinstance(packet,dict) else None
                    right_n = packet.get('red_right_n') if isinstance(packet,dict) else None
                    left_cx  = packet.get('red_left_cx')  if isinstance(packet,dict) else None
                    right_cx = packet.get('red_right_cx') if isinstance(packet,dict) else None


                    has_bottle=any(
                        (d.get('cls') in self.lock.target_cls)
                        and float(d.get('conf') or 0)>=self.lock.min_conf
                        and float(d.get('red_ratio') or 0)>=self.lock.red_min
                        for d in (packet.get('detections') or []) if isinstance(d,dict))
                    nonbottle_red = any(
                        (d.get('cls') in (27, 31, 33)) and float(d.get('conf') or 0) >= self.lock.min_conf
                        for d in (packet.get('detections') or []) if isinstance(d, dict))


                    if nonbottle_red:

                        self._bag_side = 1 if rcx >= self.cx_center else -1
                    other_cx = None

                    main_side = 1 if rcx >= self.cx_center else -1
                    if (self._bag_side is not None and main_side == self._bag_side
                            and not has_bottle):

                        if main_side > 0:                       # 书包在右侧 → 去左
                            if isinstance(left_n,(int,float)) and left_n >= 20 and isinstance(left_cx,(int,float)):
                                other_cx = left_cx
                        else:                                   # 书包在左侧 → 去右
                            if isinstance(right_n,(int,float)) and right_n >= 20 and isinstance(right_cx,(int,float)):
                                other_cx = right_cx


                    if other_cx is not None:
                        err = other_cx - self.cx_center
                    if abs(err) > self.red_release_px:       # 偏差超过松手阈值(45px)

                        d = 4 if err > 0 else 3

                        frac=min(1.0,max(0.0,abs(err)/500.0))
                        spd=self.red_min_spd+(self.red_cue_spd-self.red_min_spd)*frac
                        c=Cmd(d,spd,'SEEK_RED')
                    elif has_bottle and ((rng_ok and rng>self.range_stop_mm)
                                         or (not rng_ok and isinstance(packet.get('red_h'),(int,float))
                                             and packet['red_h'] < self.close_h*720.0*0.7)):

                        if rng_ok:
                            span=max(1.0,float(self.d_far-self.d_near))
                            fr=min(1.0,max(0.0,(rng-self.d_near)/span))
                            spd=self.fwd_near+(self.fwd_far-self.fwd_near)*fr
                        else:
                            spd=self.fwd_near
                        c=Cmd(1,spd,'APPROACH_RED')
                    else:


                        c=Cmd(1,self.fwd_near,'APPROACH_RED') if (self.continuous or rng_ok) \
                          else Cmd(0,0,'RED_CENTERED')
                    self.last_cmd=c
                    if c.spd>0: self.last_move=c
                    return c
            lost_ms = now - self._lost_at


            if (self.search_enable and self._bearing_yaw is not None
                    and lost_ms >= (self.grace_ms+self.hold_ms)):
                yaw=status.get('yaw')
                if isinstance(yaw,(int,float)):
                    dyaw=(self._bearing_yaw-yaw+180.0)%360.0-180.0
                    if abs(dyaw)>self.bearing_tol_deg:

                        d=6 if dyaw>0 else 5
                        c=Cmd(d,self.bearing_spd,'SEEK_BEARING')
                        self.last_cmd=c; self.last_move=c
                        return c

                    self._bearing_yaw=None
            if self.search_enable and lost_ms > self.search_max_ms:
                c = Cmd(0, 0, 'NO_BLIND_MOTION')
                self.last_cmd = c
                return c


            if self.ctrl is None and self.search_enable:
                self._build_ctrl()
            if self.ctrl is not None and self.coast_ms<=0:
                c=self.ctrl.step(None, now, range_mm=None)

                if c.dir==2:
                    c=Cmd(0,0,'NO_BLIND_MOTION')
                c.spd=min(self.max_spd,max(0,c.spd))
                self.last_cmd=c
                if c.spd>0: self.last_move=c
                return c

            if self._lost_at is None: self._lost_at = now


            can_coast = (self.last_move is not None
                         and self.last_move.dir in (1,3,4))
            if (self.coast_ms>0 and can_coast
                    and (now-self._lost_at)<=self.coast_ms
                    and isinstance(rng,(int,float)) and rng>self.range_stop_mm+50):
                c=Cmd(self.last_move.dir,self.last_move.spd,'COAST')
                self.last_cmd=c
                return c
            return self.stop(self.lock.state)
        if not rng_ok:


            if target['h'] >= self.close_h*720.0*0.85:
                self.last_cmd=self.stop('INVALID_RANGE_CLOSE')
                return self.last_cmd


        if rng_ok and rng<=self.range_stop_mm:


            if not self.follow:self.arrived=True
            return self.stop('RANGE_STOP')
        if (not self.continuous) and target['h']>=self.close_h*720:
            return self.stop('BOTTLE_TOO_CLOSE')
        if (not self.continuous) and now < self.settle_until:
            self.last_cmd=Cmd(0,0,'TRIM_SETTLE');return self.last_cmd
        if (not self.continuous) and not self.trim_closed_loop and self.trim_deadline is not None and now>=self.trim_deadline:
            self.trim_deadline=None;self.settle_until=now+800
            return self.stop('TRIM_SETTLE')

        if self._need_still:
            vl=status.get('velL');vr=status.get('velR')
            try:
                still=(abs(float(vl))<=self.still_vel_max and abs(float(vr))<=self.still_vel_max)
            except Exception:
                still=False
            self._still_cnt = self._still_cnt+1 if still else 0
            if self._still_cnt>=self.still_frames:
                self._need_still=False
            else:
                return self.stop('STILL_CONFIRM')
        if not self.lock.new_frame:return self.last_cmd
        if self.ctrl is None: self._build_ctrl()


        raw_cx = target['cx']
        if self._cx_ema is None:
            self._cx_ema = float(raw_cx)
        self._cx_ema = (1.0 - self._cx_ema_alpha) * self._cx_ema + self._cx_ema_alpha * float(raw_cx)
        vis=(44,round(self._cx_ema),round(target['cy']),round(target['w']),round(target['h']),round(target['conf']*100))

        aligned=abs(raw_cx-self.cx_center)<=170
        cmd=self.ctrl.step(vis,now,range_mm=(rng if (aligned and rng_ok) else None))


        if self.trim_closed_loop:
            ae_now=abs(target['cx']-self.cx_center)
            if cmd.dir in (3,4) and cmd.spd>0:
                if self._trim_start is None:                 # 本段第一次 trim
                    self._trim_start=now;self._trim_err0=ae_now
                    self._trim_best=ae_now;self._trim_stall=0
                else:
                    if ae_now < self._trim_best - self.trim_progress_px:
                        self._trim_best=ae_now;self._trim_stall=0      # 有进展
                    else:
                        self._trim_stall+=1                            # 没进展
                stalled   = self._trim_stall >= self.trim_stall_frames
                timeout   = (now - self._trim_start) > self.trim_max_ms
                overshoot = (self._trim_err0 is not None and
                             ae_now > self._trim_err0 + self.trim_overshoot_px)
                if stalled or timeout or overshoot:
                    why='stall' if stalled else ('timeout' if timeout else 'overshoot')
                    self._trim_start=None;self._trim_err0=None
                    self._trim_best=None;self._trim_stall=0
                    if not self.continuous:                 # 连续模式：不停，继续转
                        self.settle_until=now+self.trim_settle_ms
                        return self.stop('TRIM_SETTLE_'+why)
                self.last_cmd=cmd
                if cmd.spd>0: self.last_move=cmd
                return cmd
            else:                                            # 不是 trim ⇒ 本段结束
                self._trim_start=None;self._trim_err0=None
                self._trim_best=None;self._trim_stall=0
        if cmd.dir in (3,4) and cmd.spd>0 and self.trim_deadline is None:self.trim_deadline=now+300
        elif cmd.dir not in (3,4) and self.trim_deadline is not None:
            self.trim_deadline=None
            if not self.continuous:
                self.settle_until=now+800;return self.stop('TRIM_SETTLE')


        if cmd.dir==2:cmd=Cmd(0,0,'NO_BLIND_MOTION')
        cmd.spd=min(self.max_spd,max(0,cmd.spd));self.last_cmd=cmd
        if cmd.spd>0: self.last_move=cmd
        if cmd.state=='ARRIVED':self.arrived=True
        return cmd
