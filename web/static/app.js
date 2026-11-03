// 上位机交互：遥控心跳、任务请求、遥测轮询与视频刷新。

/* ★ 2026-10-26 人员识别看板状态：必须用 var 在最前面声明。
   poll() 在脚本中段就被调用一次，若用 let 声明在后面会命中 TDZ
   （ReferenceError: Cannot access 'pOn' before initialization）⇒ 整页 JS 挂掉。 */
var pOn = false, pBusy = false;
const $=s=>document.querySelector(s);
// 速度记忆：手机上不用每次都调
let spd = +(localStorage.getItem('spd') || 30);
let holding=null, timer=null;

function post(p,d){return fetch(p,{method:'POST',headers:{'Content-Type':'application/json'},
  body:JSON.stringify(d||{})}).then(r=>r.json()).catch(e=>({ok:false,err:String(e)}));}

// ---- 按住移动 ----
document.querySelectorAll('button[data-dir]').forEach(b=>{
  const dir=+b.dataset.dir;
  const start=e=>{e.preventDefault(); holding=dir; pulse();};
  const stop =e=>{e.preventDefault(); if(holding===dir){holding=null; post('/api/stop');}};
  b.addEventListener('pointerdown',start);
  b.addEventListener('pointerup',stop);
  b.addEventListener('pointerleave',stop);
  b.addEventListener('pointercancel',stop);
});
function pulse(){ if(holding===null) return; post('/api/motion',{dir:holding,spd:spd}); }

// 150ms 发一次（服务端有效期 350ms，够冗余）
timer=setInterval(pulse,150);

$('#estop').addEventListener('pointerdown',e=>{e.preventDefault();
  holding=null; post('/api/estop').then(r=>flash('急停已发送'));});

$('#spd').addEventListener('input',e=>{spd=+e.target.value;
  $('#spdv').textContent=spd; localStorage.setItem('spd',spd);});
$('#spd').value = spd; $('#spdv').textContent = spd;

function flash(t){ $('#msg').textContent=t; }

// ---- 开关 ----
function bindSw(id, path, body){
  const el=$(id);
  el.addEventListener('click',()=>{
    if(el.classList.contains('dis')) { flash('该功能暂不可用'); return; }
    const on=!el.classList.contains('on');
    post(path, Object.assign({on:on}, body||{})).then(r=>{
      if(r.ok) el.classList.toggle('on', on); flash(r.msg||'');
    });
  });
}
function bindFpgaMode(id, mode) {
  const el=$(id); let pending=false;
  el.addEventListener('click',()=>{
    if(pending) return;
    pending=true; holding=null;
    post('/api/fpga_mode',{mode:el.classList.contains('on')?0:mode})
      .then(r=>flash(r.msg||''))
      .catch(()=>flash('模式请求失败，请检查连接'))
      .finally(()=>{pending=false; poll();});
  });
}
bindFpgaMode('#sw-avoid',1);
bindFpgaMode('#sw-nav',4);
bindSw('#sw-t6','/api/task6',{mode:'follow'});
bindSw('#sw-prev','/api/preview');
/* 目标识别看板：把"同步 FPGA OSD"勾选状态一起发过去 */
(function(){
  const sw = $('#sw-person');
  if (sw) sw.addEventListener('click', ()=>{
    if (sw.classList.contains('dis')) return;
    const on = !sw.classList.contains('on');
    const uart = !!($('#pUart') && $('#pUart').checked);
    post('/api/person', {on:on, uart:uart}).then(r=>{
      if (r.ok) sw.classList.toggle('on', on); flash(r.msg||'');
    });
  });
  const cb = $('#pUart');
  if (cb) cb.addEventListener('change', ()=>{
    /* 已经在跑 ⇒ 用新设置重启一次，让 OSD 同步立刻生效 */
    if (sw && sw.classList.contains('on')) {
      const uart = cb.checked;
      post('/api/person', {on:false}).then(()=>{
        setTimeout(()=>post('/api/person', {on:true, uart:uart})
          .then(r=>{ if(!r.ok) flash(r.msg||''); }), 900);
      });
    }
  });
})();

// 目标类型 / 跟随模式：本地设置，启动语义导航时带上
const tgt=$('#tgt');
tgt.value = 'bottle';
tgt.addEventListener('change',()=>localStorage.setItem('tgt', tgt.value));
$('#sw-follow').addEventListener('click',()=>{
  $('#sw-follow').classList.toggle('on');
  localStorage.setItem('fol', $('#sw-follow').classList.contains('on')?'1':'0');
});
if(localStorage.getItem('fol')==='0') $('#sw-follow').classList.remove('on');

// 语义导航开关：把"目标 + 是否跟随"一起发过去
$('#sw-t6s').addEventListener('click',()=>{
  const el=$('#sw-t6s'), on=!el.classList.contains('on');
  post('/api/task6',{on:on, mode:'semantic', tasks:tgt.value,
                     follow:$('#sw-follow').classList.contains('on')})
    .then(r=>{ if(r.ok) el.classList.toggle('on',on); flash(r.msg||''); });
});

// ---- 状态轮询 ----
let lastOk = 0, connOk = null, connSince = 0;
function setConn(ok){
  if(connOk === ok) return;
  connOk = ok;
  const b=$('#badge'), t=$('#badgeTxt');
  b.classList.toggle('ok', ok);
  b.classList.toggle('bad', !ok);
  t.textContent = ok ? '已连接' : '已断开';
  if(ok) connSince = Date.now();
  if(!ok) flash('与板子的连接断开，请检查 WiFi');
}
function setSw(id,on){ const el=$(id); if(el) el.classList.toggle('on',!!on); }
function poll(){
  fetch('/api/status',{cache:'no-store'}).then(r=>r.json()).then(s=>{
    lastOk = Date.now();
    setConn(true);
    $('#hdrInfo').textContent = '板子 '+location.host+' · 已连接 '+
      Math.floor((Date.now()-connSince)/1000)+'s';
    $('#s-rng').textContent = s.rng==null?'–':s.rng+' mm';
    $('#s-yaw').textContent = s.yaw==null?'–':s.yaw.toFixed(2)+'°';
    $('#s-vl').textContent  = s.velL+' mm/s';
    $('#s-vr').textContent  = s.velR+' mm/s';
    $('#s-st').textContent  = s.fpga_state==null?'–':s.fpga_state;
    $('#s-sent').textContent= s.sent;
    /* ★ P3.2：三态。tlm_ok 只代表"曾经收到"，必须看 age 才知当前是否有效 */
    if (s.tlm_valid) {
      $('#fpgaStat').innerHTML = '<span class="dot ok"></span>FPGA 遥测正常 · 命令源 '+(s.src||'-')
        + ' · '+(s.tlm_age_ms|0)+'ms';
    } else if (s.tlm_ever) {
      $('#fpgaStat').innerHTML = '<span class="dot bad"></span>FPGA 遥测<b>已过期</b>（'
        + (s.tlm_age_ms|0) + 'ms 未更新）· 数据不可用于控制';
    } else {
      $('#fpgaStat').innerHTML = '<span class="dot bad"></span>FPGA 遥测<b>从未收到</b>';
    }
    (function(){
      const sel = $('#osdSel');
      if (!sel) return;
      if (s.osd_mode != null && document.activeElement !== sel) sel.value = String(s.osd_mode);
      if (!sel._bound) { sel._bound = true;
        sel.addEventListener('change', ()=>{ post('/api/osd', {mode: parseInt(sel.value,10)}); }); }
    })();
    /* 跟随目标开关：切 红瓶/红书包/人（下次启动任务六生效） */
    (function(){
      const sel = $('#tgt');
      if (!sel) return;
      if (s.tgt_key && document.activeElement !== sel) sel.value = s.tgt_key;
      if (!sel._bound) { sel._bound = true;
        sel.addEventListener('change', ()=>{ post('/api/target', {key: sel.value}); }); }
    })();
    /* ★ P3.4：任务六控制器状态（state/dir/speed/lock/DRY-LIVE） */
    fetch('/api/task6_status',{cache:'no-store'}).then(r=>r.json()).then(j=>{
      const el = $('#t6Stat'); if(!el) return;
      // 状态文件超过 2 秒没更新 ⇒ 任务六已停或状态陈旧，直接显示"未运行"，
      // 不再把上一次运行残留的旧状态数字刷出来（那样页面看起来很乱）。
      if (j.state == null || j.age_ms == null || j.age_ms >= 2000) {
        el.textContent = '任务六状态：未运行';
        el.style.color = '';
        return;
      }
      const mode = j.live ? 'LIVE' : 'DRY';
      el.textContent = '任务六状态：' + mode + ' · ' + j.state
        + ' · dir=' + j.dir + ' spd=' + j.speed
        + ' · lock=' + (j.lock_state||'-')
        + ' · frame=' + (j.frame_id==null?'-':j.frame_id);
      el.style.color = (j.speed>0) ? '#b3541e' : '';
    }).catch(()=>{});
    setSw('#sw-avoid', s.fpga_mode===1);
    setSw('#sw-nav', s.fpga_mode===4);
    if($('#pUart') && s.person) $('#pUart').checked=!!s.person_uart;
    setSw('#sw-t6',  s.task6===true && s.task6_mode==='follow');
    setSw('#sw-t6s', s.task6===true && s.task6_mode==='semantic');
    setSw('#sw-prev', s.preview);
    setSw('#sw-person', s.person);
    pOn = !!s.person;
    if(s.msg) flash(s.msg);
  }).catch(()=>{ setConn(false); });
}
setInterval(poll,500); poll();

// ---- 网页内实时画面（2026-10-25）----
// 走 /api/frame.jpg 读抓帧链路的 /tmp/frame.jpg，5fps 刷新（够看，省流量）。
// 页面在后台时停抓，回来立刻恢复；连续失败就提示，不刷屏。
// ★★ 2026-10-30：改成 MJPEG 连续流（/stream.mjpg），不再 4fps 轮询抓帧。
//   浏览器把 multipart/x-mixed-replace 当视频流连续渲染；断开自动重连。
(function(){
  const im=$('#cam'); if(!im) return;
  const tip=$('#camTip');
  const src='/stream.mjpg';
  function play(){
    if(document.hidden) return;                    // 后台不占带宽
    im.src = src + '?t=' + Date.now();
    if(tip) tip.textContent='连续画面（MJPEG）';
  }
  im.onerror=function(){
    if(tip) tip.textContent='画面断开，重连中…';
    setTimeout(play, 1500);
  };
  document.addEventListener('visibilitychange', function(){
    if(document.hidden){ im.src=''; }               // 切后台就断开
    else { play(); }
  });
  play();
})();

// ---- 人员识别看板（2026-10-26）----
// 1) BMP 串行拉取（约 3fps，与 SSD 输出节拍一致）
// 2) 状态 JSON 每 400ms 一次：有人 ⇒ 绿底"检测到人员 87%"，无人 ⇒ 灰底
function personTick(){
  if(document.hidden || pBusy || !pOn) return;
  const im=$('#pimg'); if(!im) return;
  pBusy=true;
  const nxt=new Image();
  nxt.onload=()=>{ im.src=nxt.src; im.style.display='block'; pBusy=false; };
  nxt.onerror=()=>{ pBusy=false; };
  nxt.src='/api/person.bmp?t='+Date.now();
}
function personStat(){
  if(!pOn) return;
  fetch('/api/person.json',{cache:'no-store'}).then(r=>r.json()).then(j=>{
    const b=$('#pBanner');
    /* ★ 2026-10-27 修：原来用 `Date.now()/1000 - j.ts < 3` 判新鲜度，
     *   但 j.ts 是**检测程序自己的运行秒数**（如 15841.8），不是 epoch 时间戳
     *   ⇒ 差值恒为 ~1.79e9 ⇒ 永远判"不新鲜" ⇒ 明明画面里已经框住了人，
     *   横幅却一直显示"画面中未检测到目标"。
     *   改成读服务端按文件 mtime 算出的 age_ms。 */
    const fresh = Number.isFinite(j.age_ms) && j.age_ms < 300;
    const reds=(j.detections||[]).filter(d=>d.cls===44 && d.conf>=0.4 && d.red_ratio>=0.18);
    const rt=$('#red-target'); if(rt)rt.textContent=fresh?('红瓶候选：'+reds.length+'；任务六仅选择红瓶，透明瓶不参与'):'红瓶候选：检测数据过期';
    /* ★ 2026-10-27：检测是**间歇**的（SSD 逐帧抖动，实测约一半帧有框），
     *   若逐帧显示，横幅会一直在"检测到/未检测到"之间闪。
     *   这里做 2.5 秒保持：命中过就继续保持，超过 2.5s 没命中才显示"未检测到"。 */
    const nowMs = Date.now();
    if (fresh && j.n > 0) { window.__lastHit = {t: nowMs, j: j}; }
    const hold = window.__lastHit && (nowMs - window.__lastHit.t) < 2500;
    if(hold){
      const jj = (fresh && j.n > 0) ? j : window.__lastHit.j;
      const nm=(jj.name||'').toUpperCase();
      if(fresh && (j.bottle_count||0)>0){b.textContent='检测到瓶子 BOTTLE ×'+j.bottle_count+'  最高 '+Math.round(j.bottle_conf*100)+'%'; b.style.background='#14532d'; b.style.color='#ffffff'; return;}
      b.textContent=(nm?('检测到 '+nm):'检测到目标')+'  ×'+jj.n
                    +'  最高 '+Math.round((jj.conf||0)*100)+'%';
      b.style.background='#e6f7ec'; b.style.color='#137a3f';
    }else{
      b.textContent='画面中未检测到目标';
      b.style.background='#eef2f7'; b.style.color='#5a6b80';
    }
  }).catch(()=>{});
}
function personLoop(){
  const on = pOn;               // 状态由 /api/status 同步（见 poll()）
  const sw = $('#sw-person');
  if (sw) { if (on) sw.classList.add('on'); else sw.classList.remove('on'); }
  const im=$('#pimg');
  if(im) im.style.display = on ? (im.src ? 'block':'none') : 'none';
  if(!on){
    const b=$('#pBanner');
    if(b){ b.textContent='未开启'; b.style.background='#eef2f7'; b.style.color='#5a6b80'; }
  }
  if(on){ personTick(); personStat(); }
  setTimeout(personLoop, 400);
}
personLoop();
/* 2026-10-30: camTick 已删除，画面改用 MJPEG 流，这里不再需要 */
// 看门狗：2 秒没有任何成功响应 ⇒ 判为断开（即使浏览器没报错）
setInterval(()=>{ if(lastOk && Date.now()-lastOk > 2000) setConn(false); }, 500);
