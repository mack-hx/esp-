/**
 * @file web_page.c
 * @brief 调参网页的 HTML/JS (整页字符串常量, 由 web_server.c 以 text/html 返回)
 * @note  滑块完全由 param.h 的参数表动态生成, 前端不硬编码任何字段名。
 *        往 param.h 加一行参数, 这里不用改, 网页自动多一个滑块。
 * @author ESP32 平衡轮腿机器人
 * @date 2026-10-04
 */

#include "web_page.h"

/* 大段 HTML/JS 用 C11 原样字符串 (R"HTML(...)HTML") 包住, 免去转义地狱。
 * ESP-IDF 用的是 GCC, 支持 C11 raw string。 */
const char WEB_PAGE_HTML[] = R"HTML(<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>Balance Robot 调参</title>
<style>
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{margin:0;background:#12151a;color:#e6e8ec;font:15px/1.5 -apple-system,system-ui,"PingFang SC","Microsoft YaHei",sans-serif;padding-bottom:40px}
header{position:sticky;top:0;z-index:10;background:#1b1f27;padding:10px 14px;border-bottom:1px solid #2a303c}
h1{margin:0 0 5px;font-size:17px}
#status{font-size:12px;color:#8b93a3;display:flex;gap:10px;flex-wrap:wrap}
#status b{color:#5ad1a0;font-weight:600}
#status b.warn{color:#e8b04b}
.bar{display:flex;gap:8px;margin-top:9px;flex-wrap:wrap}
button{flex:1 1 90px;min-width:90px;padding:9px 8px;font-size:14px;font-weight:600;border:0;border-radius:8px;cursor:pointer;color:#fff}
button:active{filter:brightness(1.25)}
button:disabled{opacity:.45}
section{margin:12px 10px;background:#1b1f27;border-radius:10px;overflow:hidden}
section>h2{margin:0;padding:9px 12px;font-size:14px;background:#222732;cursor:pointer;user-select:none}
.gbody{padding:4px 12px 12px}
.gbody.hide{display:none}
.row{padding:9px 0;border-bottom:1px solid #262c36}
.row:last-child{border-bottom:0}
.top{display:flex;justify-content:space-between;align-items:baseline;gap:8px}
.name{font-size:14px}
.val{font-size:15px;font-weight:700;color:#5ad1a0;font-variant-numeric:tabular-nums}
.hint{font-size:11px;color:#7c8494;margin-top:2px}
.hint .df{opacity:.55}
.hint .df.mod{opacity:1;color:#e8b04b}
input[type=range]{width:100%;margin:8px 0 0;height:28px;background:transparent;-webkit-appearance:none}
input[type=range]::-webkit-slider-runnable-track{height:6px;background:#333a47;border-radius:3px}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:26px;height:26px;margin-top:-10px;border-radius:50%;background:#5ad1a0}
input[type=range]::-moz-range-track{height:6px;background:#333a47;border-radius:3px}
input[type=range]::-moz-range-thumb{width:26px;height:26px;border-radius:50%;background:#5ad1a0;border:0}
.tag{display:inline-block;font-size:10px;color:#12151a;background:#6a7488;border-radius:4px;padding:0 5px;margin-left:5px}
.mgrid{display:grid;grid-template-columns:repeat(2,1fr);gap:7px;margin:8px 0}
.mgrid3{display:grid;grid-template-columns:repeat(3,1fr);gap:7px;margin:8px 0}
.mgrid button,.mgrid3 button{background:#2b4f7d;padding:11px 4px;font-size:15px}
.mgrid button.on{background:#2f7d5c}
#btnCal{background:#2f7d5c;font-size:17px;padding:15px}
#btnCal2{background:#5b4a8a;font-size:15px;padding:12px}
canvas{width:100%;height:120px;display:block;background:#0d1014;border-radius:6px;margin:8px 0 4px}
#toast{position:fixed;left:50%;bottom:26px;transform:translateX(-50%);background:#2f7d5c;color:#fff;padding:9px 18px;border-radius:20px;font-size:14px;opacity:0;transition:opacity .25s;pointer-events:none;z-index:99}
.warnbox{background:#3a2a12;border-left:3px solid #e8b04b;padding:8px 10px;border-radius:5px;font-size:12px;color:#f0d9ae;margin:8px 0}
.warnbox b{color:#e8b04b}
table{width:100%;border-collapse:collapse;font-size:12px;margin:6px 0}
th,td{padding:5px 4px;text-align:right;border-bottom:1px solid #262c36}
th:first-child,td:first-child{text-align:left}
th{color:#8b93a3;font-weight:600}
a.del{color:#e8b04b;text-decoration:none;font-weight:700}
</style>
</head>
<body>
<header>
  <h1>Balance Robot 调参</h1>
  <div id="status">
    <span>WiFi <b id="sWifi">-</b></span>
    <span>接入 <b id="sCli">-</b></span>
    <span>Pitch <b id="sPitch">-</b></span>
    <span>原始 <b id="sPitchRaw">-</b></span>
    <span>Gyro <b id="sGyro">-</b></span>
    <span>腿角 <b id="sServo">-</b></span>
    <span>控制 <b id="sBal">-</b></span>
  </div>
  <div class="bar">
    <button id="btnSave" style="background:#2f7d5c">保存到 NVS</button>
    <button id="btnDefault" style="background:#7a4b2f">恢复默认</button>
    <button id="btnReload" style="background:#4a5568">刷新</button>
  </div>
  <div class="bar">
    <button id="btnServo" style="background:#5b4a8a">舵机回中</button>
    <button id="btnReboot" style="background:#6a3030">重启设备</button>
  </div>
</header>

<section>
  <h2 onclick="tog(8)">① 配平标定（先做这个）</h2>
  <div id="g8" class="gbody hide">
    <div class="warnbox">
      <b>为什么先做这个</b><br>
      腿换一个角度，「平衡」的姿态就不是车身竖直了。
      这张表就是告诉控制器：<b>腿在 X 度时，我该盯多少度才算平衡</b>。<br>
      没标定 = 腿一动车就往一个方向偏，永远站不起来。
    </div>
    <div class="warnbox">
      <b>怎么标（30 秒一个点）</b><br>
      1. 把车<b>扶住</b>（放桌上 / 夹具 / 手扶），电机全程刹车，不会自己跑。<br>
      2. 滑块拨到 30°，点「① 拨腿到这里」→ 两路腿转到 30°<br>
      3. <b>用手把车身扶到接近平衡</b>（车会往一边倒，你就往另一边扶）<br>
      4. 稳住，再点「② 记下这个角度」→ 读 Pitch 存进表<br>
      5. 换角度重复：<b>40 / 50 / 60 / 70 / 80 / 90 / 100</b>（6~8 个点够了）<br>
      6. 全部录完，点顶部<b>「保存到 NVS」</b>才落盘
    </div>
    <div class="row">
      <div class="top"><span class="name">腿的角度</span><span class="val" id="calDeg">80°</span></div>
      <input type="range" id="rDeg" min="30" max="100" step="1" value="80"
             oninput="document.getElementById('calDeg').textContent=this.value+'°'">
      <div class="hint">每次都要<b>换一个角度</b>。全用同一个角度记是没意义的。</div>
    </div>
    <div class="mgrid">
      <button id="btnMove">① 拨腿到这里</button>
      <button id="btnCal">② 记下这个角度</button>
    </div>
    <div class="hint" id="calMsg" style="margin:6px 0;font-size:12px">
      还没开始 —— 先点「① 拨腿到这里」
    </div>
    <table>
      <tr><th>#</th><th>腿角</th><th>平衡俯仰角</th><th></th></tr>
      <tbody id="calRows"><tr><td colspan="4" style="text-align:center;color:#7c8494">还没有配平点</td></tr></tbody>
    </table>
    <div class="mgrid3">
      <button onclick="tmClear()" style="background:#6a3030">清空重录</button>
      <button onclick="tmLoad()" style="background:#4a5568">刷新表</button>
      <button onclick="tmNext()" style="background:#5b4a8a">跳下一个角度</button>
    </div>
    <div class="hint" id="tmVerdict" style="margin-top:6px;font-size:12px"></div>
  </div>
</section>

<section>
  <h2 onclick="tog(9)">② 平衡控制</h2>
  <div id="g9" class="gbody hide">
    <div class="mgrid">
      <button id="balOn" onclick="balance(1)">▶ 启动平衡</button>
      <button id="balOff" onclick="balance(0)" style="background:#7a4b2f">■ 停止回中</button>
    </div>
    <div class="row">
      <div class="hint">状态 <b id="bState">-</b>　电机 <b id="bMot">-</b>　门控 <b id="bArm">-</b></div>
      <div class="hint">目标倾角 <b id="bTgt">-</b>　平衡输出 <b id="bOut">-</b>　轮速 <b id="bSpeed">-</b></div>
      <div class="hint">配平补偿 <b id="bTrim">-</b>　周期 <b id="bDt">-</b></div>
    </div>
    <div class="mgrid">
      <button onclick="bStop()" style="background:#b33a3a">■ 急停（清积分）</button>
      <button onclick="bReload()" style="background:#4a5568">重读参数</button>
    </div>
    <div class="warnbox" style="background:#2a1a1a;border-color:#e05a5a;color:#f0c4c4">
      <b style="color:#e05a5a">第一次试车请务必：</b>
      <ol style="margin:4px 0 0;padding-left:18px">
        <li>先做配平标定（上面 ① 面板），没标定车动起来就往一边倒</li>
        <li>把 <b>motor_max</b> 压到 <b>25</b> 以内（默认已经是 30，往下调）</li>
        <li>扶正车 → 启动 → 看到 <b>门控</b> 跳到 500ms 后电机才开始输出</li>
        <li>观察车往<b>倒的方向</b>冲：这是对的（方向已烧死在固件里）</li>
        <li>方向对了，再慢慢加 <b>bal_kp</b>（默认 40 是占位值）</li>
        <li>全程<b>扶着车</b>，随时准备断电</li>
      </ol>
    </div>
  </div>
</section>

<section>
  <h2 onclick="tog(10)">③ 俯仰角波形（诊断用）</h2>
  <div id="g10" class="gbody hide">
    <div class="row">
      <div class="hint">最近 120 个采样点。<b style="color:#5ad1a0">绿线 = 修正后的 Pitch</b>
      （已扣零偏和配平补偿，<b>这条线在配平位应该稳在 0 附近</b>），
      <b style="color:#e8b04b">黄线 = 原始 Pitch</b>。</div>
    </div>
    <canvas id="chart" width="600" height="240"></canvas>
    <div class="hint" style="text-align:right">
      当前 <b id="wNow">-</b>　量程 <b id="wRange">-</b>
      <button id="btnPause" style="flex:none;padding:5px 12px;margin-left:8px;font-size:12px;background:#4a5568">暂停</button>
    </div>
  </div>
</section>

<div id="groups"></div>
<div id="toast"></div>

<script>
var ITEMS=[], pend={}, sending=0, resend=0;
var GRPN={0:'平衡环（直立）',1:'速度环（前进后退）',2:'限幅 / 死区 / 方向'};
/* 默认展开: 0=平衡环 1=速度环 2=限幅组 8=配平标定 9=平衡控制 10=波形 */
var OPEN={0:1,1:0,2:0,8:1,9:0,10:0};

function toast(t){var e=document.getElementById('toast');e.textContent=t;e.style.opacity=1;
  clearTimeout(e.t);e.t=setTimeout(function(){e.style.opacity=0;},1500);}
function $(id){return document.getElementById(id);}
function set(id,v){var e=$(id);if(e)e.textContent=v;}
function fmt(it,v){return Number(v).toFixed(it.decimals);}

function build(){
  var h='',last=-1,i,it;
  for(i=0;i<ITEMS.length;i++){
    it=ITEMS[i];
    if(it.grp!==last){
      if(last>=0) h+='</div></section>';
      h+='<section><h2 onclick="tog('+it.grp+')">'+(GRPN[it.grp]||('组 '+it.grp))+
         ' <span class="tag" id="c'+it.grp+'">0 项</span></h2>'+
         '<div id="g'+it.grp+'" class="gbody'+(OPEN[it.grp]?'':' hide')+'">';
      last=it.grp;
    }
    h+='<div class="row"><div class="top"><span class="name">'+it.label+
       (it.unit?'<span class="tag">'+it.unit+'</span>':'')+'</span>'+
       '<span class="val" id="v_'+it.key+'">'+fmt(it,it.v)+'</span></div>'+
       '<input type="range" id="r_'+it.key+'" min="'+it.min+'" max="'+it.max+
       '" step="'+it.step+'" value="'+it.v+'" oninput="setv(\''+it.key+'\',this.value)">'+
       '<div class="hint">'+it.hint+' <span class="df" id="d_'+it.key+'">默认 '+fmt(it,it.def)+'</span></div></div>';
  }
  if(last>=0) h+='</div></section>';
  $('groups').innerHTML=h;
  var cnt={};
  for(i=0;i<ITEMS.length;i++)cnt[ITEMS[i].grp]=(cnt[ITEMS[i].grp]||0)+1;
  for(var g in cnt){var e=$('c'+g);if(e)e.textContent=cnt[g]+' 项';}
}
function tog(g){$('g'+g).classList.toggle('hide');}
function get(k){for(var i=0;i<ITEMS.length;i++)if(ITEMS[i].key===k)return ITEMS[i];return null;}

function setv(k,v){
  var it=get(k);if(!it)return;
  $('v_'+k).textContent=fmt(it,v);
  var mod=Math.abs(v-it.def)>1e-9;
  $('d_'+k).className='df'+(mod?' mod':'');
  $('r_'+k).style.background=mod?'rgba(232,176,75,.12)':'';
  if(mod){pend[k]=v;}else{delete pend[k];}
  flush();
}

/* 滑块拖动会连续触发, 合并成 80ms 一次的下发, 避免刷爆 ESP32 */
var timer=null;
function flush(){
  if(timer)return;
  timer=setTimeout(function(){timer=null;post();},80);
}
function post(){
  if(sending){resend=1;return;}
  if(!Object.keys(pend).length)return;
  var o={},p=pend;pend={};
  for(var k in p)o[k]=p[k];
  sending=1;
  fetch('/api/params',{method:'POST',body:JSON.stringify(o)})
    .then(function(){if(resend){resend=0;post();}else sending=0;})
    .catch(function(){sending=0;toast('下发失败');});
}

/* ========== 配平标定 ========== */
var tmPts=[];        /* [{deg, pitch}] */
var tmLastDeg=null;  /* 上次「拨腿」到的角度 */

/* 标定期间禁用按钮, 防止连点导致请求打架 */
function tmBusy(on){
  $('btnMove').disabled=on;$('btnCal').disabled=on;
  $('btnMove').textContent=on?'腿在动…':'① 拨腿到这里';
  $('btnCal').textContent=on?'读数中…':'② 记下这个角度';
}
function tmMsg(html,ok){
  var e=$('calMsg');e.innerHTML=html;e.style.color=ok?'#5ad1a0':'#8b93a3';
}
function tmPost(body){
  return fetch('/api/trim',{method:'POST',body:JSON.stringify(body)})
    .then(function(r){return r.json();});
}

$('btnMove').onclick=function(){
  var d=Number($('rDeg').value);
  tmBusy(true);
  tmMsg('腿转到 <b>'+d+'°</b>，等姿态稳定…<br>'
       +'<b style="color:#e8b04b">现在用手把车身扶到接近平衡</b>，稳住后点「② 记下这个角度」');
  tmPost({act:'move',deg:d}).then(function(r){
    if(!r.ok){tmMsg('失败: '+(r.msg||'?'));return;}
    tmLastDeg=r.deg;
    $('rDeg').value=r.deg;
    $('calDeg').textContent=r.deg+'°';
    tmMsg('腿已在 <b>'+r.deg+'°</b>，当前 Pitch <b>'+r.pitch.toFixed(2)+'°</b>（还没记）<br>'
         +'⚠ 请确认车身已经扶到接近平衡，角速度 <b>'+r.rate.toFixed(1)+'°/s</b> '
         +(Math.abs(r.rate)>8?'<span style="color:#e05a5a">← 还在晃，扶稳再记</span>':'✓ 挺稳'),true);
  }).catch(function(){tmMsg('请求失败');})
    .then(function(){tmBusy(false);});
};

$('btnCal').onclick=function(){
  var d=tmLastDeg;
  if(d===null){tmMsg('请先点「① 拨腿到这里」',false);toast('先点拨腿');return;}
  tmBusy(true);
  tmMsg('正在读 Pitch 并记录…');
  tmPost({act:'add',deg:d}).then(function(r){
    if(!r.ok){tmMsg('失败: '+(r.msg||'?'));toast('记录失败');return;}
    tmMsg('✓ 已记下 <b>'+r.deg+'°</b> → <b>'+r.pitch.toFixed(2)+'°</b>（共 '+r.n+' 点）<br>'
         +'换个角度再来：把滑块拖到别的值，重复 ①→扶→②',true);
    toast('已记录 '+r.deg+'°');
    tmLoad();
  }).catch(function(){tmMsg('请求失败');})
    .then(function(){tmBusy(false);});
};

function tmRender(){
  if(!tmPts.length){
    $('calRows').innerHTML='<tr><td colspan="4" style="text-align:center;color:#7c8494">还没有配平点</td></tr>';
    return;
  }
  var h='';
  for(var i=0;i<tmPts.length;i++){
    h+='<tr><td>'+(i+1)+'</td><td>'+tmPts[i].deg.toFixed(0)+'°</td><td>'+
       tmPts[i].pitch.toFixed(2)+'°</td>'+
       '<td><a href="#" class="del" onclick="tmDel('+tmPts[i].deg+');return false">删</a></td></tr>';
  }
  $('calRows').innerHTML=h;
}
function tmVerdict(usable,span,n){
  var e=$('tmVerdict');
  if(n===0){e.innerHTML='';return;}
  if(n<2){
    e.innerHTML='<b style="color:#e8b04b">已录 '+n+' 个点，至少要 2 个才生效。</b>';
    return;
  }
  var msg='已录 <b>'+n+'</b> 个点，俯仰角跨度 <b>'+span.toFixed(2)+'°</b>';
  if(usable){
    msg+=' <b style="color:#5ad1a0">✓ 生效中</b>。记得点顶部「保存到 NVS」。';
    if(n<5) msg+=' <span style="color:#e8b04b">建议再录几个铺满 30~100°。</span>';
  }else{
    msg+=' <b style="color:#e05a5a">✗ 不生效</b>（跨度太小，说明每次记的几乎是同一个姿态）';
  }
  e.innerHTML=msg;
}
function tmLoad(){
  return fetch('/api/trim').then(function(r){return r.json();}).then(function(d){
    tmPts=(d.pts||[]).map(function(p){return {deg:p.deg,pitch:p.pitch};});
    tmRender();
    tmVerdict(d.usable,d.span,d.n);
  }).catch(function(){});
}
function tmDel(deg){
  tmPost({act:'del',deg:deg}).then(function(){
    toast('已删 '+deg+'°');tmLoad();
  }).catch(function(){toast('请求失败');});
}
function tmClear(){
  if(!confirm('清空整张配平表? (不影响已保存的参数)'))return;
  tmPost({act:'clear'}).then(function(){
    tmPts=[];tmRender();tmVerdict(false,0,0);toast('已清空');
  }).catch(function(){toast('请求失败');});
}
/* 跳到下一个"还没录"的角度, 省得自己盯着表想下一个拨哪 */
function tmNext(){
  var has={};
  for(var i=0;i<tmPts.length;i++)has[Math.round(tmPts[i].deg)]=1;
  for(var d=30;d<=100;d+=5){if(!has[d]){$('rDeg').value=d;$('calDeg').textContent=d+'°';toast('下一个建议 '+d+'°');return;}}
  toast('30~100° 每个 5° 都录过了');
}

/* ========== 平衡控制 ========== */
function balance(on){
  fetch('/api/balance',{method:'POST',body:JSON.stringify({on:on?1:0})})
    .then(function(r){return r.json();})
    .then(function(d){
      if(d.ok)toast(on?'已启动，等门控放行':'已停止回中');
      else toast('启动被拒（车没扶正？）');
    }).catch(function(){toast('请求失败');});
}
function bStop(){
  fetch('/api/balance_stop',{method:'POST'})
    .then(function(){toast('已急停，积分已清');}).catch(function(){});
}
function bReload(){
  fetch('/api/balance_reload',{method:'POST'})
    .then(function(){toast('参数已重读');}).catch(function(){});
}

/* ========== 俯仰角波形 ==========
 * ⚠ 采样源是 /api/status 轮询 (500ms 一次), 所以这是**趋势线**,
 *   抓不到 10ms 级的姿态抖动。要看真实 100Hz 波形必须走串口。
 *   它的价值: 配平补偿配好后, 绿线应该被拉平在 0 附近。 */
var WAVEN=120, wavFix=[], wavRaw=[], wavPaused=0;
function pushWave(fix,raw){
  if(wavPaused)return;
  wavFix.push(fix);wavRaw.push(raw);
  if(wavFix.length>WAVEN){wavFix.shift();wavRaw.shift();}
  set('wNow',fix.toFixed(2)+'°');
  drawWave();
}
function drawWave(){
  var c=$('chart');if(!c)return;
  var g=c.getContext('2d'),W=c.width,H=c.height,pad=6;
  g.clearRect(0,0,W,H);
  if(wavFix.length<2){set('wRange','-');return;}
  var mn=1e9,mx=-1e9,i;
  for(i=0;i<wavFix.length;i++){var a=wavFix[i],b=wavRaw[i];
    if(a<mn)mn=a;if(a>mx)mx=a;if(b<mn)mn=b;if(b>mx)mx=b;}
  var padv=Math.max((mx-mn)*0.15,1.5);
  mn-=padv;mx+=padv;
  set('wRange',mn.toFixed(1)+' ~ '+mx.toFixed(1)+'°');
  if(0>mn&&0<mx){
    var y0=H-((0-mn)/(mx-mn))*(H-2*pad)+pad;
    g.strokeStyle='#3a4250';g.setLineDash([4,4]);g.beginPath();
    g.moveTo(pad,y0);g.lineTo(W-pad,y0);g.stroke();g.setLineDash([]);
  }
  function line(arr,color){
    g.strokeStyle=color;g.lineWidth=2;g.beginPath();
    for(var j=0;j<arr.length;j++){
      var x=pad+j*(W-2*pad)/(WAVEN-1);
      var y=H-((arr[j]-mn)/(mx-mn))*(H-2*pad)+pad;
      j?g.lineTo(x,y):g.moveTo(x,y);
    }
    g.stroke();
  }
  line(wavRaw,'#e8b04b');
  line(wavFix,'#5ad1a0');
}
$('btnPause').onclick=function(){
  wavPaused=!wavPaused;
  this.textContent=wavPaused?'继续':'暂停';
  this.style.background=wavPaused?'#2f7d5c':'#4a5568';
};

/* ========== 顶部按钮 ========== */
$('btnSave').onclick=function(){
  fetch('/api/save',{method:'POST'}).then(function(r){return r.json();})
    .then(function(d){toast(d.ok?'参数和配平表已保存':'保存失败');});
};
$('btnDefault').onclick=function(){
  if(!confirm('恢复默认参数？已保存的配平表不受影响'))return;
  fetch('/api/reset',{method:'POST'}).then(function(){
    pend={};load();toast('已恢复默认（未保存）');});
};
$('btnReload').onclick=function(){pend={};load();toast('已刷新');};
$('btnServo').onclick=function(){
  fetch('/api/servo',{method:'POST'}).then(function(){toast('舵机已回中');});
};
$('btnReboot').onclick=function(){
  if(confirm('确定重启 ESP32？未保存的参数会丢失')){
    fetch('/api/reboot',{method:'POST'}).then(function(){
      document.body.innerHTML='<p style="padding:30px;text-align:center">已重启，等 3 秒后重新连热点…</p>';
    });
  }
};

/* ========== 状态轮询 ========== */
function poll(){
  fetch('/api/status').then(function(r){return r.json();}).then(function(s){
    set('sWifi',s.wifi);set('sCli',s.clients);
    set('sPitch',s.pitch.toFixed(2)+'°');
    set('sPitchRaw',s.pitch_raw.toFixed(2)+'°');
    set('sGyro',s.gyro.toFixed(1)+'°/s');
    set('sServo',s.servo_deg+'°');
    var b=$('sBal');
    b.textContent=s.bal_state+(s.bal_on?'':' (关)');
    b.className=(s.bal_state==='BALANCING')?'':'warn';
    set('bState',s.bal_state);
    set('bMot',s.mot_a+' / '+s.mot_b);
    set('bArm',Math.round(s.arm_ms)+'ms');
    set('bTgt',s.tgt_pitch.toFixed(2)+'°');
    set('bOut',s.bal_out.toFixed(1)+'%');
    set('bSpeed',s.speed.toFixed(1)+' rpm');
    set('bTrim',s.trim_usable?s.trim_comp.toFixed(2)+'°':'未标定');
    set('bDt',s.dt_ms.toFixed(1)+'ms');
    pushWave(s.pitch,s.pitch_raw);
  }).catch(function(){});
}
function load(){
  fetch('/api/params').then(function(r){return r.json();}).then(function(d){
    ITEMS=d.items;build();
  }).catch(function(){toast('读参数失败');});
}
load();poll();tmLoad();setInterval(poll,500);
</script>
</body>
</html>
)HTML";
