/**
 * @file web_page.c
 * @brief 调参网页的 HTML/JS (整页字符串常量, 由 web_server.c 以 text/html 返回)
 * @note  滑块完全由 param.h 的参数表动态生成, 前端不硬编码任何字段名。
 *        以后往 param.h 加一行参数, 这里不用改, 网页自动多一个滑块。
 * @author ESP32 机械狗项目
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
.mgrid{display:grid;grid-template-columns:repeat(3,1fr);gap:7px;margin:8px 0}
.mgrid button{background:#2b4f7d;padding:10px 4px}
.mgrid button.on{background:#2f7d5c}
#btnStop{background:#b33a3a;width:100%;margin-top:6px;padding:12px}
canvas{width:100%;height:130px;display:block;background:#0d1014;border-radius:6px;margin:8px 0 4px}
#toast{position:fixed;left:50%;bottom:26px;transform:translateX(-50%);background:#2f7d5c;color:#fff;padding:9px 18px;border-radius:20px;font-size:14px;opacity:0;transition:opacity .25s;pointer-events:none;z-index:99}
.warnbox{background:#3a2a12;border-left:3px solid #e8b04b;padding:8px 10px;border-radius:5px;font-size:12px;color:#f0d9ae;margin:8px 0}
.warnbox b{color:#e8b04b}
table{width:100%;border-collapse:collapse;font-size:12px;margin:6px 0}
th,td{padding:5px 4px;text-align:right;border-bottom:1px solid #262c36}
th:first-child,td:first-child{text-align:left}
th{color:#8b93a3;font-weight:600}
#btnCalc{background:#2f7d5c;width:100%;margin-top:6px;padding:12px}
</style>
</head>
<body>
<header>
  <h1>Balance Robot 调参</h1>
  <div id="status">
    <span>WiFi <b id="sWifi">-</b></span>
    <span>接入 <b id="sCli">-</b></span>
    <span>Pitch <b id="sPitch">-</b></span>
    <span>Gyro <b id="sGyro">-</b></span>
    <span>编码 <b id="sEnc">-</b></span>
    <span>电机 <b id="sMotor">停止</b></span>
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
  <h2 onclick="tog(8)">平衡控制（骨架版：算法未接电机）</h2>
  <div id="g8" class="gbody hide">
    <div class="row">
      <div class="hint">1.0 只搭了控制框架，PID 输出<b>还没接到电机</b>，点启动只会让状态机走起来，车不会真的平衡。</div>
    </div>
    <div class="mgrid" style="grid-template-columns:repeat(2,1fr)">
      <button id="balOn" onclick="balance(1)">▶ 启动平衡</button>
      <button id="balOff" onclick="balance(0)" style="background:#7a4b2f">■ 停止回中</button>
    </div>
    <div class="row">
      <div class="hint">控制状态 <b id="bState">-</b>　电机 A <b id="bMotA">-</b>　电机 B <b id="bMotB">-</b></div>
      <div class="hint">目标倾角 <b id="bTgt">-</b>　平衡输出 <b id="bOut">-</b>　速度环 <b id="bSpd">-</b></div>
      <div class="hint">转向输出 <b id="bSteer">-</b>　轮速 <b id="bSpeed">-</b></div>
      <div class="hint">腿角 <b id="bLegDeg">-</b>　腿高 <b id="bLegH">-</b>　腿角补偿 <b id="bLegComp">-</b></div>
      <div class="hint">生效 Kp <b id="bKpEff">-</b>　查表 <b id="bLegOk">-</b>　Leg K <b id="bLegK">-</b></div>
    </div>
    <div class="mgrid" style="grid-template-columns:repeat(2,1fr)">
      <button onclick="bStop()" style="background:#b33a3a">■ 急停（清积分）</button>
      <button onclick="bReload()" style="background:#4a5568">重读参数</button>
    </div>
  </div>
</section>

<section>
  <h2 onclick="tog(10)">俯仰角波形（诊断用）</h2>
  <div id="g10" class="gbody hide">
    <div class="row">
      <div class="hint">最近 120 个采样点。<b style="color:#5ad1a0">绿线 = 原始 Pitch</b>，
      <b style="color:#e8b04b">黄线 = 扣掉零偏和腿角补偿后的 Pitch</b>。
      标定时看绿线随腿角怎么走。</div>
    </div>
    <canvas id="chart" width="600" height="260"></canvas>
    <div class="hint" style="text-align:right">
      当前 <b id="wRaw">-</b>　量程 <b id="wRange">-</b>
      <button id="btnPause" style="flex:none;padding:5px 12px;margin-left:8px;font-size:12px;background:#4a5568">暂停</button>
    </div>
  </div>
</section>

<section>
  <h2 onclick="tog(11)">腿高标定（实测查表，推荐）</h2>
  <div id="g11" class="gbody hide">
    <div class="warnbox">
      <b>标定前必读</b><br>
      1. 把车<b>固定住</b>（放桌上 / 夹具 / 手扶），全程<b>不要落地</b>。<br>
      2. 车身必须<b>纹丝不动</b>。本工具会自动强制刹车并停掉平衡控制，
      电机不会转，车不会自己动 —— 但你扶车的力要稳住。<br>
      3. 关键：<b>只让腿动，车身不能动</b>。这样测出的差值才是「腿的贡献」。<br>
      4. 每录一个点：点「动腿并读取」→ ESP32 读 Pitch → <b>你拿尺子量轮心离地高度</b>
      填进去 → 点「录入这个点」。<br>
      5. 建议录 <b>6~8 个点</b>，腿角从 30° 到 100° 均匀铺开。
      录完点顶部<b>「保存到 NVS」</b>才落盘。
    </div>
    <div class="row">
      <div class="top"><span class="name">腿角</span><span class="val" id="calDeg">80°</span></div>
      <input type="range" id="rDeg" min="30" max="100" step="1" value="80"
             oninput="document.getElementById('calDeg').textContent=this.value+'°'">
      <div class="hint">腿角范围要拉开，铺满整个工作行程</div>
    </div>
    <div class="row">
      <div class="top"><span class="name">实测腿高</span><span class="val" id="calH">?</span></div>
      <input type="number" id="nH" min="20" max="200" step="0.5" value="60"
             style="width:100%;padding:10px;font-size:16px;font-weight:700;color:#5ad1a0;
                    background:#0d1014;border:1px solid #333a47;border-radius:6px">
      <div class="hint">轮心离地面的垂直距离（mm）—— <b>这个数只能你量</b>，ESP32 量不了自己</div>
    </div>
    <div class="mgrid" style="grid-template-columns:repeat(2,1fr)">
      <button id="btnMove" onclick="lmMove()" style="background:#2b4f7d">动腿并读取<br><span style="font-size:11px">等 1 秒稳定</span></button>
      <button id="btnAdd" onclick="lmAdd()" style="background:#2f7d5c">录入这个点<br><span style="font-size:11px">存高度+Pitch</span></button>
    </div>
    <div class="hint" id="calMsg" style="margin:6px 0">
      还没动腿 —— 先点「动腿并读取」
    </div>
    <table>
      <tr><th>#</th><th>腿角</th><th>腿高</th><th>Pitch</th><th></th></tr>
      <tbody id="calRows"><tr><td colspan="5" style="text-align:center;color:#7c8494">还没有标定点</td></tr></tbody>
    </table>
    <div class="mgrid" style="grid-template-columns:repeat(2,1fr)">
      <button onclick="lmClear()" style="background:#7a4b2f">清空重来</button>
      <button onclick="lmFitK()" style="background:#5b4a8a">反推线性 Leg K<br><span style="font-size:11px">兜底方案</span></button>
    </div>
    <div class="hint" id="lmVerdict" style="margin-top:6px"></div>
  </div>
</section>

<section>
  <h2 onclick="tog(9)">电机手动测试（调试用，通电前清空车轮）</h2>
  <div id="g9" class="gbody hide">
    <div class="mgrid">
      <button id="mA" onclick="motor(1)">电机 A</button>
      <button id="mB" onclick="motor(2)">电机 B</button>
      <button id="mAB" onclick="motor(3)">A + B</button>
    </div>
    <div class="row">
      <div class="top"><span class="name">速度</span><span class="val" id="vSpd">40</span></div>
      <input type="range" id="rSpd" min="0" max="100" step="1" value="40" oninput="document.getElementById('vSpd').textContent=this.value">
      <div class="hint">点电机按钮 = 启动，再点同一按钮 = 停；速度用上面滑块调</div>
    </div>
    <div class="row">
      <div class="top"><span class="name">方向</span><span class="val" id="vDir">正转</span></div>
      <input type="range" id="rDir" min="0" max="1" step="1" value="0" oninput="document.getElementById('vDir').textContent=this.value?'反转':'正转'">
    </div>
    <button id="btnStop" onclick="stopAll()">■ 急停（两路刹车）</button>
  </div>
</section>

<div id="groups"></div>
<div id="toast"></div>

<script>
var ITEMS=[], pend={}, sending=0, resend=0;
var GRPN={0:'平衡环（直立）',1:'速度环（前进后退）',2:'转向环（左右转）',3:'限幅 / 滤波 / 硬件'};
/* 默认展开的分组 (3 = 限幅组, 8 = 平衡控制, 9 = 电机测试, 10 = 波形, 11 = 标定, 平时用不到, 默认收起) */
var OPEN={0:1,1:1,2:1,3:0,8:0,9:0,10:0,11:0};

function toast(t){var e=document.getElementById('toast');e.textContent=t;e.style.opacity=1;
  clearTimeout(e.t);e.t=setTimeout(function(){e.style.opacity=0;},1400);}
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
    var c=$('c'+it.grp);
  }
  if(last>=0) h+='</div></section>';
  $('groups').innerHTML=h;
  /* 统计每组项目数 */
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

/* 滑块拖动时会连续触发, 合并成 80ms 一次的下发, 避免刷爆 ESP32 */
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

/* ---- 平衡控制 ---- */
function balance(on){
  fetch('/api/balance',{method:'POST',body:JSON.stringify({on:on?1:0})})
    .then(function(r){return r.json();})
    .then(function(d){
      if(d.ok)toast(on?'平衡已启动':'已停止回中');
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

/* ---- 俯仰角波形 ----
 * ⚠ 采样源是 /api/status 轮询 (500ms 一次), 所以这里画的是**趋势线**,
 *   抓不到 10ms 级的姿态抖动。要看真实 100Hz 波形必须走串口。
 *   它的价值在于: 标定时看绿线随腿角怎么走、补偿后黄线是否被拉平。 */
var WAVEN=120, wavRaw=[], wavFix=[], wavPaused=0;
function pushWave(raw,fix){
  if(wavPaused)return;
  wavRaw.push(raw);wavFix.push(fix);
  if(wavRaw.length>WAVEN){wavRaw.shift();wavFix.shift();}
  set('wRaw',raw.toFixed(2)+'°');
  drawWave();
}
function drawWave(){
  var c=$('chart');if(!c)return;
  var g=c.getContext('2d'),W=c.width,H=c.height,pad=6;
  g.clearRect(0,0,W,H);
  if(wavRaw.length<2){set('wRange','-');return;}

  /* 自适应量程: 上下各留 15% 余量, 并至少张开 4° 免得噪声被拉得夸张 */
  var mn=1e9,mx=-1e9,i;
  for(i=0;i<wavRaw.length;i++){var a=wavRaw[i],b=wavFix[i];
    if(a<mn)mn=a;if(a>mx)mx=a;if(b<mn)mn=b;if(b>mx)mx=b;}
  var padv=Math.max((mx-mn)*0.15,2.0);
  mn-=padv;mx+=padv;
  set('wRange',mn.toFixed(1)+' ~ '+mx.toFixed(1)+'°');

  /* 0° 参考线 */
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
  line(wavFix,'#e8b04b');
  line(wavRaw,'#5ad1a0');
}
$('btnPause').onclick=function(){
  wavPaused=!wavPaused;
  this.textContent=wavPaused?'继续':'暂停';
  this.style.background=wavPaused?'#2f7d5c':'#4a5568';
};

/* ---- 腿高标定 (实测查表) ----
 *
 * 流程是"半自动"的: ESP32 能读 Pitch, 但**量不了腿高**,
 * 所以每录一个点都要你先动腿、再量高度、再提交。
 */
var lmPts=[], lmCur=null;   /* lmCur = 上次"动腿并读取"拿到的 deg/pitch */

function lmPost(body){
  return fetch('/api/legmap',{method:'POST',body:JSON.stringify(body)})
    .then(function(r){return r.json();});
}
function lmMsg(t,ok){
  var e=$('calMsg');
  e.innerHTML=t;e.style.color=ok?'#5ad1a0':'#8b93a3';
}

/* 检查高度是否单调 (四连杆在正常行程内轮心高度必然单调变化) */
function lmCheckMono(){
  for(var i=0;i+1<lmPts.length;i++){
    if(lmPts[i+1].h < lmPts[i].h - 1.0)
      return '第 '+(i+1)+'→'+(i+2)+' 点腿高反而变小了。四连杆在正常行程内'+
             '高度必然单调，录错了 → 重新检查这个点的尺子读数';
  }
  return null;
}

function lmRender(){
  var b=$('calRows');
  if(!lmPts.length){
    b.innerHTML='<tr><td colspan="5" style="text-align:center;color:#7c8494">还没有标定点</td></tr>';
    return;
  }
  var h='';
  for(var i=0;i<lmPts.length;i++){
    h+='<tr><td>'+(i+1)+'</td><td>'+lmPts[i].deg+'°</td><td>'+
       lmPts[i].h.toFixed(1)+'</td><td>'+lmPts[i].pitch.toFixed(2)+'°</td>'+
       '<td><a href="#" style="color:#e8b04b" onclick="lmDel('+i+');return false">删</a></td></tr>';
  }
  b.innerHTML=h;

  var mono=lmCheckMono();
  if(mono){lmMsg('<b style="color:#e8b04b">⚠ '+mono+'</b>');return;}

  if(lmPts.length<2){
    lmMsg('已录 '+lmPts.length+' 个点，至少要 2 个',false);return;
  }
  var lo=lmPts[0].deg, hi=lo;
  for(var j=0;j<lmPts.length;j++){
    if(lmPts[j].deg<lo)lo=lmPts[j].deg;
    if(lmPts[j].deg>hi)hi=lmPts[j].deg;
  }
  lmMsg('已录 <b>'+lmPts.length+'</b> 个点，腿角跨度 '+(hi-lo).toFixed(0)+
        '°'+(hi-lo<20?' <b style="color:#e8b04b">（太窄，建议铺满 30~100°）</b>':' ✓')+
        '<br>建议 6~8 个点铺满整个行程，然后点顶部<b>「保存到 NVS」</b>',true);
}

/* 按钮①: 动腿到指定角度, 等稳定, 读回 Pitch */
function lmMove(){
  var d=Number($('rDeg').value);
  lmMsg('腿在动…等姿态稳定约 1 秒，请扶住车身');
  $('btnMove').disabled=true;$('btnAdd').disabled=true;
  lmPost({act:'move',deg:d}).then(function(r){
    if(!r.ok){lmMsg('失败: '+(r.msg||'?'));toast('读取失败');return;}
    lmCur={deg:r.deg,pitch:r.pitch};
    lmMsg('腿已到 <b>'+r.deg+'°</b>，当前 Pitch <b>'+r.pitch.toFixed(2)+'°</b><br>'+
          '现在拿尺子量<b>轮心离地高度</b>，填到上面那个框，然后点「录入这个点」',true);
    $('nH').focus();
  }).catch(function(){lmMsg('请求失败');})
    .then(function(){
      $('btnMove').disabled=false;$('btnAdd').disabled=false;
    });
}

/* 按钮②: 把「刚才读到的 Pitch」+「你量的腿高」一起存进表 */
function lmAdd(){
  if(!lmCur){lmMsg('请先点「动腿并读取」',false);toast('先点动腿并读取');return;}
  var h=parseFloat($('nH').value);
  if(!(h>=20&&h<=200)){lmMsg('腿高要填 20~200 之间的数字',false);toast('腿高不合法');return;}

  lmPost({act:'add',deg:lmCur.deg,h:h}).then(function(r){
    if(!r.ok){lmMsg('失败: '+(r.msg||'?'));toast('录入失败');return;}
    /* 重新拉整表, 以服务端排序结果为准 (服务端按腿角升序插的) */
    lmLoad();
    toast('第 '+r.n+' 个点已录: '+r.deg+'° → '+r.h.toFixed(1)+'mm');
    /* 提示下一个建议腿角, 避免不知道往哪拨 */
    var next=nextDeg();
    if(next!==null)$('rDeg').value=next;
  }).catch(function(){lmMsg('请求失败');});
}

function nextDeg(){
  if(!lmPts.length)return 30;
  var target=30+lmPts.length*10;
  if(target>100)return null;
  return target;
}

function lmDel(i){
  var d=lmPts[i].deg;
  lmPts.splice(i,1);
  /* 单条删除走 clear+逐条重加太笨, 直接让用户重录该点;
     这里只更新本地显示, 提示如何处理 */
  lmRender();
  lmMsg('已从列表移除 '+d+'°。ESP32 里那条还在 —— '+
        '要真删掉请点「清空重来」重录一次',false);
}

function lmLoad(){
  fetch('/api/legmap').then(function(r){return r.json();})
    .then(function(d){
      lmPts=(d.pts||[]).map(function(p){return {deg:p.deg,h:p.h,pitch:p.pitch};});
      lmRender();
      var v=$('lmVerdict');
      if(d.usable&&d.n>=4){
        v.innerHTML='<b style="color:#5ad1a0">✓ 标定可用</b>：'+
          d.n+' 个点，补偿和 Kp 调度都会按表走。记得点「保存到 NVS」。';
      }else if(d.n<2){
        v.innerHTML='标定点不足 2 个，<b>不做任何几何补偿</b>。'+
          '腿改变角度时车会往一个方向偏。';
      }else{
        v.innerHTML='<b style="color:#e8b04b">数据有 '+(d.n)+' 个点但判定不可用</b>'+
          '（多半是高度非单调或腿角跨度不够）→ 补几个点，或点「清空重来」重录。';
      }
    }).catch(function(){toast('读标定表失败');});
}

function lmClear(){
  if(!confirm('清空所有腿高标定点？'))return;
  lmPost({act:'clear'}).then(function(){
    lmPts=[];lmCur=null;lmRender();
    $('lmVerdict').innerHTML='';
    toast('已清空');
  }).catch(function(){toast('请求失败');});
}

/* 兜底: 用标定点反推线性 leg_k (查表不好用时的临时方案) */
function lmFitK(){
  if(lmPts.length<2){toast('至少要 2 个点');return;}
  fetch('/api/legmap_fit_legk',{method:'POST'})
    .then(function(r){return r.json();})
    .then(function(r){
      if(!r.ok){toast('算不了: '+(r.msg||'?'));return;}
      var it=get('leg_k');
      if(it){$('r_leg_k').value=r.k;setv('leg_k',r.k);}
      var msg='线性 Leg K = <b>'+r.k.toFixed(4)+'</b>（'+r.n+' 点拟合）<br>';
      if(r.res_ratio>0.35){
        msg+='<b style="color:#e8b04b">残差占变化幅度 '+(r.res_ratio*100).toFixed(0)+
             '%，说明 Pitch~腿角不是直线关系，线性补偿会丢掉很多量。'+
             '建议继续用查表，或多录几个点让曲线更准。</b>';
      }else{
        msg+='残差占变化幅度 '+(r.res_ratio*100).toFixed(0)+'%，线性近似还算准。';
      }
      $('lmVerdict').innerHTML=msg+'<br>已填入 leg_k 滑块，记得点「保存到 NVS」。';
      toast('Leg K 已算出');
    }).catch(function(){toast('请求失败');});
}

/* ---- 电机手动测试 ---- */
var mOn=0;
function motor(which){
  mOn=(mOn===which)?0:which;
  $('mA').className = mOn===1?'on':'';
  $('mB').className = mOn===2?'on':'';
  $('mAB').className= mOn===3?'on':'';
  post_motor(mOn);
}
function post_motor(which){
  var body=JSON.stringify({on:which,
    speed:Number($('rSpd').value), dir:Number($('rDir').value)});
  fetch('/api/motor',{method:'POST',body:body}).catch(function(){});
}
function stopAll(){
  mOn=0;$('mA').className='';$('mB').className='';$('mAB').className='';
  fetch('/api/stop',{method:'POST'}).then(function(){toast('已急停');});
}
/* 松手/切后台自动停, 防止网页关了电机还转 */
window.addEventListener('pointerup',function(){if(mOn)post_motor(mOn);});
window.addEventListener('beforeunload',function(){
  if(mOn){navigator.sendBeacon&&navigator.sendBeacon('/api/stop','');
          fetch&&fetch('/api/stop',{method:'POST',keepalive:true});}
});

$('btnSave').onclick=function(){
  fetch('/api/save',{method:'POST'}).then(function(r){return r.json();})
    .then(function(d){toast(d.ok?'参数和标定表已保存':'保存失败');});
};
$('btnDefault').onclick=function(){
  if(!confirm('恢复默认参数？已保存的腿高标定表不受影响'))return;
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

function load(){
  fetch('/api/params').then(function(r){return r.json();}).then(function(d){
    ITEMS=d.items;build();
  }).catch(function(){toast('读参数失败');});
}
function poll(){
  fetch('/api/status').then(function(r){return r.json();}).then(function(s){
    set('sWifi',s.wifi);set('sCli',s.clients);
    set('sPitch',s.pitch.toFixed(1)+'°');
    set('sGyro',s.gyro.toFixed(1)+'°/s');
    set('sEnc',s.enc1+' / '+s.enc2);
    var m=$('sMotor');m.textContent=s.motor;
    m.className=s.motor==='停止'?'':'warn';
    /* 平衡控制状态 */
    var b=$('sBal');
    b.textContent=s.bal_state+(s.bal_on?'':' (关)');
    b.className=(s.bal_state==='BALANCING')?'':'warn';
    set('bState',s.bal_state);
    set('bMotA',s.mot_a);set('bMotB',s.mot_b);
    set('bTgt',s.tgt_pitch.toFixed(2)+'°');
    set('bOut',s.bal_out.toFixed(1)+'%');
    set('bSpd',s.spd_out.toFixed(2)+'°');
    set('bSteer',s.steer_out.toFixed(1)+'%');
    set('bSpeed',s.speed.toFixed(1)+' rpm');
    set('bLegDeg',s.leg_deg+'°');
    set('bLegH',s.leg_h>0?s.leg_h.toFixed(1)+'mm':'未标定');
    set('bLegComp',s.leg_comp.toFixed(2)+'°');
    set('bKpEff',s.kp_eff.toFixed(1));
    var lok=$('bLegOk');
    lok.textContent=s.leg_usable?'可用':'不可用';
    lok.style.color=s.leg_usable?'#5ad1a0':'#e8b04b';
    set('bLegK',s.leg_k.toFixed(3));
    /* 波形: 修正后 = 原始 + 零偏 - 腿角补偿, 反推出原始值显示 */
    var fix=s.pitch, raw=fix+s.leg_comp;
    pushWave(raw,fix);
  }).catch(function(){});
}
load();poll();lmLoad();setInterval(poll,500);
</script>
</body>
</html>
)HTML";
