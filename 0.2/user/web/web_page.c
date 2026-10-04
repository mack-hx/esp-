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
#toast{position:fixed;left:50%;bottom:26px;transform:translateX(-50%);background:#2f7d5c;color:#fff;padding:9px 18px;border-radius:20px;font-size:14px;opacity:0;transition:opacity .25s;pointer-events:none;z-index:99}
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
/* 默认展开的分组 (3 = 限幅组, 平时用不到, 默认收起) */
var OPEN={0:1,1:1,2:1,3:0,9:0};

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
  fetch('/api/save',{method:'POST'}).then(function(){toast('已保存到 NVS');});
};
$('btnDefault').onclick=function(){
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
  }).catch(function(){});
}
load();poll();setInterval(poll,500);
</script>
</body>
</html>
)HTML";
