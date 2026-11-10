/**
 * @file wifi_provision_page.c
 * @brief AP 配网页的 HTML (内嵌字符串)
 * @note  极简版: 一个表单填 SSID/密码, POST /api/wifi, 保存到 NVS 后 ESP32 重启
 * @author ESP32 机械狗项目
 */

#include "wifi_provision_page.h"

/* C11 raw string 避免引号转义地狱 */
const char WIFI_PROV_PAGE_HTML[] = R"HTML(<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>BalanceRobot</title>
<style>
body{margin:0;padding:20px;background:#111;color:#eee;font-family:system-ui,sans-serif}
.wrap{max-width:400px;margin:0 auto}
h1{color:#5ad1a0;margin:0 0 10px}
h2{font-size:16px;color:#eee;margin:20px 0 8px}
label{display:block;margin:12px 0 4px;font-size:14px}
input,select{width:100%;padding:10px;margin:4px 0 8px;background:#222;color:#eee;border:1px solid #444;border-radius:4px;font:inherit}
input:focus{border-color:#5ad1a0;outline:0}
button{width:100%;padding:10px;margin-top:12px;background:#2f7d5c;color:#fff;border:0;border-radius:4px;cursor:pointer;font-weight:600}
button:active{opacity:.8}
button:disabled{opacity:.5;cursor:not-allowed}
.hint{font-size:12px;color:#999;margin-top:2px}
.warn{background:#3a2a12;border-left:3px solid #e8b04b;padding:10px;margin-top:12px;font-size:12px}
.tab{display:flex;gap:6px;margin-bottom:16px}
.tab button{margin:0;padding:8px 12px;background:#333;color:#999;flex:1}
.tab button.active{background:#2f7d5c;color:#fff}
.tab-content{display:none}
.tab-content.active{display:block}
#toast{position:fixed;bottom:20px;left:50%;transform:translateX(-50%);background:#2f7d5c;color:#fff;padding:8px 16px;border-radius:16px;opacity:0;transition:opacity .2s}
</style>
</head>
<body>
<div class="wrap">
<h1>BalanceRobot</h1>

<div class="tab">
<button class="tabBtn active" data-tab="wifi">WiFi</button>
<button class="tabBtn" data-tab="mqtt">MQTT</button>
</div>

<div id="wifi" class="tab-content active">
<label>WiFi名</label>
<input id="ssid" type="text" placeholder="WiFi SSID">
<label>密码</label>
<input id="pass" type="password" placeholder="留空=开放网络">
<div class="hint">2.4GHz only</div>
</div>

<div id="mqtt" class="tab-content">
<label>产品ID</label>
<input id="pid" type="text" placeholder="Product ID">
<label>设备名</label>
<input id="dev" type="text" placeholder="Device Name">
<label>AccessKey</label>
<input id="ak" type="password" placeholder="Secret Key">
<div style="display:flex;gap:8px;margin-top:8px">
<button id="btnMain" style="margin:0;flex:1;background:#333">主车</button>
<button id="btnSub" style="margin:0;flex:1;background:#333">副车</button>
</div>
<div class="hint">拉取当前设备配置或载入已记录配置</div>
</div>

<button id="btnSave">保存</button>
<div class="warn">保存后设备将重启</div>
</div>
<div id="toast"></div>

<script>
var $=id=>document.getElementById(id);
var toast=t=>{let e=$('toast');e.textContent=t;e.style.opacity='1';setTimeout(()=>e.style.opacity='0',2e3);};
document.querySelectorAll('.tabBtn').forEach(btn=>{btn.onclick=()=>{
  document.querySelectorAll('.tabBtn').forEach(b=>b.classList.remove('active'));
  document.querySelectorAll('.tab-content').forEach(c=>c.classList.remove('active'));
  btn.classList.add('active');$(btn.dataset.tab).classList.add('active');};
});
try{let s=localStorage.getItem('ssid');if(s)$('ssid').value=s;}catch(e){}
$('btnMain').onclick=()=>{
  fetch('/api/cfg/get').then(r=>r.json()).then(d=>{
    if(d.pid)$('pid').value=d.pid;if(d.dev)$('dev').value=d.dev;if(d.ak)$('ak').value=d.ak;
    try{localStorage.setItem('rob_main',JSON.stringify({pid:d.pid,dev:d.dev,ak:d.ak}));}catch(e){}
    toast('已拉取主车配置');
  }).catch(()=>toast('拉取失败'));
};
$('btnSub').onclick=()=>{
  try{
    let sub=localStorage.getItem('rob_sub');
    if(sub){
      let d=JSON.parse(sub);
      if(d.pid)$('pid').value=d.pid;if(d.dev)$('dev').value=d.dev;if(d.ak)$('ak').value=d.ak;
      toast('已载入副车配置');
    }else{
      fetch('/api/cfg/get').then(r=>r.json()).then(d=>{
        if(d.pid)$('pid').value=d.pid;if(d.dev)$('dev').value=d.dev;if(d.ak)$('ak').value=d.ak;
        localStorage.setItem('rob_sub',JSON.stringify({pid:d.pid,dev:d.dev,ak:d.ak}));
        toast('已拉取副车配置');
      }).catch(()=>toast('拉取失败'));
    }
  }catch(e){toast('副车配置加载失败');}
};
$('btnSave').onclick=()=>{
  let s=$('ssid').value.trim(),p=$('pass').value,pid=$('pid').value.trim(),dev=$('dev').value.trim(),ak=$('ak').value.trim();
  if(!s){toast('请输入WiFi名');$('ssid').focus();return;}
  if(p.length>0&&p.length<8){toast('密码8位+');$('pass').focus();return;}
  try{localStorage.setItem('ssid',s);}catch(e){}
  $('btnSave').disabled=true;$('btnSave').textContent='保存中…';
  fetch('/api/wifi',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({ssid:s,pass:p,pid,dev,ak})})
    .then(r=>r.json()).then(d=>{
      if(d.ok){toast('✓ 保存成功，重启中…');setTimeout(()=>{document.body.innerHTML='<div class="wrap"><h1>已保存</h1><p>设备重启中，尝试连接'+s+'</p></div>';},1000);}
      else{toast('✗ 失败: '+(d.msg||'error'));$('btnSave').disabled=false;$('btnSave').textContent='保存';}
    }).catch(()=>{toast('✗ 网络错误');$('btnSave').disabled=false;$('btnSave').textContent='保存';});
};
</script>
</body>
</html>
)HTML";
