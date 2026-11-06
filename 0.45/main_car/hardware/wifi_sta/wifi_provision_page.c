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
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<meta http-equiv="refresh" content="3600">
<title>配网 - BalanceRobot</title>
<style>
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{margin:0;background:#12151a;color:#e6e8ec;font:16px/1.5 -apple-system,system-ui,"PingFang SC","Microsoft YaHei",sans-serif;padding:20px}
.wrap{max-width:420px;margin:0 auto}
h1{font-size:22px;margin:0 0 8px;color:#5ad1a0}
h2{font-size:16px;margin:24px 0 8px;color:#e6e8ec;font-weight:600}
.sub{color:#8b93a3;font-size:13px;margin-bottom:16px}
label{display:block;margin:14px 0 6px;font-size:14px;color:#e6e8ec;font-weight:500}
input,select{width:100%;padding:10px 12px;font-size:15px;background:#1b1f27;color:#e6e8ec;border:1px solid #2a303c;border-radius:6px;outline:none}
input:focus,select:focus{border-color:#5ad1a0}
button{width:100%;padding:12px;margin-top:18px;font-size:16px;font-weight:600;background:#2f7d5c;color:#fff;border:0;border-radius:6px;cursor:pointer}
button:hover:not(:disabled){filter:brightness(1.1)}
button:active:not(:disabled){filter:brightness(1.2)}
button:disabled{opacity:.5;cursor:not-allowed}
button.alt{background:#3a4a52;margin-top:10px}
.hint{font-size:12px;color:#8b93a3;margin-top:4px}
.warn{background:#3a2a12;border-left:3px solid #e8b04b;padding:10px 12px;border-radius:5px;font-size:13px;color:#f0d9ae;margin-top:18px}
.tab{display:flex;gap:8px;margin-bottom:20px}
.tab button{margin:0;padding:10px 16px;background:#2a303c;color:#8b93a3;flex:1}
.tab button.active{background:#2f7d5c;color:#fff}
.tab-content{display:none}
.tab-content.active{display:block}
#toast{position:fixed;left:50%;bottom:30px;transform:translateX(-50%);background:#2f7d5c;color:#fff;padding:10px 20px;border-radius:20px;font-size:14px;opacity:0;transition:opacity .25s;pointer-events:none;z-index:99}
.success{background:#2f5c3a;border-left:3px solid #5ad1a0}
</style>
</head>
<body>
<div class="wrap">
  <h1>机械狗配网</h1>
  <div class="sub">连接 WiFi 和 OneNET 云平台</div>

  <div class="tab">
    <button class="tabBtn active" data-tab="manual">手动配置</button>
    <button class="tabBtn" data-tab="import">导入配置</button>
  </div>

  <!-- 标签页 1: 手动配置 -->
  <div id="manual" class="tab-content active">
    <h2>WiFi 信息</h2>
    <label>WiFi 名称</label>
    <select id="ssid" onchange="document.getElementById('ssidText').value=this.value">
      <option value="">-- 扫描可用网络 --</option>
    </select>
    <div class="hint" id="scanHint">点击按钮扫描周围 WiFi</div>
    <button class="alt" id="btnScan">扫描 WiFi</button>

    <label style="margin-top:14px">或手动输入 WiFi 名</label>
    <input id="ssidText" type="text" placeholder="例如: MyHomeWiFi" autocomplete="off">

    <label>WiFi 密码</label>
    <input id="pass" type="password" placeholder="留空表示开放 WiFi" autocomplete="off">
    <div class="hint">仅支持 2.4GHz (ESP32 限制)</div>

    <h2>OneNET MQTT 配置</h2>
    <label>产品 ID</label>
    <input id="pid" type="text" placeholder="例如: 6YlUqr9lue" autocomplete="off">
    <div class="hint">从 OneNET Studio 平台复制</div>

    <label>设备名</label>
    <input id="dev" type="text" placeholder="例如: balancing_dog" autocomplete="off">

    <label>AccessKey 密钥</label>
    <input id="ak" type="password" placeholder="Base64 密钥" autocomplete="off">
    <div class="hint">设备级的长期密钥</div>

    <button id="btnSave">保存配置</button>
  </div>

  <!-- 标签页 2: 导入配置 -->
  <div id="import" class="tab-content">
    <h2>从 App 导入</h2>
    <label>App 配置字符串</label>
    <textarea id="appConfig" style="width:100%;height:80px;padding:10px;background:#1b1f27;color:#e6e8ec;border:1px solid #2a303c;border-radius:6px;resize:none;font-family:monospace;font-size:12px" placeholder="粘贴从 App 复制的配置 JSON"></textarea>
    <button class="alt" id="btnImportApp">导入 App 配置</button>

    <h2 style="margin-top:24px">手机已保存的 WiFi</h2>
    <div class="hint">点击按钮读取系统 WiFi 列表</div>
    <button class="alt" id="btnGetPhoneWiFi">读取手机 WiFi</button>
    <div id="phoneWiFiList" style="margin-top:12px"></div>
  </div>

  <div class="warn">
    <b>注意:</b> 保存后设备将重启。连接成功后此热点消失; 失败则重新开启。
  </div>
</div>
<div id="toast"></div>

<script>
var $=function(id){return document.getElementById(id);};
var toast=function(t,ok){var e=$('toast');e.textContent=t;e.className=ok?'success':'';e.style.opacity=1;
  setTimeout(function(){e.style.opacity=0;},2500);};

// 标签切换
document.querySelectorAll('.tabBtn').forEach(function(btn){
  btn.onclick=function(){
    document.querySelectorAll('.tabBtn').forEach(function(b){b.classList.remove('active');});
    document.querySelectorAll('.tab-content').forEach(function(c){c.classList.remove('active');});
    btn.classList.add('active');
    $(btn.dataset.tab).classList.add('active');
  };
});

// 自动填上次 SSID
try{var s=localStorage.getItem('last_ssid');if(s)$('ssidText').value=s;}catch(e){}

// 扫描 WiFi
$('btnScan').onclick=function(){
  $('btnScan').disabled=true;$('btnScan').textContent='扫描中…';
  fetch('/api/wifi/scan').then(r=>r.json()).then(d=>{
    $('btnScan').disabled=false;$('btnScan').textContent='扫描 WiFi';
    if(d.networks){
      var html='';
      d.networks.forEach(function(net){
        html+='<option value="'+net.ssid+'">'+net.ssid+' (信号:'+net.rssi+')</option>';
      });
      $('ssid').innerHTML='<option>-- 选择 WiFi --</option>'+html;
      $('scanHint').textContent='找到 '+d.networks.length+' 个网络';
    }else toast('扫描失败',false);
  }).catch(function(){
    $('btnScan').disabled=false;$('btnScan').textContent='扫描 WiFi';
    toast('网络错误',false);
  });
};

// 导入 App 配置
$('btnImportApp').onclick=function(){
  var cfg=$('appConfig').value.trim();
  if(!cfg){toast('请粘贴配置字符串',false);return;}
  try{
    var obj=JSON.parse(cfg);
    if(obj.pid)$('pid').value=obj.pid;
    if(obj.dev)$('dev').value=obj.dev;
    if(obj.ak)$('ak').value=obj.ak;
    if(obj.ssid)$('ssidText').value=obj.ssid;
    if(obj.pass)$('pass').value=obj.pass;
    toast('导入成功',true);
    document.querySelector('.tabBtn').click();
  }catch(e){
    toast('配置格式错误',false);
  }
};

// 读取手机 WiFi
$('btnGetPhoneWiFi').onclick=function(){
  if(!navigator.permissions||!navigator.permissions.query){
    toast('浏览器不支持读取 WiFi',false);return;
  }
  var list=$('phoneWiFiList');list.innerHTML='<p style="color:#8b93a3">需要手机系统支持...</p>';
  // 实际上浏览器无法直接访问手机 WiFi 列表，这里只是占位
  // 真实场景下需要 App 通过其他方式 (蓝牙/QR码) 传递
};

// 保存配置
$('btnSave').onclick=function(){
  var s=$('ssidText').value.trim();
  var p=$('pass').value;
  var pid=$('pid').value.trim();
  var dev=$('dev').value.trim();
  var ak=$('ak').value.trim();

  if(!s){toast('请选择或输入 WiFi 名',false);$('ssidText').focus();return;}
  if(p.length>0 && p.length<8){toast('密码至少 8 位',false);$('pass').focus();return;}
  /* MQTT 字段可以留空，后续用 App 或配网页再配 */
  if(!pid || !dev || !ak){
    if(!confirm('MQTT 配置未完整，设备将无法连接云平台。\\n\\n可以先只配 WiFi，用 App 配网页稍后设置。\\n\\n确认继续?')){return;}
  }

  try{localStorage.setItem('last_ssid',s);}catch(e){}
  $('btnSave').disabled=true;
  $('btnSave').textContent='保存中…';

  fetch('/api/wifi',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({ssid:s,pass:p,pid:pid||'',dev:dev||'',ak:ak||''})})
    .then(r=>r.json())
    .then(d=>{
      if(d.ok){
        $('btnSave').textContent='已保存，重启中…';
        toast('✓ 配置已保存',true);
        setTimeout(function(){
          document.body.innerHTML='<div class="wrap"><h1 style="color:#5ad1a0">保存成功</h1>'+
            '<div class="sub">设备重启中, 尝试连接 <b>'+s+'</b> 热点...<br><br>'+
            '连接成功后此热点会消失。失败则 30 秒内重新开启。</div></div>';
        },1500);
      }else{
        toast('✗ 保存失败: '+(d.msg||'未知错误'),false);
        $('btnSave').disabled=false;
        $('btnSave').textContent='保存配置';
      }
    }).catch(()=>{
      toast('✗ 请求失败',false);
      $('btnSave').disabled=false;
      $('btnSave').textContent='保存配置';
    });
};
</script>
</body>
</html>
)HTML";
