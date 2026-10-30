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
<title>配网 - BalanceRobot</title>
<style>
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{margin:0;background:#12151a;color:#e6e8ec;font:16px/1.5 -apple-system,system-ui,"PingFang SC","Microsoft YaHei",sans-serif;padding:20px}
.wrap{max-width:420px;margin:0 auto}
h1{font-size:22px;margin:0 0 8px;color:#5ad1a0}
.sub{color:#8b93a3;font-size:13px;margin-bottom:24px}
label{display:block;margin:18px 0 6px;font-size:14px;color:#e6e8ec}
input{width:100%;padding:12px 14px;font-size:15px;background:#1b1f27;color:#e6e8ec;border:1px solid #2a303c;border-radius:8px;outline:none}
input:focus{border-color:#5ad1a0}
button{width:100%;padding:14px;margin-top:24px;font-size:16px;font-weight:600;background:#2f7d5c;color:#fff;border:0;border-radius:8px;cursor:pointer}
button:active{filter:brightness(1.25)}
button:disabled{opacity:.5}
.hint{font-size:12px;color:#8b93a3;margin-top:6px}
.warn{background:#3a2a12;border-left:3px solid #e8b04b;padding:10px 12px;border-radius:5px;font-size:13px;color:#f0d9ae;margin-top:18px}
#toast{position:fixed;left:50%;bottom:30px;transform:translateX(-50%);background:#2f7d5c;color:#fff;padding:10px 20px;border-radius:20px;font-size:14px;opacity:0;transition:opacity .25s;pointer-events:none;z-index:99}
</style>
</head>
<body>
<div class="wrap">
  <h1>机械狗配网</h1>
  <div class="sub">输入家里的 WiFi 名和密码, 设备会保存并自动重启.</div>

  <label>WiFi 名称 (SSID)</label>
  <input id="ssid" type="text" placeholder="例如: MyHomeWiFi" autocomplete="off">

  <label>WiFi 密码</label>
  <input id="pass" type="password" placeholder="至少 8 位, 开放 WiFi 留空" autocomplete="off">
  <div class="hint">仅支持 2.4GHz 频段 (ESP32 不支持 5GHz)</div>

  <button id="btnSave">保存并重启</button>

  <div class="warn">
    <b>注意:</b> 配网完成后此热点会消失, 设备会尝试连你填的 WiFi.
    失败的话, 设备会再次开此热点, 你可以重新填.
  </div>
</div>
<div id="toast"></div>

<script>
var $=function(id){return document.getElementById(id);};
var toast=function(t){var e=$('toast');e.textContent=t;e.style.opacity=1;
  setTimeout(function(){e.style.opacity=0;},2000);};

// 自动填一下上次填的 SSID
try{
  var saved=localStorage.getItem('last_ssid');
  if(saved)$('ssid').value=saved;
}catch(e){}

$('btnSave').onclick=function(){
  var s=$('ssid').value.trim();
  var p=$('pass').value;
  if(!s){toast('请填写 WiFi 名');$('ssid').focus();return;}
  if(p.length>0 && p.length<8){toast('密码至少 8 位, 或留空表示开放 WiFi');$('pass').focus();return;}
  try{localStorage.setItem('last_ssid',s);}catch(e){}
  $('btnSave').disabled=true;
  $('btnSave').textContent='保存中…';
  fetch('/api/wifi',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({ssid:s,pass:p})})
    .then(function(r){return r.json();})
    .then(function(d){
      if(d.ok){
        $('btnSave').textContent='已保存, 设备重启中…';
        toast('保存成功, 等待设备重启');
        setTimeout(function(){
          document.body.innerHTML='<div class="wrap"><h1 style="color:#e8b04b">已保存</h1>'+
            '<div class="sub">设备正在重启, 尝试连接 <b style="color:#5ad1a0">'+s+'</b>.<br><br>'+
            '如果连接成功, 此热点 <b>BalanceRobot_Setup</b> 会消失.<br>'+
            '如果连接失败, 设备会重新开此热点, 请回到本页面重新填.</div></div>';
        },1000);
      }else{
        toast('保存失败: '+(d.msg||'未知错误'));
        $('btnSave').disabled=false;
        $('btnSave').textContent='保存并重启';
      }
    }).catch(function(){
      toast('请求失败, 请重试');
      $('btnSave').disabled=false;
      $('btnSave').textContent='保存并重启';
    });
};
</script>
</body>
</html>
)HTML";
