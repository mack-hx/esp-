/**
 * @file wifi_sta.c
 * @brief WiFi STA 模式 + AP 配网实现
 *
 * 流程:
 *   1. wifi_sta_start() 检查 NVS: 有 ssid+pass 走 STA, 没有走 AP 配网
 *   2. STA 连上后关闭配网页服务器 (释放 80 端口), onenet_client 可启动
 *   3. AP 模式下收到 /api/wifi POST → 存 NVS → 软重启
 *
 * @note  WiFi 事件处理用 esp_event 异步模式
 * @author ESP32 机械狗项目
 */

#include "wifi_sta.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include <string.h>
#include <stdio.h>

/* 配网页 HTML（内嵌） */
static const char WIFI_PROV_PAGE_HTML[] = R"HTML(<!DOCTYPE html>
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
input:focus,select:focus{border-color:#5ad1a0;outline:0}
button{width:100%;padding:10px;margin-top:12px;background:#2f7d5c;color:#fff;border:0;border-radius:4px;cursor:pointer;font-weight:600;font-size:14px}
button.alt{background:#333;margin-top:6px;padding:8px}
button:active{opacity:.8}
button:disabled{opacity:.5;cursor:not-allowed}
.hint{font-size:12px;color:#999;margin-top:2px}
.warn{background:#3a2a12;border-left:3px solid #e8b04b;padding:10px;margin-top:12px;font-size:12px}
.tab{display:flex;gap:6px;margin-bottom:16px}
.tab button{margin:0;padding:8px 12px;background:#333;color:#999;flex:1}
.tab button.active{background:#2f7d5c;color:#fff}
.tab-content{display:none}
.tab-content.active{display:block}
.wifilist{max-height:200px;overflow-y:auto;border:1px solid #444;border-radius:4px;background:#222;margin-top:6px}
.wifiitem{padding:8px;border-bottom:1px solid #333;cursor:pointer}
.wifiitem:last-child{border-bottom:0}
.wifiitem:hover{background:#333}
.wifissid{font-weight:600}
.wifisignal{font-size:11px;color:#999}
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
<button class="alt" id="btnScan">扫描WiFi</button>
<label style="margin-top:12px">密码</label>
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
<button class="alt" id="btnFromApp" style="width:100%;margin-top:12px">从App导入配置</button>
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
$('btnScan').onclick=()=>{
  $('btnScan').disabled=true;$('btnScan').textContent='扫描中…';
  fetch('/api/wifi/scan').then(r=>r.json()).then(d=>{
    $('btnScan').disabled=false;$('btnScan').textContent='扫描WiFi';
    if(d.networks&&d.networks.length){
      let list=document.createElement('div');list.className='wifilist';
      d.networks.forEach(net=>{
        let item=document.createElement('div');item.className='wifiitem';
        item.innerHTML='<div class="wifissid">'+net.ssid+'</div><div class="wifisignal">信号: '+net.rssi+' dBm</div>';
        item.onclick=()=>{$('ssid').value=net.ssid;toast('已选: '+net.ssid);};
        list.appendChild(item);
      });
      let old=document.querySelector('.wifilist');if(old)old.remove();
      $('btnScan').parentNode.insertBefore(list, $('btnScan').nextSibling);
      toast('扫到 '+d.networks.length+' 个');
    }else toast('扫描失败');
  }).catch(()=>{$('btnScan').disabled=false;$('btnScan').textContent='扫描WiFi';toast('网络错误');});
};
$('btnFromApp').onclick=()=>{
  fetch('/api/cfg/get').then(r=>{
    if(!r.ok){toast('✗ 服务端错误: '+r.status);return;}
    return r.json();
  }).then(d=>{
    if(!d){toast('✗ JSON 解析失败');return;}
    if(d.pid||d.dev||d.ak){
      $('pid').value=d.pid||'';
      $('dev').value=d.dev||'';
      $('ak').value=d.ak||'';
      toast('✓ 已从App导入MQTT配置');
    }else{
      toast('⚠ App未配置OneNet');
    }
  }).catch(e=>{
    toast('✗ 导入失败: '+e.message);
  });
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
)HTML";

static const char *TAG = "WIFI_STA";

/* ========== 状态机 ========== */
static wifi_sta_state_t s_state = WIFI_STA_STOPPED;
static int s_retry_num = 0;
static EventGroupHandle_t s_wifi_event_group = NULL;

/* WiFi 事件组的位定义 */
#define WIFI_STA_CONNECTED_BIT   BIT0   /* 已连上 */
#define WIFI_STA_FAIL_BIT        BIT1   /* 重试超限失败 */

/* STA 模式重试次数 - 需要足够多以便 DHCP 有时间获取 IP */
#define WIFI_STA_MAX_RETRY       15

/* STA 模式拿到的 IP (连上后填, 没连上时全 0) */
static esp_netif_ip_info_t s_sta_ip = {0};
static char s_sta_ssid[33] = {0};

/* AP 模式下的 netif 句柄 (创建后持有, 关闭时清理) */
static esp_netif_t *s_ap_netif = NULL;

/* 配网页 httpd 实例 (AP 模式下持有, STA 模式下销毁) */
static httpd_handle_t s_prov_httpd = NULL;

/* 真实生效的 AP SSID (基础名 + MAC 后两字节, 避免多台设备同 SSID) */
static char s_ap_ssid[32] = {0};

/* ========== NVS 读写 ========== */

/**
 * @brief 从 NVS 读保存的 SSID 和密码
 * @return true=读到了完整 ssid+pass
 */
static bool load_credentials_from_nvs(char *ssid, size_t ssid_len,
                                      char *pass, size_t pass_len)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(WIFI_STA_NVS_NS, NVS_READONLY, &h);
    if (err != ESP_OK) {
        return false;
    }

    size_t s_len = ssid_len, p_len = pass_len;
    err = nvs_get_str(h, WIFI_STA_NVS_KEY_SSID, ssid, &s_len);
    if (err != ESP_OK) {
        nvs_close(h);
        return false;
    }
    err = nvs_get_str(h, WIFI_STA_NVS_KEY_PASS, pass, &p_len);
    nvs_close(h);
    if (err != ESP_OK) {
        return false;
    }
    return (strlen(ssid) > 0);
}

/**
 * @brief 把 SSID+密码写 NVS (用单独 handle 避免读写冲突)
 * @return ESP_OK 成功
 */
static esp_err_t save_credentials_to_nvs_impl(const char *ssid, const char *pass)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(WIFI_STA_NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open 失败: %s", esp_err_to_name(err));
        return err;
    }
    err = nvs_set_str(h, WIFI_STA_NVS_KEY_SSID, ssid);
    if (err == ESP_OK) {
        err = nvs_set_str(h, WIFI_STA_NVS_KEY_PASS, pass);
    }
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

/**
 * @brief 公开 API: 保存 WiFi 凭证 (用于 BLE 和配网页)
 */
esp_err_t wifi_sta_save_credentials(const char *ssid, const char *pass)
{
    return save_credentials_to_nvs_impl(ssid, pass);
}

/**
 * @brief 清除 WiFi 配置，重启后进入 AP 模式（用于重新配网）
 */
void wifi_sta_force_ap_mode(void)
{
    nvs_handle_t h;
    if (nvs_open(WIFI_STA_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_key(h, WIFI_STA_NVS_KEY_SSID);
        nvs_erase_key(h, WIFI_STA_NVS_KEY_PASS);
        nvs_commit(h);
        nvs_close(h);
        ESP_LOGI(TAG, "已清除配置，15s 后重启进入 AP 模式");
    }
}

/* ========== 配网页 HTTP handler ========== */

/**
 * @brief GET / → 返回配网页 HTML
 */
static esp_err_t prov_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, WIFI_PROV_PAGE_HTML, HTTPD_RESP_USE_STRLEN);
}

/**
 * @brief 从 JSON 里取一个字符串字段
 * @param json 原始 JSON
 * @param key  字段名
 * @param out  [出参] 取出值
 * @param cap  out 大小
 * @return true=找到
 */
static bool json_str(const char *json, const char *key, char *out, size_t cap)
{
    if (out == NULL || cap == 0) return false;
    out[0] = '\0';

    char pat[24];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char *p = strstr(json, pat);
    if (p == NULL) return false;
    p += strlen(pat);
    while (*p == ' ' || *p == ':' || *p == '\t') p++;
    if (*p != '"') return false;
    p++;
    size_t i = 0;
    while (*p && *p != '"' && i < cap - 1) {
        /* JSON 中 \\ 和 \" 要反转义, 简化版: 遇到反斜杠跳过下一字符 */
        if (*p == '\\' && *(p + 1)) {
            p += 2;
            continue;
        }
        out[i++] = *p++;
    }
    out[i] = '\0';
    return (i > 0);
}

/**
 * @brief GET /api/wifi/scan → 扫描周围 WiFi，返回 JSON 数组
 */
static esp_err_t prov_get_wifi_scan_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json; charset=utf-8");
    
    /* 触发 WiFi 扫描 */
    wifi_scan_config_t config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = true,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 0,
        .scan_time.active.max = 150
    };
    esp_wifi_scan_start(&config, true);
    
    uint16_t ap_count = 0;
    esp_wifi_scan_get_ap_num(&ap_count);
    
    char *resp = malloc(4096);
    if (!resp) {
        httpd_resp_sendstr(req, "{\"networks\":[],\"error\":\"memory\"}");
        return ESP_OK;
    }
    
    strcpy(resp, "{\"networks\":[");
    wifi_ap_record_t *ap_list = malloc(ap_count * sizeof(wifi_ap_record_t));
    if (ap_list) {
        esp_wifi_scan_get_ap_records(&ap_count, ap_list);
        for (int i = 0; i < ap_count && i < 30; i++) {
            if (i > 0) strcat(resp, ",");
            char item[256] = {0};
            snprintf(item, sizeof(item), 
                "{\"ssid\":\"%s\",\"rssi\":%d,\"channel\":%d}",
                ap_list[i].ssid, ap_list[i].rssi, ap_list[i].primary);
            strcat(resp, item);
        }
        free(ap_list);
    }
    strcat(resp, "]}");
    
    httpd_resp_sendstr(req, resp);
    free(resp);
    esp_wifi_scan_stop();
    return ESP_OK;
}

/**
 * @brief Captive Portal: 用户访问任何网址都跳到配网页根目录
 *        浏览器会自动检测连接检查网址 (如 connectivitycheck 开头或 captive 开头等),
 *        自动重定向到配网页
 */
static esp_err_t prov_captive_portal_handler(httpd_req_t *req)
{
    /* 跳过 API 请求，让特定处理器接管 */
    if (strncmp(req->uri, "/api/", 5) == 0) {
        return ESP_FAIL;  /* 让 httpd 返回 404 */
    }
    
    /* 其他请求重定向到首页（Captive Portal） */
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, "", 0);
    return ESP_OK;
}

/**
 * @brief POST /api/wifi → 收 {ssid, pass, pid, dev, ak}, 存 NVS, 返回 ok
 *        之后会软重启
 */
static esp_err_t prov_post_wifi_handler(httpd_req_t *req)
{
    char buf[512] = {0};
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, "{\"ok\":false,\"msg\":\"empty body\"}", -1);
        return ESP_OK;
    }
    buf[ret] = '\0';

    char ssid[33] = {0}, pass[65] = {0}, pid[32] = {0}, dev[64] = {0}, ak[128] = {0};
    if (!json_str(buf, "ssid", ssid, sizeof(ssid))) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, "{\"ok\":false,\"msg\":\"missing ssid\"}", -1);
        return ESP_OK;
    }
    json_str(buf, "pass", pass, sizeof(pass));   /* pass 可空 (开放 WiFi) */
    json_str(buf, "pid", pid, sizeof(pid));      /* OneNET pid */
    json_str(buf, "dev", dev, sizeof(dev));      /* OneNET dev */
    json_str(buf, "ak", ak, sizeof(ak));         /* OneNET ak */

    /* 保存 WiFi 配置 */
    esp_err_t err = save_credentials_to_nvs_impl(ssid, pass);
    if (err != ESP_OK) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, "{\"ok\":false,\"msg\":\"nvs save failed\"}", -1);
        return ESP_OK;
    }

    /* 保存 OneNET 配置 (如果提供了的话) */
    if (pid[0] || dev[0] || ak[0]) {
        extern esp_err_t onenet_config_save(const char *pid, const char *dev, const char *ak);
        /* 只传非空的字段，NVS 会保留之前的值 */
        err = onenet_config_save(pid[0] ? pid : NULL, dev[0] ? dev : NULL, ak[0] ? ak : NULL);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "OneNET 配置保存失败: %s (WiFi 已保存，继续重启)", esp_err_to_name(err));
        }
    }

    ESP_LOGI(TAG, "已保存配置: SSID=%s, pid=%s, dev=%s, ak=%s, 重启中...", 
             ssid, pid[0] ? "已设置" : "空", dev[0] ? "已设置" : "空", ak[0] ? "已设置" : "空");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"ok\":true}", -1);

    /* 延迟 500ms 再重启, 让 HTTP 响应飞回浏览器 */
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;  /* 不会到这里 */
}

/**
 * @brief 简单 DNS 服务器: 所有查询都返回 192.168.4.1 (用于 Captive Portal)
 */

#include "lwip/netdb.h"
#include "lwip/err.h"
#include "lwip/udp.h"
#include "lwip/dns.h"

#define DNS_PORT 53

static struct udp_pcb *dns_pcb = NULL;
static TaskHandle_t dns_task = NULL;

/** DNS 消息头结构 */
typedef struct {
    uint16_t id;
    uint16_t flags;
    uint16_t qdcount;
    uint16_t ancount;
    uint16_t nscount;
    uint16_t arcount;
} dns_hdr_t;

static void dns_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                             const ip_addr_t *addr, u16_t port)
{
    /* 极简 DNS 应答: 无脑返回 192.168.4.1
     * 这是 Captive Portal 的标准做法，让手机自动打开配网页 */
    if (!p || p->len < 12) {
        pbuf_free(p);
        return;
    }

    uint8_t *buf = (uint8_t *)p->payload;
    dns_hdr_t *req = (dns_hdr_t *)buf;
    uint16_t id = req->id;
    uint16_t flags = 0x8480;  /* 响应 + 标准查询 + 权威应答 */

    /* 构造响应包 */
    static uint8_t resp[512];
    int pos = 0;

    /* 头 */
    memcpy(resp + pos, &id, 2); pos += 2;
    uint16_t f = htons(flags);
    memcpy(resp + pos, &f, 2); pos += 2;
    uint16_t qdcount = ntohs(req->qdcount);
    uint16_t c = htons(1); memcpy(resp + pos, &c, 2); pos += 2;  /* ancount */
    c = htons(0); memcpy(resp + pos, &c, 2); pos += 2;  /* nscount */
    memcpy(resp + pos, &c, 2); pos += 2;  /* arcount */

    /* 复制查询域名 (不解析, 直接抄) */
    int qlen = 0;
    uint8_t *q = buf + 12;
    while (qlen < p->len - 12 - 4 && q[qlen] != 0) {
        if (q[qlen] > 63) break;
        qlen += 1 + q[qlen];
    }
    if (qlen < p->len - 12 - 4) qlen += 5;  /* 类型 + 类 */
    if (pos + qlen < (int)sizeof(resp)) {
        memcpy(resp + pos, q, qlen);
        pos += qlen;
    }

    /* 应答记录: 指向查询域名的指针 + 类型 A + 类 IN + TTL + 长度 + IP */
    if (pos + 16 < (int)sizeof(resp)) {
        uint16_t ptr = htons(0xc00c);  /* 指针指向位置 12 */
        memcpy(resp + pos, &ptr, 2); pos += 2;
        uint16_t type_class = htons((1 << 8) | 1);  /* A (1) + IN (1) */
        memcpy(resp + pos, &type_class, 2); pos += 2;
        uint32_t ttl = htonl(60);
        memcpy(resp + pos, &ttl, 4); pos += 4;
        uint16_t rdlen = htons(4);
        memcpy(resp + pos, &rdlen, 2); pos += 2;
        uint8_t ipv4[] = {192, 168, 4, 1};
        memcpy(resp + pos, ipv4, 4); pos += 4;
    }

    struct pbuf *pb = pbuf_alloc(PBUF_TRANSPORT, pos, PBUF_RAM);
    if (pb) {
        memcpy(pb->payload, resp, pos);
        udp_sendto(pcb, pb, addr, port);
        pbuf_free(pb);
    }
    pbuf_free(p);
}

static esp_err_t start_dns_spoofing(void)
{
    if (dns_pcb) return ESP_OK;

    dns_pcb = udp_new_ip_type(IPADDR_TYPE_V4);
    if (!dns_pcb) {
        ESP_LOGE(TAG, "DNS PCB 创建失败");
        return ESP_FAIL;
    }

    esp_err_t err = udp_bind(dns_pcb, IP_ADDR_ANY, DNS_PORT);
    if (err != ERR_OK) {
        ESP_LOGE(TAG, "DNS 端口绑定失败: %d", err);
        udp_remove(dns_pcb);
        dns_pcb = NULL;
        return ESP_FAIL;
    }

    udp_recv(dns_pcb, dns_recv_callback, NULL);
    ESP_LOGI(TAG, "DNS 劫持已启动 (所有查询 → 192.168.4.1, 用于 Captive Portal)");
    return ESP_OK;
}

static void stop_dns_spoofing(void)
{
    if (dns_pcb) {
        udp_remove(dns_pcb);
        dns_pcb = NULL;
        ESP_LOGI(TAG, "DNS 劫持已关闭");
    }
}

/**
 * @brief GET /api/cfg/get → 返回已保存的 WiFi 和 MQTT 配置 (用于 App 导入)
 */
static esp_err_t prov_get_config_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json; charset=utf-8");
    
    char ssid[33] = {0}, pass[65] = {0};
    char pid[32] = {0}, dev[64] = {0}, ak[128] = {0};
    
    /* 读 WiFi 配置 */
    load_credentials_from_nvs(ssid, sizeof(ssid), pass, sizeof(pass));
    
    /* 读 OneNet 配置 */
    extern void onenet_config_load(char *pid, size_t pid_len, 
                                   char *dev, size_t dev_len, 
                                   char *ak, size_t ak_len);
    onenet_config_load(pid, sizeof(pid), dev, sizeof(dev), ak, sizeof(ak));
    
    char resp[512] = {0};
    snprintf(resp, sizeof(resp), 
             "{\"ssid\":\"%s\",\"pass\":\"%s\",\"pid\":\"%s\",\"dev\":\"%s\",\"ak\":\"%s\"}",
             ssid[0] ? ssid : "", 
             pass[0] ? pass : "",
             pid[0] ? pid : "", 
             dev[0] ? dev : "", 
             ak[0] ? ak : "");
    
    httpd_resp_sendstr(req, resp);
    return ESP_OK;
}

/**
 * @brief 启动配网页 HTTP server (AP 模式下)
 */
static esp_err_t start_provision_httpd(void)
{
    if (s_prov_httpd != NULL) return ESP_OK;  /* 已在跑 */

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.uri_match_fn = httpd_uri_match_wildcard;
    cfg.max_uri_handlers = 8;
    cfg.stack_size = 8192;

    esp_err_t err = httpd_start(&s_prov_httpd, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "配网页 httpd 启动失败: %s", esp_err_to_name(err));
        return err;
    }

    /* GET / → 配网页 */
    httpd_uri_t uri_get = {
        .uri = "/", .method = HTTP_GET,
        .handler = prov_get_handler, .user_ctx = NULL
    };
    httpd_register_uri_handler(s_prov_httpd, &uri_get);

    /* GET / 也支持兜底, 浏览器没填路径时会自动跳到根 */
    httpd_uri_t uri_index = {
        .uri = "/index.html", .method = HTTP_GET,
        .handler = prov_get_handler, .user_ctx = NULL
    };
    httpd_register_uri_handler(s_prov_httpd, &uri_index);

    /* GET /api/wifi/scan → 扫描结果 */
    httpd_uri_t uri_scan = {
        .uri = "/api/wifi/scan", .method = HTTP_GET,
        .handler = prov_get_wifi_scan_handler, .user_ctx = NULL
    };
    httpd_register_uri_handler(s_prov_httpd, &uri_scan);

    /* POST /api/wifi → 收配置 */
    httpd_uri_t uri_post = {
        .uri = "/api/wifi", .method = HTTP_POST,
        .handler = prov_post_wifi_handler, .user_ctx = NULL
    };
    httpd_register_uri_handler(s_prov_httpd, &uri_post);

    /* GET /api/cfg/get → 返回已保存配置 (用于 App 导入) */
    httpd_uri_t uri_cfg_get = {
        .uri = "/api/cfg/get", .method = HTTP_GET,
        .handler = prov_get_config_handler, .user_ctx = NULL
    };
    httpd_register_uri_handler(s_prov_httpd, &uri_cfg_get);

    /* Captive Portal: 兜底所有其他请求，重定向到根 */
    httpd_uri_t uri_catchall = {
        .uri = "/*", .method = HTTP_GET,
        .handler = prov_captive_portal_handler, .user_ctx = NULL
    };
    httpd_register_uri_handler(s_prov_httpd, &uri_catchall);

    ESP_LOGI(TAG, "配网页已启动: http://" WIFI_STA_AP_IP "/");
    return ESP_OK;
}

/**
 * @brief 关闭配网页 HTTP server (STA 模式下)
 */
static void stop_provision_httpd(void) __attribute__((unused));
static void stop_provision_httpd(void)
{
    if (s_prov_httpd == NULL) return;
    httpd_stop(s_prov_httpd);
    s_prov_httpd = NULL;
    stop_dns_spoofing();
    ESP_LOGI(TAG, "配网页已关闭");
}

/* ========== AP 模式 (配网用) ========== */

/**
 * @brief 启动 AP 模式 + 配网页
 * @note  使用 AP+STA 混合模式以支持 WiFi 扫描 (纯 AP-only 无法扫描周围网络)
 */
static esp_err_t start_ap_mode(void)
{
    ESP_LOGI(TAG, "NVS 无 WiFi 配置, 进入 AP 配网模式 (AP+STA 混合以支持扫描)");

    s_state = WIFI_STA_AP_MODE;

    /* 拼一个带 MAC 后缀的 SSID, 避免多台设备同名热点 */
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    snprintf(s_ap_ssid, sizeof(s_ap_ssid), "%s_%02X%02X",
             WIFI_STA_AP_SSID, mac[4], mac[5]);

    /* 使用 AP+STA 混合模式以支持 WiFi 扫描 */
    esp_err_t ret = esp_wifi_set_mode(WIFI_MODE_APSTA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_mode(AP+STA) 失败: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 先创建 AP netif, 再设置 IP 和 WiFi config */
    s_ap_netif = esp_netif_create_default_wifi_ap();
    if (s_ap_netif == NULL) {
        ESP_LOGE(TAG, "创建 AP netif 失败");
        return ESP_FAIL;
    }

    /* 创建 STA netif (用于混合模式扫描, 不会主动连接任何网络) */
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    if (sta_netif == NULL) {
        ESP_LOGW(TAG, "创建 STA netif 失败 (非致命, 但扫描可能受限)");
    }

    /* create_default_wifi_ap() 已自动起 DHCP server, 必须先停 DHCP 再设 IP,
     * 否则 esp_netif_set_ip_info() 返回 ESP_ERR_ESP_NETIF_DHCP_NOT_STOPPED,
     * AP 启动流程会中断。 */
    esp_netif_dhcps_stop(s_ap_netif);

    /* 固定 IP 192.168.4.1
     * esp_ip4addr_aton() 返回 uint32_t 网络序数值, 赋给 esp_ip4_addr_t 字段 */
    esp_netif_ip_info_t ipinfo = {0};
    ipinfo.ip.addr      = esp_ip4addr_aton((const char *)WIFI_STA_AP_IP);
    ipinfo.netmask.addr = esp_ip4addr_aton((const char *)"255.255.255.0");
    ipinfo.gw.addr      = esp_ip4addr_aton((const char *)WIFI_STA_AP_IP);
    ret = esp_netif_set_ip_info(s_ap_netif, &ipinfo);
    if (ret != ESP_OK) {
        /* 软失败: 继续往下走, 至少保证 esp_wifi_start() 被调用, 热点能广播出来 */
        ESP_LOGW(TAG, "设置 AP IP 失败: %s (继续, 用默认 192.168.4.1)", esp_err_to_name(ret));
    }

    /* 显式启动 DHCP server */
    ret = esp_netif_dhcps_start(s_ap_netif);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "DHCP server 启动失败: %s (可能已自动启用)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "DHCP server 已启动");
    }

    /* 配置 AP WiFi 参数 - WPA2 + 固定密码 */
    wifi_config_t wc = {0};
    strlcpy((char *)wc.ap.ssid, s_ap_ssid, sizeof(wc.ap.ssid));
    wc.ap.ssid_len = strlen(s_ap_ssid);
    wc.ap.channel = WIFI_STA_AP_CHANNEL;
    wc.ap.max_connection = WIFI_STA_AP_MAX_CONN;
    /* 用 WPA2 + 固定密码 (和 WIFI_STA_AP_PASSWORD / 日志提示保持一致)。
     * 不用开放网络: 部分手机会隐藏开放热点, 搜不到。 */
    wc.ap.authmode = WIFI_AUTH_WPA2_PSK;
    strlcpy((char *)wc.ap.password, WIFI_STA_AP_PASSWORD, sizeof(wc.ap.password));
    wc.ap.beacon_interval = 100;

    ret = esp_wifi_set_config(WIFI_IF_AP, &wc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_config(AP) 失败: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "配网热点配置完成: SSID=%s  密码=%s", s_ap_ssid, WIFI_STA_AP_PASSWORD);

    /* 启动 DNS 劫持 (Captive Portal 自动弹网页) */
    start_dns_spoofing();

    /* 起配网页 */
    return start_provision_httpd();
}

/* ========== STA 模式 (连家里 WiFi) ========== */

/**
 * @brief 启动 STA 模式连家里 WiFi
 * @note  使用纯 STA 模式确保 DHCP 能正常工作
 */
static esp_err_t start_sta_mode(const char *ssid, const char *pass)
{
    ESP_LOGI(TAG, ">>> start_sta_mode() 被调用, SSID=%s", ssid);
    s_state = WIFI_STA_CONNECTING;
    s_retry_num = 0;
    strlcpy(s_sta_ssid, ssid, sizeof(s_sta_ssid));
    memset(&s_sta_ip, 0, sizeof(s_sta_ip));

    /* 直接使用 esp_netif_create_default_wifi_sta() 创建 netif
     * ESP-IDF 会自动处理 DHCP client */
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    if (sta_netif == NULL) {
        ESP_LOGE(TAG, "创建STA netif失败");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "STA netif 创建成功");

    /* 设置纯 STA 模式 */
    esp_err_t ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_mode(STA) 失败: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "WiFi 模式设置为 STA");

    /* 配置 STA WiFi 参数 */
    wifi_config_t sta_wc = {0};
    strlcpy((char *)sta_wc.sta.ssid, ssid, sizeof(sta_wc.sta.ssid));
    strlcpy((char *)sta_wc.sta.password, pass, sizeof(sta_wc.sta.password));

    /* 设置 BSSID 自动模式 (默认) */
    sta_wc.sta.bssid_set = false;

    /* 设置认证模式 */
    if (pass == NULL || pass[0] == '\0') {
        sta_wc.sta.threshold.authmode = WIFI_AUTH_OPEN;
    } else {
        sta_wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    }

    ret = esp_wifi_set_config(WIFI_IF_STA, &sta_wc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_config(STA) 失败: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "WiFi 配置完成, SSID=%s, 等待连接...", ssid);
    return ESP_OK;
}

/* ========== WiFi 事件处理 ========== */

/**
 * @brief WiFi 事件回调
 *        - STA 连上 → 拿 IP → 关配网页 (AP 仍保留做兜底)
 *        - STA 连失败 → 重试, 超过次数进 FAILED
 *        - AP 模式客户端连/断 → 打印日志
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    ESP_LOGI(TAG, "WiFi事件: base=%s, id=%ld", event_base, (long)event_id);

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "STA 启动, 开始连接...");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_STA_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGW(TAG, "STA 断连, 重试 (%d/%d)", s_retry_num, WIFI_STA_MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_STA_FAIL_BIT);
            s_state = WIFI_STA_FAILED;
            ESP_LOGE(TAG, "STA 重试超限, 连接失败");
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        /* WiFi 已连接，等待 DHCP 自动获取 IP */
        ESP_LOGI(TAG, "WiFi已连接, 等待DHCP获取IP...");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)event_data;
        s_sta_ip = e->ip_info;
        ESP_LOGI(TAG, "★ STA 已连上, IP: " IPSTR, IP2STR(&e->ip_info.ip));
        s_retry_num = 0;
        s_state = WIFI_STA_CONNECTED;
        xEventGroupSetBits(s_wifi_event_group, WIFI_STA_CONNECTED_BIT);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *e = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "AP 客户端连入: " MACSTR, MAC2STR(e->mac));
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *e = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "AP 客户端断开: " MACSTR, MAC2STR(e->mac));
    } else {
        ESP_LOGW(TAG, "未知WiFi事件: base=%s, id=%ld", event_base, (long)event_id);
    }
}

/* ========== 公共 API ========== */

esp_err_t wifi_sta_start(void)
{
    ESP_LOGI(TAG, "wifi_sta_start called, state=%d", s_state);

    if (s_state != WIFI_STA_STOPPED) {
        ESP_LOGW(TAG, "已在运行 (state=%d), 不重复启动", s_state);
        return ESP_OK;
    }

    /* 1. 初始化 TCP/IP + 默认事件循环 (幂等, main 已调过也 OK) */
    esp_err_t ret = esp_netif_init();
    if (ret != ESP_OK) return ret;
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) return ret;

    /* 2. 创建事件组 (STA 等待用) */
    s_wifi_event_group = xEventGroupCreate();
    if (s_wifi_event_group == NULL) return ESP_ERR_NO_MEM;

    /* 3. 注册事件 */
    ret = esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WIFI事件注册失败: %s", esp_err_to_name(ret));
    }
    ret = esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IP事件注册失败: %s", esp_err_to_name(ret));
    }
    ESP_LOGI(TAG, "WiFi事件处理器已注册, state=%d", s_state);

    /* 4. WiFi 驱动初始化 */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init 失败: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 5. 查 NVS, 决定走 STA 还是 AP, 然后设置模式并启动 */
    char ssid[33] = {0}, pass[65] = {0};
    if (load_credentials_from_nvs(ssid, sizeof(ssid), pass, sizeof(pass))) {
        /* 已有 WiFi 配置 → STA 模式
         * 顺序: 先创建netif, 再设置模式, 最后启动 */
        ret = start_sta_mode(ssid, pass);
        if (ret != ESP_OK) return ret;
        ret = esp_wifi_set_mode(WIFI_MODE_STA);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "esp_wifi_set_mode(STA) 失败: %s", esp_err_to_name(ret));
            return ret;
        }
    } else {
        /* 首次使用或被清空 → AP 配网模式 */
        ret = start_ap_mode();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi 模式启动失败: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 6. 启动 WiFi 驱动 */
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start 失败: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "esp_wifi_start() 成功, WiFi驱动已启动");

    return ESP_OK;
}

wifi_sta_state_t wifi_sta_get_state(void)
{
    return s_state;
}
esp_err_t wifi_sta_get_ip(esp_netif_ip_info_t *info)
{
    if (info == NULL) return ESP_ERR_INVALID_ARG;
    if (s_state != WIFI_STA_CONNECTED) return ESP_ERR_INVALID_STATE;
    *info = s_sta_ip;
    return ESP_OK;
}

esp_err_t wifi_sta_get_ssid(char *buf, uint32_t len)
{
    if (buf == NULL || len == 0) return ESP_ERR_INVALID_ARG;
    if (s_state != WIFI_STA_CONNECTED) return ESP_ERR_INVALID_STATE;
    strlcpy(buf, s_sta_ssid, len);
    return ESP_OK;
}

/**
 * @brief 启动 AP 模式 + 配网页 (用于运行时切换配网)
 * @note  从 STA 模式切换时会关闭 STA，切换到纯 AP 模式
 */
/**
 * @brief 获取当前 AP 配网热点的真实 SSID (含 MAC 后缀)
 * @param buf 输出缓冲区
 * @param len 缓冲区大小
 * @return ESP_OK 成功, ESP_ERR_INVALID_STATE 当前不在 AP 模式
 */
esp_err_t wifi_sta_get_ap_ssid(char *buf, uint32_t len)
{
    if (buf == NULL || len == 0) return ESP_ERR_INVALID_ARG;
    if (s_state != WIFI_STA_AP_MODE) return ESP_ERR_INVALID_STATE;
    strlcpy(buf, s_ap_ssid, len);
    return ESP_OK;
}

esp_err_t wifi_sta_start_ap(void)
{
    ESP_LOGI(TAG, "切换到 AP 配网模式");

    /* 停止现有 STA 连接 */
    if (s_state == WIFI_STA_CONNECTED || s_state == WIFI_STA_CONNECTING) {
        esp_wifi_disconnect();
    }

    /* 先创建 AP netif (必须先于 esp_wifi_set_mode) */
    if (s_ap_netif == NULL) {
        s_ap_netif = esp_netif_create_default_wifi_ap();
        if (s_ap_netif == NULL) {
            ESP_LOGE(TAG, "创建 AP netif 失败");
            return ESP_FAIL;
        }
    }

    /* 先停止DHCP，确保干净 */
    esp_netif_dhcps_stop(s_ap_netif);

    /* 设置 AP IP (192.168.4.1) */
    esp_netif_ip_info_t ipinfo = {0};
    ipinfo.ip.addr      = esp_ip4addr_aton((const char *)"192.168.4.1");
    ipinfo.netmask.addr = esp_ip4addr_aton((const char *)"255.255.255.0");
    ipinfo.gw.addr      = esp_ip4addr_aton((const char *)"192.168.4.1");
    esp_err_t ret = esp_netif_set_ip_info(s_ap_netif, &ipinfo);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "设置 AP IP 失败: %s", esp_err_to_name(ret));
    }

    /* 启动 DHCP server */
    ret = esp_netif_dhcps_start(s_ap_netif);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "DHCP server 启动失败: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "DHCP server 已启动");
    }

    /* 拼一个带 MAC 后缀的 SSID */
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    snprintf(s_ap_ssid, sizeof(s_ap_ssid), "%s_%02X%02X",
             WIFI_STA_AP_SSID, mac[4], mac[5]);

    /* 切换到 AP-only 模式 */
    ret = esp_wifi_set_mode(WIFI_MODE_AP);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_mode(AP) 失败: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 配置 AP WiFi 参数 - WPA2 + 固定密码 */
    wifi_config_t wc = {0};
    strlcpy((char *)wc.ap.ssid, s_ap_ssid, sizeof(wc.ap.ssid));
    wc.ap.ssid_len = strlen(s_ap_ssid);
    wc.ap.channel = WIFI_STA_AP_CHANNEL;
    wc.ap.max_connection = WIFI_STA_AP_MAX_CONN;
    wc.ap.authmode = WIFI_AUTH_WPA2_PSK;
    strlcpy((char *)wc.ap.password, WIFI_STA_AP_PASSWORD, sizeof(wc.ap.password));
    wc.ap.beacon_interval = 100;

    ret = esp_wifi_set_config(WIFI_IF_AP, &wc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_config(AP) 失败: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 启动配网页 */
    ret = start_provision_httpd();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "配网页启动失败: %s", esp_err_to_name(ret));
        return ret;
    }

    s_state = WIFI_STA_AP_MODE;
    ESP_LOGI(TAG, "AP 配网已开启: SSID=%s  IP=http://192.168.4.1/", s_ap_ssid);

    return ESP_OK;
}
