/**
 * @file wifi_sta.h
 * @brief WiFi STA 模式 + AP 配网
 *
 * 流程: 启动读 NVS `wifi_ssid`/`wifi_pass`; 有存走 STA 连接, 没存开 AP 热点
 *       (默认 SSID BalanceRobot_Setup) + 配网页 (192.168.4.1), 保存后软重启。
 *       主循环看到 STA 连上后再启动 onenet_client。
 *
 * @note  AP 模式仅用于配网, 不提供调参网页
 * @author ESP32 机械狗项目
 */

#ifndef WIFI_STA_H
#define WIFI_STA_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_netif.h"

/* ========== AP 配网模式配置 (配网时开的临时热点) ========== */
#define WIFI_STA_AP_SSID     "BalanceRobot_Setup"   /* AP 配网热点名 (固定, 配网期间不能改) */
#define WIFI_STA_AP_PASSWORD "12345678"             /* AP 密码, 至少 8 位 */
#define WIFI_STA_AP_CHANNEL  1
#define WIFI_STA_AP_MAX_CONN 4
#define WIFI_STA_AP_IP       "192.168.4.1"          /* 配网页地址 */

/* ========== STA 连接配置 ========== */
#define WIFI_STA_MAX_RETRY    15                     /* STA 连接最大重试次数 (每次约5s, 共75s足够DHCP获取IP) */
#define WIFI_STA_NVS_NS      "wifi_cfg"             /* NVS 命名空间 */
#define WIFI_STA_NVS_KEY_SSID "ssid"                /* NVS key: 已保存的 SSID */
#define WIFI_STA_NVS_KEY_PASS "pass"                /* NVS key: 已保存的密码 */

/* ========== 状态 ========== */

/**
 * @brief WiFi 状态机
 */
typedef enum {
    WIFI_STA_STOPPED = 0,    // 未启动
    WIFI_STA_AP_MODE,        // AP 配网模式 (等待用户填 WiFi)
    WIFI_STA_CONNECTING,     // STA 正在连家里 WiFi
    WIFI_STA_CONNECTED,      // STA 已连上
    WIFI_STA_FAILED,         // STA 连失败 (超过重试次数)
} wifi_sta_state_t;

/* ========== 函数声明 ========== */

/**
 * @brief 启动 WiFi (自动选 STA 或 AP 配网模式)
 * @note  必须在 nvs_flash_init() 之后调用 (WiFi 驱动要用 NVS)
 * @return ESP_OK 启动指令成功下发, 异步结果看事件 / wifi_sta_get_state()
 */
esp_err_t wifi_sta_start(void);

/**
 * @brief 查询当前状态
 */
wifi_sta_state_t wifi_sta_get_state(void);
/**
 * @brief 获取当前 STA 模式下的 IP 信息
 * @param info [出参] IP 信息
 * @return ESP_OK 成功, ESP_ERR_INVALID_STATE 还没连上
 */
esp_err_t wifi_sta_get_ip(esp_netif_ip_info_t *info);

/**
 * @brief 获取当前已连接的 SSID (STA 模式)
 * @param buf 输出缓冲区
 * @param len 缓冲区大小
 * @return ESP_OK 成功, ESP_ERR_INVALID_STATE 当前不在 STA 模式
 */
esp_err_t wifi_sta_get_ssid(char *buf, uint32_t len);

/**
 * @brief 获取当前 AP 配网热点的真实 SSID (含 MAC 后缀)
 * @param buf 输出缓冲区
 * @param len 缓冲区大小
 * @return ESP_OK 成功, ESP_ERR_INVALID_STATE 当前不在 AP 模式
 * @note  真实 SSID = WIFI_STA_AP_SSID + "_" + MAC 后两字节 (大写 hex),
 *        手机搜热点必须按这个名字找
 */
esp_err_t wifi_sta_get_ap_ssid(char *buf, uint32_t len);

/**
 * @brief 切换到 AP 配网模式 (运行时调用，不重启)
 * @note  用于在已连接状态下按K4进入配网模式
 * @return ESP_OK 成功
 */
esp_err_t wifi_sta_start_ap(void);
#endif // WIFI_STA_H
