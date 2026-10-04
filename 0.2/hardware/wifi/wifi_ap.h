/**
 * @file wifi_ap.h
 * @brief WiFi 热点 (AP 模式) 驱动 — 供手机/电脑连上后用浏览器调参
 * @note  本项目只做 AP 模式, 不连路由器:
 *        - ESP32 自己开热点, SSID 见 WIFI_AP_SSID
 *        - 手机/电脑连上这个热点后, 直接浏览器打开 http://192.168.4.1
 *        - IP 固定 192.168.4.1, 端口 80 (HTTP)
 *
 * @note  为什么不用 STA 模式:
 *        调试现场不一定有路由器 / 路由器可能改过密码, AP 模式最省事。
 *        缺点: 手机连上后没有外网, 只能访问本机页面 —— 调参场景够用。
 * @author ESP32 机械狗项目
 * @date 2026-10-04
 */

#ifndef WIFI_AP_H
#define WIFI_AP_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_netif.h"

/* ========== 热点配置 ========== */

/** 热点名称 (SSID), 手机搜到的就是这个 */
#define WIFI_AP_SSID        "BalanceRobot"
/** 热点密码, 至少 8 位 (WPA2-PSK); 设为空则用开放网络 */
#define WIFI_AP_PASSWORD    "12345678"
/** 热点信道 (1-13) */
#define WIFI_AP_CHANNEL     1
/** 一次最多接入几个客户端 */
#define WIFI_AP_MAX_CONN    4

/* ========== 固定 IP (AP 模式下自己就是网关) ========== */
#define WIFI_AP_IP          "192.168.4.1"
#define WIFI_AP_GATEWAY     "192.168.4.1"
#define WIFI_AP_NETMASK     "255.255.255.0"

/* ========== 连接状态 ========== */

/**
 * @brief 热点状态
 */
typedef enum {
    WIFI_AP_STOPPED = 0,   // 未启动
    WIFI_AP_STARTING,      // 启动中
    WIFI_AP_READY,         // 热点已开, 等待客户端
    WIFI_AP_ERROR          // 启动失败
} wifi_ap_state_t;

/* ========== 函数声明 ========== */

/**
 * @brief 启动 WiFi 热点
 * @note  必须在 nvs_flash_init() 之后调用 (WiFi 驱动要用 NVS 存配网信息)
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t wifi_ap_start(void);

/**
 * @brief 停止 WiFi 热点
 * @return ESP_OK 成功
 */
esp_err_t wifi_ap_stop(void);

/**
 * @brief 查询当前状态
 * @return 状态枚举
 */
wifi_ap_state_t wifi_ap_get_state(void);

/**
 * @brief 获取热点自身的 IP 信息 (固定 192.168.4.1)
 * @param info [出参] IP 信息结构体指针
 * @return ESP_OK 成功, ESP_ERR_INVALID_STATE 热点未启动
 */
esp_err_t wifi_ap_get_ip(esp_netif_ip_info_t *info);

/**
 * @brief 获取当前热点实际使用的 SSID
 * @note  实际 SSID = WIFI_AP_SSID + "_" + MAC 后两字节 (用于区分多台设备),
 *        比 WIFI_AP_SSID 长, 显示/提示必须用本函数拿到的真实值
 * @param buf 输出缓冲区
 * @param len 缓冲区大小
 */
void wifi_ap_get_ssid(char *buf, uint32_t len);

/**
 * @brief 获取当前已连接的客户端数量
 * @return 客户端数 (0 = 没人连)
 */
uint8_t wifi_ap_get_client_count(void);

#endif // WIFI_AP_H
