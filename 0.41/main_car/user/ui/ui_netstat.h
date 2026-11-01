/**
 * @file ui_netstat.h
 * @brief UI 网络状态页
 * @note  显示:
 *        - WiFi 当前模式 (AP 配网 / STA 连家里 / 失败)
 *        - 连上的 SSID / IP
 *        - OneNet MQTT 连接状态
 *
 *        按键:
 *        - K1 返回菜单
 *        - K2 舵机回中
 *        - K3 清零编码器
 *        - K4 长按 3s 触发重新配网 (清 NVS 重启)
 *
 * @author ESP32 机械狗项目
 */

#ifndef UI_NETSTAT_H
#define UI_NETSTAT_H

#include "lcd_ui.h"

esp_err_t ui_netstat_register(void);

#endif // UI_NETSTAT_H
