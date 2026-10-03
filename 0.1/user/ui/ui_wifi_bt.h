/**
 * @file ui_wifi_bt.h
 * @brief UI 蓝牙 & WiFi 调试页
 * @note  当前工程尚未实现 WiFi / 蓝牙驱动, 此页仅做框架与占位:
 *        - 显示 WiFi / BT 模块状态 (OFF / INIT / READY)
 *        - 预留 K2/K3 操作钩子, 待驱动就绪后填充
 * @author ESP32 机械狗项目
 * @date 2026-10-01
 */

#ifndef UI_WIFI_BT_H
#define UI_WIFI_BT_H

#include "lcd_ui.h"

esp_err_t ui_wifi_bt_register(void);

#endif // UI_WIFI_BT_H