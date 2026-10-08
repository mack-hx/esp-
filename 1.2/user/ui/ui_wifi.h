/**
 * @file ui_wifi.h
 * @brief UI WiFi / 网页调参页
 * @note  屏幕太小, 调参走网页, 本页只负责告诉用户"去哪儿调":
 *        - 热点名 / 密码
 *        - 打开的网址
 *        - 当前接了几个客户端
 * @author ESP32 机械狗项目
 * @date 2026-10-04
 */

#ifndef UI_WIFI_H
#define UI_WIFI_H

#include "lcd_ui.h"

esp_err_t ui_wifi_register(void);

#endif // UI_WIFI_H
