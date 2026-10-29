/**
 * @file ui_home.h
 * @brief UI 主页 (上电默认显示)
 * @note  显示项目标题 / 时间 / 提示
 *        K1 进入菜单页
 * @author ESP32 机械狗项目
 */

#ifndef UI_HOME_H
#define UI_HOME_H

#include "lcd_ui.h"

/**
 * @brief 注册主页到 UI 系统
 */
esp_err_t ui_home_register(void);

#endif // UI_HOME_H