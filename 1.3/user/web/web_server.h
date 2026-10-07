/**
 * @file web_server.h
 * @brief 调参网页服务器 (HTTP, 端口 80)
 * @note  必须先 wifi_ap_start() 再调 web_server_start()
 *
 * @note  接口一览:
 *   GET  /            调参主页 (滑块界面, HTML 内嵌在 web_page.c)
 *   GET  /api/params  取全部参数 (含名称/范围/默认值/当前值)
 *   POST /api/params  批量设置参数, body = {"bal_kp":12.5,...}
 *   POST /api/save    保存参数 + 配平表到 NVS
 *   POST /api/reset   恢复默认参数 (仅内存, 不影响配平表)
 *   POST /api/servo   舵机回中
 *   POST /api/reboot  重启
 *   POST /api/balance 启停平衡控制, body = {"on":1|0}
 *   POST /api/balance_stop   急停 (清 PID 积分)
 *   POST /api/balance_reload 重新读取控制参数
 *   GET  /api/status  取实时状态 (姿态/陀螺/编码器/WiFi/平衡控制)
 *
 *   GET  /api/trim    取整张配平表
 *   POST /api/trim    配平标定, body = {"act":"move|add|read|del|clear", "deg":80}
 *
 * @author ESP32 平衡轮腿机器人
 * @date 2026-10-04
 */

#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include "esp_err.h"

#define WEB_PORT         80      /* HTTP 端口 (网址不用写端口) */

/**
 * @brief 启动调参网页服务器
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t web_server_start(void);

/**
 * @brief 停止调参网页服务器
 * @return ESP_OK 成功
 */
esp_err_t web_server_stop(void);

#endif // WEB_SERVER_H
