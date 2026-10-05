/**
 * @file web_server.h
 * @brief 调参网页服务器 (HTTP, 端口 80)
 * @note  必须先 wifi_ap_start() 再调 web_server_start()
 *
 * @note  接口一览:
 *   GET  /            调参主页 (滑块界面, HTML 内嵌在 web_page.c)
 *   GET  /api/params  取全部参数 (含名称/范围/默认值/当前值)
 *   GET  /api/status  取实时状态 (陀螺仪/编码器/WiFi/平衡控制)
 *   POST /api/params  批量设置参数, body = {"bal_kp":12.5,...}
 *   POST /api/save    保存参数到 NVS
 *   POST /api/reset   恢复默认参数 (仅内存)
 *   POST /api/servo   舵机回中
 *   POST /api/balance 启停平衡控制, body = {"on":1|0}
 *   GET  /api/legmap  取左右两腿的标定表
 *   POST /api/legmap  录入/动腿/读取, body={"act":"clear|clear_all|move|read|add","leg":"L|R","deg":80,"h":81.5}
 *   POST /api/legmap_fit_legk  用标定点反推线性 leg_k (查表不可用时的兜底), body={"leg":"L|R"}
 * @author ESP32 机械狗项目
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
