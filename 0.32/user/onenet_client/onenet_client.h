/**
 * @file onenet_client.h
 * @brief OneNet MQTT 客户端 (连 mqtts.heclouds.com, 走物模型 topic)
 * @date 2026-10-10
 *
 * 平台: OneNet Studio (Masa 平台)
 *   - Broker:   mqtts://mqtts.heclouds.com:1883
 *   - clientId: 设备名 (balancing_dog)
 *   - username: 产品 ID (6YlUqr9lue)
 *   - password: 用设备级 AccessKey 计算的 token (见 mqtt_client.c)
 *
 * 物模型 topic (本客户端订阅 / 发布, 跟 app/common/topic.js 1:1 对齐):
 *   发布:   $sys/{pid}/{dev}/thing/property/post    上报属性
 *   发布:   $sys/{pid}/{dev}/thing/event/post       上报事件
 *   订阅:   $sys/{pid}/{dev}/thing/property/set     云下发的属性 (移动指令 vx/vy/w)
 *   订阅:   $sys/{pid}/{dev}/thing/property/desired/get/reply   期望值应答
 *   订阅:   $sys/{pid}/{dev}/thing/service/{name}/invoke         服务调用
 *
 * 关键配置写在 sdkconfig.defaults:
 *   CONFIG_MQTT_PROTOCOL_311=y
 *   CONFIG_MQTT_TRANSPORT_SSL=y
 *   CONFIG_MQTT_TRANSPORT_WEBSOCKET_DISABLED=y
 *
 * 启动入口: mqtt_client_start() (在 main.c 的 app_main() 里调)
 */

#ifndef ONENET_CLIENT_H
#define ONENET_CLIENT_H

#include "esp_err.h"
#include <stdbool.h>

/* ========== 编译期配置 (用户填的) ==========
 * 这里写死项目对应的产品/设备. 后续如果换项目, 改这三行 + 重算 token. */
#define ONENET_PRODUCT_ID    "6YlUqr9lue"
#define ONENET_DEVICE_NAME   "balancing_dog"
/** 设备级 AccessKey (Base64 字符串), 跟 App 端 onenet.js 里写的一致 */
#define ONENET_DEVICE_AK     "YlpFVXlONlMzTUNnMHRocTRXSG04SkZMbmVrdEdmQTI="
/** Token 有效期 (秒), 跟 App 端 tokenExpireSec 一致 */
#define ONENET_TOKEN_TTL     (86400)

/* ========== 对外 API ========== */

/**
 * @brief 启动 MQTT 客户端 (后台任务)
 * @return ESP_OK 成功
 */
esp_err_t mqtt_client_start(void);

/**
 * @brief 上报一帧设备属性 (走 $sys/.../thing/property/post)
 * @param json_payload OneJSON 格式的完整属性 post body
 *                     (含 id / version / params 三段, 不带 topic)
 * @return ESP_OK 已加入发送队列
 */
esp_err_t mqtt_client_post_property(const char *json_payload);

/**
 * @brief 上报一个事件 (走 $sys/.../thing/event/post)
 * @param name 事件名 (fallen / armed / error)
 * @param json_params 形如 {"pitch":{"value":25.1,"time":1700000000000}}
 */
esp_err_t mqtt_client_post_event(const char *name, const char *json_params);

/**
 * @brief 查询当前连接状态
 * @return true=已连上 broker
 */
bool mqtt_client_is_connected(void);

#endif // ONENET_CLIENT_H
