/**
 * @file onenet_client.h
 * @brief OneNet MQTT 客户端 (连 mqtts.heclouds.com, 走物模型 topic)
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

/* ========== 默认编译期配置 ==========
 * AccessKey 从 NVS 读, 无需 onenet_secret.h; 配网页配置后存 NVS */
#define ONENET_DEFAULT_PRODUCT_ID    "6YlUqr9lue"
#define ONENET_DEFAULT_DEVICE_NAME   "balancing_dog"

/* ========== 对外 API ========== */

/**
 * @brief 保存 OneNET 配置到 NVS (产品ID / 设备名 / AccessKey)
 * @param pid  产品 ID (可为 NULL，保持现有值)
 * @param dev  设备名 (可为 NULL，保持现有值)
 * @param ak   AccessKey base64 (可为 NULL，保持现有值)
 * @return ESP_OK 成功
 */
esp_err_t onenet_config_save(const char *pid, const char *dev, const char *ak);

/**
 * @brief 从 NVS 读 OneNET 配置 (不存在时用默认值)
 * @param pid_out  [出参] 产品 ID (缓冲至少 32 字节)
 * @param dev_out  [出参] 设备名 (缓冲至少 64 字节)
 * @param ak_out   [出参] AccessKey (缓冲至少 128 字节, 为空表示未配置)
 */
void onenet_config_load(char *pid_out, size_t pid_cap, char *dev_out, size_t dev_cap, 
                        char *ak_out, size_t ak_cap);

/**
 * @brief 启动 MQTT 客户端 (后台任务)
 * @note  内部建一条任务处理 MQTT 事件循环, 不需要外部喂事件
 * @note  AccessKey 为空时不启动，返回 ESP_FAIL
 * @return ESP_OK 成功
 */
esp_err_t mqtt_client_start(void);
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

/* ========== 远程指令 (OneNET 与蓝牙共用) ========== */

/**
 * @brief 远程移动指令
 * @param v 指令值: 0 停; ±0.2~0.8 前进(+)/后退(-) 档位 20~80%;
 *          ±1.2~1.8 左转(+)/右转(-) 档位 20~80%
 */
void remote_move(float v);

/**
 * @brief 执行一条服务 (OneNET 服务调用与蓝牙 SV 指令共用)
 * @param name  stop / stand / calibrate / param_get / param_set / param_save /
 *              trim_record / trim_save / trim_clear / trim_get
 * @param in    入参文本 (param_set: 参数 JSON; 其它服务可为 NULL)
 * @param out   [出参] 返回文本 (param_get / trim_* 有内容), 可为 NULL
 * @param cap   out 容量
 * @return 成功返回 out 内文本长度 (无返回文本为 0); 失败或服务名未知返回 -1
 */
int remote_service(const char *name, const char *in, char *out, size_t cap);

/**
 * @brief 设置灯 / 舵机 / 电磁铁属性
 * @param json 扁平 JSON, 键: led_on / led_color / led_brightness / led_preset /
 *             servo1_angle / servo2_angle / magnet_on
 */
void remote_property(const char *json);

/**
 * @brief 标记蓝牙直控是否连接; 连接期间忽略 OneNET 下发的全部控制 (属性设置 / 服务调用 / 移动期望值)
 * @param on true = 已连接
 */
void remote_set_ble_link(bool on);

/**
 * @brief 远程急停: 清移动指令 + 停平衡 + 刹车; LCD 在平衡页时退回主页
 */
void remote_stop(void);

/**
 * @brief 远程起立: 开平衡, 成功后 LCD 切到平衡页 (倾角过大被拒时不切)
 */
void remote_stand(void);

#endif // ONENET_CLIENT_H
