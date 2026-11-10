/**
 * @file ble_ctrl.h
 * @brief BLE 直控 (NimBLE 外设, 无配对码)
 *
 * GATT: 服务 0xFFE0, 特征 0xFFE1 (写 / 无响应写 / 通知), 内容为 ASCII 文本
 *   App → 车:  "M:<v>"   移动指令, v 编码同 remote_move()
 *              "STOP"    急停
 *              "STAND"   起立
 *              "PROP:<json>\n"       灯 / 舵机 / 电磁铁属性, 无应答
 *              "SV:<name>[:<入参>]\n"  服务调用, name 同 remote_service():
 *                  calibrate / param_get / param_set / param_save /
 *                  trim_record / trim_save / trim_clear / trim_get
 *              长指令 (PROP / SV / CFG) 由 App 按 20 字节分片写入
 *   车 → App:  "P:<pitch>"  俯仰角 (度, 1 位小数), 每 BLE_NOTIFY_MS 一帧
 *              "A:<name>:<1|0>:<返回文本>\n"  SV 应答, 按 MTU 分片通知,
 *              发送期间不插 P 帧
 *
 * 非零移动指令超过 BLE_MOVE_TIMEOUT_MS 没刷新、或蓝牙断开时, 移动指令归零 (不关平衡)。
 * 蓝牙连接期间车端忽略 OneNET 下发的全部控制指令 (MQTT 只上报数据)。
 */
#ifndef BLE_CTRL_H
#define BLE_CTRL_H

#include "esp_err.h"
#include <stdbool.h>

#define BLE_DEVICE_NAME      "BalanceDog"   /* 广播名, App 按此名字识别主车 */
#define BLE_NOTIFY_MS        50             /* pitch 通知周期 (ms) */
#define BLE_MOVE_TIMEOUT_MS  600            /* 移动指令保活超时 (ms) */

/**
 * @brief 启动 BLE 协议栈并开始广播
 * @return ESP_OK 成功, 其它为 NimBLE 初始化错误
 */
esp_err_t ble_ctrl_start(void);

/**
 * @brief 是否有手机连着
 * @return true 已连接
 */
bool ble_ctrl_is_connected(void);

/**
 * @brief 取已连接手机的蓝牙地址
 * @param out  输出缓冲, 格式 "AA:BB:CC:DD:EE:FF"
 * @param cap  缓冲长度 (至少 18)
 * @return true 已连接且取到地址; false 未连接, out 不变
 */
bool ble_ctrl_get_peer(char *out, size_t cap);

#endif // BLE_CTRL_H
