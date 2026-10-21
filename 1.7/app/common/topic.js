/**
 * @file topic.js
 * @brief OneNet 物模型 topic 集中出口 (App + ESP32 通用)
 * @date 2026-10-10
 *
 * 协议: OneNet Studio 物模型 (Masa 平台)
 *   文档: https://open.iot.10086.cn/doc/iot_platform/book/device-connect&manager/thing-model/
 *
 * Topic 命名规范 (OneNet 物模型):
 *   属性上报 (设备→云):  $sys/{pid}/{device-name}/thing/property/post
 *   属性上报应答 (云→设备): $sys/{pid}/{device-name}/thing/property/post/reply
 *   属性设置 (云→设备):  $sys/{pid}/{device-name}/thing/property/set
 *   属性设置应答 (设备→云): $sys/{pid}/{device-name}/thing/property/set/reply
 *   属性获取 (云→设备):  $sys/{pid}/{device-name}/thing/property/get
 *   属性获取应答 (设备→云): $sys/{pid}/{device-name}/thing/property/get/reply
 *   期望值获取 (设备→云): $sys/{pid}/{device-name}/thing/property/desired/get
 *   期望值获取应答 (云→设备): $sys/{pid}/{device-name}/thing/property/desired/get/reply
 *   事件上报 (设备→云):  $sys/{pid}/{device-name}/thing/event/post
 *   服务调用 (云→设备):  $sys/{pid}/{device-name}/thing/service/{identifier}/invoke
 *   服务调用应答 (设备→云): $sys/{pid}/{device-name}/thing/service/{identifier}/invoke/reply
 *
 * 注意: ESP32 端还没接 MQTT, 写 mqtt_client.c 时 1:1 翻译成 C 字符串.
 */

import { getConfig } from './config.js'

/**
 * @brief 拼 OneNet 物模型 topic (跟产品 ID + 设备名绑定)
 * @param {string} suffix 例如 'thing/property/post'
 */
function sysTopic(suffix) {
  const cfg = getConfig()
  return `$sys/${cfg.productId}/${cfg.deviceName}/${suffix}`
}

export const T = {
  /** 设备基础路径, 调试用 */
  base: () => `$sys/${getConfig().productId}/${getConfig().deviceName}`,

  // ---- 属性 (设备状态 / 移动指令) ----
  propertyPost:      () => sysTopic('thing/property/post'),
  propertyPostReply: () => sysTopic('thing/property/post/reply'),
  propertySet:       () => sysTopic('thing/property/set'),
  propertySetReply:  () => sysTopic('thing/property/set/reply'),
  propertyGet:       () => sysTopic('thing/property/get'),
  propertyGetReply:  () => sysTopic('thing/property/get/reply'),
  desiredGet:        () => sysTopic('thing/property/desired/get'),
  desiredGetReply:   () => sysTopic('thing/property/desired/get/reply'),

  // ---- 事件 ----
  eventPost: () => sysTopic('thing/event/post'),

  // ---- 服务 (按 identifier 拼后缀) ----
  serviceInvoke:      (name) => sysTopic(`thing/service/${name}/invoke`),
  serviceInvokeReply: (name) => sysTopic(`thing/service/${name}/invoke/reply`)
}
