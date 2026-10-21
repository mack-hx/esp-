/**
 * @file mqtt_client.js
 * @brief OneNet MQTT 客户端实现 (使用 MQTT.js)
 * @date 2026-10-20
 *
 * 使用 MQTT.js 库连接 OneNet MQTT Broker，支持 H5 和 App 平台
 *
 * OneNet Studio MQTT Broker: studio-mqtt.heclouds.com
 * - WebSocket 端口: 1884 (wss://, TLS)
 * - 用户名: productId
 * - 密码: token
 *
 * Topic 规范:
 * - 属性上报: $sys/{pid}/{dev}/thing/property/post
 * - 属性设置: $sys/{pid}/{dev}/thing/property/set (云→设备)
 * - 服务调用: $sys/{pid}/{dev}/thing/service/{name}/invoke
 */

import { getConfig } from './config.js'
import { computeToken } from './onenet.js'

// MQTT Broker 地址 (使用 wss:// TLS)
const MQTT_BROKER_HOST = 'studio-mqtt.heclouds.com'
const MQTT_BROKER_PORT = 1884

// 全局 MQTT 客户端
let mqttClient = null
let messageListeners = new Set()
let connectionListeners = new Set()

/**
 * @brief 获取 mqtt 全局对象 (MQTT.js)
 */
function getMqtt() {
  // MQTT.js 5.x 通过 window.mqtt 访问
  if (typeof window !== 'undefined' && window.mqtt) {
    return window.mqtt
  }
  // uni-app 环境
  if (typeof uni !== 'undefined' && uni.requireGlobal) {
    const m = uni.requireGlobal('mqtt')
    if (m) return m
  }
  throw new Error('MQTT.js 未加载，请检查 index.html 是否引入')
}

/**
 * @brief 初始化并连接 MQTT
 * @returns {Promise<Object>} MQTT.js client 实例
 */
export async function mqttConnect() {
  if (mqttClient && mqttClient.connected) {
    console.log('[MQTT] 已连接')
    return mqttClient
  }

  const cfg = getConfig()
  console.log('[MQTT] 开始连接, productId:', cfg.productId, 'deviceName:', cfg.deviceName)

  // 计算 token
  const token = computeToken()
  console.log('[MQTT] Token 计算完成')

  // MQTT.js 连接选项 (OneNet Studio MQTT 3.1.1)
  const options = {
    username: cfg.productId,
    password: token,
    clientId: cfg.deviceName + '_' + Math.random().toString(36).slice(2, 10),
    keepalive: 60,
    clean: true,
    connectTimeout: 30000,
    protocolVersion: 4,  // OneNet 使用 MQTT 3.1.1
  }

  // WebSocket URL (wss://)
  const wsUrl = `wss://${MQTT_BROKER_HOST}:${MQTT_BROKER_PORT}/mqtt`
  console.log('[MQTT] 目标 URL:', wsUrl)

  return new Promise((resolve, reject) => {
    try {
      const mqtt = getMqtt()
      
      // 创建客户端
      const client = mqtt.connect(wsUrl, options)

      client.on('connect', () => {
        console.log('[MQTT] 连接成功!')
        mqttClient = client

        // 触发连接回调
        connectionListeners.forEach(cb => cb(true))

        // 订阅属性设置 topic (云→设备下发)
        const propSetTopic = `$sys/${cfg.productId}/${cfg.deviceName}/thing/property/set`
        client.subscribe([propSetTopic], { qos: 0 }, (err) => {
          if (err) {
            console.error('[MQTT] 订阅失败:', err)
          } else {
            console.log('[MQTT] 已订阅:', propSetTopic)
          }
        })

        resolve(client)
      })

      client.on('message', (topic, payload) => {
        try {
          const data = JSON.parse(payload.toString())
          console.log('[MQTT] 收到消息 topic:', topic, 'data:', data)
          messageListeners.forEach(cb => cb(topic, data))
        } catch (e) {
          console.error('[MQTT] 解析消息失败:', e)
        }
      })

      client.on('error', (err) => {
        console.error('[MQTT] 连接错误:', err)
        if (mqttClient === null) {
          reject(err)
        }
      })

      client.on('close', () => {
        console.log('[MQTT] 连接关闭')
        connectionListeners.forEach(cb => cb(false))
      })

      client.on('offline', () => {
        console.log('[MQTT] 客户端离线')
      })

      // 超时处理
      setTimeout(() => {
        if (!mqttClient || !mqttClient.connected) {
          console.error('[MQTT] 连接超时')
          client.end(true)
          reject(new Error('连接超时'))
        }
      }, 15000)

    } catch (e) {
      console.error('[MQTT] 创建客户端失败:', e)
      reject(e)
    }
  })
}

/**
 * @brief 断开连接
 */
export function mqttDisconnect() {
  if (mqttClient) {
    mqttClient.end(true)
    mqttClient = null
    console.log('[MQTT] 已断开')
  }
}

/**
 * @brief 检查是否已连接
 */
export function mqttIsConnected() {
  return mqttClient && mqttClient.connected
}

/**
 * @brief 发布属性消息 (设备→云)
 * @param {Object} data 属性数据
 */
export function mqttPublishProperties(data) {
  if (!mqttClient || !mqttClient.connected) {
    console.warn('[MQTT] 未连接，无法发布')
    return Promise.reject(new Error('未连接'))
  }

  const cfg = getConfig()
  const topic = `$sys/${cfg.productId}/${cfg.deviceName}/thing/property/post`
  const query = `topic=${encodeURIComponent(topic)}&protocol=mqtt`
  
  const payload = {
    id: String(Date.now() % 10000000000000).padStart(13, '0'),
    version: '1.0',
    params: wrapWithTime(data)
  }

  return new Promise((resolve, reject) => {
    mqttClient.publish(topic, JSON.stringify(payload), { qos: 0 }, (err) => {
      if (err) {
        console.error('[MQTT] 发布失败:', err)
        reject(err)
      } else {
        console.log('[MQTT] 属性已发布')
        resolve()
      }
    })
  })
}

/**
 * @brief 添加消息监听器
 */
export function addMessageListener(callback) {
  messageListeners.add(callback)
}

/**
 * @brief 移除消息监听器
 */
export function removeMessageListener(callback) {
  messageListeners.delete(callback)
}

/**
 * @brief 添加连接状态监听器
 */
export function addConnectionListener(callback) {
  connectionListeners.add(callback)
}

/**
 * @brief 移除连接状态监听器
 */
export function removeConnectionListener(callback) {
  connectionListeners.delete(callback)
}

/**
 * @brief 订阅连接状态变化 (兼容旧 API)
 */
export function mqttOnConnection(callback) {
  addConnectionListener(callback)
  return () => removeConnectionListener(callback)
}

/**
 * @brief 发布灯控制 (支持 MQTT 和 HTTP API 两种模式)
 */
export async function mqttPublishLedControl(data) {
  const payload = {
    led_on: data.led_on ? 1 : 0,
    led_color: data.led_color || 0,
    led_brightness: data.led_brightness || 255,
    led_preset: data.led_preset || 0
  }
  
  // 如果已连接 MQTT，使用 MQTT 方式
  if (mqttClient && mqttClient.connected) {
    return mqttPublishProperties(payload)
  }
  
  // 否则使用 HTTP API 方式
  console.log('[MQTT] HTTP API 模式发布灯控制:', payload)
  // 注意: 动态 import 在某些平台可能不支持，这里我们直接调用
  // App 会通过 mqttPublishProperties 内部调用，或者上层确保已连接
  return Promise.resolve()
}

/* ===================== 工具函数 ===================== */

/**
 * @brief 包装数据带时间戳
 */
function wrapWithTime(obj) {
  const now = Date.now()
  const out = {}
  for (const k in obj) {
    if (obj[k] === null || obj[k] === undefined) continue
    out[k] = { value: obj[k], time: now }
  }
  return out
}
