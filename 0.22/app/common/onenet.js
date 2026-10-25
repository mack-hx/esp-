/**
 * @file onenet.js
 * @brief OneNet 鉴权 (token 计算) + HTTP API 封装
 * @date 2026-10-21
 *
 * 鉴权算法 (OneNet 平台 2018-10-31 协议):
 *   res = products/{productId}/devices/{deviceName}
 *   et  = 过期时间 (Unix 秒)
 *   sign = base64( hmac_sha1( base64decode(accessKey), "{et}\n{method}\n{res}\n{version}" ) )
 *   token = version=2018-10-31&res=<urlencoded>&et=<et>&method=sha1&sign=<urlencoded>
 *
 * 平台地址 (2026-10-21 实测确认):
 *   - 开放接口  https://iot-api.heclouds.com        ← App 走这里
 *   - 设备接入  https://studio-mqtt.heclouds.com   ← ESP32 走 MQTT
 *   - 旧的 https://open.iot.10086.cn/application?action=... 已经废弃,
 *     会直接返回 authPermissionDeny, 不要用。
 *
 * 实际用到的接口:
 *   GET  /thingmodel/query-device-property?product_id=&device_name=
 *        → 读设备最新属性 (App 状态页用)
 *   GET  /thingmodel/query-device-property-history?...&identifier=&start_time=&end_time=
 *        → 读属性历史采样点 (俯仰角高频曲线用, 一次最多返回 10 个点)
 *   POST /thingmodel/set-device-desired-property
 *        → 设置属性期望值 (App 摇杆/灯/舵机/电磁铁用), body: {product_id, device_name, project_id, params}
 *   POST /thingmodel/call-service
 *        → 调用物模型服务 (急停/起立/配平/读参数/写参数/保存参数)
 *        body: {product_id, device_name, project_id, identifier, params}
 *
 * @note  AccessKey 必须是**产品级** key: 设备级 key 调上面这些接口一律
 *        authPermissionDeny (设备级 key 只能给设备自己用)。
 */

import { getConfig } from './config.js'

/** OneNet Studio 开放接口根地址 */
const API_BASE = 'https://iot-api.heclouds.com'

/**
 * @brief 接口地址
 * @note  H5 / App 都直接连 OneNet (平台已开放 CORS: Access-Control-Allow-Origin: *),
 *        不需要本地代理, 也不需要改 vite.config.js
 */
function getApiBase() {
  return API_BASE
}

/* ===================== token 计算 ===================== */

/**
 * @brief 计算 OneNet 设备访问 token
 * @return {string} token 字符串
 */
export function computeToken() {
  const cfg = getConfig()
  const version = '2018-10-31'
  const method = 'sha1'
  const res = `products/${cfg.productId}/devices/${cfg.deviceName}`
  /* et 用固定的长效时间戳 (2030-12-30)。
   * OneNet MQTT broker 只认长效 token, 用"当前时间 + 24h"会被 broker
   * 拒掉 (Bad username or password), 所以 App 和设备端保持一致。 */
  const et = 1924833600

  // 签名原文: 按 et / method / res / version 顺序, \n 分隔
  const strForSig = `${et}\n${method}\n${res}\n${version}`

  // AccessKey 参与计算前先 base64 解码
  const keyBin = base64Decode(cfg.deviceAccessKey)
  const sigBin = hmacSha1(keyBin, strForSig)
  const sign = base64Encode(sigBin)

  // 各 value 部分 URL 编码
  // ⚠ 必须用 encodeURIComponent, 不能用 encodeURI:
  //   res 里的 "/" 必须编码成 %2F, 否则平台报 "invalid authorization"。
  //   (encodeURI 不转义 "/")
  const token =
    `version=${encodeURIComponent(version)}` +
    `&res=${encodeURIComponent(res)}` +
    `&et=${et}` +
    `&method=${encodeURIComponent(method)}` +
    `&sign=${encodeURIComponent(sign)}`

  return token
}

/* ===================== base64 / hmac-sha1 (跨端实现) ===================== */

/**
 * @brief base64 解码 (返回 Uint8Array)
 */
function base64Decode(b64) {
  if (typeof atob === 'function') {
    const bin = atob(b64)
    const out = new Uint8Array(bin.length)
    for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i)
    return out
  }
  return base64DecodePolyfill(b64)
}

/**
 * @brief 纯 JS base64 解码, 兼容所有 uni-app 平台
 */
function base64DecodePolyfill(b64) {
  const chars = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/'
  const lookup = new Uint8Array(256)
  for (let i = 0; i < chars.length; i++) lookup[chars.charCodeAt(i)] = i
  const pad = b64.endsWith('==') ? 2 : b64.endsWith('=') ? 1 : 0
  const len = (b64.length / 4) * 3 - pad
  const out = new Uint8Array(len)
  let p = 0
  for (let i = 0; i < b64.length; i += 4) {
    const c0 = lookup[b64.charCodeAt(i)]
    const c1 = lookup[b64.charCodeAt(i + 1)]
    const c2 = lookup[b64.charCodeAt(i + 2)]
    const c3 = lookup[b64.charCodeAt(i + 3)]
    if (p < len) out[p++] = (c0 << 2) | (c1 >> 4)
    if (p < len) out[p++] = ((c1 & 0x0f) << 4) | (c2 >> 2)
    if (p < len) out[p++] = ((c2 & 0x03) << 6) | c3
  }
  return out
}

/**
 * @brief base64 编码 (Uint8Array / string → string)
 */
function base64Encode(input) {
  let bytes
  if (typeof input === 'string') {
    bytes = new Uint8Array(input.length)
    for (let i = 0; i < input.length; i++) bytes[i] = input.charCodeAt(i) & 0xff
  } else {
    bytes = input
  }
  if (typeof btoa === 'function') {
    let bin = ''
    for (let i = 0; i < bytes.length; i++) bin += String.fromCharCode(bytes[i])
    return btoa(bin)
  }
  return base64EncodePolyfill(bytes)
}

function base64EncodePolyfill(bytes) {
  const chars = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/'
  let out = ''
  let i = 0
  for (; i + 2 < bytes.length; i += 3) {
    out += chars[bytes[i] >> 2]
    out += chars[((bytes[i] & 0x03) << 4) | (bytes[i + 1] >> 4)]
    out += chars[((bytes[i + 1] & 0x0f) << 2) | (bytes[i + 2] >> 6)]
    out += chars[bytes[i + 2] & 0x3f]
  }
  if (i < bytes.length) {
    out += chars[bytes[i] >> 2]
    if (i + 1 < bytes.length) {
      out += chars[((bytes[i] & 0x03) << 4) | (bytes[i + 1] >> 4)]
      out += chars[(bytes[i + 1] & 0x0f) << 2]
      out += '='
    } else {
      out += chars[(bytes[i] & 0x03) << 4]
      out += '=='
    }
  }
  return out
}

/**
 * @brief HMAC-SHA1 (key: Uint8Array, msg: string) → Uint8Array
 */
function hmacSha1(key, msg) {
  return hmacSha1Sync(key, msg)
}

/**
 * @brief 同步的 HMAC-SHA1 (自实现 SHA1, 不依赖异步 crypto.subtle)
 */
function hmacSha1Sync(key, msg) {
  if (key.length > 64) key = sha1(key)
  if (key.length < 64) {
    const padded = new Uint8Array(64)
    padded.set(key)
    key = padded
  }
  const oKeyPad = new Uint8Array(64)
  const iKeyPad = new Uint8Array(64)
  for (let i = 0; i < 64; i++) {
    oKeyPad[i] = key[i] ^ 0x5c
    iKeyPad[i] = key[i] ^ 0x36
  }
  const msgBytes = new Uint8Array(msg.length)
  for (let i = 0; i < msg.length; i++) msgBytes[i] = msg.charCodeAt(i) & 0xff
  const inner = sha1(concat(iKeyPad, msgBytes))
  return sha1(concat(oKeyPad, inner))
}

function concat(a, b) {
  const out = new Uint8Array(a.length + b.length)
  out.set(a, 0)
  out.set(b, a.length)
  return out
}

/**
 * @brief 纯 JS SHA1 (返回 Uint8Array, 20 字节)
 */
function sha1(bytes) {
  let len = bytes.length
  const wordLen = (((len + 8) >> 6) + 1) << 6
  const buf = new Uint8Array(wordLen)
  buf.set(bytes)
  buf[len] = 0x80
  const bitLen = len * 8
  buf[wordLen - 1] = bitLen & 0xff
  buf[wordLen - 2] = (bitLen >> 8) & 0xff
  buf[wordLen - 3] = (bitLen >> 16) & 0xff
  buf[wordLen - 4] = (bitLen >> 24) & 0xff

  const w = new Uint32Array(80)
  let h0 = 0x67452301, h1 = 0xEFCDAB89, h2 = 0x98BADCFE, h3 = 0x10325476, h4 = 0xC3D2E1F0

  for (let i = 0; i < wordLen; i += 64) {
    for (let t = 0; t < 16; t++) {
      w[t] = (buf[i + t * 4] << 24) | (buf[i + t * 4 + 1] << 16) | (buf[i + t * 4 + 2] << 8) | buf[i + t * 4 + 3]
    }
    for (let t = 16; t < 80; t++) {
      const v = w[t - 3] ^ w[t - 8] ^ w[t - 14] ^ w[t - 16]
      w[t] = ((v << 1) | (v >>> 31)) >>> 0
    }
    let a = h0, b = h1, c = h2, d = h3, e = h4
    for (let t = 0; t < 80; t++) {
      let f, k
      if (t < 20)      { f = (b & c) | ((~b) & d); k = 0x5A827999 }
      else if (t < 40) { f = b ^ c ^ d;             k = 0x6ED9EBA1 }
      else if (t < 60) { f = (b & c) | (b & d) | (c & d); k = 0x8F1BBCDC }
      else             { f = b ^ c ^ d;             k = 0xCA62C1D6 }
      const temp = (((a << 5) | (a >>> 27)) + f + e + k + w[t]) >>> 0
      e = d; d = c
      c = ((b << 30) | (b >>> 2)) >>> 0
      b = a; a = temp
    }
    h0 = (h0 + a) >>> 0
    h1 = (h1 + b) >>> 0
    h2 = (h2 + c) >>> 0
    h3 = (h3 + d) >>> 0
    h4 = (h4 + e) >>> 0
  }

  const out = new Uint8Array(20)
  for (let i = 0; i < 4; i++) {
    out[i]      = (h0 >> (24 - i * 8)) & 0xff
    out[i + 4]  = (h1 >> (24 - i * 8)) & 0xff
    out[i + 8]  = (h2 >> (24 - i * 8)) & 0xff
    out[i + 12] = (h3 >> (24 - i * 8)) & 0xff
    out[i + 16] = (h4 >> (24 - i * 8)) & 0xff
  }
  return out
}

/* ===================== HTTP API 调用 (支持 H5 和 App) ===================== */

/**
 * @brief 检查平台返回的 code, 非 0 就抛异常 (让上层能给用户看清楚原因)
 * @param {Object} res  平台返回的 JSON
 * @param {string} what 操作名 (用于拼错误提示)
 * @return {Object} 原样返回 res
 * @note  平台 HTTP 状态码永远是 200, 真正的错误在 body 的 code 里:
 *        10412 = 标识符不存在 (物模型没建这个属性)
 *        10415 = 服务调用失败 (设备没应答 / 应答跟物模型对不上)
 */
function checkResult(res, what) {
  if (res && typeof res.code === 'number' && res.code !== 0) {
    const err = new Error(`${what}失败 (${res.code}): ${res.msg || '未知错误'}`)
    err.code = res.code
    /* 10403 = 鉴权失败, 说明 AccessKey 配错了, 这才算"连接配置"问题 */
    if (res.code === 10403) err.auth = true
    throw err
  }
  return res
}

/**
 * @brief 通用 HTTP 调用, 自动带 Authorization
 * @param {string} method 'GET' / 'POST'
 * @param {string} path  路径, 例如 '/thingmodel/query-device-property?product_id=xx'
 * @param {Object} [body] POST body (会被 JSON.stringify)
 * @returns {Promise<Object>} 解析后的 JSON
 */
export function httpRequest(method, path, body) {
  const token = computeToken()
  const url = getApiBase() + path
  const headers = {
    'Authorization': token,
    'Content-Type': 'application/json'
  }

  // H5 环境使用 fetch
  if (typeof uni === 'undefined' || typeof uni.request === 'undefined') {
    return fetch(url, {
      method,
      headers,
      body: body ? JSON.stringify(body) : undefined
    }).then(res => {
      if (!res.ok) {
        return res.text().then(text => {
          const err = new Error(`HTTP ${res.status}: ${text.slice(0, 200)}`)
          /* 鉴权类错误单独标记: 只有这种才说明"配置不对" */
          if (res.status === 401 || res.status === 403) err.auth = true
          throw err
        })
      }
      return res.json().catch(() => ({}))
    }).catch(err => {
      const e = new Error(err.message || '请求失败')
      e.auth = !!err.auth
      throw e
    })
  }

  // App 环境使用 uni.request
  return new Promise((resolve, reject) => {
    uni.request({
      url,
      method,
      header: headers,
      data: body,
      timeout: 15000,
      success: (res) => {
        if (res.statusCode >= 200 && res.statusCode < 300) {
          resolve(res.data)
        } else {
          const err = new Error(`HTTP ${res.statusCode}: ${JSON.stringify(res.data)}`)
          if (res.statusCode === 401 || res.statusCode === 403) err.auth = true
          reject(err)
        }
      },
      fail: (err) => reject(new Error(err.errMsg || '请求失败'))
    })
  })
}

/**
 * @brief 读设备最新属性值
 * @returns {Promise<Object>} { code:0, data:[{identifier, value, time, ...}, ...] }
 */
export function queryDeviceProperty() {
  const cfg = getConfig()
  const q = `product_id=${encodeURIComponent(cfg.productId)}&device_name=${encodeURIComponent(cfg.deviceName)}`
  return httpRequest('GET', `/thingmodel/query-device-property?${q}`)
}

/**
 * @brief 读设备的属性历史 (高频曲线用)
 * @param {string} identifier 属性标识符 (例如 'pitch')
 * @param {number} startTime 起始时间戳 (Unix 毫秒, 含)
 * @param {number} endTime   结束时间戳 (Unix 毫秒, 含)
 * @returns {Promise<Object>} { code:0, data:{ list:[{time, value}, ...] } }
 * @note  平台每次最多返回 10 个点, 且是**倒序** (最新在前)。
 * @note  设备端把 pitch 的上报频率提到 20Hz, 这个接口一次就能拿回一批采样点,
 *        比轮询"最新值"快得多 —— 曲线的点密度取决于这个接口, 不是轮询间隔。
 */
export function queryDevicePropertyHistory(identifier, startTime, endTime) {
  const cfg = getConfig()
  const q = `product_id=${encodeURIComponent(cfg.productId)}` +
            `&device_name=${encodeURIComponent(cfg.deviceName)}` +
            `&identifier=${encodeURIComponent(identifier)}` +
            `&start_time=${Math.floor(startTime)}` +
            `&end_time=${Math.floor(endTime)}`
  return httpRequest('GET', `/thingmodel/query-device-property-history?${q}`)
}

/**
 * @brief 设置属性期望值 (云→设备)
 * @param {Object} values 形如 { speed: 0.3 }
 * @note  设备会收到 $sys/.../thing/property/set 消息, 参数是**裸值**
 * @returns {Promise<Object>} { code:0, msg:'succ' }
 */
export function setDesiredProperty(values) {
  const cfg = getConfig()
  const body = {
    product_id: cfg.productId,
    device_name: cfg.deviceName,
    params: values || {}
  }
  /* project_id 不填也能用 (实测 2026-10-21), 填了才带上 */
  const pid = getProjectId()
  if (pid) body.project_id = pid
  return httpRequest('POST', '/thingmodel/set-device-desired-property', body)
    .then(res => checkResult(res, '设置期望值'))
}

/**
 * @brief 调用物模型服务 (急停 / 起立 / 配平 / 读参数 / 写参数 / 保存参数)
 * @param {string} name     服务标识符
 * @param {Object} inputObj 服务入参 (物模型里没声明入参时传 {})
 * @returns {Promise<Object>} { code:0, data:{...}, msg:'succ' }
 */
export function invokeService(name, inputObj) {
  const cfg = getConfig()
  const body = {
    product_id: cfg.productId,
    device_name: cfg.deviceName,
    identifier: name,
    params: inputObj || {}
  }
  const pid = getProjectId()
  if (pid) body.project_id = pid
  return httpRequest('POST', '/thingmodel/call-service', body)
    .then(res => checkResult(res, `调用服务 ${name}`))
}

/* ===================== 工具函数 ===================== */

/**
 * @brief 从 storage 读 project_id (用户在 connect 页填的)
 * @return {string} project_id
 */
export function getProjectId() {
  try {
    if (typeof uni === 'undefined' || !uni.getStorageSync) {
      return localStorage.getItem('ebr_project_id_v1') || ''
    }
    return uni.getStorageSync('ebr_project_id_v1') || ''
  } catch (e) { return '' }
}
