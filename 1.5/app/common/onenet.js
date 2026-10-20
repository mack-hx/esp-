/**
 * @file onenet.js
 * @brief OneNet 鉴权 (token 计算) + HTTP API 封装
 * @date 2026-10-10
 *
 * 鉴权算法 (OneNet 平台 2018-10-31 协议):
 *   res = products/{productId}/devices/{deviceName}
 *   et  = (现在 + tokenExpireSec) 的 Unix 秒
 *   sign = base64( hmac_sha1( base64decode(accessKey), "{et}\n{method}\n{res}\n{version}" ) )
 *   token = version=2018-10-31&res=<urlencoded>&et=<et>&method=sha1&sign=<urlencoded>
 *
 * HTTP API 调用方式: 走 HTTPS, 鉴权头为 Authorization: <token>
 * 设备级 AccessKey 权限: 关联设备后有该设备最大权限 (可读写数据 / 下发命令等)
 *
 * 文档: https://open.iot.10086.cn/doc/iot_platform/book/device-connect&manager/device-auth.html
 */

import { getConfig } from './config.js'

/** OneNet Studio HTTP API 根地址 (新版 Masa 平台) */
const API_BASE = 'https://open.iot.10086.cn'

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
  const et = Math.floor(Date.now() / 1000) + (cfg.tokenExpireSec || 86400)

  // 签名原文: 按 et / method / res / version 顺序, \n 分隔
  const strForSig = `${et}\n${method}\n${res}\n${version}`

  // AccessKey 参与计算前先 base64 解码
  const keyBin = base64Decode(cfg.deviceAccessKey)
  const sigBin = hmacSha1(keyBin, strForSig)
  const sign = base64Encode(sigBin)

  // 各 value 部分 URL 编码
  const token =
    `version=${encodeURI(version)}` +
    `&res=${encodeURI(res)}` +
    `&et=${et}` +
    `&method=${encodeURI(method)}` +
    `&sign=${encodeURI(sign)}`

  return token
}

/* ===================== base64 / hmac-sha1 (跨端实现) ===================== */

/**
 * @brief base64 解码 (返回 Uint8Array)
 *   uni-app / 小程序 / H5 都能用这个纯 JS 实现, 避免依赖 nodejs Buffer
 */
function base64Decode(b64) {
  if (typeof atob === 'function') {
    const bin = atob(b64)
    const out = new Uint8Array(bin.length)
    for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i)
    return out
  }
  // 小程序 / app-plus: 走 uni 提供的 polyfill (如果有)
  if (typeof uni !== 'undefined' && typeof uni.arrayBufferToBase64 === 'function') {
    // 反向没有, 退到下面的 base64Encode 逻辑里做对偶
    return base64DecodePolyfill(b64)
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
 *   优先用 crypto.subtle, 小程序/app 走 fallback SHA1
 */
function hmacSha1(key, msg) {
  if (typeof crypto !== 'undefined' && crypto.subtle) {
    // 同步: 用同步 SHA1 + 自实现 HMAC
    return hmacSha1Sync(key, msg)
  }
  return hmacSha1Sync(key, msg)
}

/**
 * @brief 同步的 HMAC-SHA1 (用内置的 SHA1 实现, 不依赖异步 crypto.subtle)
 */
function hmacSha1Sync(key, msg) {
  // key 太长时先 hash 一次
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
  // 64-bit 大端长度 (高 32 位用 0, JS 位运算 32 位)
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

/* ===================== HTTP API 调用 ===================== */

/**
 * @brief 通用 HTTP 调用, 自动带 Authorization
 * @param {string} method 'GET' / 'POST'
 * @param {string} path 路径, 例如 '/device/thing/property/post?topic=...'
 * @param {Object} [body] POST body (会被 JSON.stringify)
 * @returns {Promise<Object>} 解析后的 JSON
 */
export function httpRequest(method, path, body) {
  const token = computeToken()
  const url = API_BASE + path
  const headers = {
    'Authorization': token,
    'Content-Type': 'application/json'
  }
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
          reject(new Error(`HTTP ${res.statusCode}: ${JSON.stringify(res.data)}`))
        }
      },
      fail: (err) => reject(new Error(err.errMsg || '请求失败'))
    })
  })
}

/**
 * @brief 属性上报 (设备状态 / 移动指令 → OneNet)
 *   HTTP 路径: POST /device/thing/property/post
 *   URL 参数:  topic=$sys/{pid}/{device-name}/thing/property/post, protocol=http
 */
export function postProperties(paramsObj) {
  const cfg = getConfig()
  const topic = `$sys/${cfg.productId}/${cfg.deviceName}/thing/property/post`
  const query = `topic=${encodeURIComponent(topic)}&protocol=http`
  const body = {
    id: String(Date.now() % 10000000000000).padStart(13, '0'),
    version: '1.0',
    params: wrapWithTime(paramsObj)
  }
  return httpRequest('POST', `/device/thing/property/post?${query}`, body)
}

/**
 * @brief 事件上报
 *   HTTP 路径: POST /device/thing/event/post
 */
export function postEvent(name, outObj) {
  const cfg = getConfig()
  const topic = `$sys/${cfg.productId}/${cfg.deviceName}/thing/event/post`
  const query = `topic=${encodeURIComponent(topic)}&protocol=http`
  const params = {}
  for (const k in outObj) {
    params[k] = { value: outObj[k], time: Date.now() }
  }
  const body = {
    id: String(Date.now() % 10000000000000).padStart(13, '0'),
    version: '1.0',
    name,
    params
  }
  return httpRequest('POST', `/device/thing/event/post?${query}`, body)
}

/**
 * @brief 设置属性期望值 (云→设备, App 推命令用)
 *   HTTP 路径: POST /application?action=SetDeviceDesiredProperty&version=1
 *   注意: project_id 需要填, 这是从 OneNet Studio 项目概览拿的 ID
 *
 * @param {string} projectId  项目 ID
 * @param {Object} values     形如 { vx: 0.3, vy: 0, w: 0 }
 */
export function setDesiredProperty(projectId, values) {
  const cfg = getConfig()
  const body = {
    project_id: projectId,
    product_id: cfg.productId,
    device_name: cfg.deviceName,
    params: values
  }
  return httpRequest('POST', '/application?action=SetDeviceDesiredProperty&version=1', body)
}

/**
 * @brief 读设备最新属性值
 *   HTTP 路径: GET /device/thing/property/get?topic=...&protocol=http
 *   注: 这个接口实际是从设备影子读, 设备主动上报后才会有
 */
export function getDeviceProperties() {
  const cfg = getConfig()
  const topic = `$sys/${cfg.productId}/${cfg.deviceName}/thing/property/get`
  const query = `topic=${encodeURIComponent(topic)}&protocol=http`
  return httpRequest('GET', `/device/thing/property/get?${query}`)
}

/**
 * @brief 服务调用 (App 调 ESP32 的 stop / stand / calibrate / param_get / param_set / param_save)
 *   HTTP 路径: POST /device/thing/service/{name}/invoke
 *   URL 参数:  topic=$sys/{pid}/{device-name}/thing/service/{name}/invoke, protocol=http
 */
export function invokeService(name, inputObj) {
  const cfg = getConfig()
  const topic = `$sys/${cfg.productId}/${cfg.deviceName}/thing/service/${name}/invoke`
  const query = `topic=${encodeURIComponent(topic)}&protocol=http`
  const params = {}
  for (const k in (inputObj || {})) {
    params[k] = { value: inputObj[k] }
  }
  const body = {
    id: String(Date.now() % 10000000000000).padStart(13, '0'),
    version: '1.0',
    params
  }
/**
 * @brief 服务调用 (App 调 ESP32 的 stop / stand / calibrate / param_get / param_set / param_save)
 *   HTTP 路径: POST /application?action=InvokeService&version=1
 *   Body 格式: {project_id, product_id, device_name, service_identifier, args}
 *   args 是 JSON 字符串
 */
export function invokeService(name, inputObj) {
  const cfg = getConfig()
  const args = Object.keys(inputObj || {}).length > 0
    ? JSON.stringify(inputObj)
    : '{}'
  const body = {
    project_id: getProjectId(),
    product_id: cfg.productId,
    device_name: cfg.deviceName,
    service_identifier: name,
    args
  }
  return httpRequest('POST', '/application?action=InvokeService&version=1', body)
}

/**
 * @brief 读设备最新属性值 (轮询设备最近一次上报的值)
 *   HTTP 路径: GET /application?action=QueryDeviceProperty&version=1
 *   返回: { data: { list: [{identifier, value, time, data_type}, ...] } }
 */
export function queryDeviceProperty() {
  const cfg = getConfig()
  const pid = getProjectId()
  const q = [
    'action=QueryDeviceProperty',
    'version=1',
    `project_id=${encodeURIComponent(pid)}`,
    `product_id=${encodeURIComponent(cfg.productId)}`,
    `device_name=${encodeURIComponent(cfg.deviceName)}`
  ].join('&')
  return httpRequest('GET', `/application?${q}`)
}

/* ===================== 工具 ===================== */

/** 从 storage 读 project_id (用户在 connect 页填的) */
function getProjectId() {
  try { return uni.getStorageSync('ebr_project_id_v1') || '' } catch (e) { return '' }
}

function wrapWithTime(obj) {
  const now = Date.now()
  const out = {}
  for (const k in obj) {
    if (obj[k] === null || obj[k] === undefined) continue
    out[k] = { value: obj[k], time: now }
  }
  return out
}
