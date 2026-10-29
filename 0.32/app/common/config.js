/**
 * @file config.js
 * @brief OneNet 连接配置 (产品ID / 设备名 / 设备级 AccessKey)
 * @date 2026-10-10
 *
 * 数据全部存 uni.storage (H5 走 localStorage, 小程序走 wx.storage).
 * 修改后立即生效.
 *
 * OneNet 相关字段:
 *   - productId:  产品 ID (OneNet 后台产品详情页可见)
 *   - deviceName: 设备名 (OneNet 后台设备详情页可见)
 *   - deviceAccessKey: 设备级 AccessKey (Base64 字符串, 用于生成 token)
 *   - tokenExpireSec: token 有效期 (秒), 默认 86400 = 1 天
 */

const STORAGE_KEY = 'ebr_config_v3'

/**
 * @brief 默认配置 (OneNet 产品 6YlUqr9lue / 设备 balancing_dog)
 * @note  accessKey 用**设备级** AccessKey (跟 ESP32 端同一把) 即可:
 *        开放接口 (query-device-property / set-device-desired-property /
 *        call-service) 都通。另一把产品级的 key 反而会返回 10403 无效鉴权。
 * @note  project_id 选填: 期望值下发和服务调用, 带不带平台都收。
 */
const DEFAULT_CONFIG = {
  // === OneNet 设备三元组 ===
  productId: '6YlUqr9lue',
  deviceName: 'balancing_dog',
  deviceAccessKey: 'YlpFVXlONlMzTUNnMHRocTRXSG04SkZMbmVrdEdmQTI=',

  // === HTTP API 调优 ===
  tokenExpireSec: 86400,   // 24h

  // === 内部标识 ===
  clientId: 'app_' + Math.random().toString(36).slice(2, 10)
}

let _config = null

export function initConfig() {
  if (_config) return _config
  try {
    const stored = uni.getStorageSync(STORAGE_KEY)
    if (stored && typeof stored === 'object') {
      _config = Object.assign({}, DEFAULT_CONFIG, stored)
    } else {
      _config = Object.assign({}, DEFAULT_CONFIG)
    }
  } catch (e) {
    console.warn('[config] 读取 storage 失败, 用默认值:', e)
    _config = Object.assign({}, DEFAULT_CONFIG)
  }
  return _config
}

export function getConfig() {
  if (!_config) initConfig()
  return _config
}

/**
 * @brief 本地是否已经配过 OneNet (产品ID/设备名/AccessKey 都填了)
 * @return {boolean} 配全了返回 true
 * @note  用来决定 App 启动后要不要自动做一次连通性检查
 */
export function hasConfig() {
  const c = getConfig()
  return !!(c.productId && c.deviceName && c.deviceAccessKey)
}

export function saveConfig(patch) {
  if (!_config) initConfig()
  Object.assign(_config, patch)
  try {
    uni.setStorageSync(STORAGE_KEY, _config)
    console.log('[config] 已保存:', _config)
  } catch (e) {
    console.error('[config] 写 storage 失败:', e)
  }
  return _config
}

export function resetConfig() {
  _config = Object.assign({}, DEFAULT_CONFIG)
  uni.setStorageSync(STORAGE_KEY, _config)
  return _config
}
