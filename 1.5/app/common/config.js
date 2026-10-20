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

const STORAGE_KEY = 'ebr_config_v2'

/**
 * @brief 默认配置 (从 README / 物模型文档复制过来的用户项目)
 */
const DEFAULT_CONFIG = {
  // === OneNet 设备三元组 ===
  productId: '6YlUqr9lue',
  deviceName: 'balancing_dog',
  deviceAccessKey: 'y1uJyXF9uU8jo9Wl21tRdeHX69eNx6Gti21Z85udgZk=',

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
