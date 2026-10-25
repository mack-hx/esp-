/**
 * @file oneapi.js
 * @brief OneNet HTTP API 连接状态机 (替代原 mqtt.js)
 * @date 2026-10-10
 *
 * 原因: App 端不再走 MQTT, 改走 OneNet HTTP API.
 *   - 状态机: 'idle' | 'ready' | 'error'
 *   - ready: 一次 token 计算成功的状态; token 失效 (HTTP 401) 时退到 error
 *   - 业务层用 onState(cb) 订阅状态变化
 *
 * 不在这里保存 token, 每次调用 onenet.js 里的接口都现算 (HMAC 算起来很轻量, 不缓存).
 */

class OneApiManager {
  constructor() {
    this.state = 'idle'   // 'idle' | 'ready' | 'error'
    this.error = null
    /** token 上次计算成功的 Unix 毫秒, 用于决定是否需要重算 */
    this._lastTokenAt = 0
    /** token 提前 5 分钟过期, 强制重算 */
    this._TOKEN_TTL_MS = 50 * 60 * 1000
    this.stateListeners = new Set()
  }

  getState() {
    return { state: this.state, error: this.error }
  }

  onState(cb) {
    this.stateListeners.add(cb)
    cb(this.state, this.error)
    return () => this.stateListeners.delete(cb)
  }

  _setState(state, error = null) {
    this.state = state
    this.error = error
    this.stateListeners.forEach(cb => {
      try { cb(state, error) } catch (e) { console.error(e) }
    })
  }

  /**
   * @brief 主动"就绪"检查: 真的调一次平台接口, 不只是算 token
   *   算 token 成功不代表有权限, 设备级 key 也能算出 token, 但一调接口就被拒。
   *   所以这里直接拉一次设备属性, 能拿到数据才算配置正确。
   * @return {Promise<boolean>} true=配置可用
   */
  async ensureReady() {
    try {
      const { queryDeviceProperty } = await import('./onenet.js')
      const res = await queryDeviceProperty()
      if (res && res.code === 0) {
        this._lastTokenAt = Date.now()
        this._setState('ready', null)
        return true
      }
      const msg = (res && res.msg) ? '平台返回: ' + res.msg : '平台返回异常'
      this._setState('error', msg)
      return false
    } catch (e) {
      this._setState('error', e.message || String(e))
      return false
    }
  }

  /**
   * @brief token 是否需要重算
   */
  tokenStale() {
    return Date.now() - this._lastTokenAt > this._TOKEN_TTL_MS
  }

  /**
   * @brief 标记 token 计算 / HTTP 401 错误
   */
  markError(msg) {
    this._setState('error', msg)
  }
}

export const oneApi = new OneApiManager()
