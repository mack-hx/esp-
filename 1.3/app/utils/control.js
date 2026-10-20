/**
 * @file control.js
 * @brief 控制指令封装 (走 OneNet HTTP API, 不再走 MQTT)
 * @date 2026-10-10
 *
 * 命令下发分两类:
 *   - 持续指令 (vx/vy/w 移动): 走 SetDeviceDesiredProperty 写期望值
 *       设备订阅 $sys/.../thing/property/desired/get/reply 时会收到
 *   - 单次动作 (急停 / 起立 / 配平): 走 invokeService('stop' / 'stand' / 'calibrate')
 *   - 参数读写: 走 invokeService('param_get' / 'param_set' / 'param_save')
 *
 * 频率: 移动指令 50ms 一帧, 跟之前一样.
 *   - 期望值是"幂等"的, 重复写相同值不会触发设备回调, 改用 vx 极小时做归零判断
 *   - 后端是 HTTP POST, 一次访问就一帧, 没有 50ms 合并之类的优化
 */

import { oneApi } from '../common/oneapi.js'
import { setDesiredProperty, invokeService } from '../common/onenet.js'
import { getConfig } from '../common/config.js'

class Controller {
  constructor() {
    this.active = { vx: 0, vy: 0, w: 0 }
    this.timer = null
    this.PERIOD_MS = 50
    this.projectId = ''   // 从 connect 页填, 存在 storage 里
  }

  /** 读 projectId (从 storage) */
  _getProjectId() {
    try {
      return uni.getStorageSync('ebr_project_id_v1') || ''
    } catch (e) { return '' }
  }

  /** 检查 OneNet API 是否就绪 */
  _check() {
    if (oneApi.getState().state !== 'ready') {
      uni.showToast({ title: '请先在连接页配置 OneNet', icon: 'none' })
      return false
    }
    const pid = this._getProjectId()
    if (!pid) {
      uni.showToast({ title: '缺少项目 ID, 请到连接页填', icon: 'none' })
      return false
    }
    return true
  }

  /**
   * @brief 持续发移动指令 (50ms 一帧, HTTP POST)
   */
  startMove(dir, speed = 0.6) {
    if (!this._check()) return
    this.active = this._dirToVel(dir, speed)
    this._publishMove()
    if (this.timer) clearInterval(this.timer)
    this.timer = setInterval(() => this._publishMove(), this.PERIOD_MS)
  }

  stopMove() {
    this.active = { vx: 0, vy: 0, w: 0 }
    if (this.timer) {
      clearInterval(this.timer)
      this.timer = null
    }
    this._publishMove()
  }

  _dirToVel(dir, speed) {
    switch (dir) {
      case 'forward': return { vx: +speed, vy: 0, w: 0 }
      case 'back':    return { vx: -speed, vy: 0, w: 0 }
      case 'left':    return { vx: 0,     vy: 0, w: +speed }
      case 'right':   return { vx: 0,     vy: 0, w: -speed }
      default:        return { vx: 0,     vy: 0, w: 0 }
    }
  }

  async _publishMove() {
    const pid = this._getProjectId()
    try {
      await setDesiredProperty(pid, this.active)
    } catch (e) {
      oneApi.markError(e.message)
    }
  }

  /** 急停 (单次) - 走服务调用 */
  async emergencyStop() {
    if (!this._check()) return
    if (this.timer) { clearInterval(this.timer); this.timer = null }
    this.active = { vx: 0, vy: 0, w: 0 }
    try {
      await invokeService('stop', {})
    } catch (e) {
      oneApi.markError(e.message)
    }
  }

  /** 起立 */
  async stand() {
    if (!this._check()) return
    try {
      await invokeService('stand', {})
    } catch (e) {
      oneApi.markError(e.message)
    }
  }

  /** 配平 */
  calibrate() {
    if (!this._check()) return
    uni.showModal({
      title: '配平标定',
      content: '请把车摆正, 然后点确定. 标定过程约 3 秒, 期间不要动车.',
      success: (res) => {
        if (res.confirm) {
          invokeService('calibrate', {}).catch(e => oneApi.markError(e.message))
          uni.showLoading({ title: '配平中...' })
          setTimeout(() => uni.hideLoading(), 3500)
        }
      }
    })
  }

  /** 读参数 (返回 values 对象)
   * 物模型定义 output: values: string (JSON 字符串), 需要 parse */
  async getParams() {
    if (!this._check()) return null
    try {
      const res = await invokeService('param_get', {})
      // res: { code: 0, data: { values: '{"bal_kp":40,...}' } }
      if (res && res.data && res.data.values) {
        return JSON.parse(res.data.values)
      }
      return null
    } catch (e) {
      oneApi.markError(e.message)
      return null
    }
  }

  /** 写参数 (物模型 input: values: string, 传 JSON 字符串)
   * param_set 成功后再调 param_save 落盘 */
  async setParams(values) {
    if (!this._check()) return false
    try {
      // values 是 JS 对象, 物模型 string 类型, 先 stringify
      await invokeService('param_set', { values: JSON.stringify(values) })
      await invokeService('param_save', {})
      return true
    } catch (e) {
      oneApi.markError(e.message)
      return false
    }
  }
}

export const controller = new Controller()
