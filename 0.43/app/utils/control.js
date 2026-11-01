/**
 * @file control.js
 * @brief 控制指令封装 (走 OneNet 开放接口)
 * @date 2026-10-21
 *
 * 命令下发分两类:
 *   - 持续指令 (移动 / 灯 / 舵机 / 电磁铁): 走 setDesiredProperty
 *       → 平台把期望值推给设备的 $sys/.../thing/property/set 消息
 *   - 单次动作 (急停 / 起立 / 配平 / 参数读写): 走 invokeService
 *       → 平台把服务调用推给设备的 $sys/.../thing/service/{name}/invoke 消息
 *
 * @note  移动指令走物模型里已有的 `speed` 属性 (标幺 -1~1):
 *        平台会把期望值 push 给设备, 设备直接当速度环目标用。
 * @note  下发频率: 值变化时立即发, 另外每 1s 补发一次保活
 *        (平台只在期望值变化时 push, 补发是为了让设备在重连后能恢复指令)。
 *
 * @note  移动指令的编码 (物模型属性 `speed`, 范围 -10~10):
 *        0                        → 停止
 *        ±0.2 / ±0.4 / ±0.6 / ±0.8 → 前进(+) / 后退(-), 档位 20/40/60/80 (%)
 *        ±1.2 / ±1.4 / ±1.6 / ±1.8 → 左转(+) / 右转(-), 档位 20/40/60/80 (%)
 *        档位跟车上的 UI 编码器调试页同源 (motor_spd_levels[]),
 *        所以 App 上的档位和屏上调出来的档位是一回事。
 *        档位代表"目标轮速的百分比", 车端把它换算成目标轮速后走速度环。
 *
 * @note  控制对象 (target):
 *        'main' 主车 / 'sub' 副车 / 'both' 两车 (对接合体后, 同一指令同时发给两个设备)。
 *        参数 / 配平这类单设备操作在 'both' 时落到主车。
 *
 * @note  蓝牙直控: 某辆车的蓝牙已连上时, 它的移动 / 急停 / 起立走蓝牙 (common/ble.js),
 *        其余指令仍走 OneNet。蓝牙移动指令每 BLE_KEEPALIVE_MS 补发一次 (车端 600ms 超时归零)。
 */

import { oneApi } from '../common/oneapi.js'
import { setDesiredProperty, invokeService } from '../common/onenet.js'
import { PARAM_VER } from '../common/paramDef.js'
import * as ble from '../common/ble.js'

/** 移动指令的属性名 (物模型里必须存在这个标识符) */
const MOVE_PROP = 'speed'

/** 保活补发周期 (毫秒) */
const KEEPALIVE_MS = 1000

/** 蓝牙移动指令补发周期 (毫秒), 必须小于车端 BLE_MOVE_TIMEOUT_MS (600) */
const BLE_KEEPALIVE_MS = 200

/** 速度档位 (PWM 百分比), 必须与车上的 motor_spd_levels[] 一致 */
export const SPEED_LEVELS = [20, 40, 60, 80]

/** 方向 → 指令符号 (前进/左转为正, 后退/右转为负) */
const DIR_SIGN = {
  forward: 1,
  back: -1,
  left: 1,
  right: -1
}

/** 转向指令的基址 (>1 表示转向, 与设备端解码一致) */
const TURN_BASE = 1.0

class Controller {
  constructor() {
    this.active = { speed: 0 }
    this.timer = null
    /** 当前控制对象: 'main' | 'sub' | 'both' */
    this.target = 'main'
    /** 上次通过 OneNet 发移动指令的时刻 (毫秒) */
    this._netMoveAt = 0
  }

  /**
   * @brief 切换控制对象 (切换前先停掉正在下发的移动指令)
   * @param {string} t 'main' | 'sub' | 'both'
   */
  setTarget(t) {
    if (t === this.target) return
    if (this.timer) this.stopMove()
    this.target = t
  }

  /** 当前控制对象展开成设备列表 */
  _devices() {
    return this.target === 'both' ? ['main', 'sub'] : [this.target]
  }

  /** 单设备操作 (参数 / 配平) 用的设备 */
  _single() {
    return this.target === 'sub' ? 'sub' : 'main'
  }

  /** 把同一个期望值同时下发给当前控制对象的所有设备 */
  _setAll(values) {
    return Promise.all(this._devices().map(d => setDesiredProperty(values, d)))
  }

  /** 把同一个服务调用同时发给当前控制对象的所有设备 */
  _invokeAll(name, input) {
    return Promise.all(this._devices().map(d => invokeService(name, input, d)))
  }

  /**
   * @brief 实时指令: 蓝牙已连的车走蓝牙, 其余走 OneNet 服务调用
   * @param bleText 蓝牙指令文本 ("STOP" / "STAND")
   * @param service OneNet 服务名
   */
  _realtimeAll(bleText, service) {
    return Promise.all(this._devices().map(d =>
      ble.isConnected(d) ? ble.send(d, bleText) : invokeService(service, {}, d)))
  }

  /** 当前控制对象是否全部连着蓝牙 */
  _allBle() {
    return this._devices().every(d => ble.isConnected(d))
  }

  /** 检查 OneNet API 是否就绪 (实时指令在所有目标车都连着蓝牙时不需要 OneNet) */
  _check(realtime = false) {
    if (realtime && this._allBle()) return true
    if (oneApi.getState().state !== 'ready') {
      uni.showToast({ title: '请先在连接页配置 OneNet', icon: 'none' })
      return false
    }
    return true
  }

  /**
   * @brief 持续发移动指令 (按下方向键时调用)
   * @param dir   'forward' | 'back' | 'left' | 'right'
   * @param level 档位 (PWM 百分比, 取 SPEED_LEVELS 里的值: 20/40/60/80)
   * @note  车没起立 (平衡没开) 时, 设备才退回开环直驱, 方便调电机/编码器。
   * @note  平台的 speed 属性范围是 -10~10, 指令值由 _dirToCmd() 编码。
   */
  startMove(dir, level = 60) {
    if (!this._check(true)) return
    if (!(dir in DIR_SIGN)) return
    this.active = { [MOVE_PROP]: this._dirToCmd(dir, level) }
    this._publishMove(true)
    if (this.timer) clearInterval(this.timer)
    this.timer = setInterval(() => this._publishMove(false), BLE_KEEPALIVE_MS)
  }

  /** 停止移动 (目标速度归零) */
  stopMove() {
    this.active = { [MOVE_PROP]: 0 }
    if (this.timer) {
      clearInterval(this.timer)
      this.timer = null
    }
    this._publishMove(true)
  }

  /**
   * @brief 方向 + 档位 → 指令值 (设备端 apply_move_cmd() 的逆运算)
   * @param dir   方向
   * @param level 档位 (PWM 百分比)
   * @return 指令值
   *
   * 编码规则 (与设备端一致):
   *   前进/后退: ±档位/100                → ±0.2 / ±0.4 / ±0.6 / ±0.8
   *   左转/右转: ±(1 + 档位/100)          → ±1.2 / ±1.4 / ±1.6 / ±1.8
   */
  _dirToCmd(dir, level) {
    const lv = Number(level) / 100
    const sign = DIR_SIGN[dir]
    const isTurn = (dir === 'left' || dir === 'right')
    return +(sign * (isTurn ? (TURN_BASE + lv) : lv)).toFixed(2)
  }

  /**
   * @brief 推一帧移动指令
   * @param force true = OneNet 也立即发; false = OneNet 按 KEEPALIVE_MS 节流 (蓝牙每次都发)
   */
  async _publishMove(force) {
    const now = Date.now()
    const netDue = force || now - this._netMoveAt >= KEEPALIVE_MS
    if (netDue) this._netMoveAt = now
    const v = this.active[MOVE_PROP]
    try {
      await Promise.all(this._devices().map(d => {
        if (ble.isConnected(d)) return ble.send(d, 'M:' + v)
        return netDue ? setDesiredProperty(this.active, d) : null
      }))
    } catch (e) {
      this.reportError(e)
    }
  }

  /**
   * @brief 统一的失败处理 (页面也可以直接调这个来显示错误)
   * @param {Error} e 平台返回的错误
   * @note  只有**鉴权类**错误 (AccessKey 配错) 才把连接状态打成错误;
   *        单条指令失败 (物模型缺属性之类的) 只提示, 不影响其它指令。
   */
  reportError(e) {
    if (e && e.auth) {
      oneApi.markError(e.message)
    }
    if (e && e.code === 10412) {
      uni.showToast({ title: '物模型缺这个属性, 请在 OneNet 后台补上', icon: 'none' })
    } else {
      const msg = (e && (e.message || e.errMsg)) || String(e)
      uni.showToast({ title: msg.slice(0, 30), icon: 'none' })
    }
  }

  /** 急停 (单次服务调用) */
  async emergencyStop() {
    if (!this._check(true)) return
    if (this.timer) { clearInterval(this.timer); this.timer = null }
    this.active = { [MOVE_PROP]: 0 }
    try {
      await this._realtimeAll('STOP', 'stop')
    } catch (e) {
      this.reportError(e)
    }
  }

  /** 起立 */
  async stand() {
    if (!this._check(true)) return
    try {
      await this._realtimeAll('STAND', 'stand')
    } catch (e) {
      this.reportError(e)
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
          this._invokeAll('calibrate', {}).catch(e => this.reportError(e))
          uni.showLoading({ title: '配平中...' })
          setTimeout(() => uni.hideLoading(), 3500)
        }
      }
    })
  }

  /**
   * @brief 读全部 PID 参数
   * @return {Promise<Object|null>} { bal_kp: 40, ... }
   * @note  出错时往上抛, 让页面能把平台的具体原因显示出来
   *        (比如物模型里 values 的长度限制不够, 平台会回 "string length error")
   */
  async getParams() {
    if (!this._check()) return null
    try {
      const res = await invokeService('param_get', {}, this._single())
      /* 物模型 output: values (string), 内容是设备给的扁平 JSON 字符串 */
      if (res && res.data && res.data.values) {
        return JSON.parse(res.data.values)
      }
      return null
    } catch (e) {
      this.reportError(e)
      throw e
    }
  }

  /**
   * @brief 写全部 PID 参数 (写完自动落盘)
   * @param {Object} values 形如 { bal_kp: 40, ... }
   * @return {Promise<boolean>} 是否成功
   * @note  出错时往上抛, 让页面能把平台的具体原因显示出来
   */
  async setParams(values) {
    if (!this._check()) return false
    try {
      /* 物模型 input: values (string), 所以要 stringify。
       * 带上 "_v" = App 内置的参数表版本号, 车端比对不一致就告警
       * (见 onenet_client.c 的 param_set 分支)。 */
      const payload = Object.assign({}, values, { _v: PARAM_VER })
      const r1 = await invokeService('param_set', { values: JSON.stringify(payload) }, this._single())
      if (!r1) return false
      const r2 = await invokeService('param_save', {}, this._single())
      return !!r2
    } catch (e) {
      this.reportError(e)
      throw e
    }
  }

  /**
   * @brief 让**车端**把参数恢复成它自己的出厂默认值 (并落盘)
   * @return {Promise<boolean>} 是否成功
   */
  async resetParams() {
    if (!this._check()) return false
    try {
      /* 物模型 input: values (string), 所以要 stringify。
       * 带上 "_v" = App 内置的参数表版本号, 车端比对不一致就告警
       * (见 onenet_client.c 的 param_set 分支)。
       * ⚠ 恢复默认也必须带 "_v": 车端把"没有版本号"当成"旧版 App"标记,
       *   会给用户弹提示条说参数可能被旧版 App 改坏。 */
      const r1 = await invokeService('param_set', {
        values: JSON.stringify({ _reset: true, _v: PARAM_VER })
      }, this._single())
      if (!r1) return false
      const r2 = await invokeService('param_save', {}, this._single())
      return !!r2
    } catch (e) {
      this.reportError(e)
      throw e
    }
  }

  /**
   * @brief 发送灯控制
   * @param {Object} payload { led_on, led_color, led_brightness }
   * @note  这几个属性物模型里还没有, 平台会回 10412, 提示用户去补物模型
   */
  async sendLed(payload) {
    if (!this._check()) return
    try {
      await this._setAll(payload)
    } catch (e) {
      this.reportError(e)
      throw e
    }
  }

  /**
   * @brief 发送电磁铁控制 (只有主车有对接模块)
   * @param {Object} payload { magnet_on }
   */
  async sendMagnet(payload) {
    if (!this._check()) return
    try {
      await setDesiredProperty(payload, 'main')
    } catch (e) {
      this.reportError(e)
      throw e
    }
  }

  /**
   * @brief 发送舵机控制
   * @param {Object} payload { servo1_angle, servo2_angle }
   */
  async sendServo(payload) {
    if (!this._check()) return
    try {
      await this._setAll(payload)
    } catch (e) {
      this.reportError(e)
      throw e
    }
  }


  /**
   * @brief 记下一个配平标定点 (当前舵机角 → 当前原始 pitch)
   * @return {Promise<string|null>} 车端回的文本, 例如 "ok, phi=60, pitch=-1.23, n=3"
   * @note  记之前要**用手把车身扶到接近平衡**, 电机全程刹车, 车不能起立。
   * @note  车端存的是**原始 pitch** (没扣任何补偿的那个), 不是页面上显示的
   *        修正后 pitch —— 修正链里还会再扣一次, 存错了会重复扣。
   */
  async trimRecord() {
    if (!this._check()) return null
    try {
      const res = await invokeService('trim_record', {}, this._single())
      if (res && res.data && res.data.values) return String(res.data.values)
      return null
    } catch (e) {
      this.reportError(e)
      throw e
    }
  }

  /**
   * @brief 把配平表落盘 (NVS, 断电不丢)
   * @return {Promise<string|null>} 车端回的文本
   */
  async trimSave() {
    if (!this._check()) return null
    try {
      const res = await invokeService('trim_save', {}, this._single())
      if (res && res.data && res.data.values) return String(res.data.values)
      return null
    } catch (e) {
      this.reportError(e)
      throw e
    }
  }

  /**
   * @brief 清空配平表并落盘 (换机械结构后用)
   * @return {Promise<string|null>} 车端回的文本
   */
  async trimClear() {
    if (!this._check()) return null
    try {
      const res = await invokeService('trim_clear', {}, this._single())
      if (res && res.data && res.data.values) return String(res.data.values)
      return null
    } catch (e) {
      this.reportError(e)
      throw e
    }
  }

  /**
   * @brief 读回配平表
   * @return {Promise<Object|null>} { n, ref, pz, pts: [[舵机角, pitch], ...] }
   * @note  车端回的 values 是一段 JSON 字符串, 这里解析成对象方便页面用。
   *        ref = 参考角 (servo_c), pz = 配平零点 (°),
   *        pts = 每个点的 [舵机角, 原始 pitch]。
   */
  async trimGet() {
    if (!this._check()) return null
    try {
      const res = await invokeService('trim_get', {}, this._single())
      if (res && res.data && res.data.values) {
        return JSON.parse(res.data.values)
      }
      return null
    } catch (e) {
      this.reportError(e)
      throw e
    }
  }

}

export const controller = new Controller()
