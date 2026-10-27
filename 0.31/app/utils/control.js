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
 *        原来用的 `vx` / `w` 在物模型里并不存在, 平台会回
 *        10412 "identifier not exist", 指令根本下发不下去。
 * @note  下发频率: 值变化时立即发, 另外每 1s 补发一次保活
 *        (平台只在期望值变化时 push, 补发是为了让设备在重连后能恢复指令)。
 *
 * @note  移动指令的编码 (物模型属性 `speed`, 范围 -10~10):
 *        0                        → 停止
 *        ±0.2 / ±0.4 / ±0.6 / ±0.8 → 前进(+) / 后退(-), 档位 20/40/60/80 (%)
 *        ±1.2 / ±1.4 / ±1.6 / ±1.8 → 左转(+) / 右转(-), 档位 20/40/60/80 (%)
 *        档位跟车上的 UI 编码器调试页同源 (motor_spd_levels[]),
 *        所以 App 上的档位和屏上调出来的档位是一回事。
 *        ⚠ 2026-10-26: 档位现在代表"目标轮速的百分比", 车端把它换算成
 *           目标轮速后走速度环 (不再直接给 PWM)。
 */

import { oneApi } from '../common/oneapi.js'
import { setDesiredProperty, invokeService } from '../common/onenet.js'
import { PARAM_VER } from '../common/paramDef.js'

/** 移动指令的属性名 (物模型里必须存在这个标识符) */
const MOVE_PROP = 'speed'

/** 保活补发周期 (毫秒) */
const KEEPALIVE_MS = 1000

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
  }

  /** 检查 OneNet API 是否就绪 */
  _check() {
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
   * @note  设备收到后走**速度环**: 档位换算成目标轮速 → 目标倾角 → 平衡环。
   *        平衡环全程在线, 不会因为按方向键就被暂停
   *        (2026-10-26 改: 以前是"直驱", 直接给 PWM, 平衡环整个暂停 ——
   *         车站着的时候一推, 平衡环等于被关掉了, 松手时车已经歪了,
   *         平衡环接手必然剧烈纠偏 → "一让走就剧烈抖")。
   * @note  车没起立 (平衡没开) 时, 设备才退回开环直驱, 方便调电机/编码器。
   * @note  平台的 speed 属性范围是 -10~10, 指令值由 _dirToCmd() 编码。
   */
  startMove(dir, level = 60) {
    if (!this._check()) return
    if (!(dir in DIR_SIGN)) return
    this.active = { [MOVE_PROP]: this._dirToCmd(dir, level) }
    this._publishMove()
    if (this.timer) clearInterval(this.timer)
    this.timer = setInterval(() => this._publishMove(), KEEPALIVE_MS)
  }

  /** 停止移动 (目标速度归零) */
  stopMove() {
    this.active = { [MOVE_PROP]: 0 }
    if (this.timer) {
      clearInterval(this.timer)
      this.timer = null
    }
    this._publishMove()
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

  /** 推一帧移动指令 */
  async _publishMove() {
    try {
      await setDesiredProperty(this.active)
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
      const msg = (e && e.message) ? e.message : String(e)
      uni.showToast({ title: msg.slice(0, 30), icon: 'none' })
    }
  }

  /** 急停 (单次服务调用) */
  async emergencyStop() {
    if (!this._check()) return
    if (this.timer) { clearInterval(this.timer); this.timer = null }
    this.active = { [MOVE_PROP]: 0 }
    try {
      await invokeService('stop', {})
    } catch (e) {
      this.reportError(e)
    }
  }

  /** 起立 */
  async stand() {
    if (!this._check()) return
    try {
      await invokeService('stand', {})
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
          invokeService('calibrate', {}).catch(e => this.reportError(e))
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
      const res = await invokeService('param_get', {})
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
      const r1 = await invokeService('param_set', { values: JSON.stringify(payload) })
      if (!r1) return false
      const r2 = await invokeService('param_save', {})
      return !!r2
    } catch (e) {
      this.reportError(e)
      throw e
    }
  }

  /**
   * @brief 让**车端**把参数恢复成它自己的出厂默认值 (并落盘)
   * @return {Promise<boolean>} 是否成功
   * @note  2026-10-26: 以前"恢复默认"是把 App 内置的默认值表整份下发,
   *        而那份表早就跟车端脱节 (还是 v7 的老值), 结果把调好的参数覆盖成
   *        "速度环开着 + 输出上限 30%" → 车只朝一个方向转, 根本稳不住。
   *        现在改成下发 {"_reset":true}, 由车端 param_reset_default() 恢复,
   *        默认值永远以车端 param.c 的表为准。
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
      })
      if (!r1) return false
      const r2 = await invokeService('param_save', {})
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
      await setDesiredProperty(payload)
    } catch (e) {
      this.reportError(e)
      throw e
    }
  }

  /**
   * @brief 发送电磁铁控制
   * @param {Object} payload { magnet_on }
   */
  async sendMagnet(payload) {
    if (!this._check()) return
    try {
      await setDesiredProperty(payload)
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
      await setDesiredProperty(payload)
    } catch (e) {
      this.reportError(e)
      throw e
    }
  }

  /* ================= 配平表标定 (0.31 新增) =================
   * 车腿高度由舵机角决定, 换腿高后"平衡姿态"的车身俯仰角会差约 10°,
   * 配平表 (舵机角 → 平衡俯仰角) 就是用来自动补偿这个差的。
   * 车端有 4 个服务专门做这件事, 这里一一对应封装。 */

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
      const res = await invokeService('trim_record', {})
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
      const res = await invokeService('trim_save', {})
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
      const res = await invokeService('trim_clear', {})
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
      const res = await invokeService('trim_get', {})
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
