<template>
  <view class="container">
    <!-- 顶部状态条 -->
    <view class="top-bar">
      <view class="conn-state" :class="'conn-' + state">
        <text class="dot"></text>
        <text>{{ connText }}</text>
      </view>
      <view class="device-id" @click="goConnect">
        <text>{{ devicePath }} ›</text>
      </view>
    </view>

    <!-- 控制方式: 合体 (对接后两车同时控制) / 分体 (主车、副车分别控制) -->
    <view class="speed-card">
      <view class="card-title">控制方式</view>
      <view class="speed-options">
        <view class="speed-pill" :class="{ active: target === 'both', disabled: !docked }" @click="onTarget('both')">合体</view>
        <view class="speed-pill" :class="{ active: target === 'main' }" @click="onTarget('main')">主车</view>
        <view class="speed-pill" :class="{ active: target === 'sub' }" @click="onTarget('sub')">副车</view>
      </view>
      <text class="mode-hint">{{ modeHint }}</text>
    </view>

    <!-- 急停 / 起立 / 配平 (移到顶部) -->
    <view class="action-bar">
      <view class="big-btn btn-stop" @click="onEmergencyStop">
        <text>急停</text>
      </view>
      <view v-if="target !== 'both'" class="big-btn btn-stand" @click="onStand">
        <text>起立</text>
      </view>
      <view v-if="target !== 'both'" class="big-btn btn-calib" @click="onCalibrate">
        <text>配平</text>
      </view>
    </view>

    <!-- 速度档位 -->
    <view class="speed-card">
      <view class="card-title">速度档位</view>
      <view class="speed-options">
        <view
          v-for="(opt, i) in speedOptions"
          :key="i"
          class="speed-pill"
          :class="{ active: speed === opt.value }"
          @click="speed = opt.value"
        >
          {{ opt.label }}
        </view>
      </view>
    </view>

    <!-- 主控制盘: 左转-前进-右转 / 中间显示 / 后退 -->
    <view class="pad-card">
      <view class="card-title">方向控制</view>
      <view class="pad">
        <!-- 左转 -->
        <view
          v-if="target !== 'both'"
          class="pad-btn pad-btn-left"
          :class="{ active: held === 'left' }"
          @touchstart="onDown('left')"
          @touchend="onUp"
          @touchcancel="onUp"
          @mousedown="onDown('left')"
          @mouseup="onUp"
          @mouseleave="onUp"
        >
          <text class="pad-icon">◀</text>
          <text class="pad-label">左转</text>
        </view>

        <!-- 前进 -->
        <view
          class="pad-btn pad-btn-top"
          :class="{ active: held === 'forward' }"
          @touchstart="onDown('forward')"
          @touchend="onUp"
          @touchcancel="onUp"
          @mousedown="onDown('forward')"
          @mouseup="onUp"
          @mouseleave="onUp"
        >
          <text class="pad-icon">▲</text>
          <text class="pad-label">前进</text>
        </view>

        <!-- 右转 -->
        <view
          v-if="target !== 'both'"
          class="pad-btn pad-btn-right"
          :class="{ active: held === 'right' }"
          @touchstart="onDown('right')"
          @touchend="onUp"
          @touchcancel="onUp"
          @mousedown="onDown('right')"
          @mouseup="onUp"
          @mouseleave="onUp"
        >
          <text class="pad-icon">▶</text>
          <text class="pad-label">右转</text>
        </view>

        <!-- 后退 -->
        <view
          class="pad-btn pad-btn-bottom"
          :class="{ active: held === 'back' }"
          @touchstart="onDown('back')"
          @touchend="onUp"
          @touchcancel="onUp"
          @mousedown="onDown('back')"
          @mouseup="onUp"
          @mouseleave="onUp"
        >
          <text class="pad-icon">▼</text>
          <text class="pad-label">后退</text>
        </view>

        <!-- 中央提示 -->
        <view class="pad-center">
          <text class="pad-center-text">{{ target === 'both' ? '合体行进' : '原地转向' }}</text>
          <text class="pad-center-sub">档位: {{ speedLabel }}</text>
        </view>
      </view>
    </view>

    <!-- 对接模块 (只有主车有) -->
    <view class="pad-card" v-if="target !== 'sub'">
      <view class="card-title">对接模块</view>

      <!-- 执行对接按钮 -->
      <view class="docking-row">
        <button class="magnet-btn" :class="{ active: magnetState.on }" @click="onToggleMagnet">
          {{ magnetState.on ? '取消对接' : '执行对接' }}
        </button>
      </view>

      <!-- 微动开关状态 -->
      <view class="switch-status">
        <text class="switch-title">对接检测</text>
        <view class="switch-indicators">
          <view class="switch-item">
            <view class="switch-dot" :class="{ pressed: switchState.sw1 }"></view>
            <text class="switch-name">开关1</text>
            <text class="switch-desc">左</text>
          </view>
          <view class="switch-align">
            <text v-if="switchState.sw1 && switchState.sw2" class="align-ok">✓ 对正</text>
            <text v-else-if="switchState.sw1 || switchState.sw2" class="align-miss">△ 歪了</text>
            <text v-else class="align-none">— 未吸合</text>
          </view>
          <view class="switch-item">
            <view class="switch-dot" :class="{ pressed: switchState.sw2 }"></view>
            <text class="switch-name">开关2</text>
            <text class="switch-desc">右</text>
          </view>
        </view>
      </view>
    </view>

    <!-- 姿态控制 (快捷姿态 + 跳舞) -->
    <view class="pad-card">
      <view class="card-title">姿态控制</view>
      <view class="pose-btns">
        <view class="pose-btn pose-btn-squat" @click="onPoseSquat">
          <text class="pose-icon">⊓</text>
          <text class="pose-label">蹲下</text>
        </view>
        <view class="pose-btn pose-btn-stand" @click="onPoseStand">
          <text class="pose-icon">⊏</text>
          <text class="pose-label">起立</text>
        </view>
        <view class="pose-btn pose-btn-dance" :class="{ active: dancing }" @click="onPoseDance">
          <text class="pose-icon">♪</text>
          <text class="pose-label">{{ dancing ? '停止' : '跳舞' }}</text>
        </view>
      </view>
      <view class="pose-more" @click="goPosePage">
        <text>姿态页 (跳舞频率可调) ›</text>
      </view>
    </view>

    <!-- 舵机控制 (范围 30-120° 与 ESP32 servo.h 同步) -->
    <view class="pad-card">
      <view class="card-title">舵机控制</view>
      <view class="servo-item">
        <text class="servo-label">当前角度</text>
        <text class="servo-value">{{ servoAngle }}°</text>
      </view>
      <slider :value="servoAngle" min="30" max="120" step="1" show-value active-color="#1aad19" @change="e => servoAngle = e.detail.value" />
      <view class="servo-btns">
        <button class="servo-btn" @click="onServoLeft">左</button>
        <button class="servo-btn servo-btn-center" @click="onServoBoth">双</button>
        <button class="servo-btn" @click="onServoRight">右</button>
      </view>
    </view>

    <!-- RGB 彩灯控制 (SK6812 只有 RGB 三通道) -->
    <view class="pad-card">
      <view class="card-title">RGB 彩灯</view>
      <view class="rgb-sliders">
        <view class="rgb-item">
          <text class="rgb-label" style="color:#ff4444">R</text>
          <slider :value="ledState.r" min="0" max="255" step="1" show-value active-color="#ff4444" block-size="28" @changing="e => onRgbChange(e, 'r')" />
        </view>
        <view class="rgb-item">
          <text class="rgb-label" style="color:#44ff44">G</text>
          <slider :value="ledState.g" min="0" max="255" step="1" show-value active-color="#44ff44" block-size="28" @changing="e => onRgbChange(e, 'g')" />
        </view>
        <view class="rgb-item">
          <text class="rgb-label" style="color:#4444ff">B</text>
          <slider :value="ledState.b" min="0" max="255" step="1" show-value active-color="#4444ff" block-size="28" @changing="e => onRgbChange(e, 'b')" />
        </view>
        <view class="rgb-item">
          <text class="rgb-label" style="color:#1aad19">亮度</text>
          <slider :value="ledState.brightness" min="0" max="255" step="1" show-value active-color="#1aad19" block-size="28" @changing="e => onRgbChange(e, 'brightness')" />
        </view>
      </view>
    </view>

    <!-- 提示 -->
    <view class="tip-mask" v-if="tip">
      <view class="tip-box"><text>{{ tip }}</text></view>
    </view>
  </view>
</template>

<script>
import { oneApi } from '../../common/oneapi.js'
import { controller } from '../../utils/control.js'
import { getConfig } from '../../common/config.js'
import { queryDeviceProperty } from '../../common/onenet.js'

export default {
  data() {
    return {
      state: 'idle',
      held: null,
      /* 速度档位 (PWM 百分比): 必须与车上的 motor_spd_levels[] 一致,
       * 也就是屏上"编码器调试"页调出来的 20/40/60/80 四档 */
      speed: 60,
      speedOptions: [
        { label: '慢', value: 20 },
        { label: '中', value: 40 },
        { label: '快', value: 60 },
        { label: '极速', value: 80 }
      ],
      _cfg: { productId: '', deviceName: '', subDeviceName: '' },
      /* 控制对象: 'both' 合体 / 'main' 主车 / 'sub' 副车 */
      target: controller.target,
      tip: '',
      // RGBW 灯
      ledState: {
        on: false,
        r: 255, g: 0, b: 0, w: 0,
        brightness: 255,
        preset: -1
      },
      colorPresets: [
        { name: '关', color: '#333' },
        { name: '红', color: '#ff4444' },
        { name: '绿', color: '#44ff44' },
        { name: '蓝', color: '#4444ff' },
        { name: '黄', color: '#ffff44' },
        { name: '青', color: '#44ffff' },
        { name: '紫', color: '#ff44ff' },
        { name: '白', color: '#ffffff' }
      ],
      // 电磁铁
      magnetState: { on: false },
      // 微动开关状态
      switchState: { sw1: false, sw2: false },
      // 舵机 (范围 30-120° 与 ESP32 servo.h 同步: SERVO_LIMIT_MIN=30, SERVO_LIMIT_MAX=120)
      servoAngle: 100,
      // 跳舞状态
      dancing: false,
      _danceTimer: null,
      _dancePhase: false, // false=蹲下(100°), true=起立(40°)
      /**
       * App 最近一次下发给设备的值 + 下发时刻。
       * 用途: 轮询到设备上报的值时, 只有"跟 App 下发的值不一样"才认为是
       * 外部 (物理开关/其它会话) 改的, 这时用设备的值覆盖界面;
       * 刚下发过的那几秒内不覆盖, 免得把用户正在拖的滑块弹回去。
       */
      _sent: {
        magnet_on: null, led_on: null, led_color: null,
        led_brightness: null, servo1_angle: null
      },
      _sentAt: {}
    }
  },
  computed: {
    connText() {
      switch (this.state) {
        case 'ready': return '已就绪'
        case 'error': return '错误'
        default:      return '未连接'
      }
    },
    speedLabel() {
      const s = this.speedOptions.find(o => o.value === this.speed)
      return s ? `${s.label} ${s.value}%` : ''
    },
    devicePath() {
      const c = this._cfg || {}
      const name = this.target === 'both' ? `${c.deviceName} + ${c.subDeviceName}`
                 : this.target === 'sub'  ? c.subDeviceName : c.deviceName
      return '$sys/' + c.productId + '/' + name
    },
    /** 对接成功: 电磁铁吸合 + 两个微动开关都压下 */
    docked() {
      return this.magnetState.on && this.switchState.sw1 && this.switchState.sw2
    },
    modeHint() {
      if (this.target === 'both') return '合体: 两车同时收指令, 关平衡直驱轮子 (暂不支持转向)'
      if (this.target === 'sub') return '分体: 只控制副车'
      return this.docked ? '已对接, 可切到合体' : '分体: 只控制主车'
    }
  },
  watch: {
    /* 对接成功自动进合体; 取消对接自动回分体 (主车) */
    docked(v) {
      if (v && this.target !== 'both') this._applyTarget('both')
      if (!v && this.target === 'both') this._applyTarget('main')
    }
  },
  onShow() {
    const c = getConfig()
    this._cfg = { productId: c.productId, deviceName: c.deviceName, subDeviceName: c.subDeviceName }
    this.target = controller.target
    this._unsubState = oneApi.onState((s) => { this.state = s })
    this._startSwitchPoll()
  },
  onUnload() {
    if (this._unsubState) this._unsubState()
    if (this._pollTimer) clearInterval(this._pollTimer)
    if (this._danceTimer) clearInterval(this._danceTimer)
    controller.stopMove()
  },
  onHide() {
    controller.stopMove()
    this.held = null
    this._stopDance()
  },
  methods: {
    /**
     * @brief 用户点控制方式
     * @param {string} t 'both' | 'main' | 'sub'
     */
    onTarget(t) {
      if (t === 'both' && !this.docked) {
        this._showTip('还没对接成功, 不能合体')
        return
      }
      this._applyTarget(t)
    },
    /**
     * @brief 切换控制对象
     * @note  进合体时给两车发急停 (关掉各自平衡), 之后方向键走车端开环直驱
     */
    _applyTarget(t) {
      this.held = null
      controller.setTarget(t)
      this.target = t
      if (t === 'both') controller.emergencyStop()
      this._showTip(t === 'both' ? '合体控制' : t === 'sub' ? '控制副车' : '控制主车')
    },
    onDown(dir) {
      this.held = dir
      controller.startMove(dir, this.speed)
    },
    onUp() {
      this.held = null
      controller.stopMove()
    },
    onEmergencyStop() {
      controller.emergencyStop()
      this.held = null
      this._showTip('急停已发送')
    },
    onStand() {
      controller.stand()
      this._showTip('起立指令已发送')
    },
    onCalibrate() {
      controller.calibrate()
    },
    goConnect() {
      uni.navigateTo({ url: '/pages/connect/connect' })
    },
    goPosePage() {
      uni.navigateTo({ url: '/pages/pose/pose' })
    },
    _showTip(msg) {
      this.tip = msg
      if (this._tipTimer) clearTimeout(this._tipTimer)
      this._tipTimer = setTimeout(() => { this.tip = '' }, 1500)
    },
    // RGBW 灯控制
    onLedSwitch(e) {
      this.ledState.on = e.detail.value
      this.ledState.preset = -1
    },
    onPresetColor(index) {
      this.ledState.preset = index
      const presets = [
        { r: 0, g: 0, b: 0, w: 0 },
        { r: 255, g: 0, b: 0, w: 0 },
        { r: 0, g: 255, b: 0, w: 0 },
        { r: 0, g: 0, b: 255, w: 0 },
        { r: 255, g: 255, b: 0, w: 0 },
        { r: 0, g: 255, b: 255, w: 0 },
        { r: 255, g: 0, b: 255, w: 0 },
        { r: 255, g: 255, b: 255, w: 255 }
      ]
      const p = presets[index]
      if (p) {
        this.ledState.r = p.r; this.ledState.g = p.g
        this.ledState.b = p.b; this.ledState.w = p.w
      }
      this.ledState.on = index !== 0
    },
    onRgbChange(e, channel) {
      this.ledState[channel] = e.detail.value
      this.ledState.preset = -1
      this.ledState.on = true
      // 滑动时直接发送，不 await 避免阻塞
      this._sendLed()
    },
    onSendLedControl() {
      this._sendLed()
        .then(() => this._showTip('灯控制已发送'))
        .catch(e => controller.reportError(e))
    },
    /**
     * @brief 下发灯控制
     * @note  led_color 用 0x00RRGGBB (和设备端解析一致); SK6812 只有 RGB 三通道
     */
    _sendLed() {
      const { r, g, b, brightness, on } = this.ledState
      const colorVal = (r << 16) | (g << 8) | b
      this._markSent({ led_on: on ? 1 : 0, led_color: colorVal, led_brightness: brightness })
      return controller.sendLed({ led_on: on ? 1 : 0, led_color: colorVal, led_brightness: brightness })
    },

    /**
     * @brief 记下"App 刚下发了什么值"
     * @param {Object} values 形如 { led_on: 1 }
     */
    _markSent(values) {
      const now = Date.now()
      Object.keys(values).forEach(k => {
        this._sent[k] = values[k]
        this._sentAt[k] = now
      })
    },

    /**
     * @brief 用设备上报的值同步界面
     * @param {string} key  标识符
     * @param {number} val  设备上的值
     * @note  刚下发过 (2.5s 内) 不覆盖, 防止把用户正在拖的滑块弹回去
     */
    _syncFromDevice(key, val) {
      const now = Date.now()
      if (now - (this._sentAt[key] || 0) < 2500) return
      if (Number(this._sent[key]) === Number(val)) return
      this._sent[key] = val
      this._sentAt[key] = now
      if (key === 'magnet_on') this.magnetState.on = !!Number(val)
      if (key === 'led_on') this.ledState.on = !!Number(val)
      if (key === 'led_brightness') this.ledState.brightness = Number(val)
      if (key === 'led_color') {
        const rgb = Number(val)
        this.ledState.r = (rgb >> 16) & 0xFF
        this.ledState.g = (rgb >> 8) & 0xFF
        this.ledState.b = rgb & 0xFF
      }
      if (key === 'servo1_angle') this.servoAngle = Number(val)
    },

    onToggleMagnet() {
      this.magnetState.on = !this.magnetState.on
      this._markSent({ magnet_on: this.magnetState.on ? 1 : 0 })
      controller.sendMagnet({ magnet_on: this.magnetState.on ? 1 : 0 })
        .then(() => this._showTip(this.magnetState.on ? '执行对接' : '取消对接'))
        .catch(e => controller.reportError(e))
    },
    // 微动开关轮询
    _startSwitchPoll() {
      if (this._pollTimer) clearInterval(this._pollTimer)
      this._pollTimer = setInterval(() => this._fetchSwitch(), 500)
      this._fetchSwitch()
    },
    async _fetchSwitch() {
      try {
        /* 对接状态 (电磁铁 / 微动开关) 只看主车; 灯/舵机跟当前控制的车 */
        const res = await queryDeviceProperty('main')
        /* 平台返回: { code:0, data:[{identifier, value, ...}] } */
        if (res && res.code === 0 && Array.isArray(res.data)) {
          const props = {}
          res.data.forEach(item => { props[item.identifier] = item.value })
          this.switchState.sw1 = !!Number(props.sw1)
          this.switchState.sw2 = !!Number(props.sw2)
          if (props.magnet_on !== undefined) this._syncFromDevice('magnet_on', props.magnet_on)
          if (this.target === 'sub') {
            const r2 = await queryDeviceProperty('sub')
            if (!(r2 && r2.code === 0 && Array.isArray(r2.data))) return
            Object.keys(props).forEach(k => delete props[k])
            r2.data.forEach(item => { props[item.identifier] = item.value })
          }

          /* 执行器状态以**车上的实际值**为准:
           * 电磁铁/灯可能被物理开关或上一轮会话改过,
           * App 瞎猜的话按钮显示的状态跟车上对不上。 */
          const SYNC_KEYS = ['led_on', 'led_brightness', 'led_color', 'servo1_angle']
          SYNC_KEYS.forEach(k => {
            if (props[k] !== undefined) this._syncFromDevice(k, props[k])
          })
        }
      } catch (e) {
        // 忽略查询错误
      }
    },
    onSendMagnet() {
      controller.sendMagnet({ magnet_on: this.magnetState.on ? 1 : 0 })
        .then(() => this._showTip('电磁铁已发送'))
        .catch(e => controller.reportError(e))
    },
    // 舵机控制 (左=舵机1, 右=舵机2, 双=两路一起)
    onServoLeft() {
      this.onSendServo('servo1_angle')
    },
    onServoBoth() {
      this.onSendServo('both')
    },
    onServoRight() {
      this.onSendServo('servo2_angle')
    },
    /**
     * @brief 下发舵机角度
     * @param {string} which 'servo1_angle' | 'servo2_angle' | 'both'
     */
    onSendServo(which) {
      const payload = {}
      if (which === 'servo1_angle' || which === 'both') payload.servo1_angle = this.servoAngle
      if (which === 'servo2_angle' || which === 'both') payload.servo2_angle = this.servoAngle
      const label = which === 'servo1_angle' ? '左' : which === 'servo2_angle' ? '右' : '双'
      this._markSent(payload)
      controller.sendServo(payload)
        .then(() => this._showTip(`舵机 ${label} 已设为 ${this.servoAngle}°`))
        .catch(e => controller.reportError(e))
    },

    // ========== 姿态控制 ==========

    /**
     * @brief 蹲下姿态 (100°)
     */
    onPoseSquat() {
      this._stopDance()
      this.servoAngle = 100
      const payload = { servo1_angle: 100, servo2_angle: 100 }
      this._markSent(payload)
      controller.sendServo(payload)
        .then(() => this._showTip('蹲下姿态 100°'))
        .catch(e => controller.reportError(e))
    },

    /**
     * @brief 起立姿态 (40°)
     */
    onPoseStand() {
      this._stopDance()
      this.servoAngle = 40
      const payload = { servo1_angle: 40, servo2_angle: 40 }
      this._markSent(payload)
      controller.sendServo(payload)
        .then(() => this._showTip('起立姿态 40°'))
        .catch(e => controller.reportError(e))
    },

    /**
     * @brief 跳舞: 周期性切换 40° ↔ 100° (1Hz 频率)
     */
    onPoseDance() {
      if (this.dancing) {
        this._stopDance()
      } else {
        this._startDance()
      }
    },

    /**
     * @brief 启动跳舞定时器 (1Hz, 每 1s 切换一次姿态)
     */
    _startDance() {
      this.dancing = true
      this._dancePhase = false
      this._showTip('开始跳舞 ♪')
      
      // 立即执行第一次
      this._danceStep()
      
      // 每 1s 切换一次
      this._danceTimer = setInterval(() => {
        this._danceStep()
      }, 1000)
    },

    /**
     * @brief 停止跳舞
     */
    _stopDance() {
      if (this._danceTimer) {
        clearInterval(this._danceTimer)
        this._danceTimer = null
      }
      if (this.dancing) {
        this.dancing = false
        this._showTip('停止跳舞')
      }
    },

    /**
     * @brief 执行一次跳舞动作
     */
    _danceStep() {
      this._dancePhase = !this._dancePhase
      const angle = this._dancePhase ? 40 : 100
      this.servoAngle = angle
      
      const payload = { servo1_angle: angle, servo2_angle: angle }
      this._markSent(payload)
      controller.sendServo(payload).catch(e => {
        // 跳舞时发送失败不弹错误，避免打断
        console.error('跳舞指令发送失败', e)
      })
    }
  }
}
</script>

<style scoped>
.container {
  padding-bottom: 40rpx;
}

.top-bar {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 20rpx 32rpx;
  background: #fff;
  border-bottom: 1rpx solid #eee;
}

.conn-state {
  display: flex;
  align-items: center;
  font-size: 26rpx;
  font-weight: 500;
}

.conn-state .dot {
  width: 16rpx;
  height: 16rpx;
  border-radius: 50%;
  margin-right: 12rpx;
}

.conn-ready .dot { background: #1aad19; }
.conn-ready { color: #1aad19; }
.conn-error .dot { background: #d9534f; }
.conn-error { color: #d9534f; }
.conn-idle .dot { background: #aaa; }
.conn-idle { color: #888; }

.device-id {
  font-size: 26rpx;
  color: #888;
}

.speed-card {
  background: #fff;
  margin: 20rpx;
  border-radius: 12rpx;
  padding: 20rpx 24rpx;
  box-shadow: 0 2rpx 8rpx rgba(0, 0, 0, 0.06);
}

.card-title {
  font-size: 30rpx;
  font-weight: 600;
  color: #1aad19;
  margin-bottom: 16rpx;
  border-left: 6rpx solid #1aad19;
  padding-left: 12rpx;
}

.speed-options {
  display: flex;
  gap: 16rpx;
}

.speed-pill {
  flex: 1;
  text-align: center;
  padding: 16rpx 0;
  border-radius: 60rpx;
  background: #f5f5f5;
  color: #666;
  font-size: 28rpx;
  border: 2rpx solid transparent;
  transition: all 0.15s;
}

.speed-pill.active {
  background: #1aad19;
  color: #fff;
  border-color: #1aad19;
  font-weight: 600;
}

.pad-card {
  background: #fff;
  margin: 20rpx;
  border-radius: 12rpx;
  padding: 24rpx;
  box-shadow: 0 2rpx 8rpx rgba(0, 0, 0, 0.06);
}

/* 十字键布局: 左转-前进-右转 在同一行，pad-center 在中间 */
.pad {
  display: grid;
  grid-template-columns: 1fr 1fr 1fr;
  grid-template-rows: 1fr 1fr 1fr;
  gap: 12rpx;
  margin-top: 20rpx;
  height: 360rpx;
}

.pad-btn {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  background: #f7f7f7;
  border-radius: 16rpx;
  border: 2rpx solid #e0e0e0;
  user-select: none;
  -webkit-user-select: none;
  transition: all 0.1s;
}

.pad-btn.active {
  background: #1aad19;
  border-color: #168b16;
  transform: scale(0.96);
}

.pad-btn.active .pad-icon,
.pad-btn.active .pad-label {
  color: #fff;
}

.pad-icon {
  font-size: 52rpx;
  color: #1aad19;
  line-height: 1;
}

.pad-label {
  font-size: 24rpx;
  color: #666;
  margin-top: 8rpx;
}

/* 左转 - 第二行第一列（与 pad-center 同行） */
.pad-btn-left { grid-area: 2 / 1 / 3 / 2; }
/* 前进 - 第一行第二列 */
.pad-btn-top { grid-area: 1 / 2 / 2 / 3; }
/* 右转 - 第二行第三列（与 pad-center 同行） */
.pad-btn-right { grid-area: 2 / 3 / 3 / 4; }
/* 后退 - 第三行第二列 */
.pad-btn-bottom { grid-area: 3 / 2 / 4 / 3; }

.pad-center {
  grid-area: 2 / 2 / 3 / 3;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  background: #fafafa;
  border-radius: 16rpx;
  color: #999;
  text-align: center;
  border: 2rpx dashed #e8e8e8;
}

.pad-center-text {
  font-size: 26rpx;
}

.pad-center-sub {
  font-size: 22rpx;
  margin-top: 8rpx;
  color: #bbb;
}

.speed-pill.disabled {
  opacity: 0.4;
}
.mode-hint {
  display: block;
  margin-top: 12rpx;
  font-size: 22rpx;
  color: #888;
}

/* 对接模块样式 */
.docking-row {
  display: flex;
  align-items: center;
  gap: 16rpx;
  padding: 8rpx 0;
}

.docking-btn,
.magnet-btn {
  padding: 10rpx 32rpx;
  border-radius: 8rpx;
  font-size: 26rpx;
  background: #f0f0f0;
  color: #666;
  border: 1rpx solid #ddd;
  line-height: 1.6;
}

.magnet-btn.active {
  background: #d9534f;
  color: #fff;
  border-color: #c9302c;
}

.switch-status {
  margin-top: 20rpx;
  padding-top: 16rpx;
  border-top: 1rpx solid #eee;
}

.switch-title {
  font-size: 26rpx;
  color: #666;
  display: block;
  margin-bottom: 12rpx;
}

.switch-indicators {
  display: flex;
  align-items: center;
  justify-content: space-between;
}

.switch-item {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 6rpx;
}

.switch-dot {
  width: 36rpx;
  height: 36rpx;
  border-radius: 50%;
  background: #e0e0e0;
  border: 2rpx solid #ccc;
  transition: all 0.2s;
}

.switch-dot.pressed {
  background: #1aad19;
  border-color: #168b16;
  box-shadow: 0 0 8rpx rgba(26, 173, 25, 0.5);
}

.switch-name {
  font-size: 24rpx;
  color: #666;
}

.switch-desc {
  font-size: 22rpx;
  color: #aaa;
}

.switch-align {
  text-align: center;
}

.align-ok {
  font-size: 30rpx;
  color: #1aad19;
  font-weight: 600;
}

.align-miss {
  font-size: 30rpx;
  color: #f0ad4e;
  font-weight: 600;
}

.align-none {
  font-size: 26rpx;
  color: #aaa;
}

/* RGBW 彩灯样式 */
.led-row {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 12rpx 0;
}

.led-label {
  font-size: 28rpx;
  color: #333;
  min-width: 140rpx;
}

.color-presets {
  display: flex;
  flex-wrap: wrap;
  gap: 12rpx;
  padding: 12rpx 0;
}

.color-btn {
  width: 80rpx;
  height: 60rpx;
  border-radius: 10rpx;
  display: flex;
  align-items: center;
  justify-content: center;
  border: 4rpx solid transparent;
  transition: all 0.2s;
}

.color-btn.active {
  border-color: #1aad19;
  transform: scale(1.05);
}

.color-btn-text {
  font-size: 20rpx;
  color: #fff;
  text-shadow: 0 1rpx 2rpx rgba(0,0,0,0.5);
}

.rgb-sliders {
  padding: 8rpx 0;
}

.rgb-item {
  display: flex;
  align-items: center;
  padding: 4rpx 0;
  min-height: 60rpx;
}

.rgb-item uni-slider,
.rgb-item uni-slider > div {
  width: 100% !important;
}

.rgb-label {
  width: 40rpx;
  font-size: 26rpx;
  font-weight: 600;
  color: #333;
}

/* 舵机样式 */
.servo-item {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 8rpx 0;
}

.servo-label {
  font-size: 26rpx;
  color: #666;
}

.servo-value {
  font-size: 28rpx;
  font-weight: 600;
  color: #1aad19;
  font-family: monospace;
}

.servo-btns {
  display: flex;
  gap: 16rpx;
  padding: 16rpx 0 8rpx;
}

.servo-btn {
  flex: 1;
  padding: 16rpx;
  border-radius: 8rpx;
  font-size: 28rpx;
  text-align: center;
  background: #f0f0f0;
  color: #333;
  border: 1rpx solid #ddd;
}

.servo-btn-center {
  background: #1aad19;
  color: #fff;
  border-color: #168b16;
}

.btn-warning {
  background: #f0ad4e;
  color: #fff;
  border: none;
  border-radius: 8rpx;
  padding: 16rpx 32rpx;
  font-size: 28rpx;
}

.action-bar {
  display: flex;
  gap: 16rpx;
  padding: 0 20rpx;
  margin: 20rpx 0;
}

.big-btn {
  flex: 1;
  height: 120rpx;
  display: flex;
  align-items: center;
  justify-content: center;
  border-radius: 16rpx;
  font-size: 34rpx;
  font-weight: 600;
  color: #fff;
  user-select: none;
  transition: all 0.1s;
}

.big-btn:active { transform: scale(0.96); }

.btn-stop { background: #d9534f; }
.btn-stand { background: #1aad19; }
.btn-calib { background: #f0ad4e; }

/* 姿态控制样式 */
.pose-btns {
  display: flex;
  gap: 16rpx;
  padding: 8rpx 0;
}

/* 姿态页入口 */
.pose-more {
  padding: 12rpx 0 0;
  text-align: right;
  font-size: 24rpx;
  color: #1aad19;
}

.pose-btn {
  flex: 1;
  height: 140rpx;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  border-radius: 16rpx;
  background: #f7f7f7;
  border: 2rpx solid #e0e0e0;
  user-select: none;
  transition: all 0.1s;
}

.pose-btn:active {
  transform: scale(0.96);
}

.pose-btn.active {
  background: #1aad19;
  border-color: #168b16;
}

.pose-btn.active .pose-icon,
.pose-btn.active .pose-label {
  color: #fff;
}

.pose-icon {
  font-size: 52rpx;
  color: #1aad19;
  line-height: 1;
  margin-bottom: 8rpx;
}

.pose-label {
  font-size: 26rpx;
  color: #666;
  font-weight: 500;
}

.pose-btn-squat .pose-icon { color: #f0ad4e; }
.pose-btn-stand .pose-icon { color: #1aad19; }
.pose-btn-dance .pose-icon { color: #5bc0de; }

.pose-btn.active .pose-icon {
  color: #fff;
}

.tip-mask {
  position: fixed;
  top: 0; left: 0; right: 0; bottom: 0;
  display: flex;
  align-items: center;
  justify-content: center;
  pointer-events: none;
  z-index: 999;
}

.tip-box {
  background: rgba(0, 0, 0, 0.75);
  color: #fff;
  padding: 20rpx 40rpx;
  border-radius: 12rpx;
  font-size: 30rpx;
}
</style>
