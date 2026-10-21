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

    <!-- 急停 / 起立 / 配平 -->
    <view class="action-bar">
      <view class="big-btn btn-stop" @click="onEmergencyStop">
        <text>急停</text>
      </view>
      <view class="big-btn btn-stand" @click="onStand">
        <text>起立</text>
      </view>
      <view class="big-btn btn-calib" @click="onCalibrate">
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

    <!-- 主控制盘 -->
    <view class="pad-card">
      <view class="card-title">方向控制</view>

      <!-- 十字键容器 -->
      <view class="dpad">
        <!-- 左侧: 左转 -->
        <view
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

        <!-- 中间列: 上(前进) + 中间显示 + 下(后退) -->
        <view class="dpad-center-col">
          <!-- 前进 -->
          <view
            class="pad-btn pad-center-btn"
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

          <!-- 中间显示 -->
          <view class="pad-center">
            <text class="pad-center-text">双轮差速</text>
            <text class="pad-center-sub">档位: {{ speedLabel }}</text>
          </view>

          <!-- 后退 -->
          <view
            class="pad-btn pad-center-btn"
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
        </view>

        <!-- 右侧: 右转 -->
        <view
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
      </view>

      <!-- 舵机角度滑动条 -->
      <view class="servo-section">
        <view class="card-title">舵机角度</view>
        <view class="servo-slider-row">
          <text class="servo-bound">{{ servoMin }}</text>
          <slider
            class="servo-slider"
            :min="servoMin"
            :max="servoMax"
            :step="1"
            :value="servoAngle"
            :block-size="22"
            active-color="#1aad19"
            background-color="#e0e0e0"
            @change="onServoAngleChange"
          />
          <text class="servo-bound">{{ servoMax }}</text>
        </view>
        <view class="servo-value-display">
          <text class="servo-value-text">{{ servoAngle }}°</text>
        </view>
        <!-- 左/双/右 按钮 -->
        <view class="servo-btn-row">
          <view class="servo-btn" @click="applyServo(0)">
            <text>◀ 左</text>
          </view>
          <view class="servo-btn" @click="applyServo(2)">
            <text>◈ 双</text>
          </view>
          <view class="servo-btn" @click="applyServo(1)">
            <text>右 ▶</text>
          </view>
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

export default {
  data() {
    return {
      state: 'idle',
      held: null,
      speed: 0.6,
      speedOptions: [
        { label: '慢', value: 0.3 },
        { label: '中', value: 0.6 },
        { label: '快', value: 0.9 }
      ],
      _cfg: { productId: '', deviceName: '' },
      tip: '',
      // 舵机参数 (与 ESP32 servo.h 同步: SERVO_LIMIT_MIN=30, SERVO_LIMIT_MAX=120, SERVO_DEFAULT_ANGLE=100)
      servoMin: 30,
      servoMax: 120,
      servoAngle: 100
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
      return s ? s.label : ''
    },
    devicePath() {
      return '$sys/' + (this._cfg || {}).productId + '/' + (this._cfg || {}).deviceName
    }
  },
  onShow() {
    const c = getConfig()
    this._cfg = { productId: c.productId, deviceName: c.deviceName }
    this._unsubState = oneApi.onState((s) => { this.state = s })
  },
  onUnload() {
    if (this._unsubState) this._unsubState()
    controller.stopMove()
  },
  onHide() {
    controller.stopMove()
    this.held = null
  },
  methods: {
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
    onServoAngleChange(e) {
      this.servoAngle = e.detail.value
    },
    async applyServo(servoId) {
      // servoId: 0=左舵机(servo1), 1=右舵机(servo2), 2=同时设置两路
      await controller.setServoAngle(servoId, this.servoAngle)
      const names = ['左舵机', '右舵机', '双舵机']
      this._showTip(`${names[servoId]} → ${this.servoAngle}°`)
    },
    goConnect() {
      uni.navigateTo({ url: '/pages/connect/connect' })
    },
    _showTip(msg) {
      this.tip = msg
      if (this._tipTimer) clearTimeout(this._tipTimer)
      this._tipTimer = setTimeout(() => { this.tip = '' }, 1500)
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

/* 急停/起立/配平 顶部动作条 */
.action-bar {
  display: flex;
  gap: 12rpx;
  padding: 20rpx;
}

/* 十字键布局 */
/* 十字键布局 — 使用 Grid 确保三列高度一致、垂直居中 */
.dpad {
  display: grid;
  grid-template-columns: 140rpx 1fr 140rpx;
  align-items: center;
  gap: 12rpx;
  margin-bottom: 16rpx;
}

/* 中间列高度 = 前进100 + gap12 + 中间80 + gap12 + 后退100 = 304rpx */
.dpad-center-col {
  display: flex;
  flex-direction: column;
  justify-content: space-between;
  align-items: center;
  height: 304rpx;
}

/* 十字键按钮 */
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

/* 左右按钮 — 与中间列等高 304rpx */
.pad-btn-left,
.pad-btn-right {
  height: 304rpx;
  flex-shrink: 0;
  align-self: stretch;
}

/* 中间上下按钮 */
.pad-center-btn {
  width: 200rpx;
  height: 100rpx;
  flex-shrink: 0;
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

/* 中间显示区 */
.pad-center {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  background: #fafafa;
  border-radius: 16rpx;
  color: #999;
  text-align: center;
  border: 2rpx dashed #e8e8e8;
  width: 200rpx;
  height: 80rpx;
}

.pad-center-text {
  font-size: 26rpx;
}

.pad-center-sub {
  font-size: 22rpx;
  margin-top: 8rpx;
  color: #bbb;
}

/* 舵机控制区 */
.servo-section {
  margin-top: 20rpx;
  padding-top: 16rpx;
  border-top: 1rpx solid #f0f0f0;
}

.servo-slider-row {
  display: flex;
  align-items: center;
  gap: 8rpx;
}

.servo-bound {
  font-size: 24rpx;
  color: #999;
  width: 36rpx;
  text-align: center;
}

.servo-slider {
  flex: 1;
}

.servo-value-display {
  text-align: center;
  margin: 8rpx 0 16rpx;
}

.servo-value-text {
  font-size: 36rpx;
  font-weight: 700;
  color: #1aad19;
}

/* 左/双/右 按钮 */
.servo-btn-row {
  display: flex;
  gap: 12rpx;
}

.servo-btn {
  flex: 1;
  height: 80rpx;
  display: flex;
  align-items: center;
  justify-content: center;
  background: #f5f5f5;
  border-radius: 12rpx;
  border: 2rpx solid #e0e0e0;
  font-size: 28rpx;
  color: #555;
  user-select: none;
  transition: all 0.1s;
}

.servo-btn:active {
  background: #1aad19;
  border-color: #168b16;
  color: #fff;
  transform: scale(0.97);
}

/* 底部大按钮 (已移到 pad-card 顶部, 保留样式复用) */
.big-btn {
  flex: 1;
  height: 88rpx;
  display: flex;
  align-items: center;
  justify-content: center;
  border-radius: 12rpx;
  font-size: 30rpx;
  font-weight: 600;
  color: #fff;
  user-select: none;
  transition: all 0.1s;
}

.big-btn:active { transform: scale(0.96); }

.btn-stop { background: #d9534f; }
.btn-stand { background: #1aad19; }
.btn-calib { background: #f0ad4e; }

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
