<template>
  <view class="container">
    <!-- 连接状态条 -->
    <view class="top-bar">
      <view class="conn-state" :class="'conn-' + state">
        <text class="dot"></text>
        <text>{{ connText }}</text>
      </view>
      <text class="last-seen" v-if="lastSeen">最近心跳: {{ lastSeen }}s 前</text>
      <text class="last-seen" v-else>无数据</text>
    </view>

    <!-- 姿态 -->
    <view class="card">
      <view class="card-title">姿态</view>
      <view class="pose-grid">
        <view class="pose-item">
          <text class="pose-label">Pitch</text>
          <text class="pose-value" :class="poseClass('pitch')">{{ fmt(pose.pitch, 1) }}°</text>
        </view>
        <view class="pose-item">
          <text class="pose-label">Roll</text>
          <text class="pose-value">{{ fmt(pose.roll, 1) }}°</text>
        </view>
        <view class="pose-item">
          <text class="pose-label">Yaw</text>
          <text class="pose-value">{{ fmt(pose.yaw, 1) }}°</text>
        </view>
        <view class="pose-item">
          <text class="pose-label">Gz</text>
          <text class="pose-value">{{ fmt(pose.gz, 1) }}°/s</text>
        </view>
      </view>
      <!-- 简易姿态条 -->
      <view class="pose-bar">
        <view class="pose-bar-track">
          <view class="pose-bar-fill" :style="pitchBarStyle"></view>
          <view class="pose-bar-center"></view>
        </view>
        <text class="pose-bar-text">倾斜方向</text>
      </view>
    </view>

    <!-- 电机 -->
    <view class="card">
      <view class="card-title">电机</view>
      <view class="motor-row">
        <view class="motor-cell">
          <text class="motor-label">左轮</text>
          <text class="motor-value">{{ fmt(motor.left, 0) }}%</text>
          <view class="motor-bar">
            <view class="motor-fill" :style="motorBarStyle('left')"></view>
          </view>
        </view>
        <view class="motor-cell">
          <text class="motor-label">右轮</text>
          <text class="motor-value">{{ fmt(motor.right, 0) }}%</text>
          <view class="motor-bar">
            <view class="motor-fill" :style="motorBarStyle('right')"></view>
          </view>
        </view>
      </view>
      <view class="row">
        <text class="label">速度</text>
        <text class="value">{{ fmt(motor.speed, 2) }} m/s</text>
      </view>
      <view class="row">
        <text class="label">编码器</text>
        <text class="value">L: {{ motor.enc_l || 0 }} / R: {{ motor.enc_r || 0 }}</text>
      </view>
    </view>

    <!-- 电池 -->
    <view class="card">
      <view class="card-title">电池</view>
      <view class="row">
        <text class="label">电压</text>
        <text class="value" :class="battClass">{{ fmt(battery.voltage, 2) }} V</text>
      </view>
      <view class="row">
        <text class="label">电量</text>
        <text class="value">{{ battery.percent || '--' }} %</text>
      </view>
      <view class="batt-bar">
        <view class="batt-bar-fill" :class="battBarClass" :style="battBarStyle"></view>
      </view>
    </view>

    <!-- 设备信息 -->
    <view class="card">
      <view class="card-title">设备</view>
      <view class="row">
        <text class="label">设备 ID</text>
        <text class="value">{{ deviceId }}</text>
      </view>
      <view class="row">
        <text class="label">MQTT 主题</text>
        <text class="value mono">{{ T.base() }}</text>
      </view>
      <view class="row">
        <text class="label">消息数</text>
        <text class="value">{{ msgCount }}</text>
      </view>
    </view>
  </view>
</template>

<script>
import { oneApi } from '../../common/oneapi.js'
import { queryDeviceProperty } from '../../common/onenet.js'
import { getConfig } from '../../common/config.js'

export default {
  data() {
    return {
      T: { base: () => '$sys/' + (getConfig().productId || '') + '/' + (getConfig().deviceName || '') },
      state: 'idle',
      deviceId: '',
      pose: { pitch: 0, roll: 0, yaw: 0, gz: 0 },
      motor: { left: 0, right: 0, speed: 0, enc_l: 0, enc_r: 0 },
      battery: { voltage: 0, percent: 0 },
      deviceState: '--',
      lastSeen: null,
      msgCount: 0,
      _now: Date.now(),
      _pollTick: null
    }
  },
  computed: {
    connText() {
      switch (this.state) {
        case 'ready':    return '已就绪'
        case 'error':    return '错误'
        default:         return '未连接'
      }
    },
    poseClass() {
      return (k) => {
        if (k === 'pitch') {
          const p = Math.abs(this.pose.pitch)
          if (p > 20) return 'pose-warn'
          if (p > 10) return 'pose-mid'
          return ''
        }
        return ''
      }
    },
    pitchBarStyle() {
      // pitch: -45° ~ +45° 映射到 -50% ~ +50% (中心 0%)
      const p = Math.max(-45, Math.min(45, this.pose.pitch))
      const x = (p / 45) * 50
      return `left: ${50 + x}%; transform: translateX(-50%);`
    },
    motorBarStyle() {
      return (k) => {
        const v = this.motor[k] || 0
        const w = Math.max(0, Math.min(100, Math.abs(v)))
        return `width: ${w}%; background: ${v < 0 ? '#d9534f' : '#1aad19'};`
      }
    },
    battClass() {
      const v = this.battery.voltage || 0
      if (v < 6.5) return 'batt-warn'
      if (v < 7.0) return 'batt-mid'
      return ''
    },
    battBarClass() {
      const v = this.battery.voltage || 0
      if (v < 6.5) return 'batt-warn'
      if (v < 7.0) return 'batt-mid'
      return ''
    },
    battBarStyle() {
      const p = this.battery.percent || 0
      return `width: ${Math.max(0, Math.min(100, p))}%;`
    }
  },
  onShow() {
    this.deviceId = (getConfig().productId || '') + '/' + (getConfig().deviceName || '')
    this._unsubState = oneApi.onState((s) => { this.state = s })
    // 轮询: 每 1s 拉一次设备属性
    this._pollTick = setInterval(() => this._poll(), 1000)
    this._poll()
  },
  onUnload() {
    if (this._unsubState) this._unsubState()
    if (this._pollTick) clearInterval(this._pollTick)
  },
  methods: {
    async _poll() {
      if (oneApi.getState().state !== 'ready') return
      try {
        const res = await queryDeviceProperty()
        // 返回: { data: { list: [{identifier, value, time, data_type}, ...] } }
        if (res && res.success && res.data && Array.isArray(res.data.list)) {
          const obj = {}
          res.data.list.forEach(p => { obj[p.identifier] = p.value })
          this._bumpSeen()
          this.msgCount++
          if (obj.pitch   !== undefined) this.pose.pitch  = Number(obj.pitch)
          if (obj.roll    !== undefined) this.pose.roll   = Number(obj.roll)
          if (obj.yaw     !== undefined) this.pose.yaw    = Number(obj.yaw)
          if (obj.gz      !== undefined) this.pose.gz     = Number(obj.gz)
          if (obj.motor_l !== undefined) this.motor.left  = Number(obj.motor_l)
          if (obj.motor_r !== undefined) this.motor.right = Number(obj.motor_r)
          if (obj.speed   !== undefined) this.motor.speed = Number(obj.speed)
          if (obj.enc_l   !== undefined) this.motor.enc_l = Number(obj.enc_l)
          if (obj.enc_r   !== undefined) this.motor.enc_r = Number(obj.enc_r)
          if (obj.state)                 this.deviceState = String(obj.state)
        }
      } catch (e) {
        // 静默失败, 下个 tick 继续
      }
    },
    _bumpSeen() {
      this.lastSeen = 0
    },
    fmt(v, dec) {
      if (v === null || v === undefined || Number.isNaN(v)) return '--'
      return Number(v).toFixed(dec)
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

.last-seen {
  font-size: 24rpx;
  color: #888;
}

.card-title {
  font-size: 30rpx;
  font-weight: 600;
  color: #1aad19;
  margin-bottom: 16rpx;
  border-left: 6rpx solid #1aad19;
  padding-left: 12rpx;
}

.pose-grid {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 16rpx 20rpx;
  margin-bottom: 24rpx;
}

.pose-item {
  display: flex;
  justify-content: space-between;
  align-items: baseline;
  padding: 8rpx 0;
}

.pose-label { font-size: 26rpx; color: #888; }
.pose-value { font-size: 34rpx; font-weight: 600; color: #333; font-family: monospace; }
.pose-mid { color: #f0ad4e; }
.pose-warn { color: #d9534f; }

.pose-bar {
  margin-top: 20rpx;
}

.pose-bar-track {
  position: relative;
  width: 100%;
  height: 16rpx;
  background: #f0f0f0;
  border-radius: 8rpx;
  margin-bottom: 8rpx;
}

.pose-bar-center {
  position: absolute;
  left: 50%;
  top: -4rpx;
  bottom: -4rpx;
  width: 2rpx;
  background: #999;
  transform: translateX(-50%);
}

.pose-bar-fill {
  position: absolute;
  top: 0;
  width: 6rpx;
  height: 100%;
  background: #1aad19;
  border-radius: 3rpx;
  transition: left 0.2s, transform 0.2s;
}

.pose-bar-text {
  font-size: 22rpx;
  color: #aaa;
}

.motor-row {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 20rpx;
  margin-bottom: 20rpx;
}

.motor-cell {
  display: flex;
  flex-direction: column;
  gap: 8rpx;
}

.motor-label { font-size: 24rpx; color: #888; }
.motor-value { font-size: 30rpx; font-weight: 600; color: #333; font-family: monospace; }

.motor-bar {
  width: 100%;
  height: 12rpx;
  background: #f0f0f0;
  border-radius: 6rpx;
  overflow: hidden;
}

.motor-fill {
  height: 100%;
  background: #1aad19;
  border-radius: 6rpx;
  transition: width 0.2s, background 0.2s;
}

.batt-bar {
  width: 100%;
  height: 18rpx;
  background: #f0f0f0;
  border-radius: 9rpx;
  overflow: hidden;
  margin-top: 16rpx;
}

.batt-bar-fill {
  height: 100%;
  background: #1aad19;
  border-radius: 9rpx;
  transition: width 0.3s, background 0.3s;
}

.batt-mid { color: #f0ad4e; }
.batt-warn { color: #d9534f; }
.batt-bar-fill.batt-mid { background: #f0ad4e; }
.batt-bar-fill.batt-warn { background: #d9534f; }

.mono { font-family: monospace; font-size: 24rpx; }
</style>
