<template>
  <view class="container">
    <!-- 连接状态条 -->
    <view class="top-bar">
      <view class="conn-state" :class="'conn-' + state">
        <text class="dot"></text>
        <text>{{ connText }}</text>
      </view>
      <text class="last-seen" v-if="lastSeen !== null">心跳: {{ lastSeen }}s</text>
      <text class="last-seen" v-else>无数据</text>
    </view>

    <!-- Pitch 折线图 -->
    <view class="card">
      <view class="card-title">Pitch 实时曲线</view>
      <canvas canvas-id="pitchCanvas" class="chart-canvas" @touchstart="onTouchStart" @touchend="onTouchEnd"></canvas>
      <view class="chart-labels">
        <text class="chart-label">{{ pitchMin }}°</text>
        <text class="chart-label chart-label-center">0°</text>
        <text class="chart-label">{{ pitchMax }}°</text>
      </view>
    </view>

    <!-- 参数入口 (调参统一到"参数"页, 那边跟车上的参数表是一一对应的) -->
    <view class="card">
      <view class="card-title">参数调节</view>
      <view class="pid-item">
        <text class="pid-label">PID / 限幅 / 舵机中位…</text>
      </view>
      <text class="pid-value">共 16 项, 改完记得点"保存"才会下发到车</text>
      <view class="action-row">
        <button class="btn btn-primary" @click="goParams">去调参</button>
      </view>
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
        <text class="label">遥控指令</text>
        <text class="value">{{ moveText }}</text>
      </view>
      <view class="row">
        <text class="label">编码器</text>
        <text class="value">L: {{ motor.enc_l || 0 }} / R: {{ motor.enc_r || 0 }}</text>
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

const MAX_POINTS = 100  // 最多显示100个点
const PITCH_RANGE = 30  // 显示范围 ±30°

export default {
  data() {
    return {
      state: 'idle',
      deviceId: '',
      pose: { pitch: 0, roll: 0, yaw: 0, gz: 0 },
      motor: { left: 0, right: 0, speed: 0, enc_l: 0, enc_r: 0 },
      deviceState: '--',
      lastSeen: null,
      _lastDataAt: 0,
      msgCount: 0,
      // 折线图数据
      pitchHistory: [],
      ctx: null,
      canvasWidth: 0,
      canvasHeight: 0,
      _cssW: 0,      // canvas 的 CSS 像素宽 (绘图坐标用)
      _cssH: 0,      // canvas 的 CSS 像素高 (绘图坐标用)
      isDrawing: false,
      _pollTick: null,
      _animationId: null
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
    pitchMin() { return '-' + PITCH_RANGE },
    pitchMax() { return '+' + PITCH_RANGE },
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
    motorBarStyle() {
      return (k) => {
        const v = this.motor[k] || 0
        const w = Math.max(0, Math.min(100, Math.abs(v)))
        return `width: ${w}%; background: ${v < 0 ? '#d9534f' : '#1aad19'};`
      }
    },
    /**
     * @brief 遥控指令文字
     * @note  `speed` 属性是打包指令 (见 app/utils/control.js 的编码说明):
     *        0=停止, ±0.2~0.8=前进/后退, ±1.2~1.8=左转/右转
     */
    moveText() {
      const v = Number(this.motor.speed) || 0
      const a = Math.abs(v)
      if (a < 0.05) return '停止'
      const level = Math.round((a > 1.0 ? a - 1.0 : a) * 100)
      let name = ''
      if (a <= 1.0) name = v > 0 ? '前进' : '后退'
      else name = v > 0 ? '左转' : '右转'
      return `${name} ${level}%`
    }
  },
  onShow() {
    this.deviceId = (getConfig().productId || '') + '/' + (getConfig().deviceName || '')
    this._unsubState = oneApi.onState((s) => { this.state = s })
    
    // 高频轮询: 200ms 拉一次数据
    this._pollTick = setInterval(() => this._poll(), 200)
    this._poll()
    
    // 启动绘制循环
    this._startDrawLoop()
    
    // 初始化 Canvas
    this.$nextTick(() => this._initCanvas())
  },
  onUnload() {
    if (this._unsubState) this._unsubState()
    if (this._pollTick) clearInterval(this._pollTick)
    if (this._animationId) cancelAnimationFrame(this._animationId)
  },
  onHide() {
    if (this._pollTick) clearInterval(this._pollTick)
    if (this._animationId) cancelAnimationFrame(this._animationId)
  },
  methods: {
    _initCanvas() {
      // #ifdef H5
      /* ⚠ uni-app 的 H5 端会把 <canvas canvas-id="xxx"> 渲染成
       *   <canvas class="uni-canvas-canvas">, **不会保留 canvas-id 属性**,
       *   所以 document.querySelector('canvas[canvas-id="..."]') 永远取不到,
       *   表现就是曲线框空白。这里按 class 找, 兜底再取第一个 canvas。 */
      const canvas = document.querySelector('canvas.uni-canvas-canvas') ||
                     document.querySelector('canvas')
      if (canvas) {
        this.ctx = canvas.getContext('2d')
        /* canvas.width 是设备像素 (2x), ctx.scale(2,2) 后绘图用 CSS 像素,
         * 所以 canvasWidth/Height 存的是 CSS 尺寸 (绘制坐标不用除以 2) */
        const w = canvas.offsetWidth || 300
        const h = canvas.offsetHeight || 150
        this.canvasWidth = canvas.width = w * 2
        this.canvasHeight = canvas.height = h * 2
        this.ctx.scale(2, 2)
        this._cssW = w
        this._cssH = h
      }
      // #endif
    },
    
    _startDrawLoop() {
      const draw = () => {
        /* canvas 可能在页面切换后才渲染出来, 这里兜底再取一次 */
        if (!this.ctx) this._initCanvas()
        if (this.pitchHistory.length > 1) {
          this._drawChart()
        }
        this._animationId = requestAnimationFrame(draw)
      }
      draw()
    },
    
    _drawChart() {
      // #ifdef H5
      if (!this.ctx) return
      
      const ctx = this.ctx
      /* ctx.scale(2,2) 之后绘图坐标用 CSS 像素, 直接用 _cssW/_cssH,
       * 不要再除以 2 (否则图形只画在左上角 1/4 区域) */
      const w = this._cssW || this.canvasWidth / 2 || 300
      const h = this._cssH || this.canvasHeight / 2 || 150
      const padding = 10
      const chartW = w - padding * 2
      const chartH = h - padding * 2
      
      // 清空画布
      ctx.fillStyle = '#fafafa'
      ctx.fillRect(0, 0, w, h)
      
      // 画网格线
      ctx.strokeStyle = '#e0e0e0'
      ctx.lineWidth = 0.5
      ctx.setLineDash([4, 4])
      
      // 水平网格
      for (let i = 0; i <= 4; i++) {
        const y = padding + (chartH / 4) * i
        ctx.beginPath()
        ctx.moveTo(padding, y)
        ctx.lineTo(w - padding, y)
        ctx.stroke()
      }
      
      // 零线
      ctx.strokeStyle = '#888'
      ctx.lineWidth = 1
      ctx.setLineDash([])
      ctx.beginPath()
      const zeroY = padding + chartH / 2
      ctx.moveTo(padding, zeroY)
      ctx.lineTo(w - padding, zeroY)
      ctx.stroke()
      
      // 画折线
      const points = this.pitchHistory
      if (points.length < 2) return
      
      ctx.strokeStyle = '#1aad19'
      ctx.lineWidth = 2
      ctx.setLineDash([])
      ctx.beginPath()
      
      const stepX = chartW / (MAX_POINTS - 1)
      const scaleY = chartH / (PITCH_RANGE * 2)
      
      points.forEach((p, i) => {
        const x = padding + i * stepX
        const y = zeroY - p * scaleY
        if (i === 0) {
          ctx.moveTo(x, y)
        } else {
          ctx.lineTo(x, y)
        }
      })
      ctx.stroke()
      
      // 画数据点
      ctx.fillStyle = '#1aad19'
      points.forEach((p, i) => {
        const x = padding + i * stepX
        const y = zeroY - p * scaleY
        ctx.beginPath()
        ctx.arc(x, y, 3, 0, Math.PI * 2)
        ctx.fill()
      })
      
      // 显示当前值
      const currentPitch = points[points.length - 1]
      ctx.fillStyle = '#333'
      ctx.font = '24px sans-serif'
      ctx.textAlign = 'right'
      ctx.fillText(currentPitch.toFixed(1) + '°', w - padding - 5, padding + 25)
      // #endif
    },
    
    async _poll() {
      if (oneApi.getState().state !== 'ready') return
      /* 刷新"心跳"秒数: 距离上一次收到平台数据过了多久 */
      if (this._lastDataAt) {
        this.lastSeen = Math.floor((Date.now() - this._lastDataAt) / 1000)
      }
      try {
        const res = await queryDeviceProperty()
        /* 平台返回: { code:0, data:[{identifier, value, time, ...}], msg:'succ' } */
        if (res && res.code === 0 && Array.isArray(res.data)) {
          const obj = {}
          res.data.forEach(p => { obj[p.identifier] = p.value })
          this._bumpSeen()
          this.msgCount++
          if (obj.pitch   !== undefined) {
            const newPitch = Number(obj.pitch)
            this.pose.pitch = newPitch
            this.pitchHistory.push(newPitch)
            if (this.pitchHistory.length > MAX_POINTS) {
              this.pitchHistory.shift()
            }
          }
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
        // 静默失败
      }
    },

    _bumpSeen() {
      /* 记录最后一次收到平台数据的时间, 供"心跳"显示用 */
      this._lastDataAt = Date.now()
      this.lastSeen = 0
    },

    fmt(v, dec) {
      if (v === null || v === undefined || Number.isNaN(v)) return '--'
      return Number(v).toFixed(dec)
    },

    onTouchStart() {
      this.isDrawing = true
    },

    onTouchEnd() {
      this.isDrawing = false
    },


    goParams() {
      uni.navigateTo({ url: '/pages/params/params' })
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

.card {
  background: #fff;
  margin: 20rpx;
  border-radius: 12rpx;
  padding: 20rpx;
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

.chart-canvas {
  width: 100%;
  height: 300rpx;
  background: #fafafa;
  border-radius: 8rpx;
}

.chart-labels {
  display: flex;
  justify-content: space-between;
  padding: 8rpx 10rpx 0;
}

.chart-label {
  font-size: 22rpx;
  color: #888;
  font-family: monospace;
}

.chart-label-center {
  position: absolute;
  left: 50%;
  transform: translateX(-50%);
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

.row {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 12rpx 0;
}

.label { font-size: 26rpx; color: #888; }
.value { font-size: 26rpx; color: #333; }

/* PID 样式 */
.pid-item {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 8rpx 0 4rpx;
}

.pid-label {
  font-size: 26rpx;
  color: #666;
  font-family: monospace;
}

.pid-value {
  font-size: 28rpx;
  font-weight: 600;
  color: #1aad19;
  font-family: monospace;
}

.action-row {
  display: flex;
  gap: 16rpx;
  padding: 16rpx 0 8rpx;
}

.btn {
  flex: 1;
  padding: 16rpx;
  border-radius: 8rpx;
  font-size: 28rpx;
  text-align: center;
  border: none;
}

.btn-primary {
  background: #1aad19;
  color: #fff;
}

.btn-ghost {
  background: #f5f5f5;
  color: #666;
  border: 1rpx solid #ddd;
}
</style>
