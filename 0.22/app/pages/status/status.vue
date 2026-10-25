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
      <view class="chart-info">
        <text class="chart-info-item">采样点: {{ pitchSeries.length }}</text>
        <text class="chart-info-item">数据延迟: {{ latencyText }}</text>
      </view>
      <view class="chart-info">
        <text class="chart-info-item">请求耗时: {{ _rttMs }}ms</text>
        <text class="chart-info-item">时钟偏差: {{ skewText }}</text>
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
import { queryDeviceProperty, queryDevicePropertyHistory, getServerNowMs, getServerSkewMs } from '../../common/onenet.js'
import { getConfig } from '../../common/config.js'

const MAX_POINTS = 120  // 最多显示120个点
const PITCH_RANGE = 30  // 显示范围 ±30°
/* 俯仰角高速通道参数:
 *   - 设备端 50ms 上报一次 (20Hz)
 *   - App 每 120ms 拉一次历史, 一次能拿回 10 个点 → 曲线点密度 ~80 点/秒
 *   - 每次拉取的时间窗口 (ms), 要盖住"两次拉取之间"新产生的点 */
const PITCH_POLL_MS   = 120
const PITCH_WINDOW_MS = 3000
/* 查询窗口左右各留的余量 (ms)。
 * 平台的属性历史接口是按时间窗口查的, 如果窗口右端用**手机本地时钟**算,
 * 而手机时钟慢了几秒 (没开自动对时 / 手动改过时间), 查回来的就全是
 * 过去的数据 → 表现为"车已经动了, 要七八秒后才看到曲线变化"。
 * 窗口两端都留足余量, 平台会返回窗口内**最新**的一批点,
 * 手机时钟偏多少都照样能查到实时数据。 */
const PITCH_MARGIN_MS = 30000
/* 重绘周期 (ms):
 *   用定时器而不是 requestAnimationFrame —— App 端的逻辑层没有 DOM,
 *   rAF / document 都是 undefined, 直接调用会抛 ReferenceError。 */
const REDRAW_MS = 100

/**
 * @brief 生成绘图适配器, 屏蔽各端 canvas API 的差异
 * @param ctx  H5: 原生 canvas 2d 上下文; 其它端: uni.createCanvasContext 返回的上下文
 * @param isH5 true = H5 (原生 API), false = App/小程序 (uni 命令式 API)
 * @note 所有方法的坐标单位都是 CSS 像素
 */
function createPainter(ctx, isH5) {
  if (isH5) {
    return {
      fill:      c => { ctx.fillStyle = c },
      stroke:    c => { ctx.strokeStyle = c },
      width:     w => { ctx.lineWidth = w },
      dash:      d => { ctx.setLineDash(d) },
      fillRect:  (x, y, w, h) => ctx.fillRect(x, y, w, h),
      beginPath: () => ctx.beginPath(),
      moveTo:    (x, y) => ctx.moveTo(x, y),
      lineTo:    (x, y) => ctx.lineTo(x, y),
      strokePath:() => ctx.stroke(),
      /* 画一个实心圆点 */
      dot: (x, y, r) => { ctx.beginPath(); ctx.arc(x, y, r, 0, Math.PI * 2); ctx.fill() },
      /* 画一行文字 (y 为基线) */
      text: (t, x, y, size, align) => {
        ctx.font = size + 'px sans-serif'
        ctx.textAlign = align
        ctx.fillText(t, x, y)
      },
      flush: () => {}
    }
  }
  /* App / 小程序: uni 的画布命令 API, 画完必须调 draw() 才真正上屏 */
  return {
    fill:      c => ctx.setFillStyle(c),
    stroke:    c => ctx.setStrokeStyle(c),
    width:     w => ctx.setLineWidth(w),
    dash:      d => ctx.setLineDash(d),
    fillRect:  (x, y, w, h) => ctx.fillRect(x, y, w, h),
    beginPath: () => ctx.beginPath(),
    moveTo:    (x, y) => ctx.moveTo(x, y),
    lineTo:    (x, y) => ctx.lineTo(x, y),
    strokePath:() => ctx.stroke(),
    dot: (x, y, r) => { ctx.beginPath(); ctx.arc(x, y, r, 0, Math.PI * 2); ctx.fill() },
    /* uni 的 fillText 的 y 是文字顶部, 往上挪一点让它跟 H5 的基线一致 */
    text: (t, x, y, size, align) => {
      ctx.setFontSize(size)
      ctx.setTextAlign(align)
      ctx.fillText(t, x, y - size * 0.8)
    },
    flush: () => ctx.draw()
  }
}

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
      // 折线图数据 (高速通道: 每点带毫秒时间戳)
      pitchHistory: [],
      pitchSeries: [],
      pitchLatency: null,
      _lastPointT: 0,   // 最新采样点的平台时间戳 (ms), 用来对齐查询窗口
      _rttMs: 0,        // 最近一次历史接口请求耗时 (ms), 排查网络慢用
      painter: null,    // 绘图适配器 (见 createPainter), null = 还没初始化好
      _cssW: 0,        // canvas 的 CSS 像素宽 (绘图坐标用)
      _cssH: 0,        // canvas 的 CSS 像素高 (绘图坐标用)
      isDrawing: false,
      _pollTick: null,
      _pitchTick: null,
      _redrawTick: null,
      _canvasPending: false, // App 端取画布尺寸的查询是否在进行中
      _lastDrawKey: ''       // 上次绘制时的数据指纹, 没变就不重复画
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
    },
    /**
     * @brief 数据延迟文字
     * @note  最新采样点的平台时间戳与平台当前时间的差值, 用来判断"曲线跟不跟手"
     */
    latencyText() {
      if (this.pitchLatency === null || this.pitchLatency === undefined) return '--'
      return this.pitchLatency + 'ms'
    },
    /**
     * @brief 平台时钟与手机本地时钟的偏差文字
     * @note  正数 = 平台比手机快 (手机本地时钟慢了), 这正是"车动了要好几秒
     *        才看到变化"的元凶 —— 查询窗口必须以平台时间为准
     */
    skewText() {
      const s = getServerSkewMs()
      if (!s) return '--'
      return (s > 0 ? '+' : '') + (s / 1000).toFixed(1) + 's'
    }
  },
  onShow() {
    this.deviceId = (getConfig().productId || '') + '/' + (getConfig().deviceName || '')
    this._unsubState = oneApi.onState((s) => { this.state = s })
    
    // 常规属性轮询: 200ms 拉一次最新值
    this._pollTick = setInterval(() => this._poll(), 200)
    this._poll()

    // 俯仰角高速通道: 120ms 拉一次属性历史 (一次拿回一批采样点)
    this._pitchTick = setInterval(() => this._pollPitch(), PITCH_POLL_MS)
    this._pollPitch()
    
    // 启动绘制循环
    this._startDrawLoop()
    
    // 初始化 Canvas
    this.$nextTick(() => this._initCanvas())
  },
  onUnload() {
    if (this._unsubState) this._unsubState()
    if (this._pollTick) clearInterval(this._pollTick)
    if (this._pitchTick) clearInterval(this._pitchTick)
    if (this._redrawTick) clearInterval(this._redrawTick)
    /* 画布上下文在页面隐藏后可能失效, 下次 onShow 重新建 */
    this.painter = null
    this._canvasPending = false
  },
  onHide() {
    if (this._pollTick) clearInterval(this._pollTick)
    if (this._pitchTick) clearInterval(this._pitchTick)
    if (this._redrawTick) clearInterval(this._redrawTick)
    this.painter = null
    this._canvasPending = false
  },
  methods: {
    /**
     * @brief 初始化绘图上下文
     * @note  H5 端: 直接用 DOM 里的 <canvas> 取 2d 上下文
     *        App/小程序端: 逻辑层没有 DOM, 只能用 uni.createCanvasContext 命令式 API
     */
    _initCanvas() {
      if (this.painter) return
      // #ifdef H5
      /* ⚠ uni-app 的 H5 端会把 <canvas canvas-id="xxx"> 渲染成
       *   <canvas class="uni-canvas-canvas">, **不会保留 canvas-id 属性**,
       *   所以 document.querySelector('canvas[canvas-id="..."]') 永远取不到,
       *   表现就是曲线框空白。这里按 class 找, 兜底再取第一个 canvas。 */
      const canvas = document.querySelector('canvas.uni-canvas-canvas') ||
                     document.querySelector('canvas')
      if (canvas) {
        const ctx = canvas.getContext('2d')
        /* canvas.width 是设备像素 (2x), ctx.scale(2,2) 后绘图用 CSS 像素,
         * 所以 _cssW/_cssH 存的是 CSS 尺寸 (绘制坐标不用除以 2) */
        const w = canvas.offsetWidth || 300
        const h = canvas.offsetHeight || 150
        canvas.width = w * 2
        canvas.height = h * 2
        ctx.scale(2, 2)
        this._cssW = w
        this._cssH = h
        this.painter = createPainter(ctx, true)
      }
      // #endif
      // #ifndef H5
      /* App / 小程序端: 用 uni 的画布命令 API, 尺寸靠节点查询拿到 (CSS 像素) */
      if (this._canvasPending) return
      this._canvasPending = true
      uni.createSelectorQuery()
        .select('.chart-canvas')
        .boundingClientRect((rect) => {
          this._canvasPending = false
          if (!rect || !rect.width || !rect.height) {
            /* 查不到节点尺寸时兜底: 用窗口宽度和 rpx 换算估算 */
            const info = uni.getSystemInfoSync()
            this._cssW = info.windowWidth - uni.upx2px(80)
            this._cssH = uni.upx2px(300)
          } else {
            this._cssW = rect.width
            this._cssH = rect.height
          }
          this.painter = createPainter(uni.createCanvasContext('pitchCanvas'), false)
        })
        .exec()
      // #endif
    },
    
    /**
     * @brief 启动重绘循环
     * @note  用 setInterval 而不是 requestAnimationFrame:
     *        App 端逻辑层没有 DOM, rAF / document 都是 undefined, 调用会抛
     *        "ReferenceError: requestAnimationFrame is not defined"。
     */
    _startDrawLoop() {
      if (this._redrawTick) clearInterval(this._redrawTick)
      this._redrawTick = setInterval(() => this._redraw(), REDRAW_MS)
    },
    
    /**
     * @brief 重绘一次曲线 (数据没变就不画, 减少跨层通信)
     */
    _redraw() {
      /* canvas 可能在页面切换后才渲染出来, 这里兜底再取一次
       * (App 端取尺寸是异步的, 没拿到之前不重复发起查询) */
      if (!this.painter) {
        if (!this._canvasPending) this._initCanvas()
        return
      }
      /* ⚠ 判断条件要同时看两个数组:
       *   pitchSeries  = 高速通道 (属性历史接口) 的点, 带时间戳, 是主力
       *   pitchHistory = 高速通道不通时的兜底 (最新值轮询), 元素少
       * 早期只判 pitchHistory.length > 1, 而高速通道一直正常 →
       * pitchHistory 永远只有 1 个元素 → 曲线永远不画 (空白框)。 */
      if (this.pitchSeries.length <= 1 && this.pitchHistory.length <= 1) return
      /* 数据指纹: 数据没变就不重复画 (App 端每次 draw 都要跨层通信, 很贵) */
      const series = this.pitchSeries
      let key
      if (series.length > 1) {
        key = 's' + series.length + ':' + series[series.length - 1].t
      } else if (this.pitchHistory.length > 1) {
        key = 'h' + this.pitchHistory.length + ':' + this.pitchHistory[this.pitchHistory.length - 1]
      } else {
        return
      }
      if (key === this._lastDrawKey) return
      this._lastDrawKey = key
      this._drawChart()
    },
    
    _drawChart() {
      const p = this.painter
      if (!p) return
      
      /* 绘图坐标统一用 CSS 像素 */
      const w = this._cssW || 300
      const h = this._cssH || 150
      const padding = 10
      const chartW = w - padding * 2
      const chartH = h - padding * 2
      
      // 清空画布
      p.fill('#fafafa')
      p.fillRect(0, 0, w, h)
      
      // 画网格线
      p.stroke('#e0e0e0')
      p.width(0.5)
      p.dash([4, 4])
      
      // 水平网格
      for (let i = 0; i <= 4; i++) {
        const y = padding + (chartH / 4) * i
        p.beginPath()
        p.moveTo(padding, y)
        p.lineTo(w - padding, y)
        p.strokePath()
      }
      
      // 零线
      p.stroke('#888')
      p.width(1)
      p.dash([])
      p.beginPath()
      const zeroY = padding + chartH / 2
      p.moveTo(padding, zeroY)
      p.lineTo(w - padding, zeroY)
      p.strokePath()
      
      // 画折线
      /* 高速通道的点带时间戳 (ms), 按**时间轴**摆位置:
       * 这样即使两次拉取之间漏了几个点, 曲线的疏密也能反映真实采样密度。 */
      const series = this.pitchSeries
      const points = series.length > 1 ? series.map(p2 => p2.v) : this.pitchHistory
      if (points.length < 2) return
      
      p.stroke('#1aad19')
      p.width(2)
      p.dash([])
      p.beginPath()
      
      const scaleY = chartH / (PITCH_RANGE * 2)
      /* 时间轴: 以最新点为右端, 往左铺 PITCH_WINDOW_MS 毫秒 */
      let tMin = 0, tMax = 1
      if (series.length > 1) {
        tMax = series[series.length - 1].t
        tMin = tMax - PITCH_WINDOW_MS
      }
      
      const xOf = (i) => {
        if (series.length > 1) {
          const t = series[i].t
          const ratio = (t - tMin) / (tMax - tMin)
          return padding + ratio * chartW
        }
        /* 没有时间戳的老数据 (兜底): 按序号均分 */
        return padding + i * (chartW / (points.length - 1))
      }
      
      points.forEach((v, i) => {
        const x = xOf(i)
        const y = zeroY - v * scaleY
        if (i === 0) {
          p.moveTo(x, y)
        } else {
          p.lineTo(x, y)
        }
      })
      p.strokePath()
      
      // 画数据点
      p.fill('#1aad19')
      points.forEach((v, i) => {
        p.dot(xOf(i), zeroY - v * scaleY, 3)
      })
      
      // 显示当前值
      const currentPitch = points[points.length - 1]
      p.fill('#333')
      p.text(currentPitch.toFixed(1) + '°', w - padding - 5, padding + 25, 24, 'right')
      
      // App/小程序端: 命令要 draw() 才真正上屏
      p.flush()
    },

    /**
     * @brief 拉一次俯仰角历史 (高速通道)
     * @note  平台的属性历史接口一次最多返回 10 个点 (倒序),
     *        设备端 20Hz 上报 → 每 120ms 拉一次就能攒出很密的曲线。
     * @note  返回的点带毫秒时间戳, 用来 (1) 排序去重 (2) 算数据延迟
     */
    async _pollPitch() {
      if (oneApi.getState().state !== 'ready') return
      const t0 = Date.now()
      /* 时间窗口基准:
       *   优先用**平台服务器时间** (HTTP 响应头里的 Date), 没有就用上一次
       *   拿到的最新采样点的时间戳 (那也是平台时间), 再没有才用本地时间。
       *   这样手机本地时钟偏几秒甚至几十秒, 都不会查成"过去的数据"。 */
      const anchor = Math.max(this._lastPointT || 0, getServerNowMs())
      const endTime   = anchor + PITCH_MARGIN_MS
      const startTime = anchor - PITCH_WINDOW_MS - PITCH_MARGIN_MS
      try {
        const res = await queryDevicePropertyHistory('pitch', startTime, endTime)
        this._rttMs = Date.now() - t0
        if (!res || res.code !== 0 || !res.data || !Array.isArray(res.data.list)) return
        const list = res.data.list
        if (!list.length) return

        /* 平台返回的是倒序 (最新在前), 这里转成升序再合并 */
        const pts = list
          .map(p => ({ t: Number(p.time), v: Number(p.value) }))
          .filter(p => !isNaN(p.t) && !isNaN(p.v))
          .sort((a, b) => a.t - b.t)

        /* 跟已有序列合并 (按时间戳去重) */
        const merged = this.pitchSeries.slice()
        const seen = new Set(merged.map(p => p.t))
        for (const p of pts) {
          if (!seen.has(p.t)) {
            seen.add(p.t)
            merged.push(p)
          }
        }
        merged.sort((a, b) => a.t - b.t)
        /* 只保留最近 PITCH_WINDOW_MS 毫秒的点, 最多 MAX_POINTS 个 */
        const newest = merged[merged.length - 1].t
        const keep = merged.filter(p => p.t >= newest - PITCH_WINDOW_MS)
        this.pitchSeries = keep.slice(Math.max(0, keep.length - MAX_POINTS))

        /* 最新点 → 姿态显示 + 延迟 */
        const last = this.pitchSeries[this.pitchSeries.length - 1]
        this._lastPointT = last.t
        this.pose.pitch = last.v
        /* 延迟 = 平台当前时间 − 最新采样点的时间戳 (用平台时间算, 不受手机时钟影响) */
        this.pitchLatency = Math.max(0, getServerNowMs() - last.t)
        this._bumpSeen()
        this.msgCount++
      } catch (e) {
        /* 静默失败: 历史接口不通时, 页面还有常规轮询兜底 */
      }
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
            /* 高速通道有数据时用它 (更新更快), 没有才退回这里的最新值 */
            if (this.pitchSeries.length === 0) {
              this.pose.pitch = newPitch
              this.pitchHistory.push(newPitch)
              if (this.pitchHistory.length > MAX_POINTS) {
                this.pitchHistory.shift()
              }
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

.chart-info {
  display: flex;
  justify-content: space-between;
  padding: 8rpx 10rpx 0;
}

.chart-info-item {
  font-size: 22rpx;
  color: #888;
  font-family: monospace;
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
