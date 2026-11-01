<template>
  <view class="container">
    <!-- 连接状态条 -->
    <view class="top-bar">
      <view class="conn-state" :class="'conn-' + state">
        <text class="dot"></text>
        <text>{{ connText }}</text>
      </view>
      <!-- 连接设置入口 (OneNet + 蓝牙), 任一辆车蓝牙连上时变蓝 -->
      <view class="avatar" :class="{ 'avatar-ble': anyBle }" @click="goConnect">
        <text class="avatar-text">{{ anyBle ? 'BT' : '⚙' }}</text>
      </view>
    </view>

    <!-- Pitch 折线图: 两辆车叠在同一张图里 -->
    <view class="card">
      <view class="card-title">Pitch 实时曲线</view>
      <view class="legend">
        <view class="legend-item"><text class="legend-dot lg-main"></text><text>主车 {{ fmt(cars.main.pose.pitch, 1) }}°</text></view>
        <view class="legend-item"><text class="legend-dot lg-sub"></text><text>副车 {{ fmt(cars.sub.pose.pitch, 1) }}°</text></view>
      </view>
      <canvas canvas-id="pitchCanvas" class="chart-canvas" @touchstart="onTouchStart" @touchend="onTouchEnd"></canvas>
      <view class="chart-labels">
        <text class="chart-label">{{ pitchMin }}°</text>
        <text class="chart-label chart-label-center">0°</text>
        <text class="chart-label">{{ pitchMax }}°</text>
      </view>
      <view class="chart-info">
        <text class="chart-info-item">主车延迟: {{ latencyText('main') }}</text>
        <text class="chart-info-item">副车延迟: {{ latencyText('sub') }}</text>
      </view>
      <view class="chart-info">
        <text class="chart-info-item">请求耗时: {{ rttMs }}ms</text>
        <text class="chart-info-item">时钟偏差: {{ skewText }}</text>
      </view>
    </view>

    <!-- 两车实时状态对照表 -->
    <view class="card">
      <view class="card-title">实时状态</view>
      <view class="t-row t-head">
        <text class="t-name"></text>
        <text class="t-val t-main">主车</text>
        <text class="t-val t-sub">副车</text>
      </view>
      <view class="t-row">
        <text class="t-name">链路</text>
        <text v-for="w in WHICH" :key="w" class="t-val" :class="linkClass(w)">{{ linkText(w) }}</text>
      </view>
      <view class="t-row">
        <text class="t-name">数据间隔</text>
        <text v-for="w in WHICH" :key="w" class="t-val">{{ beatText(w) }}</text>
      </view>
      <view class="t-row">
        <text class="t-name">Pitch</text>
        <text v-for="w in WHICH" :key="w" class="t-val t-big" :class="poseClass(w)">{{ fmt(cars[w].pose.pitch, 1) }}°</text>
      </view>
      <view class="t-row">
        <text class="t-name">Roll</text>
        <text v-for="w in WHICH" :key="w" class="t-val">{{ fmt(cars[w].pose.roll, 1) }}°</text>
      </view>
      <view class="t-row">
        <text class="t-name">Yaw</text>
        <text v-for="w in WHICH" :key="w" class="t-val">{{ fmt(cars[w].pose.yaw, 1) }}°</text>
      </view>
      <view class="t-row">
        <text class="t-name">Gz</text>
        <text v-for="w in WHICH" :key="w" class="t-val">{{ fmt(cars[w].pose.gz, 1) }}°/s</text>
      </view>
      <view class="t-row">
        <text class="t-name">左轮</text>
        <view v-for="w in WHICH" :key="w" class="t-val t-bar-cell">
          <text>{{ fmt(cars[w].motor.left, 0) }}%</text>
          <view class="motor-bar"><view class="motor-fill" :style="motorBarStyle(w, 'left')"></view></view>
        </view>
      </view>
      <view class="t-row">
        <text class="t-name">右轮</text>
        <view v-for="w in WHICH" :key="w" class="t-val t-bar-cell">
          <text>{{ fmt(cars[w].motor.right, 0) }}%</text>
          <view class="motor-bar"><view class="motor-fill" :style="motorBarStyle(w, 'right')"></view></view>
        </view>
      </view>
      <view class="t-row">
        <text class="t-name">遥控指令</text>
        <text v-for="w in WHICH" :key="w" class="t-val">{{ moveText(w) }}</text>
      </view>
      <view class="t-row">
        <text class="t-name">编码器</text>
        <text v-for="w in WHICH" :key="w" class="t-val t-small">{{ cars[w].motor.enc_l || 0 }} / {{ cars[w].motor.enc_r || 0 }}</text>
      </view>
    </view>

    <!-- 参数入口 (调参统一到"参数"页, 那边跟车上的参数表是一一对应的) -->
    <view class="card">
      <view class="card-title">参数调节</view>
      <view class="pid-item">
        <text class="pid-label">PID / 限幅 / 舵机中位…</text>
      </view>
      <text class="pid-value">共 17 项, 改完记得点"保存"才会下发到车</text>
      <view class="action-row">
        <button class="btn btn-primary" @click="goParams">去调参</button>
      </view>
    </view>

    <!-- 配平表入口 (换腿高后要录配平表, 补偿平衡俯仰角的偏移) -->
    <view class="card">
      <view class="card-title">配平标定</view>
      <text class="pid-value">换腿高后平衡俯仰角会差约 10°, 靠配平表自动补偿</text>
      <view class="action-row">
        <button class="btn btn-ghost" @click="goTrim">去标定</button>
      </view>
    </view>
  </view>
</template>

<script>
import { oneApi } from '../../common/oneapi.js'
import { queryDeviceProperty, queryDevicePropertyHistory, getServerNowMs, getServerSkewMs } from '../../common/onenet.js'
import { onPitch, onBleState, isConnected as bleConnected } from '../../common/ble.js'

const MAX_POINTS = 120  // 最多显示120个点
const PITCH_RANGE = 60  // 显示范围 ±60° (车倾倒时角度能到 60 度以上)
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

/** 车列表: 主车 / 副车 */
const WHICH = ['main', 'sub']
/** 曲线颜色 */
const COLORS = { main: '#1aad19', sub: '#1677ff' }

/** 一辆车的空状态 */
function emptyCar() {
  return {
    pose: { pitch: null, roll: null, yaw: null, gz: null },
    motor: { left: 0, right: 0, speed: 0, enc_l: 0, enc_r: 0 },
    series: [],        // pitch 曲线点 [{ t, v }]
    src: '',           // series 的时间基准: 'ble' = 手机时钟, 'net' = 平台时钟
    lastPointT: 0,     // 最新网络采样点的平台时间戳 (ms), 对齐查询窗口用
    latency: null,     // 网络 pitch 数据延迟 (ms)
    ble: false,        // 蓝牙是否连着
    lastAt: 0,         // 最后一次收到数据的本地时刻 (ms)
    maxGap: 0,         // 最近 3 秒内两次数据的最大间隔 (ms)
    maxGapAt: 0        // maxGap 统计时刻
  }
}

export default {
  data() {
    return {
      WHICH,
      state: 'idle',
      cars: { main: emptyCar(), sub: emptyCar() },
      rttMs: 0,         // 最近一次历史接口请求耗时 (ms)
      skewMs: 0,        // 平台时间 − 手机时间 (ms)
      tick: 0,          // 每秒自增, 让"数据间隔"读数定时刷新
      painter: null,    // 绘图适配器 (见 createPainter), null = 还没初始化好
      isDrawing: false
    }
  },
  computed: {
    connText() {
      if (this.state === 'ready') return 'OneNet 已连接'
      if (this.state === 'error') return 'OneNet 配置错误'
      return 'OneNet 未连接'
    },
    anyBle() {
      return this.cars.main.ble || this.cars.sub.ble
    },
    pitchMin() { return '-' + PITCH_RANGE },
    pitchMax() { return '+' + PITCH_RANGE },
    skewText() {
      if (!this.skewMs) return '--'
      return (this.skewMs > 0 ? '+' : '') + (this.skewMs / 1000).toFixed(1) + 's'
    }
  },
  onShow() {
    this._unsubState = oneApi.onState((s) => { this.state = s })
    WHICH.forEach(w => { this.cars[w].ble = bleConnected(w) })
    this._unsubBle = onBleState((w, v) => { this.cars[w].ble = v })
    this._unsubPitch = onPitch((w, v) => this._onBlePitch(w, v))

    /* 常规属性: 两车轮流, 每车约 400ms 一次 */
    this._pollIdx = 0
    this._busy = {}   // 每车 pitch 历史请求是否在途
    this._pollTick = setInterval(() => this._poll(WHICH[this._pollIdx++ % 2]), 200)
    /* pitch 高速通道: 两车各 PITCH_POLL_MS 一次 (蓝牙连着的车跳过) */
    this._pitchTick = setInterval(() => WHICH.forEach(w => this._pollPitch(w)), PITCH_POLL_MS)
    this._redrawTick = setInterval(() => this._redraw(), REDRAW_MS)
    this._beatTick = setInterval(() => { this.tick++ }, 1000)
    this.$nextTick(() => this._initCanvas())
  },
  onHide() { this._stop() },
  onUnload() { this._stop() },
  methods: {
    /** 停掉所有定时器和订阅 (画布上下文页面隐藏后可能失效, 下次 onShow 重建) */
    _stop() {
      if (this._unsubState) { this._unsubState(); this._unsubState = null }
      if (this._unsubBle) { this._unsubBle(); this._unsubBle = null }
      if (this._unsubPitch) { this._unsubPitch(); this._unsubPitch = null }
      ;['_pollTick', '_pitchTick', '_redrawTick', '_beatTick'].forEach(k => {
        if (this[k]) { clearInterval(this[k]); this[k] = null }
      })
      this.painter = null
      this._canvasPending = false
    },

    goConnect() { uni.navigateTo({ url: '/pages/connect/connect' }) },
    goParams()  { uni.navigateTo({ url: '/pages/params/params' }) },
    goTrim()    { uni.navigateTo({ url: '/pages/trim/trim' }) },
    onTouchStart() { this.isDrawing = true },
    onTouchEnd()   { this.isDrawing = false },

    fmt(v, dec) {
      if (v === null || v === undefined || Number.isNaN(v)) return '--'
      return Number(v).toFixed(dec)
    },

    /** 链路文字: 蓝牙 / OneNet / 离线 (3 秒没数据算离线) */
    linkText(w) {
      void this.tick
      const c = this.cars[w]
      if (c.ble) return '蓝牙'
      if (c.lastAt && Date.now() - c.lastAt < 3000) return 'OneNet'
      return '离线'
    },
    linkClass(w) {
      const t = this.linkText(w)
      return t === '蓝牙' ? 'lk-ble' : (t === 'OneNet' ? 'lk-net' : 'lk-off')
    },
    /** 数据间隔: 正常时显示最近 3 秒最大间隔, 断流超过 1 秒显示已断开的秒数 */
    beatText(w) {
      void this.tick
      const c = this.cars[w]
      if (!c.lastAt) return '无数据'
      const age = Date.now() - c.lastAt
      if (age > 1000) return (age / 1000).toFixed(1) + 's'
      return c.maxGap + 'ms'
    },
    latencyText(w) {
      const c = this.cars[w]
      if (c.ble) return '蓝牙直连'
      return c.latency === null ? '--' : c.latency + 'ms'
    },
    poseClass(w) {
      const p = Math.abs(this.cars[w].pose.pitch || 0)
      if (p > 20) return 'pose-warn'
      if (p > 10) return 'pose-mid'
      return ''
    },
    motorBarStyle(w, k) {
      const v = this.cars[w].motor[k] || 0
      const pct = Math.max(0, Math.min(100, Math.abs(v)))
      return `width: ${pct}%; background: ${v < 0 ? '#d9534f' : COLORS[w]};`
    },
    /**
     * @brief 遥控指令文字 (speed 编码: 0 停, ±0.2~0.8 前进/后退, ±1.2~1.8 左转/右转)
     */
    moveText(w) {
      const v = Number(this.cars[w].motor.speed) || 0
      const a = Math.abs(v)
      if (a < 0.05) return '停止'
      const level = Math.round((a > 1.0 ? a - 1.0 : a) * 100)
      const name = a <= 1.0 ? (v > 0 ? '前进' : '后退') : (v > 0 ? '左转' : '右转')
      return `${name} ${level}%`
    },

    /** 记录一次数据到达 (统计最近 3 秒最大间隔, 断流超过 1 秒不计入) */
    _bumpSeen(w) {
      const c = this.cars[w]
      const now = Date.now()
      if (c.lastAt) {
        const gap = Math.min(now - c.lastAt, 1000)
        if (!c.maxGap || now - c.maxGapAt > 3000 || gap > c.maxGap) {
          c.maxGap = gap
          c.maxGapAt = now
        }
      }
      c.lastAt = now
    },

    /**
     * @brief 往一辆车的曲线里并入新点
     * @param w   'main' | 'sub'
     * @param pts 新点 [{ t, v }] (升序)
     * @param src 时间基准 'ble' | 'net'; 与已有序列不同时清空重来
     */
    _mergeSeries(w, pts, src) {
      const c = this.cars[w]
      const merged = c.src === src ? c.series.slice() : []
      c.src = src
      const seen = new Set(merged.map(p => p.t))
      pts.forEach(p => { if (!seen.has(p.t)) { seen.add(p.t); merged.push(p) } })
      merged.sort((a, b) => a.t - b.t)
      const newest = merged[merged.length - 1].t
      const keep = merged.filter(p => p.t >= newest - PITCH_WINDOW_MS)
      c.series = keep.slice(Math.max(0, keep.length - MAX_POINTS))
      c.pose.pitch = c.series[c.series.length - 1].v
    },

    /** 蓝牙 pitch 通知 → 曲线 (时间戳用手机本地时钟) */
    _onBlePitch(w, v) {
      this._mergeSeries(w, [{ t: Date.now(), v }], 'ble')
      this._bumpSeen(w)
    },
    /**
     * @brief 初始化绘图上下文
     * @note  H5: uni-app 把 <canvas canvas-id> 渲染成 canvas.uni-canvas-canvas (不保留 canvas-id), 按 class 取 2d 上下文;
     *        App/小程序: 逻辑层没有 DOM, 用 uni.createCanvasContext, 尺寸靠节点查询 (CSS 像素)
     */
    _initCanvas() {
      if (this.painter) return
      // #ifdef H5
      const canvas = document.querySelector('canvas.uni-canvas-canvas') ||
                     document.querySelector('canvas')
      if (canvas) {
        const ctx = canvas.getContext('2d')
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
      if (this._canvasPending) return
      this._canvasPending = true
      uni.createSelectorQuery()
        .select('.chart-canvas')
        .boundingClientRect((rect) => {
          this._canvasPending = false
          if (!rect || !rect.width || !rect.height) {
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

    /** 重绘一次曲线 (两车数据都没变就不画, App 端每次 draw 都要跨层通信) */
    _redraw() {
      if (!this.painter) {
        if (!this._canvasPending) this._initCanvas()
        return
      }
      const key = WHICH.map(w => {
        const s = this.cars[w].series
        return s.length + ':' + (s.length ? s[s.length - 1].t : 0)
      }).join('|')
      if (key === this._lastDrawKey) return
      this._lastDrawKey = key
      this._drawChart()
    },

    /**
     * @brief 画两车 pitch 曲线 (主车绿, 副车蓝)
     * @note  每条曲线以自己的最新点为右端、往左铺 PITCH_WINDOW_MS:
     *        蓝牙点用手机时钟, 网络点用平台时钟, 两者不对齐
     */
    _drawChart() {
      const p = this.painter
      if (!p) return
      const w = this._cssW || 300
      const h = this._cssH || 150
      const padding = 10
      const chartW = w - padding * 2
      const chartH = h - padding * 2
      const zeroY = padding + chartH / 2
      const scaleY = chartH / (PITCH_RANGE * 2)

      p.fill('#fafafa')
      p.fillRect(0, 0, w, h)

      /* 水平网格 */
      p.stroke('#e0e0e0')
      p.width(0.5)
      p.dash([4, 4])
      for (let i = 0; i <= 4; i++) {
        const y = padding + (chartH / 4) * i
        p.beginPath()
        p.moveTo(padding, y)
        p.lineTo(w - padding, y)
        p.strokePath()
      }

      /* 零线 */
      p.stroke('#888')
      p.width(1)
      p.dash([])
      p.beginPath()
      p.moveTo(padding, zeroY)
      p.lineTo(w - padding, zeroY)
      p.strokePath()

      WHICH.forEach(car => {
        const s = this.cars[car].series
        if (s.length < 2) return
        const tMax = s[s.length - 1].t
        const tMin = tMax - PITCH_WINDOW_MS
        const xOf = t => padding + (t - tMin) / PITCH_WINDOW_MS * chartW
        p.stroke(COLORS[car])
        p.width(2)
        p.beginPath()
        s.forEach((pt, i) => {
          const y = zeroY - pt.v * scaleY
          if (i === 0) p.moveTo(xOf(pt.t), y)
          else p.lineTo(xOf(pt.t), y)
        })
        p.strokePath()
        p.fill(COLORS[car])
        p.dot(xOf(tMax), zeroY - s[s.length - 1].v * scaleY, 3)
      })

      p.flush()
    },

    /**
     * @brief 拉一辆车的 pitch 属性历史 (高速通道, 一次最多 10 个点, 倒序)
     * @param w 'main' | 'sub'; 该车蓝牙连着时跳过 (曲线走蓝牙通知)
     * @note  查询窗口以平台时间为基准, 两端各留 PITCH_MARGIN_MS, 手机时钟偏差不影响
     */
    async _pollPitch(w) {
      const c = this.cars[w]
      if (c.ble || oneApi.getState().state !== 'ready') return
      if (this._busy[w]) return
      this._busy[w] = true
      const t0 = Date.now()
      const anchor = Math.max(c.lastPointT || 0, getServerNowMs())
      try {
        const res = await queryDevicePropertyHistory('pitch',
          anchor - PITCH_WINDOW_MS - PITCH_MARGIN_MS, anchor + PITCH_MARGIN_MS, w)
        this.rttMs = Date.now() - t0
        this.skewMs = getServerSkewMs()
        const list = res && res.code === 0 && res.data && Array.isArray(res.data.list) ? res.data.list : []
        const pts = list
          .map(x => ({ t: Number(x.time), v: Number(x.value) }))
          .filter(x => !isNaN(x.t) && !isNaN(x.v))
          .sort((a, b) => a.t - b.t)
        /* 蓝牙可能在请求期间连上, 此时丢弃网络点 */
        if (pts.length && !c.ble) {
          this._mergeSeries(w, pts, 'net')
          c.lastPointT = c.series[c.series.length - 1].t
          c.latency = Math.max(0, getServerNowMs() - c.lastPointT)
          this._bumpSeen(w)
        }
      } catch (e) {
        /* 静默失败: 常规轮询仍会更新其它读数 */
      }
      this._busy[w] = false
    },

    /**
     * @brief 拉一辆车的最新属性 (姿态 / 电机 / 遥控指令 / 编码器)
     * @param w 'main' | 'sub'
     * @note  pitch 只在该车曲线为空时才用这里的值 (曲线有数据时以曲线最新点为准)
     */
    async _poll(w) {
      if (oneApi.getState().state !== 'ready') return
      const c = this.cars[w]
      try {
        const res = await queryDeviceProperty(w)
        this.skewMs = getServerSkewMs()
        if (!res || res.code !== 0 || !Array.isArray(res.data)) return
        const o = {}
        res.data.forEach(x => { o[x.identifier] = x.value })
        if (!c.ble) this._bumpSeen(w)
        if (o.pitch !== undefined && !c.series.length) c.pose.pitch = Number(o.pitch)
        if (o.roll    !== undefined) c.pose.roll   = Number(o.roll)
        if (o.yaw     !== undefined) c.pose.yaw    = Number(o.yaw)
        if (o.gz      !== undefined) c.pose.gz     = Number(o.gz)
        if (o.motor_l !== undefined) c.motor.left  = Number(o.motor_l)
        if (o.motor_r !== undefined) c.motor.right = Number(o.motor_r)
        if (o.speed   !== undefined) c.motor.speed = Number(o.speed)
        if (o.enc_l   !== undefined) c.motor.enc_l = Number(o.enc_l)
        if (o.enc_r   !== undefined) c.motor.enc_r = Number(o.enc_r)
      } catch (e) {
        /* 静默失败 */
      }
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

.avatar {
  width: 60rpx;
  height: 60rpx;
  margin-left: 20rpx;
  border-radius: 50%;
  background: #1aad19;
  display: flex;
  align-items: center;
  justify-content: center;
}

.avatar-ble { background: #1677ff; }

.avatar-text {
  color: #fff;
  font-size: 26rpx;
  font-weight: 600;
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
.pose-mid { color: #f0ad4e; }
.pose-warn { color: #d9534f; }

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
.legend {
  display: flex;
  gap: 32rpx;
  margin-bottom: 12rpx;
  font-size: 24rpx;
  color: #555;
}

.legend-item {
  display: flex;
  align-items: center;
}

.legend-dot {
  width: 20rpx;
  height: 6rpx;
  border-radius: 3rpx;
  margin-right: 10rpx;
}

.lg-main { background: #1aad19; }
.lg-sub { background: #1677ff; }

.t-row {
  display: flex;
  align-items: center;
  padding: 12rpx 0;
  border-bottom: 1rpx solid #f3f3f3;
}

.t-row:last-child { border-bottom: none; }

.t-head { padding-top: 0; }

.t-name {
  width: 150rpx;
  font-size: 26rpx;
  color: #888;
}

.t-val {
  flex: 1;
  font-size: 26rpx;
  color: #333;
  font-family: monospace;
  text-align: center;
}

.t-main { color: #1aad19; font-weight: 600; font-family: inherit; }
.t-sub { color: #1677ff; font-weight: 600; font-family: inherit; }
.t-big { font-size: 32rpx; font-weight: 600; }
.t-small { font-size: 22rpx; }

.t-bar-cell {
  display: flex;
  flex-direction: column;
  align-items: center;
  padding: 0 12rpx;
}

.lk-ble { color: #1677ff; }
.lk-net { color: #1aad19; }
.lk-off { color: #aaa; }
</style>
