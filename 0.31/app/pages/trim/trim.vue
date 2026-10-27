<template>
  <view class="container">
    <!-- 连接状态条 -->
    <view class="top-bar">
      <view class="conn-state" :class="'conn-' + state">
        <text class="dot"></text>
        <text>{{ connText }}</text>
      </view>
      <text class="top-text">配平表标定</text>
    </view>

    <!-- 说明: 为什么要标定 -->
    <view class="card">
      <view class="card-title">为什么需要标定</view>
      <text class="note">
        这台车的腿高由舵机角度决定。腿一换高度, 车"平衡姿态"时的车身俯仰角就会跟着变,
        实测从最低到最高能差约 10°。车端有一张"配平表"(舵机角 → 平衡俯仰角),
        控制器每周期都会自动扣掉这个偏差, 换腿高就不用重新配平。
        ⚠ 这张表必须手工录点, 下面就是录入界面。
      </text>
    </view>

    <!-- 拨腿 -->
    <view class="card">
      <view class="card-title">拨腿</view>
      <view class="row">
        <text class="label">目标舵机角</text>
        <text class="value">{{ servoAngle }}°</text>
      </view>
      <slider
        class="slider"
        :min="servoMin"
        :max="servoMax"
        :step="1"
        :value="servoAngle"
        :block-size="22"
        activeColor="#1aad19"
        backgroundColor="#e0e0e0"
        @change="onSliderChange"
        @changing="onSliderChange"
        show-value
      />
      <view class="row">
        <text class="label">车端实际舵机角</text>
        <text class="value">{{ curServo }}°</text>
      </view>
      <view class="action-row">
        <button class="btn btn-primary" @click="onMoveLeg">拨腿</button>
      </view>
      <text class="hint">拨完等 1 秒左右让舵机转到位, 再用手把车身扶到接近平衡。</text>
    </view>

    <!-- 实时读数 -->
    <view class="card">
      <view class="card-title">实时读数</view>
      <view class="row">
        <text class="label">原始 pitch (记录用)</text>
        <text class="value" :class="pitchClass">{{ fmt(rp, 2) }}°</text>
      </view>
      <view class="row">
        <text class="label">修正后 pitch (控制器看的)</text>
        <text class="value">{{ fmt(pitch, 2) }}°</text>
      </view>
      <view class="row">
        <text class="label">配平零点</text>
        <text class="value">{{ fmt(pz, 2) }}°</text>
      </view>
      <view class="row">
        <text class="label">参考角 servo_c</text>
        <text class="value">{{ fmt(ref, 1) }}°</text>
      </view>
      <text class="hint">
        标定时看的是"原始 pitch": 表里存的就是它。判断姿态靠手感 ——
        车会往哪边倒就往另一边扶, 扶到"快要平衡"再点「记下这个点」。
        修正后 pitch 是控制器实际用的那个 (扣了配平表补偿和配平零点),
        表还没录满时它跟"平衡姿态"对不上号, 不能拿来判断姿态。
        配平零点 = servo_c 那一档的平衡角, 由遥控页的「配平」按钮采集。
      </text>
    </view>

    <!-- 记录 / 保存 / 清空 -->
    <view class="card">
      <view class="card-title">标定操作</view>
      <text class="hint">
        步骤: ① 选一个舵机角 → ② 点「拨腿」 → ③ 用手把车扶到接近平衡 →
        ④ 点「记下这个点」 → ⑤ 换角度重复 6~8 次 → ⑥ 点「保存到 NVS」落盘。
      </text>
      <view class="action-row">
        <button class="btn btn-primary" @click="onRecord">记下这个点</button>
        <button class="btn btn-ghost" @click="onSave">保存到 NVS</button>
        <button class="btn btn-warn" @click="onClear">清空表</button>
      </view>
    </view>

    <!-- 已录的点 -->
    <view class="card">
      <view class="card-title">已录点 ({{ n }} 个)</view>
      <view class="row">
        <text class="label">表内跨度 (max-min)</text>
        <text class="value">{{ fmt(span, 2) }}°</text>
      </view>
      <text class="hint">
        跨度是表里俯仰角的最大值减最小值。换腿高时俯仰角应该差好几度;
        跨度接近 0 说明每次都扶成了同一个姿态, 这张表没有用。
      </text>
      <view class="table">
        <view class="tr th">
          <text class="td c1">舵机角</text>
          <text class="td c2">原始 pitch</text>
        </view>
        <view class="tr" v-for="(p, i) in pts" :key="i">
          <text class="td c1">{{ p[0] }}°</text>
          <text class="td c2">{{ fmt(p[1], 2) }}°</text>
        </view>
        <view class="tr" v-if="!pts.length">
          <text class="td c1 empty">还没有录点</text>
        </view>
      </view>
    </view>

    <view class="bottom-spacer"></view>
  </view>
</template>

<script>
/**
 * @file trim.vue
 * @brief 配平表标定页 (0.31 新增)
 *
 * 配平表 = "舵机角 → 平衡俯仰角" 的一维表, 车端用来自动补偿换腿高带来的
 * 平衡俯仰角偏差 (实测从最低到最高差约 10°)。
 *
 * 车端提供的 4 个服务:
 *   trim_record  记一个点 (当前舵机角 → 当前原始 pitch)
 *   trim_save    表落盘 (NVS, 断电不丢)
 *   trim_clear   清空表并落盘 (换机械结构后用)
 *   trim_get     读回整张表 (JSON 字符串: {n, ref, pz, pts})
 *
 * @note  为什么页面上的 pitch 与表里的 pitch 不是同一个数:
 *        页面上显示的是遥测上报的**修正后** pitch (已经扣了配平表补偿和配平零点),
 *        表里存的是车端取的**原始** pitch (没扣任何补偿) —— 修正链里还会再扣一次
 *        trim_pitch(), 存修正后的会重复扣, 补偿就废了。所以两者本来就不该相等。
 */
import { controller } from '../../utils/control.js'
import { oneApi } from '../../common/oneapi.js'
import { queryDeviceProperty } from '../../common/onenet.js'

/** 舵机角滑块范围 (与车端 servo 的限位一致) */
const SERVO_MIN = 30
const SERVO_MAX = 120

/** 遥测属性轮询周期 (ms) */
const POLL_MS = 500

/** 配平表刷新周期 (ms): 表里带当前原始 pitch, 标定时要看着它扶车,
 *  所以刷得快一点 (服务调用是 MQTT 往返, 1 秒一次够用了) */
const TRIM_MS = 1000

export default {
  data() {
    return {
      state: 'idle',
      servoMin: SERVO_MIN,   // 滑块下限 (°)
      servoMax: SERVO_MAX,   // 滑块上限 (°)
      servoAngle: 80,        // 滑块上的目标舵机角 (°)
      curServo: 0,           // 车端上报的实际舵机角 (°)
      pitch: 0,              // 遥测上报的修正后 pitch (°)
      rp: 0,                 // 车端上报的当前原始 pitch (°, 记录用)
      pz: 0,                 // 配平零点 (°), 车端 balance_get_pitch_zero()
      ref: 80,               // 配平表参考角 (= param 的 servo_c)
      pts: [],               // 已录点: [[舵机角, 原始pitch], ...]
      n: 0,                  // 点数
      span: 0,               // 表内俯仰角跨度 (max-min)
      _pollTick: null,
      _trimTick: null
    }
  },
  computed: {
    connText() {
      switch (this.state) {
        case 'ready':  return '已连接'
        case 'error':  return '连接错误'
        default:       return '未连接'
      }
    },
    pitchClass() {
      /* 高亮的是原始 pitch (标定时要盯着它扶车) */
      const p = Math.abs(this.rp)
      if (p > 20) return 'warn'
      if (p > 10) return 'mid'
      return ''
    }
  },
  onShow() {
    /* 重复进入时先把上一次的订阅退掉 (避免订阅叠加) */
    if (this._unsubState) { this._unsubState(); this._unsubState = null }
    this._unsubState = oneApi.onState((s) => { this.state = s })
    /* 遥测轮询: 读 pitch / servo1_angle (照状态页那套读属性的做法) */
    this._pollTick = setInterval(() => this._poll(), POLL_MS)
    this._poll()
    /* 配平表刷新 (进入页面先拉一次, 之后定时刷新) */
    this._trimTick = setInterval(() => this._refreshTrim(), TRIM_MS)
    this._refreshTrim()
  },
  onHide() {
    this._stopTimers()
  },
  onUnload() {
    this._stopTimers()
    if (this._unsubState) this._unsubState()
  },
  methods: {
    /**
     * @brief 停掉定时器 (页面隐藏/销毁时调用, 防止定时器泄漏)
     */
    _stopTimers() {
      if (this._pollTick) { clearInterval(this._pollTick); this._pollTick = null }
      if (this._trimTick) { clearInterval(this._trimTick); this._trimTick = null }
    },

    /**
     * @brief 拉一次遥测属性 (pitch / servo1_angle)
     * @note  照状态页 status.vue 的做法: 调平台的属性查询接口, 拿回一个
     *        {identifier: value} 的表, 取自己关心的字段。
     */
    async _poll() {
      if (oneApi.getState().state !== 'ready') return
      try {
        const res = await queryDeviceProperty()
        if (res && res.code === 0 && Array.isArray(res.data)) {
          const obj = {}
          res.data.forEach(p => { obj[p.identifier] = p.value })
          if (obj.pitch !== undefined) this.pitch = Number(obj.pitch)
          if (obj.servo1_angle !== undefined) this.curServo = Number(obj.servo1_angle)
        }
      } catch (e) {
        /* 静默失败: 定时轮询, 失败了下一次再说 */
      }
    },

    /**
     * @brief 拉一次配平表 (trim_get), 刷新列表 / 配平零点 / 跨度
     */
    async _refreshTrim() {
      if (oneApi.getState().state !== 'ready') return
      try {
        const t = await controller.trimGet()
        if (!t) return
        this.n = Number(t.n) || 0
        this.ref = Number(t.ref) || 0
        this.pz = Number(t.pz) || 0
        /* rp = 车端当前原始 pitch (标定时看这个) */
        if (t.rp !== undefined && t.rp !== null) this.rp = Number(t.rp)
        this.pts = Array.isArray(t.pts) ? t.pts : []
        this.span = this._calcSpan(this.pts)
      } catch (e) {
        /* 静默失败 */
      }
    },

    /**
     * @brief 表内俯仰角跨度 (max - min)
     * @param pts 点数组 [[舵机角, pitch], ...]
     * @return 跨度 (°); 空表返回 0
     */
    _calcSpan(pts) {
      if (!pts || !pts.length) return 0
      let lo = Infinity, hi = -Infinity
      pts.forEach(p => {
        const v = Number(p[1])
        if (isNaN(v)) return
        if (v < lo) lo = v
        if (v > hi) hi = v
      })
      return (lo === Infinity) ? 0 : (hi - lo)
    },

    /**
     * @brief 滑块变化 → 记住目标舵机角
     */
    onSliderChange(e) {
      this.servoAngle = e.detail.value
    },

    /**
     * @brief 「拨腿」: 两路舵机转到选定的角度
     * @note  走物模型属性 servo1_angle / servo2_angle (跟 UI 舵机页同一条通道)。
     *        两路舵机是镜像安装的, 逻辑角始终相同, 所以两个值给一样的。
     */
    async onMoveLeg() {
      const v = Number(this.servoAngle)
      try {
        await controller.sendServo({ servo1_angle: v, servo2_angle: v })
        uni.showToast({ title: '拨腿 ' + v + '°', icon: 'none' })
      } catch (e) {
        /* reportError 已经提示过了 */
      }
    },

    /**
     * @brief 「记下这个点」: 让车端把 (当前舵机角 → 当前原始 pitch) 存进配平表
     */
    async onRecord() {
      try {
        const msg = await controller.trimRecord()
        if (!msg) {
          uni.showToast({ title: '车端没返回结果', icon: 'none' })
          return
        }
        await this._refreshTrim()
        uni.showToast({ title: msg.slice(0, 30), icon: 'none' })
      } catch (e) {
        /* reportError 已经提示过了 */
      }
    },

    /**
     * @brief 「保存到 NVS」: 落盘, 断电不丢
     */
    async onSave() {
      try {
        const msg = await controller.trimSave()
        uni.showToast({ title: (msg || 'saved').slice(0, 30), icon: 'none' })
      } catch (e) {
        /* reportError 已经提示过了 */
      }
    },

    /**
     * @brief 「清空表」: 二次确认后清空并落盘 (换机械结构后用)
     */
    onClear() {
      uni.showModal({
        title: '清空配平表',
        content: '清掉表里所有标定点并落盘? 换机械结构后需要重新录点。',
        success: async (res) => {
          if (!res.confirm) return
          try {
            const msg = await controller.trimClear()
            await this._refreshTrim()
            uni.showToast({ title: (msg || 'cleared').slice(0, 30), icon: 'none' })
          } catch (e) {
            /* reportError 已经提示过了 */
          }
        }
      })
    },

    /**
     * @brief 数字格式化
     */
    fmt(v, dec) {
      if (v === null || v === undefined || Number.isNaN(Number(v))) return '--'
      return Number(v).toFixed(dec)
    }
  }
}
</script>

<style scoped>
.container {
  padding-bottom: 40rpx;
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

.note {
  font-size: 26rpx;
  color: #666;
  line-height: 1.7;
}

.top-bar {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 20rpx 32rpx;
  background: #fff;
  border-bottom: 1rpx solid #eee;
}

.top-text {
  font-size: 26rpx;
  color: #888;
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

.hint {
  display: block;
  font-size: 22rpx;
  color: #999;
  line-height: 1.6;
  margin-top: 12rpx;
}

.row {
  display: flex;
  justify-content: space-between;
  align-items: baseline;
  padding: 12rpx 0;
}

.label { font-size: 26rpx; color: #888; }
.value { font-size: 30rpx; font-weight: 600; color: #333; font-family: monospace; }
.mid { color: #f0ad4e; }
.warn { color: #d9534f; }

.slider {
  margin: 10rpx 0 20rpx;
}

.action-row {
  display: flex;
  flex-wrap: wrap;
  gap: 16rpx;
  padding: 16rpx 0 8rpx;
}

.btn {
  flex: 1;
  min-width: 180rpx;
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

.btn-warn {
  background: #f0ad4e;
  color: #fff;
}

.table {
  margin-top: 16rpx;
  border-top: 1rpx solid #eee;
}

.tr {
  display: flex;
  padding: 12rpx 0;
  border-bottom: 1rpx solid #f5f5f5;
}

.tr.th {
  background: #fafafa;
}

.td {
  font-size: 26rpx;
  color: #333;
  font-family: monospace;
}

.td.c1 { flex: 1; }
.td.c2 { flex: 1; text-align: right; }
.td.empty { color: #aaa; font-family: sans-serif; }

.bottom-spacer {
  height: 80rpx;
}
</style>
