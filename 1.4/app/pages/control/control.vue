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

    <!-- 主控制盘: 上下左右 + 中央提示 -->
    <view class="pad-card">
      <view class="card-title">方向控制</view>
      <view class="pad">
        <!-- 上 -->
        <view
          class="pad-btn"
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

        <!-- 左 -->
        <view
          class="pad-btn"
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

        <!-- 中央提示 -->
        <view class="pad-center">
          <text class="pad-center-text">双轮差速</text>
          <text class="pad-center-sub">档位: {{ speedLabel }}</text>
        </view>

        <!-- 右 -->
        <view
          class="pad-btn"
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

        <!-- 下 -->
        <view
          class="pad-btn"
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
    </view>

    <!-- 底部动作: 急停 / 起立 / 配平 -->
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
      tip: ''
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

/* 十字键布局: 用 3x3 grid, 中心留空 */
.pad {
  display: grid;
  grid-template-columns: 1fr 1fr 1fr;
  grid-template-rows: 130rpx 130rpx 130rpx;
  gap: 16rpx;
  margin-top: 20rpx;
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

/* 位置 */
.pad-btn:nth-of-type(1) { grid-column: 2; grid-row: 1; } /* 上 */
.pad-btn:nth-of-type(2) { grid-column: 1; grid-row: 2; } /* 左 */
.pad-btn:nth-of-type(3) { grid-column: 3; grid-row: 2; } /* 右 */
.pad-btn:nth-of-type(4) { grid-column: 2; grid-row: 3; } /* 下 */

.pad-center {
  grid-column: 2;
  grid-row: 2;
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
