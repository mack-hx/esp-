<template>
  <view class="container">
    <!-- 顶部状态 -->
    <view class="top-bar">
      <view class="conn-state" :class="'conn-' + state">
        <text class="dot"></text>
        <text>{{ connText }}</text>
      </view>
      <view class="device-id" @click="goConnect">
        <text>{{ devicePath }} ›</text>
      </view>
    </view>

    <!-- 姿态动作卡片 -->
    <view class="pose-card">
      <view class="card-title">基础姿态</view>
      <view class="pose-grid">
        <view class="pose-btn" @click="onSquat">
          <text class="pose-icon">⤓</text>
          <text class="pose-label">蹲下</text>
        </view>
        <view class="pose-btn" @click="onStand">
          <text class="pose-icon">⤒</text>
          <text class="pose-label">起立</text>
        </view>
      </view>
    </view>

    <!-- 跳舞动作 -->
    <view class="pose-card">
      <view class="card-title">动作</view>
      <view class="dance-controls">
        <view class="pose-btn pose-btn-wide" :class="{ active: dancing }" @click="onDanceToggle">
          <text class="pose-icon">♪</text>
          <text class="pose-label">{{ dancing ? '停止跳舞' : '开始跳舞' }}</text>
        </view>
        <view class="freq-row">
          <text class="freq-label">频率</text>
          <slider
            :value="freq"
            :min="0.5"
            :max="3"
            :step="0.1"
            :disabled="dancing"
            @change="onFreqChange"
            activeColor="#1aad19"
            backgroundColor="#e9e9e9"
            block-size="20"
          />
          <text class="freq-value">{{ freq.toFixed(1) }} Hz</text>
        </view>
      </view>
    </view>
  </view>
</template>

<script>
import { ctrl } from '../../utils/control.js'
import { oneApi } from '../../common/oneapi.js'

export default {
  data() {
    return {
      state: 'disconnected',
      connText: '未连接',
      devicePath: '---',
      dancing: false,
      freq: 1.0,       // 跳舞频率 (Hz)
      danceTimer: null
    }
  },
  onLoad() {
    this.updateState()
    oneApi.on('state', this.updateState)
  },
  onUnload() {
    oneApi.off('state', this.updateState)
    this.stopDance()
  },
  methods: {
    updateState() {
      const s = oneApi.getState()
      this.state = s.state
      this.connText = s.connText
      this.devicePath = s.devicePath || '---'
    },
    goConnect() {
      uni.navigateTo({ url: '/pages/connect/connect' })
    },
    
    /** 蹲下: 舵机到 100° (标准蹲姿) */
    onSquat() {
      ctrl.setServo(100, 100).catch(e => {
        uni.showToast({ title: '蹲下失败', icon: 'none' })
      })
    },
    
    /** 起立: 舵机到 40° (标准站姿) */
    onStand() {
      ctrl.setServo(40, 40).catch(e => {
        uni.showToast({ title: '起立失败', icon: 'none' })
      })
    },
    
    /** 切换跳舞状态 */
    onDanceToggle() {
      if (this.dancing) {
        this.stopDance()
      } else {
        this.startDance()
      }
    },
    
    /** 开始跳舞: 左右腿交替高低 */
    startDance() {
      this.dancing = true
      let step = 0  // 0=左高右低, 1=右高左低
      
      const cycle = () => {
        if (!this.dancing) return
        
        if (step === 0) {
          // 左高(30°) 右低(110°)
          ctrl.setServo(30, 110)
        } else {
          // 左低(110°) 右高(30°)
          ctrl.setServo(110, 30)
        }
        
        step = 1 - step
      }
      
      cycle()  // 立即执行第一步
      this.danceTimer = setInterval(cycle, 1000 / this.freq / 2)  // 每半个周期切换一次
    },
    
    /** 停止跳舞: 回到中位 */
    stopDance() {
      this.dancing = false
      if (this.danceTimer) {
        clearInterval(this.danceTimer)
        this.danceTimer = null
      }
      // 回到标准站姿
      ctrl.setServo(40, 40)
    },
    
    /** 频率滑块变化 */
    onFreqChange(e) {
      this.freq = e.detail.value
      if (this.dancing) {
        // 重启跳舞以应用新频率
        this.stopDance()
        this.startDance()
      }
    }
  }
}
</script>

<style scoped>
.container {
  display: flex;
  flex-direction: column;
  min-height: 100vh;
  background: linear-gradient(180deg, #f5f7fa 0%, #e8ecf1 100%);
  padding: 0 24rpx 40rpx;
}

.top-bar {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 20rpx 0;
  margin-bottom: 24rpx;
}

.conn-state {
  display: flex;
  align-items: center;
  gap: 12rpx;
  font-size: 28rpx;
  color: #666;
}

.dot {
  width: 16rpx;
  height: 16rpx;
  border-radius: 999px;
  background: #999;
}

.conn-ready .dot { background: #1aad19; }
.conn-connecting .dot { background: #ff9800; }
.conn-disconnected .dot { background: #999; }

.device-id {
  font-size: 26rpx;
  color: #1aad19;
  padding: 8rpx 16rpx;
  background: rgba(26, 173, 25, 0.08);
  border-radius: 999px;
}

.pose-card {
  background: #fff;
  border-radius: 24rpx;
  padding: 32rpx;
  margin-bottom: 24rpx;
  box-shadow: 0 4rpx 16rpx rgba(0, 0, 0, 0.04);
}

.card-title {
  font-size: 32rpx;
  font-weight: 600;
  color: #333;
  margin-bottom: 24rpx;
}

.pose-grid {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 20rpx;
}

.pose-btn {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  height: 180rpx;
  background: linear-gradient(135deg, #f5f7fa 0%, #e8ecf1 100%);
  border-radius: 20rpx;
  border: 2rpx solid #e0e0e0;
  transition: all 0.2s;
}

.pose-btn:active {
  transform: scale(0.96);
  background: linear-gradient(135deg, #1aad19 0%, #18a017 100%);
  border-color: #1aad19;
}

.pose-btn:active .pose-icon,
.pose-btn:active .pose-label {
  color: #fff;
}

.pose-icon {
  font-size: 64rpx;
  color: #1aad19;
  margin-bottom: 12rpx;
}

.pose-label {
  font-size: 28rpx;
  color: #666;
  font-weight: 500;
}

.dance-controls {
  display: flex;
  flex-direction: column;
  gap: 24rpx;
}

.pose-btn-wide {
  width: 100%;
  height: 120rpx;
  flex-direction: row;
  gap: 16rpx;
}

.pose-btn-wide .pose-icon {
  margin-bottom: 0;
  font-size: 48rpx;
}

.pose-btn.active {
  background: linear-gradient(135deg, #ff6b6b 0%, #ee5a52 100%);
  border-color: #ff6b6b;
}

.pose-btn.active .pose-icon,
.pose-btn.active .pose-label {
  color: #fff;
}

.freq-row {
  display: flex;
  align-items: center;
  gap: 20rpx;
}

.freq-label {
  font-size: 28rpx;
  color: #666;
  min-width: 80rpx;
}

.freq-value {
  font-size: 28rpx;
  color: #1aad19;
  font-weight: 600;
  min-width: 100rpx;
  text-align: right;
}

slider {
  flex: 1;
}
</style>
