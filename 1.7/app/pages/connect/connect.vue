<template>
  <view class="container">
    <!-- 顶部连接状态卡 -->
    <view class="status-bar" :class="stateClass">
      <view class="status-dot"></view>
      <text class="status-text">{{ statusText }}</text>
    </view>

    <!-- OneNet 产品 / 设备 -->
    <view class="card">
      <view class="card-title">OneNet 产品 / 设备</view>

      <view class="form-item">
        <text class="form-label">产品 ID</text>
        <input
          v-model="form.productId"
          class="form-input"
          placeholder="OneNet 后台产品详情可见"
          placeholder-class="form-placeholder"
        />
      </view>

      <view class="form-item">
        <text class="form-label">设备名</text>
        <input
          v-model="form.deviceName"
          class="form-input"
          placeholder="OneNet 后台设备详情可见"
          placeholder-class="form-placeholder"
        />
      </view>

      <view class="form-item">
        <text class="form-label">设备级 AccessKey</text>
        <input
          v-model="form.deviceAccessKey"
          class="form-input"
          password
          placeholder="Base64 字符串, 设备详情页查看"
          placeholder-class="form-placeholder"
        />
        <text class="form-hint">用此 key 计算 token, 需勾选设备级权限</text>
      </view>

      <view class="form-item">
        <text class="form-label">项目 ID (project_id)</text>
        <input
          v-model="form.projectId"
          class="form-input"
          placeholder="OneNet 项目概览可见, 用于期望值下发"
          placeholder-class="form-placeholder"
        />
      </view>
    </view>

    <!-- 操作按钮 -->
    <view class="action-row">
      <button
        v-if="state !== 'ready'"
        class="btn btn-primary"
        :loading="connecting"
        @click="onConnect"
      >验证配置</button>

      <button
        v-else
        class="btn btn-danger"
        @click="onReset"
      >断开连接</button>

      <button
        class="btn btn-ghost"
        @click="onReset"
      >恢复默认</button>
    </view>

    <!-- 调试信息 -->
    <view class="card">
      <view class="card-title">调试</view>
      <view class="row">
        <text class="label">设备路径</text>
        <text class="value mono">{{ devicePath }}</text>
      </view>
      <view class="row">
        <text class="label">API 状态</text>
        <text class="value">{{ state === 'ready' ? '已验证' : '未验证' }}</text>
      </view>
    </view>

    <!-- 错误提示 -->
    <view v-if="errorMsg" class="error-box">
      <text>{{ errorMsg }}</text>
    </view>
  </view>
</template>

<script>
import { getConfig, saveConfig, resetConfig } from '../../common/config.js'
import { oneApi } from '../../common/oneapi.js'
import { computeToken } from '../../common/onenet.js'

export default {
  data() {
    return {
      form: {},
      state: 'idle',
      errorMsg: '',
      connecting: false
    }
  },
  computed: {
    stateClass() {
      switch (this.state) {
        case 'ready':    return 'state-ok'
        case 'error':    return 'state-err'
        default:         return 'state-idle'
      }
    },
    statusText() {
      if (this.state === 'ready') return '配置验证通过'
      if (this.state === 'error') return '配置错误'
      return '未连接'
    },
    devicePath() {
      return `$sys/${this.form.productId}/${this.form.deviceName}`
    }
  },
  onLoad() {
    this.form = Object.assign({}, getConfig())
    try {
      this.form.projectId = uni.getStorageSync('ebr_project_id_v1') || ''
    } catch (e) {
      try {
        if (typeof localStorage !== 'undefined') {
          this.form.projectId = localStorage.getItem('ebr_project_id_v1') || ''
        }
      } catch (e2) {}
    }

    this._unsubState = oneApi.onState((s, e) => {
      this.state = s
      this.errorMsg = e || ''
    })
  },
  onUnload() {
    if (this._unsubState) this._unsubState()
  },
  methods: {
    async onConnect() {
      // 1. 保存配置
      saveConfig({
        productId: this.form.productId,
        deviceName: this.form.deviceName,
        deviceAccessKey: this.form.deviceAccessKey
      })
      try {
        uni.setStorageSync('ebr_project_id_v1', this.form.projectId || '')
        if (typeof localStorage !== 'undefined') {
          localStorage.setItem('ebr_project_id_v1', this.form.projectId || '')
        }
      } catch (e) {}

      // 2. 验证 token
      this.connecting = true
      this.errorMsg = ''
      try {
        computeToken()
        oneApi._setState('ready', null)
        uni.showToast({ title: '配置验证通过', icon: 'success' })
      } catch (e) {
        this.errorMsg = 'token 计算失败: ' + (e.message || e)
        oneApi._setState('error', this.errorMsg)
      }

      this.connecting = false
    },

    onReset() {
      uni.showModal({
        title: '确认',
        content: '恢复 OneNet 默认配置? (不会影响 ESP32 端)',
        success: (res) => {
          if (res.confirm) {
            resetConfig()
            this.form = Object.assign({}, getConfig())
            try {
              uni.removeStorageSync('ebr_project_id_v1')
              if (typeof localStorage !== 'undefined') {
                localStorage.removeItem('ebr_project_id_v1')
              }
            } catch (e) {}
            this.form.projectId = ''
            oneApi._setState('idle', null)
            uni.showToast({ title: '已恢复', icon: 'success' })
          }
        }
      })
    }
  }
}
</script>

<style scoped>
.container {
  padding-bottom: 40rpx;
}

.status-bar {
  display: flex;
  align-items: center;
  padding: 24rpx 32rpx;
  margin: 20rpx;
  border-radius: 12rpx;
  background: #fff;
  box-shadow: 0 2rpx 8rpx rgba(0, 0, 0, 0.06);
}

.status-dot {
  width: 24rpx;
  height: 24rpx;
  border-radius: 50%;
  margin-right: 16rpx;
}

.state-ok .status-dot { background: #1aad19; }
.state-err .status-dot { background: #d9534f; }
.state-idle .status-dot { background: #aaa; }

.status-text {
  font-size: 32rpx;
  font-weight: 600;
}

.state-ok .status-text { color: #1aad19; }
.state-err .status-text { color: #d9534f; }
.state-idle .status-text { color: #888; }

.form-item {
  padding: 16rpx 0;
  border-bottom: 1rpx solid #f0f0f0;
}

.form-item:last-child { border-bottom: none; }

.form-label {
  display: block;
  font-size: 26rpx;
  color: #888;
  margin-bottom: 12rpx;
}

.form-input {
  width: 100%;
  font-size: 32rpx;
  color: #333;
  padding: 8rpx 0;
}

.form-placeholder {
  color: #bbb;
}

.form-hint {
  display: block;
  font-size: 22rpx;
  color: #aaa;
  margin-top: 8rpx;
}

.action-row {
  display: flex;
  flex-direction: column;
  gap: 20rpx;
  padding: 0 20rpx;
  margin: 20rpx 0;
}

.btn {
  height: 88rpx;
  line-height: 88rpx;
  border-radius: 12rpx;
  font-size: 32rpx;
}

.btn-primary {
  background: #1aad19;
  color: #fff;
}

.btn-danger {
  background: #d9534f;
  color: #fff;
}

.btn-ghost {
  background: #fff;
  color: #666;
  border: 1rpx solid #ddd;
}

.error-box {
  background: #fef0ef;
  color: #d9534f;
  padding: 20rpx;
  margin: 20rpx;
  border-radius: 12rpx;
  font-size: 26rpx;
  border: 1rpx solid #f5c6cb;
}

.row {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 12rpx 0;
}

.label { font-size: 26rpx; color: #888; }
.value { font-size: 26rpx; color: #333; }
.mono { font-family: monospace; font-size: 24rpx; word-break: break-all; }
</style>
