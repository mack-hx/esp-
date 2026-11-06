<template>
  <view class="container">
    <!-- 顶部连接状态卡 -->
    <view class="status-bar" :class="stateClass">
      <view class="status-dot"></view>
      <text class="status-text">{{ statusText }}</text>
    </view>

    <!-- 蓝牙直控: 连上后移动/急停/起立和 pitch 曲线走蓝牙 -->
    <view class="card">
      <view class="card-title">蓝牙直控</view>
      <view v-for="w in ['main', 'sub']" :key="w" class="row">
        <text class="label">{{ w === 'main' ? '主车' : '副车' }}</text>
        <text class="value" :class="ble[w] ? 'ble-on' : ''">{{ ble[w] ? '已连接' : '未连接' }}</text>
        <button v-if="ble[w]" class="btn-mini" @click="onBleDisconnect(w)">断开</button>
      </view>
      <button class="btn btn-primary" :loading="scanning" :disabled="scanning" @click="onBleScan">
        {{ scanning ? '扫描中...' : '扫描小车' }}
      </button>
      <view v-for="d in found" :key="d.deviceId" class="row">
        <text class="label">{{ d.which === 'main' ? '主车' : '副车' }} {{ d.name }} ({{ d.RSSI }}dBm)</text>
        <button class="btn-mini" :disabled="linking === d.deviceId" @click="onBleConnect(d)">
          {{ linking === d.deviceId ? '连接中' : '连接' }}
        </button>
      </view>
      <text class="form-hint">手机需打开蓝牙和定位权限。连上后移动、急停、起立和 Pitch 曲线走蓝牙, 其它功能仍走 OneNet。</text>
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
        <text class="form-label">AccessKey</text>
        <view class="secret-row">
          <input
            v-model="form.deviceAccessKey"
            class="form-input secret-input"
            :password="hideMain"
            placeholder="Base64 字符串, 设备详情 → 鉴权信息"
            placeholder-class="form-placeholder"
          />
          <text class="mini-btn" @click="hideMain = !hideMain">{{ hideMain ? '显示' : '隐藏' }}</text>
          <text class="mini-btn" @click="copySecret('main')">复制</text>
        </view>
        <text class="form-hint">设备级 AccessKey 即可 (跟 ESP32 端用同一把, 平台会同时校验产品+设备权限)</text>
      </view>

      <view class="form-item">
        <text class="form-label">副车设备名</text>
        <input
          v-model="form.subDeviceName"
          class="form-input"
          placeholder="同一产品下副车的设备名"
          placeholder-class="form-placeholder"
        />
      </view>

      <view class="form-item">
        <text class="form-label">副车 AccessKey</text>
        <view class="secret-row">
          <input
            v-model="form.subDeviceAccessKey"
            class="form-input secret-input"
            :password="hideSub"
            placeholder="副车设备详情页的设备级 AccessKey"
            placeholder-class="form-placeholder"
          />
          <text class="mini-btn" @click="hideSub = !hideSub">{{ hideSub ? '显示' : '隐藏' }}</text>
          <text class="mini-btn" @click="copySecret('sub')">复制</text>
        </view>
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

      <button class="btn btn-primary" @click="onSendToBLE('main')" v-if="ble.main">
        ➤ 发送配置到主车
      </button>
      <button class="btn btn-primary" @click="onSendToBLE('sub')" v-if="ble.sub">
        ➤ 发送配置到副车
      </button>
    </view>

    <!-- 调试信息 -->
    <view class="card">
      <view class="card-title">调试</view>
      <view class="row">
        <text class="label">主车路径</text>
        <text class="value mono">{{ devicePath }}</text>
      </view>
      <view class="row">
        <text class="label">副车路径</text>
        <text class="value mono">$sys/{{ form.productId }}/{{ form.subDeviceName }}</text>
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
import * as bleApi from '../../common/ble.js'

export default {
  data() {
    return {
      form: {},
      hideMain: true,     // 主车 AccessKey 是否掩码显示
      hideSub: true,      // 副车 AccessKey 是否掩码显示
      state: 'idle',
      errorMsg: '',
      connecting: false,
      ble: { main: false, sub: false },   // 两车蓝牙是否已连接
      scanning: false,                    // 是否在扫描
      found: [],                          // 扫到的车 [{ deviceId, name, which, RSSI }]
      linking: ''                         // 正在连接的 deviceId
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

    this._unsubState = oneApi.onState((s, e) => {
      this.state = s
      this.errorMsg = e || ''
    })
    this.ble = { main: bleApi.isConnected('main'), sub: bleApi.isConnected('sub') }
    this._unsubBle = bleApi.onBleState((w, v) => { this.ble[w] = v })
  },
  onUnload() {
    if (this._unsubState) this._unsubState()
    if (this._unsubBle) this._unsubBle()
  },
  methods: {
    async onBleScan() {
      this.found = []
      this.scanning = true
      try {
        await bleApi.scan(d => this.found.push(d))
        if (!this.found.length) uni.showToast({ title: '没扫到小车', icon: 'none' })
      } catch (e) {
        uni.showToast({ title: '蓝牙不可用: ' + (e.errMsg || e.message || e), icon: 'none' })
      }
      this.scanning = false
    },
    async onBleConnect(d) {
      this.linking = d.deviceId
      try {
        await bleApi.connect(d.which, d.deviceId)
        this.found = this.found.filter(x => x.deviceId !== d.deviceId)
        uni.showToast({ title: '蓝牙已连接', icon: 'success' })
      } catch (e) {
        uni.showToast({ title: '连接失败: ' + (e.errMsg || e.message || e), icon: 'none' })
      }
      this.linking = ''
    },
    onBleDisconnect(w) {
      bleApi.disconnect(w)
    },
    
    /** 发送 WiFi+MQTT 配置到小车 (通过 BLE)
     * @param {string} which 'main' 主车 / 'sub' 副车
     */
    async onSendToBLE(which) {
      const cfg = this.form
      if (!cfg.productId || !cfg.deviceName) {
        uni.showToast({ title: 'OneNet 配置不完整', icon: 'none' })
        return
      }
      
      // 构造 WiFi 配置命令: CFGW:{"ssid":"...","pass":"..."}
      // 这里通过 WiFi 扫描接口从网页返回，简单起见发送 MQTT 配置
      const mqttCfg = JSON.stringify({
        pid: cfg.productId.trim(),
        dev: cfg.deviceName.trim(),
        ak: cfg.deviceAccessKey.trim()
      })
      const cmd = 'CFGM:' + mqttCfg
      
      try {
        await bleApi.send(which, cmd)
        uni.showToast({ title: '配置已发送，设备重启中...', icon: 'success' })
      } catch (e) {
        uni.showToast({ title: '发送失败: ' + (e.message || e), icon: 'none' })
      }
    },
     * @param {string} which 'main' 主车 / 'sub' 副车
     */
    copySecret(which) {
      const text = which === 'sub' ? this.form.subDeviceAccessKey : this.form.deviceAccessKey
      if (!text) {
        uni.showToast({ title: 'AccessKey 为空', icon: 'none' })
        return
      }
      uni.setClipboardData({
        data: text,
        success: () => uni.showToast({ title: '已复制', icon: 'success' }),
        fail: (e) => uni.showToast({ title: '复制失败: ' + (e.errMsg || e), icon: 'none' })
      })
    },
    async onConnect() {
      // 1. 保存配置 (粘贴带进来的前后空格一律裁掉, 否则平台报 invalid authorization)
      saveConfig({
        productId: (this.form.productId || '').trim(),
        deviceName: (this.form.deviceName || '').trim(),
        deviceAccessKey: (this.form.deviceAccessKey || '').trim(),
        subDeviceName: (this.form.subDeviceName || '').trim(),
        subDeviceAccessKey: (this.form.subDeviceAccessKey || '').trim()
      })
      this.form = Object.assign({}, getConfig())

      // 2. 真的去平台拉一次设备属性, 能拉到才算配置正确
      this.connecting = true
      this.errorMsg = ''
      const ok = await oneApi.ensureReady()
      this.connecting = false
      if (ok) {
        uni.showToast({ title: '连接成功', icon: 'success' })
      } else {
        this.errorMsg = oneApi.getState().error || '连接失败'
        uni.showToast({ title: '连接失败', icon: 'none' })
      }
    },

    onReset() {
      uni.showModal({
        title: '确认',
        content: '恢复 OneNet 默认配置? (不会影响 ESP32 端)',
        success: (res) => {
          if (res.confirm) {
            resetConfig()
            this.form = Object.assign({}, getConfig())
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

.secret-row {
  display: flex;
  align-items: center;
}

.secret-input {
  flex: 1;
}

.mini-btn {
  font-size: 24rpx;
  color: #1677ff;
  padding: 8rpx 16rpx;
  white-space: nowrap;
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
.ble-on { color: #1677ff; }

.btn-mini {
  margin: 0 0 0 16rpx;
  padding: 0 24rpx;
  height: 56rpx;
  line-height: 56rpx;
  font-size: 24rpx;
  background: #fff;
  border: 1rpx solid #ddd;
}

.mono { font-family: monospace; font-size: 24rpx; word-break: break-all; }
</style>
