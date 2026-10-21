<template>
  <view class="container">
    <!-- 顶部: 版本/数据来源 -->
    <view class="top-bar">
      <text class="top-text">参数表 v7 · 共 {{ paramCount }} 项</text>
      <view class="top-actions">
        <view class="top-btn" @click="onRefresh">拉取</view>
        <view class="top-btn btn-primary" @click="onSave">保存</view>
        <view class="top-btn btn-warn" @click="onReset">恢复默认</view>
      </view>
    </view>

    <!-- 改动提示 -->
    <view v-if="dirty" class="dirty-tip">
      <text>有 {{ dirtyCount }} 项未保存</text>
      <view class="top-btn btn-primary" @click="onSave">立即保存</view>
    </view>

    <!-- 按组渲染 -->
    <view v-for="(grp, gi) in groupList" :key="gi" class="grp-card">
      <view class="grp-title">{{ grp.label }}</view>

      <view v-for="def in grp.items" :key="def.id" class="param-row">
        <view class="param-head">
          <text class="param-label">{{ def.label }}</text>
          <text class="param-hint">{{ def.hint }}</text>
        </view>
        <view class="param-ctrl">
          <view class="param-slider">
            <slider
              :min="def.min"
              :max="def.max"
              :step="def.step"
              :value="values[def.key]"
              :block-size="20"
              activeColor="#1aad19"
              backgroundColor="#e0e0e0"
              @change="onSliderChange(def, $event)"
              @changing="onSliderChange(def, $event)"
              show-value
            />
          </view>
          <view class="param-value">
            <text class="value-num">{{ formatVal(def, values[def.key]) }}</text>
            <text class="value-unit" v-if="def.unit">{{ def.unit }}</text>
            <text v-if="values[def.key] !== def.def" class="value-mark">●</text>
          </view>
        </view>
      </view>
    </view>

    <view class="bottom-spacer"></view>
  </view>
</template>

<script>
import { PARAM_DEFS, PARAM_GROUPS, getDefaultValues } from '../../common/paramDef.js'
import { controller } from '../../utils/control.js'
import { oneApi } from '../../common/oneapi.js'

export default {
  data() {
    return {
      values: getDefaultValues(),
      // 工作副本: 用户拖动滑块先改这里, 点保存才回写
      dirtyValues: getDefaultValues(),
      dirty: false
    }
  },
  computed: {
    paramCount() { return PARAM_DEFS.length },
    groupList() {
      const groups = {}
      PARAM_DEFS.forEach(d => {
        if (!groups[d.grp]) groups[d.grp] = []
        groups[d.grp].push(d)
      })
      return Object.keys(groups).map(g => ({
        key: PARAM_GROUPS[g].key,
        label: PARAM_GROUPS[g].label,
        items: groups[g]
      }))
    },
    dirtyCount() {
      return PARAM_DEFS.filter(d => this.dirtyValues[d.key] !== d.def).length
    }
  },
  onLoad() {
    this._unsubState = oneApi.onState((s) => { this.state = s })
  },
  onShow() {
    this.onRefresh()
  },
  onUnload() {
    if (this._unsubState) this._unsubState()
  },
  methods: {
    onSliderChange(def, e) {
      this.dirtyValues[def.key] = e.detail.value
      this.dirty = true
    },
    async onRefresh() {
      if (oneApi.getState().state !== 'ready') {
        uni.showToast({ title: '未连接 OneNet', icon: 'none' })
        return
      }
      try {
        uni.showLoading({ title: '拉取中...' })
        const values = await controller.getParams()
        uni.hideLoading()
        if (values) {
          this._applyValues(values)
          uni.showToast({ title: '已同步' })
        } else {
          uni.showToast({ title: '返回为空', icon: 'none' })
        }
      } catch (e) {
        uni.hideLoading()
        oneApi.markError(e.message)
        uni.showToast({ title: '拉取失败: ' + e.message, icon: 'none' })
      }
    },
    async onSave() {
      if (oneApi.getState().state !== 'ready') {
        uni.showToast({ title: '未连接 OneNet', icon: 'none' })
        return
      }
      try {
        uni.showLoading({ title: '保存中...' })
        const ok = await controller.setParams(this.dirtyValues)
        uni.hideLoading()
        if (ok) {
          this.values = Object.assign({}, this.dirtyValues)
          this.dirty = false
          uni.showToast({ title: '已保存', icon: 'success' })
        }
      } catch (e) {
        uni.hideLoading()
        oneApi.markError(e.message)
        uni.showToast({ title: '保存失败: ' + e.message, icon: 'none' })
      }
    },
    onReset() {
      uni.showModal({
        title: '恢复默认',
        content: '把所有参数恢复为出厂默认值? 还没点"保存"的话不会下发到 ESP32',
        success: (res) => {
          if (res.confirm) {
            this.dirtyValues = getDefaultValues()
            this.dirty = true
          }
        }
      })
    },
    _applyValues(obj) {
      // obj 是 {key: value} 形式, 跟 dirtyValues 结构一致
      const next = Object.assign({}, this.values)
      PARAM_DEFS.forEach(d => {
        if (obj[d.key] !== undefined && obj[d.key] !== null) {
          next[d.key] = Number(obj[d.key])
        }
      })
      this.values = next
      this.dirtyValues = Object.assign({}, next)
      this.dirty = false
    },
    formatVal(def, v) {
      return Number(v).toFixed(def.dec)
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

.top-text {
  font-size: 26rpx;
  color: #888;
}

.top-actions {
  display: flex;
  gap: 12rpx;
}

.top-btn {
  padding: 10rpx 20rpx;
  border-radius: 8rpx;
  font-size: 24rpx;
  background: #f0f0f0;
  color: #666;
  border: 1rpx solid #ddd;
}

.top-btn.btn-primary {
  background: #1aad19;
  color: #fff;
  border-color: #1aad19;
}

.top-btn.btn-warn {
  background: #f0ad4e;
  color: #fff;
  border-color: #f0ad4e;
}

.dirty-tip {
  display: flex;
  justify-content: space-between;
  align-items: center;
  background: #fff3cd;
  color: #856404;
  padding: 16rpx 24rpx;
  margin: 16rpx 20rpx;
  border-radius: 8rpx;
  font-size: 26rpx;
  border: 1rpx solid #ffe69c;
}

.grp-card {
  background: #fff;
  margin: 20rpx;
  border-radius: 12rpx;
  padding: 20rpx 24rpx;
  box-shadow: 0 2rpx 8rpx rgba(0, 0, 0, 0.06);
}

.grp-title {
  font-size: 30rpx;
  font-weight: 600;
  color: #1aad19;
  margin-bottom: 16rpx;
  border-left: 6rpx solid #1aad19;
  padding-left: 12rpx;
}

.param-row {
  padding: 20rpx 0;
  border-bottom: 1rpx solid #f0f0f0;
}

.param-row:last-child { border-bottom: none; }

.param-head {
  margin-bottom: 12rpx;
}

.param-label {
  font-size: 30rpx;
  font-weight: 500;
  color: #333;
}

.param-hint {
  display: block;
  font-size: 22rpx;
  color: #aaa;
  margin-top: 6rpx;
}

.param-ctrl {
  display: flex;
  align-items: center;
  gap: 20rpx;
}

.param-slider {
  flex: 1;
}

.param-value {
  width: 130rpx;
  text-align: right;
  display: flex;
  align-items: baseline;
  justify-content: flex-end;
  gap: 4rpx;
}

.value-num {
  font-size: 32rpx;
  font-weight: 600;
  color: #1aad19;
  font-family: monospace;
}

.value-unit {
  font-size: 22rpx;
  color: #888;
}

.value-mark {
  color: #f0ad4e;
  font-size: 20rpx;
  margin-left: 6rpx;
}

.bottom-spacer {
  height: 80rpx;
}
</style>
