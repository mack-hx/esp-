<script>
// App.vue 是 uni-app 的根组件
// 启动时从本地存储恢复 OneNet 连接配置
import { initConfig } from './common/config.js'

export default {
  onLaunch: function() {
    console.log('[App] onLaunch')
    // 恢复本地 OneNet 配置 (productId / deviceName / deviceAccessKey)
    initConfig()
    console.log('[App] OneNet 配置已恢复')
  },
  onShow: function() {
    console.log('[App] onShow')
  },
  onHide: function() {
    console.log('[App] onHide')
  }
}
</script>

<style>
/* 全局样式 */
page {
  background-color: #F8F8F8;
  font-size: 28rpx;
  color: #333;
}

/* 通用按钮重置 */
button {
  margin: 0;
  padding: 0;
}

button::after {
  border: none;
}

/* 通用卡片 */
.card {
  background: #fff;
  border-radius: 12rpx;
  padding: 24rpx;
  margin: 20rpx;
  box-shadow: 0 2rpx 8rpx rgba(0, 0, 0, 0.06);
}

.card-title {
  font-size: 32rpx;
  font-weight: 600;
  color: #1aad19;
  margin-bottom: 16rpx;
  border-left: 6rpx solid #1aad19;
  padding-left: 16rpx;
}

.label {
  font-size: 26rpx;
  color: #888;
}

.value {
  font-size: 32rpx;
  color: #333;
  font-weight: 500;
}

.row {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 16rpx 0;
  border-bottom: 1rpx solid #f0f0f0;
}

.row:last-child {
  border-bottom: none;
}
</style>
