# 2026-11-06 MQTT 配置可配置化 + 密钥零代码存储

## 目标完成
把 MQTT 三元组（产品 ID / 设备名 / AccessKey）从编译期常量改成**运行时 NVS 存储**，AP 配网页支持一次性填写 WiFi + MQTT 全参数。

## 改动清单

### 1. onenet_client.h/.c 运行时化
- 把 `ONENET_PRODUCT_ID` / `ONENET_DEVICE_NAME` 改成编译期默认值 `ONENET_DEFAULT_*`
- 删除对 `onenet_secret.h` 的依赖（不再需要填密钥文件）
- AccessKey 运行时从 NVS 读，默认空字符串（未配置）
- 新增 `onenet_config_save(pid, dev, ak)` 和 `onenet_config_load(...)` API
- Topic 改成动态缓冲 + 运行时拼接（共 9 个 topic，256 字节每个）
- mqtt_client_start() 在启动时检查 AK 为空则返回失败，不启动 MQTT

### 2. 配网页 (wifi_provision_page.c)
- 页面改为两段：WiFi 配置 + OneNET 配置
- 新增三个输入框：产品 ID、设备名、AccessKey（密码框）
- 按钮改成"保存配置并重启"，一次提交 5 个字段

### 3. wifi_sta.c POST 处理
- /api/wifi 端点改成接收 {ssid, pass, pid, dev, ak} 五元组
- WiFi 字段存到 `wifi_cfg` namespace，MQTT 字段存到 `onenet_cfg` namespace
- 调用 onenet_config_save() 保存 MQTT 参数
- 一起 commit 再重启

### 4. CMakeLists.txt 依赖
- onenet_client 组件加 `nvs_flash` 依赖

### 5. 副车同步
- 副车设备名默认值改成 `balancing_dog_sub`，其余逻辑相同

## 工作流程

### 首次使用 / 重新配置
1. 手机连接热点 `BalanceRobot_Setup_XXYY` (XX YY 是 MAC 后两字节)
2. 浏览 http://192.168.4.1
3. 填 WiFi SSID/密码 + OneNET 产品 ID/设备名/AccessKey
4. 点"保存配置并重启"
5. 设备重启，走 STA 连接 WiFi，然后用 NVS 里的 MQTT 参数启动 MQTT 客户端

### 密钥安全
- 源码仓库 `.gitignore` 已有 `onenet_secret.h`（旧配置方式的产物）
- **新方式不再有代码中的密钥** ✓
- AccessKey 填过一次后存 NVS，固件重烧不会丢失
- App 配网页和设备 AP 配网页都可以更新密钥

## 编译验证
- ✓ 主车编译通过
- ✓ 副车编译通过
- RAM: 32.5% / Flash: 32.8% (正常)

## 测试步骤（待手动执行）
1. 烧录新固件到主车
2. 启动时应进入 AP 模式（NVS 无配置）
3. 手机连接配网热点，打开配网页
4. 填入控制台新 key (SHA256: ebfb8bc7da96e04288bf8164f63a0a68122ecd4e193ce32644bcce291aba1bf6)
5. 重启后应能连接 WiFi 并启动 MQTT
6. 用 App 控制验证功能正常

## 跨版本兼容性
- 0.45 是首次支持运行时配置，旧版本（0.44）固件升级到 0.45 时 NVS 为空，会用编译默认值
- 配网页自动引导用户填写密钥
