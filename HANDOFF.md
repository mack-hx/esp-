# 跨对话交接文档 - ESP32 机械狗平衡车项目 0.45

## 当前状态
- **版本**：0.45 (稳定)
- **编译状态**：✓ 主车通过编译，副车待修复（系统 CMake 错误）
- **上次对话**：2026-11-06 21:30 WiFi 配网 + BLE 配置导入

## 已完成功能

### WiFi + 云平台配置 (完整)
- **运行时 NVS 配置**：产品 ID/设备名/AccessKey 从代码中移除 ✓
- **Captive Portal**：连接热点自动弹出配网页（DNS 劫持）✓
- **WiFi 扫描**：真实扫描列表，返回 SSID/信号强度/频道 ✓
- **可选 MQTT**：先配 WiFi 再配密钥 ✓
- **App 导入**：支持 JSON 配置字符串导入 ✓
- **BLE 配置通道**：CFGW/CFGM 指令直接配置 + 重启 ✓
- **两阶段配置**：快速 WiFi 配网 + 后补 MQTT 密钥 ✓

### 新增 API 端点
- `GET /api/wifi/scan` → 返回 WiFi 列表 (AP+STA 混合模式)
- `GET /api/cfg/get` → 返回已保存的 WiFi/MQTT 配置 (用于 App 导入)
- `POST /api/wifi` → 保存 WiFi+MQTT 配置 (含参数 trim)

### 功能清单
- ✓ 双车架构 (主车 balancing_dog + 副车 balancing_dog_sub)
- ✓ 舵机/电机/LED/磁铁/对接开关 驱动
- ✓ IMU 平衡控制 + 蓝牙遥控
- ✓ 舵机巡线识别
- ✓ 串行通信 (Modbus 从机)
- ✓ MQTT 自动上报 (位置/温度/电池等)
- ✓ 设置页面 + LED 颜色调试

## 已知问题与改进项

### 待办 (Next)
1. **副车编译修复**：macOS xcrun 架构问题 (系统层, 非代码)
   - 主车已成功编译, 代码改动已同步到副车
   - 需测试硬件验证 WiFi 扫描、DNS 劫持、BLE 配置

2. **实测验证**：
   - Captive Portal 自动弹网页
   - WiFi 扫描是否返回真实列表 (AP+STA 模式)
   - BLE 配置发送后设备是否正确重启

3. **优化空间**：
   - WiFi 扫描延迟优化 (1-1.5s)
   - DNS 答复速度优化
   - BLE JSON 解析完善度 (当前极简版本)

## 编译 / 烧录命令

```bash
# 编译 (验证)
cd 0.45/main_car && pio run

# 烧录主车
cd 0.45/main_car && pio run --target upload

# 烧录副车 (待修复)
cd 0.45/sub_car && pio run --target upload
```

## 配网流程 (最新完整版)

### 场景 1: 快速 WiFi 配网 (网页)
1. 手机连接 AP: `BalanceRobot_Setup_XXYY` (密码 `12345678`)
2. 打开浏览器 → 自动跳配网页 (Captive Portal DNS 劫持)
3. 点「扫描 WiFi」→ 选择家里的 WiFi
   - 或点「App 导入」从已保存配置导入 (读 `/api/cfg/get`)
4. 输密码 (可选，开放网络留空) → 点「保存」
5. 设备重启，连接 WiFi 进 STA 模式

### 场景 2: 从 App 导入配置 (BLE)
1. App 先用蓝牙连接主车/副车 (已连接显示「已连接」)
2. 在「OneNet 产品 / 设备」配置三元组（产品 ID、设备名、AccessKey）
3. 点「发送配置到主车」/ 「发送配置到副车」
4. 车端收到 `CFGM:{"pid":"...","dev":"...","ak":"..."}` 指令
5. 设备重启，MQTT 配置生效，连接 OneNET 云平台

### 场景 3: 从网页导入 App 配置
1. 设备在 AP 模式，手机已连接热点
2. 打开配网页，点「App 导入」
3. 调用 `GET /api/cfg/get` 获取 ESP32 已保存配置
4. 自动填入 WiFi/MQTT 三元组字段

## NVS 存储

| 命名空间 | 用途 | 字段 |
|---------|------|------|
| `wifi_cfg` | WiFi 凭证 | `ssid`, `pass` |
| `onenet_cfg` | MQTT 配置 | `pid`, `dev`, `ak` |

## BLE 协议扩展

### 指令格式
```
M:<v>                          移动 (v = -100~100)
STOP                           急停
STAND                          起立
CFGW:{"ssid":"...","pass":"..."} WiFi 配置 (JSON, 可选)
CFGM:{"pid":"...","dev":"...","ak":"..."} MQTT 配置 (JSON, 可选)
```

### 流程
- App 调用 `bleApi.send(which, cmd)` 发指令
- ESP32 BLE 回调 `ble_handle_cmd()` 解析
- WiFi/MQTT 配置指令：极简 JSON 解析 → NVS 保存 → `esp_restart()`

## 关键文件

```
0.45/
├── main_car/
│   ├── hardware/wifi_sta/
│   │   ├── wifi_sta.c        (DNS 劫持 + WiFi 扫描 + HTTP API + AP+STA 混合)
│   │   ├── wifi_sta.h        (新增 wifi_sta_save_credentials API)
│   ├── user/ble_ctrl/
│   │   ├── ble_ctrl.c        (新增 CFGW/CFGM 指令处理)
│   ├── user/onenet_client/   (MQTT 配置读写)
│   └── main/main.c           (入口)
├── sub_car/                  (同步改动)
├── app/
│   ├── pages/connect/
│   │   ├── connect.vue       (新增 onSendToBLE + App导入按钮)
│   ├── common/ble.js         (已有 BLE 发送接口)
```

## 下一步建议

1. **硬件验证**：
   - 串口看 WiFi 扫描、DNS 回应日志
   - 测试 Captive Portal 自动弹网页
   - 验证 BLE 配置指令正确解析

2. **副车修复**：
   - 确认 macOS 构建环境（xcrun 问题）
   - 或在 Linux 环境编译副车

3. **性能优化**：
   - WiFi 扫描超时缩短 (当前 150ms 可能不足)
   - DNS 查询响应时间 (实测是否 < 100ms)

4. **错误诊断**：
   - 配网页显示连接失败原因 (DNS 超时 vs 网络无连接)
   - BLE 发送失败时的重试机制

## 版本迭代记录

| 版本 | 日期 | 内容 |
|-----|------|------|
| 0.45 | 2026-10-26 | 双车架构, 完整驱动 |
| 0.45.1 | 2026-11-06 | MQTT 运行时化 |
| 0.45.2 | 2026-11-06 | Captive Portal + 真实 WiFi 扫描 |
| 0.45.3 | 2026-11-06 | BLE 配置导入 + App 按钮 |

## 编译 / 烧录命令

```bash
# 编译 (验证)
cd 0.45/main_car && pio run

# 烧录主车
cd 0.45/main_car && pio run --target upload

# 烧录副车
cd 0.45/sub_car && pio run --target upload
```

## 配网流程 (最新)

### 快速配网 (WiFi 仅)
1. 手机连接 AP: `BalanceRobot_Setup_XXYY` (XX=MAC 后两字节)
2. 打开浏览器 → 自动跳配网页 (Captive Portal)
3. 点「扫描 WiFi」→ 选择家里的 WiFi
4. 输密码 → 保存
5. 设备重启，连上 WiFi，进 STA 模式

### 补充 MQTT 密钥
1. 在配网页"手动配置"填 MQTT 三元组
2. 或用 App 配置页面 JSON 导入
3. 保存 → 重启 → 连接 OneNET

## NVS 命名空间

| 命名空间 | 用途 | 字段 |
|---------|------|------|
| `wifi_cfg` | WiFi 凭证 | `ssid`, `pass` |
| `onenet_cfg` | MQTT 配置 | `pid`, `dev`, `ak` |

## 关键文件

```
0.45/
├── main_car/
│   ├── user/onenet_client/  (MQTT 客户端, NVS 配置)
│   ├── hardware/wifi_sta/   (WiFi + 配网页, Captive Portal)
│   └── main/main.c          (入口)
├── sub_car/                 (副车, 同步改动)
```

## 下一步建议

1. **真实 WiFi 扫描**：实现后端扫描端点 (注意延迟优化)
2. **App 集成**：配网页支持从 App 接收配置
3. **错误诊断**：配网页显示连接失败原因 (MQTT AK 错 vs 网络问题)
4. **集群配置**：一个配置页批量配置多台设备

## 版本迭代记录

| 版本 | 日期 | 内容 |
|-----|------|------|
| 0.45 | 2026-10-26 | 双车架构, 完整驱动 |
| 0.45.1 | 2026-11-06 | MQTT 运行时化 (密钥从代码消失) |
| 0.45.2 | 2026-11-06 | 配网页大改 (Captive Portal + 可选 MQTT) |
