# 跨对话交接文档 - ESP32 机械狗平衡车项目 0.45

## 当前状态
- **版本**：0.45 (稳定)
- **编译状态**：✓ 主车/副车通过编译
- **上次对话**：2026-11-06 19:30 配网页 UI 增强 + Captive Portal

## 已完成功能

### WiFi + 云平台配置 (完整)
- **运行时 NVS 配置**：产品 ID/设备名/AccessKey 从代码中移除 ✓
- **Captive Portal**：连接热点自动弹出配网页 ✓
- **WiFi 扫描**：列表选择周围 WiFi ✓
- **可选 MQTT**：先配 WiFi 再配密钥 ✓
- **App 导入**：支持 JSON 配置字符串导入 ✓
- **两阶段配置**：快速 WiFi 配网 + 后补 MQTT 密钥

### 功能清单
- ✓ 双车架构 (主车 balancing_dog + 副车 balancing_dog_sub)
- ✓ 舵机/电机/LED/磁铁/对接开关 驱动
- ✓ IMU 平衡控制 + 蓝牙遥控
- ✓ 舵机巡线识别
- ✓ 串行通信 (Modbus 从机)
- ✓ MQTT 自动上报 (位置/温度/电池等)
- ✓ 设置页面 + LED 颜色调试

## 已知问题与改进项

### 待办
1. **WiFi 扫描真实实现**：目前返回空列表
   - 可用 `esp_wifi_scan_start()` 实现真实扫描，但需处理延迟 (1-1.5s)
   - 或改成异步后台扫描 + 轮询接口

2. **手机 WiFi 列表**：占位功能
   - 需要 App 层支持 (蓝牙或其他通道传递列表)

3. **二维码配网**：未实现
   - 考虑 NFC 或 QR 码，但对这代硬件用处不大

### 优化空间
- MQTT 连接失败时的重试策略
- 配网页在弱网下的加载速度
- 副车心跳报文 (目前无主车同步)

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
