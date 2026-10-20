# ESP32-S3R8N8 平衡轮腿机械狗 - 版本 0.2

**本版本为 0.2 封版归档，平衡控制算法不在本版内（0.2 只做外设驱动 + 调试 UI + 网页调参框架）。**
平衡控制从 1.0 开始，见 `../1.0/`。

## 硬件配置

- **主控**：ESP32-S3R8N8
- **PSRAM**：8MB Octal SPI
- **Flash**：8MB
- **开发框架**：ESP-IDF (PlatformIO)
- **分区表**：nvs 24K / otadata 8K / phy_init 4K / ota_0 3584K / ota_1 3584K / storage 896K

> ⚠ 自定义分区表依赖全局 board 定义
> `~/.platformio/boards/esp32-s3r8n8-8mb.json`。换机器后需重新放置，
> 否则会静默退回 1MB 单应用分区（Flash 占用分母变成 1MB，占用率显示 ~85%）。

## 本版已完成的硬件映射

编号统一为「物理左 = 1，物理右 = 2」。

| 实物位置 | 编码器 | 电机 | 舵机 |
|---|---|---|---|
| 左 | `ENCODER_1` = GPIO45/46 | `MOTOR_A` = PWMA (GPIO15) | `SERVO_1` = GPIO14（正向装，走反向映射） |
| 右 | `ENCODER_2` = GPIO41/42 | `MOTOR_B` = PWMB (GPIO16) | `SERVO_2` = GPIO13（反向装，直通） |

两路舵机调同一逻辑角度时机械上朝同一方向转（视觉对称）。
物理左轮编码器计数方向已在 `encoder_init()` 里用 `pcnt_set_mode()` 反转修正。

## 项目结构

```
0.2/
├── main/                 主程序（各外设初始化顺序 + UI 主循环）
├── hardware/             硬件外设驱动（每个外设独立文件夹）
│   ├── mpu9250/          九轴 IMU（含互补滤波求欧拉角、陀螺仪零偏校准）
│   ├── encoder/          PCNT 正交编码器
│   ├── servo/            双路 PWM 舵机（含逻辑→物理角度映射 + 后台保持任务）
│   ├── motor_driver/     双路直流电机（H 桥 PWM + 方向）
│   ├── hc595/            74HC595 移位寄存器
│   ├── pcf8574/          I2C IO 扩展（按键输入）
│   ├── tft_display/      ST7735S SPI 彩屏
│   ├── lcd/              UI 调度框架 + 6x8 字模
│   ├── led/              SK6812 RGB（RMT 直驱）
│   ├── wifi/             WiFi 热点（AP 模式）
│   ├── k230_uart/        K230 视觉串口（驱动已写，尚未接线使用）
│   ├── voice_module/     语音模块（驱动已写，尚未接线使用）
│   ├── magnet/           电磁铁（驱动已写，尚未接线使用）
│   └── i2c_scanner/      I2C 总线扫描（诊断工具）
├── user/
│   ├── param/            全局可调参数表（19 项）+ NVS 持久化
│   ├── web/              HTTP 服务器 + 调参网页
│   └── ui/               6 个屏幕页面
├── partitions.csv        8MB Flash 分区表
├── platformio.ini        PlatformIO 配置
├── sdkconfig.defaults    ESP-IDF 默认配置
└── CMakeLists.txt        组件注册
```

## 调参网页

1. 手机 / 电脑连热点 `BalanceRobot_XXXX`（密码 `12345678`，后缀 = MAC 尾 2 字节）
2. 浏览器打开 `http://192.168.4.1`
3. 拖滑块实时生效（前端 80ms 防抖），点「保存到 NVS」断电不丢
4. 手机提示「无网络」是正常的：ESP32 热点没有外网出口，不影响本地调参

屏上 `WEB TUNER` 页会显示实际 SSID、IP、MAC。

电机有 4 秒看门狗：网页被关掉或手机断网会自动停电机。

## 编译与烧录

```bash
cd 0.2
pio run              # 编译
pio run -t upload    # 烧录
pio run -t clean     # 清理
```

## 版本变更 (0.1 → 0.2)

主要工作：WiFi 热点 + 网页调参替换 BLE 方案，其余外设驱动与 UI 全部打通并验证。

- 删除 `hardware/ble/`、`user/ui/ui_wifi_bt.c`（BLE 调参方案，整体废弃）
- 新增 `hardware/wifi/`、`user/param/`、`user/web/`、`user/ui/ui_wifi.c`
- 修复 8MB Flash 分区表未生效（PlatformIO 不读 `board_build.partitions`，
  需自定义 board JSON + `CONFIG_PARTITION_TABLE_CUSTOM=y`）
- 编码器 / 舵机编号统一为「物理左=1，右=2」
- 主页精简为标题 + 舵机角 + 编码值两行
- 编码电机页重排为四行 + 5 模式（M1/M2/A+B/DIR/SPD）
- MPU9250 页改为 ACC/GYR 两列对照 + 平衡车重点参数（Pitch / Gyx / 温度 / 零偏）
- 舵机注释统一、删除未使用的 `servo_set_pulse()` / `SERVO_MID_PULSE_US`
- 修正 `ui_home.c` 编码值刷新 X 坐标与首次绘制不一致（错位 2 像素）
- `encoder_get_rpm()` 补上 `interval_ms` 校验，间隔偏差超 ±50% 返回 0

Flash 占用 24.1% (884,373 / 3,670,016)，RAM 11.6%。

详见 `../logs/2026-10-04.md`。
