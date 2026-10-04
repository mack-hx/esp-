# ESP32-S3R8N8 平衡轮腿机械狗 - 版本 0.2

ESP32-S3R8N8 平衡轮腿可分离机械狗控制系统

## 硬件配置

- **主控**：ESP32-S3R8N8
- **PSRAM**：8MB Octal SPI
- **Flash**：8MB
- **开发框架**：ESP-IDF (PlatformIO)
- **CPU频率**：240MHz

## 项目结构

```
0.2/
├── main/               # 主程序
│   ├── main.c          # 主函数
│   └── CMakeLists.txt
├── system/             # 系统级代码（RTOS、调度、配置）
├── hardware/           # 硬件外设驱动（每个外设独立文件夹）
├── user/
│   ├── pid/            # PID控制算法
│   └── ui/             # 用户界面
├── components/         # ESP-IDF组件
├── platformio.ini      # PlatformIO配置
├── sdkconfig.defaults  # ESP-IDF默认配置
└── CMakeLists.txt      # 项目构建配置
```

## 开发说明

按照项目规则开发：
- 外设驱动放在 `hardware/` 下独立文件夹
- 系统级代码放在 `system/`
- 用户应用代码放在 `user/`
- 详见项目根目录 `.cursor/rules/` 中的开发规范

## 编译与烧录

```bash
# 进入项目目录
cd 0.2

# 编译
pio run

# 烧录
pio run -t upload

# 清理构建
pio run -t clean
```

## 版本变更 (0.1 → 0.2)

主要工作：0.1 代码注释清理与精简。

- `user/ui/ui_servo.c`：合并 SERVO 调试页 3 个分散绘图函数为单一 `ui_servo_draw_static`
- `hardware/lcd/lcd_ui.c`：删除未使用的 `map_button_to_key()` 函数
- `hardware/servo/servo.c/h`：注释统一风格
- `hardware/motor_driver/motor_driver.c/h`：注释统一风格
- `hardware/tft_display/tft_display.c`：`tft_draw_string` 占位实现删除调试警告
- `main/main.c`：注释统一风格，清理冗余说明
- `user/ui/ui_home/menu/mpu9250/wifi_bt/encoder.c`：清理调试代码 / 未使用变量

**功能保持不变**：所有外设驱动、UI 页面、SERVO 双路独立控制 + 后台保持运行等核心功能完全保留。

详见 `../logs/2026-10-03.md` 最后一条记录。