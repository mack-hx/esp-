# ESP32-S3R8N8 平衡轮腿机械狗 - 版本 0.1

ESP32-S3R8N8 平衡轮腿可分离机械狗控制系统

## 硬件配置

- **主控**：ESP32-S3R8N8
- **PSRAM**：8MB Octal SPI
- **Flash**：8MB
- **开发框架**：ESP-IDF (PlatformIO)
- **CPU频率**：240MHz

## 项目结构

```
0.1/
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
cd 0.1

# 编译
pio run

# 烧录
pio run -t upload

# 串口监视
pio device monitor
```

## 开发日志

所有开发过程记录在项目根目录 `logs/` 文件夹中
