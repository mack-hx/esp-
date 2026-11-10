# 项目长期约定（ESP32 平衡轮腿机械狗）

## 版本
- 当前开发版本 0.47，见根目录 `HANDOFF.md`；0.46 只读参考。
- 本地只保留 0.46 / 0.47 两个目录；更早版本源码在 git tag（`git archive v0.45 0.45`）和 GitHub release 里。

## 结构
- 每个版本目录含 `main_car/`、`sub_car/`、`app/`；外设驱动在 `hardware/<模块>/`。
- 工程日志 `logs/YYYY-MM-DD.md`（本地只留 3 天）；交接文件 `HANDOFF.md`（≤100 行，只写结论）。

## 硬件基线
- 主控 ESP32-S3R8N8（8MB Flash / 8MB PSRAM）。
- IMU MPU9250 走 I2C1（GPIO1/2/3）；TFT 走 SPI；SK6812 走 RMT（GPIO9）。
- 74HC595 输出控制 TB6612 方向引脚和两路电磁铁。
- 完整引脚表与硬件变更见 `.workbuddy-ai/rules/pin-config.md`。

## 常用命令
- 编译 `cd 0.47/main_car && pio run`；烧录 `pio run --target upload`。
- 上下文自查 `python3 scripts/ctx_check.py`；hook 回归 `python3 scripts/test_hooks.py`。
