# ESP32 平衡轮腿可分离机械狗

基于 ESP32-S3R8N8 的平衡轮腿机械狗项目。

## 硬件配置

- **主控**：ESP32-S3R8N8
- **PSRAM**：8MB Octal SPI
- **Flash**：8MB
- **开发框架**：ESP-IDF (PlatformIO)

## 项目结构

```
project_root/
├── system/          # 系统级代码（RTOS、调度、系统配置）
├── hardware/        # 硬件外设驱动（每个外设独立子文件夹）
├── user/            # 用户应用代码
│   ├── pid/         # PID控制算法
│   └── ui/          # 用户界面
├── logs/            # 开发日志
├── 0.1/             # 历史版本（仅供参考）
└── .cursor/rules/   # 项目开发规则
```

## 版本管理

- 当前版本采用数字递增方式（如 0.1, 0.2, 1.0）
- 历史版本不得修改，仅供参考
- 所有新功能在最新版本中开发

## 开发规范

详见 `.cursor/rules/` 文件夹：
- `project-workflow.mdc` - 项目工作流程规则
- `code-structure.mdc` - 代码结构规范

## Git 仓库

- GitHub: https://github.com/mack-hx/esp-
- 版本迭代时需推送完整版本

## 开发日志

所有开发过程记录在 `logs/` 文件夹中，按日期组织。
