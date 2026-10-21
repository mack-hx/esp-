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
├── 0.2/              # 当前版本
│   ├── main/         # 入口
│   ├── hardware/     # 硬件外设驱动（每个外设独立子文件夹）
│   ├── user/         # 用户应用代码 (控制算法 / 参数 / 配平 / OneNet 客户端 / 屏 UI)
│   ├── app/          # 配套 App (uni-app, H5)
│   ├── test/         # 离线单元测试
│   └── docs/         # 版本文档
├── 0.17/             # 上一个版本（历史版本不得修改，仅供参考和回滚）
├── logs/             # 开发日志（按日期组织）+ change_log.txt
├── releases/         # 打包好的发布版本（只保留最新两版）
└── .cursor/rules/    # 项目开发规则
```

每个版本目录自带完整的 `hardware/ user/ app/`，互不依赖。

## 版本管理

- 版本号数字递增（0.01, 0.02, 0.1, 0.11 …），**仅在用户明确要求时才做版本迭代**
- **不使用 1 以上的版本号**（项目还没正式实现，1 以上不规范）
- 历史版本不得修改，仅供参考和回滚
- 所有新功能在最新版本中开发
- 本地只保留最新两版，旧版目录和 release 打包都删掉（历史代码仍在 git 历史里）

## 开发规范

详见 `.cursor/rules/` 文件夹：
- `project-workflow.mdc` - 项目工作流程规则
- `code-structure.mdc` - 代码结构规范

## Git 仓库

- GitHub: https://github.com/mack-hx/esp-
- 版本迭代时需推送完整版本

## 开发日志

所有开发过程记录在 `logs/` 文件夹，按日期组织；版本级变更加入 `logs/change_log.txt`。
