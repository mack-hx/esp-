# GitHub 发布指南

## 已完成的工作
- ✅ 所有代码已提交到本地Git
- ✅ 已创建7个版本的压缩包 (releases/)
- ✅ 已创建发布说明 (releases/RELEASE_NOTES.md)

## 手动推送到GitHub

SSH连接有问题，请选择以下方式之一：

### 方式1: 使用GitHub CLI (推荐)

```bash
# 1. 安装 GitHub CLI
brew install gh

# 2. 认证
gh auth login

# 3. 推送代码
git push origin main

# 4. 创建 Releases (可选)
gh release create v0.1.0 \
  --title "Version 0.1.0" \
  --notes-file releases/RELEASE_NOTES.md
```

### 方式2: 使用HTTPS + Personal Access Token

```bash
# 1. 在 GitHub Settings > Developer settings > Personal access tokens 生成 token

# 2. 修改 remote
git remote set-url origin https://YOUR_TOKEN@github.com/mack-hx/esp_balance_robot.git

# 3. 推送
git push origin main
```

### 方式3: 手动在GitHub网页操作

1. 访问 https://github.com/mack-hx/esp_balance_robot
2. 点击 "Upload files"
3. 拖入 `releases/` 文件夹中的压缩包
4. 提交

## 版本列表

| 版本 | 压缩包 | 大小 |
|------|--------|------|
| 0.1 | esp_balance_robot-0.1.tar.gz | 230K |
| 0.2 | esp_balance_robot-0.2.tar.gz | 277K |
| 1.0 | esp_balance_robot-1.0.tar.gz | 371K |
| 1.1 | esp_balance_robot-1.1.tar.gz | 365K |
| 1.2 | esp_balance_robot-1.2.tar.gz | 297K |
| 1.3 | esp_balance_robot-1.3.tar.gz | 1.1M |
| 1.4 | esp_balance_robot-1.4.tar.gz | 548K |

## 版本时间线

- **v0.1**: 项目初始化，基础框架
- **v0.2**: WiFi热点+网页调参，驱动全部打通
- **v1.0**: 平衡控制算法框架，串级PID
- **v1.1**: 配平表简化，腿角补偿优化
- **v1.2**: 平衡控制优化
- **v1.3**: 方向参数烘焙，App框架
- **v1.4**: WiFi连接修复，主页显示优化

## 查看压缩包位置

```bash
ls -lh /Volumes/Moving/work/01_code/03_Personal/ESP32/esp_balance_robot/releases/
```
