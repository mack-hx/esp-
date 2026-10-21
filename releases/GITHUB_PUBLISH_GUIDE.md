# GitHub 发布指南

> 仓库: https://github.com/mack-hx/esp-

## 当前状态

- ✅ 代码已提交到本地 git
- ✅ 只保留最新两版: `0.17` (上一个版本) 和 `0.2` (当前版本)
- ✅ 压缩包已打好 (`releases/`)

## 推送到 GitHub

### 方式 1: GitHub CLI (推荐)

```bash
brew install gh
gh auth login

git push origin main
```

### 方式 2: HTTPS + Personal Access Token

```bash
# 在 GitHub Settings > Developer settings > Personal access tokens 生成 token
# 注意: token 不要写进仓库里的任何文件, 用环境变量传

export GH_TOKEN=ghp_xxxxxxxxxxxx
git push origin main
```

> ⚠ **不要把 token 明文写进仓库文件**。2026-10-21 发现
> `.cursor/rules/github-config.md` 里曾经硬编码过一把 PAT, 已替换成
> `YOUR_TOKEN_HERE`, 但**明文仍在 git 历史里** (提交 `7ccc8ef`),
> 所以那把 token 必须在 GitHub 后台吊销重建。

### 方式 3: 手动在 GitHub 网页操作

1. 访问 https://github.com/mack-hx/esp-
2. 点击 "Upload files"
3. 拖入 `releases/` 里的压缩包
4. 提交

## 创建 Release

```bash
# 0.2 (当前版本)
gh release create v0.2 \
  --title "Version 0.2" \
  --notes-file releases/RELEASE_NOTES.md \
  releases/esp_balance_robot-0.2.tar.gz

# 0.17 (上一个版本)
gh release create v0.17 \
  --title "Version 0.17" \
  --notes-file releases/RELEASE_NOTES.md \
  releases/esp_balance_robot-0.17.tar.gz
```

## 版本列表

| 版本 | 压缩包 | 大小 | 说明 |
|------|--------|------|------|
| 0.2 | esp_balance_robot-0.2.tar.gz | 273K | 当前版本, 三端打通 |
| 0.17 | esp_balance_robot-0.17.tar.gz | 275K | 上一个版本 |

## 版本号规则

项目还没正式实现, 版本号**不使用 1 以上的数字**。整体缩小 10 倍:

| 旧编号 | 新编号 |
|---|---|
| 0.1 / 0.2 | 0.01 / 0.02 |
| 1.0 ~ 1.7 | 0.1 ~ 0.17 |
| 2.0 | 0.2 |

之后按 `0.01 → 0.02 → 0.1 → 0.11 → 0.12 …` 递增。

## 打包命令

```bash
cd /Volumes/Moving/work/01_code/03_Personal/ESP32/esp_balance_robot
for v in 0.17 0.2; do
  tar czf releases/esp_balance_robot-$v.tar.gz \
    --exclude='node_modules' --exclude='unpackage' --exclude='.pio' \
    --exclude='.cache' --exclude='.DS_Store' --exclude='.hbuilderx' \
    --exclude='compile_commands.json' $v
done
```

> 压缩包里只含源码, 不含 `node_modules` / `.pio` 等构建产物。
> 拿到源码后: `cd <版本>/app && npm install`, 固件用 `cd <版本> && pio run`。

## 查看压缩包位置

```bash
ls -lh /Volumes/Moving/work/01_code/03_Personal/ESP32/esp_balance_robot/releases/
```
