# GitHub 发布指南

> 仓库: https://github.com/mack-hx/esp-

## ⚠ 发布策略（重要）

- **GitHub 上保留所有历史版本**，旧 release 和旧 tag **一律不删**
- **本地只保留最新两版**（省磁盘空间），旧版本的源码在 git 历史里，需要时
  `git checkout <tag>` 就能取回
- 本地 `releases/` 也只放最新两版的压缩包；旧版的压缩包在 GitHub release 附件里

## 当前状态

- ✅ 代码已提交到本地 git 并推送到 GitHub
- ✅ GitHub 上共 13 个 release（v0.1.0 ~ v1.5、v0.17、v0.2、v0.21、v0.22、v0.30；v1.6 只有 tag 没有 release）
- ✅ 本地只保留 `0.22/` 和 `0.30/` 两个版本目录
- ✅ `releases/` 里只放最新两版的源码包

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
> `YOUR_TOKEN_HERE`, 并把未推送的历史用 `filter-branch` 重写掉了。
> 但那把 token **曾经公开过**, 必须在 GitHub 后台吊销重建。

### 方式 3: 手动在 GitHub 网页操作

1. 访问 https://github.com/mack-hx/esp-
2. 点击 "Upload files"
3. 拖入 `releases/` 里的压缩包
4. 提交

## 发布一个新版本

```bash
# 1. 打标签 (版本号用新编号, 例如 0.3)
git tag -a v0.3 -m "版本 0.3"
git push origin v0.3

# 2. 打包 (只含源码, 不含构建产物)
tar czf releases/esp_balance_robot-0.3.tar.gz \
  --exclude='node_modules' --exclude='unpackage' --exclude='.pio' \
  --exclude='.cache' --exclude='.DS_Store' --exclude='.hbuilderx' \
  --exclude='compile_commands.json' 0.3

# 3. 发 release (不要删任何已有的 release)
gh release create v0.3 \
  --title "Version 0.3" \
  --notes-file releases/RELEASE_NOTES.md \
  releases/esp_balance_robot-0.3.tar.gz
```

## GitHub 上的版本列表

| Tag | 说明 | 附件 |
|-----|------|------|
| v0.32 | 位置环默认打开 + bal_kd 0.4 生效 + trace 组件化 | esp_balance_robot-0.32.tar.gz |
| v0.31 | 位置环接入 (默认关闭) + bal_kd 1.5→0.4 | esp_balance_robot-0.31.tar.gz |
| v0.30 | 新迭代基线 (收编平衡控制三处致命修复 + App 状态页修复) | esp_balance_robot-0.30.tar.gz |
| v0.22 | 新迭代基线（0.21 的干净复制, 仅版本号不同） | esp_balance_robot-0.22.tar.gz |
| v0.21 | 20Hz pitch 高速通道 + LED/电磁铁/微动开关 检查页 | esp_balance_robot-0.21.tar.gz |
| v0.2 | 三端打通里程碑 | esp_balance_robot-0.2.tar.gz |
| v0.17 | 原 1.7, 打通三端通信 + 代码整理 | esp_balance_robot-0.17.tar.gz |
| v1.5 | OneNET SDK 集成 | esp_balance_robot-1.5.tar.gz |
| v1.4.0 | WiFi 稳定性修复 & UI 优化 | esp_balance_robot-1.4.tar.gz |
| v1.3.0 | 方向参数烘焙 & App 框架 | esp_balance_robot-1.3.tar.gz |
| v1.2.0 | 平衡控制优化 | esp_balance_robot-1.2.tar.gz |
| v1.1.0 | 配平表简化 | esp_balance_robot-1.1.tar.gz |
| v1.0.0 | 平衡控制算法框架 | esp_balance_robot-1.0.tar.gz |
| v0.2.0 | WiFi 热点 + 网页调参 | esp_balance_robot-0.2.tar.gz |
| v0.1.0 | 项目初始化 | esp_balance_robot-0.1.tar.gz |
| v1.6 | 修复 OneNET MQTT 连接（仅 tag，1.6 目录当时没进 git，无源码包） | — |

## 版本号规则

项目还没正式实现, 版本号**不使用 1 以上的数字**。整体缩小 10 倍:

| 旧编号 | 新编号 |
|---|---|
| 0.1 / 0.2 | 0.01 / 0.02 |
| 1.0 ~ 1.7 | 0.1 ~ 0.17 |
| 2.0 | 0.2 |

之后按 `0.01 → 0.02 → 0.1 → 0.11 → 0.12 …` 递增。

> 历史上已经发布过的版本**沿用原来的 tag 名** (v0.1.0 ~ v1.6), 不改名,
> 免得已有的下载链接失效。新版本才用新编号。

## 打包命令

```bash
cd /Volumes/Moving/work/01_code/03_Personal/ESP32/esp_balance_rob
for v in 0.22 0.30; do
  tar czf releases/esp_balance_robot-$v.tar.gz \
    --exclude='node_modules' --exclude='unpackage' --exclude='.pio' \
    --exclude='.cache' --exclude='.DS_Store' --exclude='.hbuilderx' \
    --exclude='compile_commands.json' $v
done
```

> 压缩包里只含源码, 不含 `node_modules` / `.pio` 等构建产物。
> 拿到源码后: `cd <版本>/app && npm install`, 固件用 `cd <版本> && pio run`。

## 取回已删除的旧版本源码

本地目录删了, 但 git 历史里都在:

```bash
git archive v1.3.0 1.3 | tar x -C /tmp/old/    # 取某个版本
git checkout v1.3.0                            # 切到那个版本看
```

## 查看压缩包位置

```bash
ls -lh /Volumes/Moving/work/01_code/03_Personal/ESP32/esp_balance_rob/releases/
```
