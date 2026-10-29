# HANDOFF

## 当前状态
- 最新版本工程：`0.32/`（0.31 为历史版本，只读不改）。
- 0.32 控制链现状：**三级串级**
  位置环(position_pid) → 锁位速度环(hold_pid) → 平衡环(balance_pid)。
  行走时由速度环(speed_pid)给倾角；站住不动时位置环接管。
  各环 PID 实现在 `user/pid/`，编排/状态机在 `user/control/balance.c`。
- 参数表：22 项，`PARAM_DATA_VERSION = 24`（车端 param.c 与 App paramDef.js 必须一致）。

## 本轮已做（2026-10-29）
1. 全工程 `@note` 注释清零（66 个文件）。
2. 整理 0.32 的代码和注释：清掉描述"改了什么"的历史流水账、修正与实际行为
   矛盾的注释（odom.h 的 100ms→10ms、balance.c 的旧架构描述、position 环
   两句互斥的说法）、补回被删掉的串级结构图和控制原理说明。
3. **补回 `user/pid/CMakeLists.txt`**（之前缺失，全新 configure 会失败），
   并把 `hold_pid.c` 加进 SRCS。
4. 删死常量 `POS_PID_V_TOTAL_MAX` / `POS_PID_KD_DEFAULT`（全工程零引用）。

## 验证状态
- `pio run` 通过：RAM 26.6%，Flash 28.4%。
- 删掉 `.pio/build` 后全新 configure + 全量编译通过（CMakeLists 修复已确认有效）。
- 从 GitHub 拉下来的干净副本再全量编译一遍，同样通过。
- App `.js` 通过 `node --check`；`.vue` 注释块闭合检查通过。

## Git 状态
- 已推送到 GitHub：`mack-hx/esp-` main 分支，提交 `217843b`
  「refactor(0.32): 整理代码与注释, 补回 user/pid/CMakeLists.txt」。
- 注意：**本地工作区本身不是 git 仓库**，历史在远端。本次是把本地 0.32
  整棵树同步进 `/tmp/esp_git_probe` 克隆再提交的（该克隆是临时目录，重启会没）。

## 约定
- 注释用 `@brief` / `@param` / `@return`；**不要引入 `@note`**（已清零）。
- `@brief` 一个函数只能有一个，不要重复用。
- 注释写"现在是什么、为什么这么做"，**不写"什么时候从什么改成什么"**
  （改动历史属于 logs/ 和 git）。
- 改固件时 `pio run` 自行验证；改 App 时 `node --check` 验证。

## 注意事项
- 本轮有**另一个 Cursor 会话同时在改 0.32 同一批文件**（14:03 那批 pid 文件）。
  它的方向是"删掉所有解释性注释"，把 pid.h / hold_pid.h / position_pid.h /
  speed_pid.h 的结构说明和原理段落删干净了；本次会话已把关键原理补回。
  继续动这几个文件前先确认当前内容。

## 待办
- 真机回归：位置环→锁位速度环→平衡环的串级改动还没上真机验证。
- `test/` 和 `scripts/` 目录只在远端仓库有（GitHub mack-hx/esp-），本地 0.32 没有；
  跑离线仿真测试需要先从远端拉这两个目录。
