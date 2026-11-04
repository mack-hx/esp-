# HANDOFF.md

## 当前状态
- **最新版本 `0.44/`**（2026-11-02 从 0.43 复制，**还没有新改动**，HBuilderX 打开 `0.44/app`）。0.43 已发布：tag v0.43 + GitHub Release + `releases/esp_balance_rob_v0.43.zip`，不再修改。
- **0.43 已发布内容（全部未烧录真机验证）**：
  - App：遥控页姿态按钮（蹲下 100° / 起立 40° / 跳舞 1Hz）+ 独立姿态页 `pages/pose`（跳舞频率可调，遥控页右下角链接进入）
  - 固件：`hardware/roll_compensator` 横滚角补偿，**默认关闭**（增益 0、死区 10°、限幅 25°），仅起立状态生效
  - 固件：`user/status_led` 状态灯自动配色（摔倒红 > 舵机粉 > 合体蓝 > 未对齐红 > 对接闪黄 > 站立绿 > 未站立黄），App/LCD 手动设色后保持
  - 姿态解算改公式：pitch `atan2(ay,az)`、roll `atan2(ax,az)`，roll 融合权重 10%，`PITCH_ZERO_REF=0`
  - LCD 主页新增 Roll 行；配平同时标定 pitch/roll 零点（NVS `bal_zero_p`/`bal_zero_r`）
  - 参数版本 **38**，`servo_c` 默认 80→100；固件版本 0.43.0、App versionCode 153
  - 脱机记录 `trace_sample_t` 加 `roll`，**68→72 字节**；`read_flash_log.py` / `capture_trace.py` 的 CSV 新增 roll 列
- 编译：0.43、0.44 的 `main_car` 均 `pio run` 通过；`scripts/run_tests.sh`（默认 0.44）55 项全过，参数表 24 项一致（版本 38）。
- 0.41 / 0.42 目录已删，源码和压缩包在 tag 与 GitHub Release 里。

## 仓库文档（2026-11-04）
- 根 `README.md` 已重写为开源介绍（功能 / 硬件引脚表 / 架构 / 快速开始 / 协议 / 工具 / 版本），新增 `LICENSE`（MIT）。
- 遗留：`0.44/main_car/README.md` 标题仍是 0.42；`0.44/README.md` 部分命令写 0.43；`0.44/app/common/config.js` 里有作者设备 AccessKey。

## 待办
1. **烧录真机验证 0.43**：pitch 新公式（能否正常直立，这是首要）、Roll 正负方向、LCD Roll 显示、配平 Roll 零点、姿态按钮与跳舞、状态灯配色。
2. App 重新打包验证姿态页与参数版本 38（建议点"恢复默认"让 `servo_c=100` 生效）。
3. Roll 补偿参数可调 + 使能开关（加入参数表或 OneNET/App 控制）——真机确认方向后再开。
4. 脱机测试：flash 记录已含 `tavg/tdiff/diff/roll`（72 字节/条），按急停或摔落后插线 `python3.11 scripts/read_flash_log.py /tmp/flog` 读出；待分析：转弯后位置不稳 / 向一侧走 / 摔倒。
5. 遥控速度环与 `BAL_POS_BLEND_RPM` 因轮速放大 2.7 倍尚未重调。
6. 静止时若再出现晃动：先查电机死区/静摩擦（静止 out 仅 ±6%，死区实测约 2.2%，参数 `deadzone` 默认 4%），不要再动位置环。

## 位置保持（LQR）现状
- 控制律：`user/pid/lqr.c`，`BAL_USE_LQR=1`（balance.c）；0 切回原串级 PID。LQR 模式下 `bal_*`/`pos_*` 滑块不生效。
- 增益：`LQR_KX -5 / KV -2.25 / KP 22 / KD 0.6 / VMAX 35`（lqr.h），单位按真实 cm（编码器两轮都四倍频）。
- 回位曲线：期望回位速度 = `sqrt(2·lqr_amax·偏差)`，限 `LQR_VMAX`=35 cm/s；`lqr_amax` 默认 8，App 在线可调。
- 位置积分：仅在 0.27~6 cm 偏差、速度 < 4 cm/s 时累积；其余指数衰减（0.3 s，锚点附近 0.5 s），不清零。积分上限 5°。
- 速度项渐隐：|v| ≤ 2 cm/s 不参与，≥ 5 cm/s 完全参与（`vel_gain`）。
- 参数表 `PARAM_DATA_VERSION`=38，车端 24 项，App `PARAM_VER`=38，改参数必须两边同步（`scripts/check_param_sync.py`）。
- 倾角限幅：位置环 8°、速度环 7°、平衡环 motor_max 95% / ang_limit 35°。起立软启动 30%/800ms。

## 工具
- 脱机 flash 记录：车离开平衡态时把最近 40 s 写进 `storage` 分区，断电不丢。读取 `python3 scripts/read_flash_log.py /tmp/flog`（约 26 s，读完板子复位）。下一次离开平衡态会覆盖旧记录，先读再测。
- 在线抓包：`python3 scripts/capture_trace.py <秒> <前缀> <串口>`。
- 真车数据：`logs/data/*.csv`（`lqr_amax8/amax30/integral_band2/band6_vmax20` 等推车记录）。

## 已否决（不要重试）
- 用辨识模型/仿真整定增益：模型与真车不符，别再靠仿真调增益。
- 超速增益刹车（仿真 G≥0.25 失稳）、叠加速度环。
- 加大 `LQR_KV` 调过冲（≥8 失稳，KV=5 出现 4 Hz 极限环）；位置偏差限幅 `ERR_SAT`（回位变慢、停在 1~5 cm）。
- 内置 `Task` 子代理不可用（Cursor 侧 bug），外部 CLI 402 欠费；不要往委派方向排障。

## 关键约定
- **发布规则（用户已明确，不用每次交代）**：发布的是"改好的上一版"，不是刚新建、还没写内容的新版。
  - 用户说"迭代到 X"：把当前版本（改好的那版）的 tag、压缩包、发布说明、GitHub Release 做完；新建的 X 目录只提交，**不打包、不打 tag、不建 Release**。
  - 流程：提交 → 推 main + tag → `git archive` 打包到 `releases/`（只留最新两版）→ 更新 `RELEASE_NOTES.md` 与 `releases/GITHUB_PUBLISH_GUIDE.md` → 建 GitHub Release（`releases/RELEASE_NOTES.md` 是根目录副本，同步覆盖）。
  - 凭据：`gh` 没登录，用 `GH_TOKEN="$(printf 'protocol=https\nhost=github.com\n\n' | git credential fill | sed -n 's/^password=//p')"` 只在当条命令的环境变量里传给 `gh release create --repo mack-hx/esp-`，不打印、不落盘。
- 版本号只在用户明确要求时递增；本地只保留最新两版（0.44、0.43），更早的在 git 历史和 tag 里。
- 本地 `logs/` 只保留最近 3 天，更早的已在 git 历史里。
- `scripts/run_tests.sh` 与 `scripts/test_hooks.py` 改完必须跑。
- 不要读自己旧会话的 transcript；跨对话结论只写本文件。
- 上下文护栏：`.cursor/hooks/context_budget.py`、`limit_verbose_output.py`；监控 `python3 scripts/ctx_check.py --daily`。
- 固件版本号来自 `main_car/version.txt`（ESP-IDF 在 CMake 配置阶段读取），改完必须 `pio run -t fullclean` 再编译，否则固件里还是旧版本字符串。
