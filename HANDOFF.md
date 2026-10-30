# HANDOFF.md

## 当前状态
- 最新版本工程：`0.41/`（2026-10-30 从 0.40 迭代，已推 GitHub）。`0.40/` 是上一版回滚基线，**不再修改**。
- 用户确认 0.41 的位置保持（回位速度、回位过冲、静止晃动）“没有问题了”，真车验证通过。
- 编译：`cd 0.41 && pio run` 通过；`scripts/run_tests.sh` 默认跑 0.41，仅 lqr 的 40° 扰动 1 项失败（原有，40° 模型可信度低）。
- 板子（`/dev/cu.usbmodem21133401`）上烧的是 0.40 同源代码，0.41 与其源码一致，未再重烧。

## 位置保持（LQR）现状
- 控制律：`user/pid/lqr.c`，`BAL_USE_LQR=1`（balance.c）；0 切回原串级 PID。LQR 模式下 `bal_*`/`pos_*` 滑块不生效。
- 增益：`LQR_KX -5 / KV -2.25 / KP 15 / KD 0.4`（lqr.h），单位已按真实 cm（编码器两轮都四倍频）。
- 回位曲线：期望回位速度 = `sqrt(2·lqr_amax·偏差)`，限 `LQR_VMAX`=20 cm/s；`lqr_amax` 默认 8，App 可在线调，不用重烧。
- 位置积分：仅在 0.27~6 cm 偏差、速度 < 4 cm/s 时累积；其余按指数衰减（0.3 s，锚点附近 0.5 s），不清零。
- 速度项渐隐：|v| ≤ 2 cm/s 不参与，≥ 5 cm/s 完全参与（`vel_gain`）。
- 参数表 `PARAM_DATA_VERSION`=35，车端 24 项，App `PARAM_VER`=35，改参数必须两边同步（`scripts/check_param_sync.py`）。
- 推到车速 60 cm/s 以上会摔（位置环倾角顶到 4.9° 上限），属推力超出可恢复范围。

## 工具
- 脱机 flash 记录：车离开平衡态时把最近 40 s 写进 `storage` 分区，断电不丢。读取：`python3 scripts/read_flash_log.py /tmp/flog`（约 26 s，读完板子复位）。
  下一次离开平衡态会覆盖旧记录，所以先读再测下一轮。
- 在线抓包：`python3 scripts/capture_trace.py <秒> <前缀> <串口>`。
- 真车数据：`logs/data/*.csv`（`lqr_amax8/amax30/integral_band2/band6_vmax20` 等推车记录）。

## 已否决（不要重试）
- 用辨识模型/仿真整定增益：模型整定失败多次，仿真与真车不符，别再靠仿真调增益。
- 超速增益刹车（仿真 G≥0.25 失稳）、叠加速度环（等于再加一份滞后速度反馈）。
- 加大 `LQR_KV` 调过冲（≥8 失稳，KV=5 出现 4 Hz 极限环）；位置偏差限幅 `ERR_SAT`（回位变慢、停在 1~5 cm 处）。
- 内置 `Task` 子代理不可用（Cursor 侧 bug），外部 CLI 402 欠费；不要往委派方向排障。

## 待办
1. `releases/` 里还没有 0.40/0.41 的发布包和发布说明（用户未要求，要打包再说）。
2. App 需要重新构建打包（`paramDef.js` 已对齐 PARAM_VER 35），否则“恢复默认”会下发旧参数。
3. 遥控速度环与 `BAL_POS_BLEND_RPM` 因轮速放大 2.7 倍尚未重调。
4. 静止时若再出现晃动：先查电机死区/静摩擦（静止时 out 仅 ±6%，死区实测约 2.2%，参数 `deadzone` 默认 4%），不要再动位置环。

## 关键约定
- `scripts/run_tests.sh` 与 `scripts/test_hooks.py` 改完必须跑。
- 版本号只在用户明确要求时递增；本地只保留最新两版（0.41、0.40），更早的在 git 历史和 tag（v0.30/v0.31/v0.32）里。
- 本地 `logs/` 只保留最近 3 天，更早的已在 git 历史里。
- 不要读自己旧会话的 transcript；跨对话结论只写本文件。
- 上下文护栏：`.cursor/hooks/context_budget.py`、`limit_verbose_output.py`；监控 `python3 scripts/ctx_check.py --daily`。
