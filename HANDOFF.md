# HANDOFF.md

## 当前状态
- **最新版本 `0.45/`**（HBuilderX 打开 `0.45/app`）。0.44 已发布：tag v0.44 + GitHub Release + `releases/esp_balance_rob_v0.44.zip`，不再修改。
- **0.45 = 0.44 + 代码清理**：删掉 LQR 上线后从不执行的串级 PID 分支（`balance_pid.c/h`、`position_pid.c/h`）、删除 6 个 LQR 下不生效的参数（`bal_kp/bal_ki/bal_kd`、`pos_kp/pos_kd/pos_ki`），参数表 24 → 18 项、版本 39；`POS_PID_LEAN_MAX` → `lqr.h` 的 `LQR_LEAN_MAX`；输出限幅从 3 层 clamp 合成 1 层。**控制行为与 0.44 一致**（同一套 LQR 增益、同一套斜坡/死区/前馈）。
- **控制行为（与 0.44 相同）**：
  - 内环 LQR（`lqr_balance_out`，KP 18 / KD 0.5），外环 = 速度环与位置环按目标轮速加权（`BAL_POS_BLEND_RPM` 5），坡度前馈 `SLOPE_FF_K=0.35`（限 ±3°）。
  - 偏航阻尼：陀螺 **Y 轴**（车体竖直轴）→ 差速环 `s_tgt_diff − 0.40·yaw_rate`，死区 10°/s、限幅 ±4 rpm。
  - 横滚闭环：`hardware/roll_compensator`（PD+弱积分，Kp 2.0 / Ki 0.4 / Kd 0.10，`ROLL_COMP_SIGN=-1`，限 ±15°），`servo_set_roll_trim()` 在写硬件时叠加。
  - Roll 零点：`s_data.roll` 存原始值，舵机补偿处扣 `s_roll_zero`；配平采均值直接赋给零点（与原始值同源）。
- 0.44 已发布内容：上述控制律 + App 姿态页 + 状态灯 + LCD Roll 行；参数版本 38（24 项）。


## 仓库文档（2026-11-04）
- 根 `README.md` 已重写为开源介绍（功能 / 硬件引脚表 / 架构 / 快速开始 / 协议 / 工具 / 版本），新增 `LICENSE`（MIT）。
- 遗留：`0.45/main_car/README.md` 标题仍是旧版号。

## 密钥安全（2026-11-04）
- 仓库 public，设备级 AccessKey 已泄露在 git 历史。**用户需去 OneNET 控制台重置 `balancing_dog` / `balancing_dog_sub` 的访问密钥**，重置后旧 key 自动作废（历史里的副本变废串）；主车/副车要重烧固件，App 连接页重填。
- 固件密钥位置：`main_car|sub_car/user/onenet_client/onenet_secret.h`（不入库），模板 `onenet_secret.example.h`。`onenet_client.h` 只留产品 ID 和设备名。
- App 默认值：`0.45/app/common/config.js` 的三元组留空，用户在连接页填一次，存 storage。
- 新文档：`0.45/docs/onenet_mqtt_guide.md`（建产品到 MQTT 报文全流程）；旧的 `app/onenet_物模型*.md` 已删。

## 待办
1. **烧录真机验证 0.45**（行为同 0.44）：单边桥抬腿回位、上坡能否爬、下坡刹不刹得住、单腿过桥是否还走歪。
2. App 重新打包验证姿态页与参数版本 39（建议点"恢复默认"，旧参数表会被丢弃重填）。
3. 横滚闭环待验证：平地起立后侧向轻推，看车身是否**回正**（`ROLL_COMP_SIGN=-1`、Kp 2.0 已按实测灵敏度 0.14 °/° 放大）。若越推越歪说明符号或增益不对，先降 `ROLL_KP` 到 1.0 再试。
4. 偏航阻尼待验证：上坡/地面走直线，看车头被带偏后是否自动回正（`YAW_DAMP_K=0.40`，符号反了改 `GYRO_YAW_SIGN=-1`）。
4. 脱机测试：flash 记录已含 `tavg/tdiff/diff/roll`（72 字节/条），按急停或摔落后插线 `python3.11 scripts/read_flash_log.py /tmp/flog` 读出；待分析：转弯后位置不稳 / 向一侧走 / 摔倒。
5. 遥控速度环与 `BAL_POS_BLEND_RPM` 因轮速放大 2.7 倍尚未重调。
6. 静止时若再出现晃动：先查电机死区/静摩擦（静止 out 仅 ±6%，死区实测约 2.2%，参数 `deadzone` 默认 4%），不要再动位置环。

## 位置保持（LQR）现状
- 控制律：`user/pid/lqr.c`（唯一实现，串级 PID 分支已删）。位置项 `lqr_pos_lean` + `lqr_pos_integral`，倾角/角速度项 `lqr_balance_out`。
- 增益：`LQR_KX -5 / KV -2.25 / KP 18 / KD 0.5 / VMAX 30`（lqr.h），单位按真实 cm（编码器两轮都四倍频）。
- 回位曲线：期望回位速度 = `sqrt(2·lqr_amax·偏差)`，限 `LQR_VMAX`=30 cm/s；`lqr_amax` 默认 8，App 在线可调。
- 位置积分：0.27~8 cm 偏差、速度 < 4 cm/s（或正在远离锚点）时累积；其余指数衰减（0.3 s，锚点附近 0.5 s），不清零。积分上限 12°。
- 速度项渐隐：|v| ≤ 2 cm/s 不参与，≥ 5 cm/s 完全参与（`vel_gain`）。
- 参数表 `PARAM_DATA_VERSION`=39，车端 18 项，App `PARAM_VER`=39，改参数必须两边同步（`scripts/check_param_sync.py`）。
- 倾角限幅：位置环 12°（`LQR_LEAN_MAX`）、速度环 10°（`SPD_PID_WALK_PITCH_MAX`）、平衡环 motor_max 95% / ang_limit 35°。起立软启动 30%/800ms。

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
- 版本号只在用户明确要求时递增；本地只保留最新两版（0.45、0.44），更早的在 git 历史和 tag 里。
- 本地 `logs/` 只保留最近 3 天，更早的已在 git 历史里。
- `scripts/run_tests.sh` 与 `scripts/test_hooks.py` 改完必须跑。
- 不要读自己旧会话的 transcript；跨对话结论只写本文件。
- 上下文护栏：`.cursor/hooks/context_budget.py`、`limit_verbose_output.py`；监控 `python3 scripts/ctx_check.py --daily`。
- 固件版本号来自 `main_car/version.txt`（ESP-IDF 在 CMake 配置阶段读取），改完必须 `pio run -t fullclean` 再编译，否则固件里还是旧版本字符串。
