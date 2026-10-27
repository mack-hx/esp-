# 工程状态交接 (HANDOFF)

> **这个文件的作用**: 换新对话时，助手读这一份就能接上之前的工作，不用从头复述。
> **维护方式**: 每次对话结束前（或每完成一个里程碑），助手更新本文件。
> **写法要求**: 只写"结论"，不写过程；控制在 100 行以内；旧内容该删就删。

---

## 1. 当前在做什么

**状态**：0.31 已完整入库 Git（135 文件）。今天修了"一直往前跑"和"高频抖动"
两个控制环问题，**待真机验证**。位置环尚未接入（缺轮径参数）。

**当前版本目录**：`0.31/`（版本号 `0.31.0`，见 `0.31/version.txt`）
**上一版本**：`0.30/`（已发布到 GitHub，勿修改）

---

## 2. 0.31 相比 0.30 改了什么

| 改动 | 位置 | 状态 |
|---|---|---|
| 编码器减速比 `gear` 1.0 → 30（编码器在电机轴上，减速箱 30:1） | `user/param/param.c` | 完成 |
| 参数表 v14，共 17 项（新增 `move_rpm_max`） | `user/param/param.c` | 完成 |
| 速度死区 3 → 5 rpm 改回 1.0（按 gear=30 重算） | `user/param/param.c` | 完成 |
| 磁力计 AK8963 bypass 探测（找不到只告警不报错） | `hardware/mpu9250/mpu9250.c` | 完成 |
| 配平标定入口：4 个云端服务 + App 页面 | `user/onenet_client/onenet_client.c`、`app/pages/trim/trim.vue` | 完成 |
| 陀螺零偏标定修复（此前因恒差 3° 永远失败，导致起立剧烈抖动） | `hardware/mpu9250/mpu9250.c` | 完成 |
| App 方向键改走速度环（平衡环全程不撒手） | `app/utils/control.js` | 完成 |
| 配平零点断电保存 / 配平表不再每次上电被擦除 | `user/param/param.c` | 完成 |
| **控制环 PID 拆成独立组件** `user/pid/`（balance/speed/steer/position 四个环各一个 .c，通用 `pid.c` 一并搬过去） | `user/pid/`、`user/control/balance.c` | 完成，`balance.c` 1313 → 1256 行 |
| 位置环只建文件 + 接口占位，**默认不接入控制流程**（工程里没有里程计，编码器只能给瞬时转速） | `user/pid/position_pid.c` | 完成，当前车行为不变 |

**10 个云端服务**（车端 `strcmp` 分支 与 App `invokeService()` 名字完全一致）：
`stop` / `stand` / `calibrate` / `param_get` / `param_set` / `param_save` / `trim_record` / `trim_save` / `trim_clear` / `trim_get`

---

## 3. 验证状态（已跑通，别重复跑）

| 项目 | 结果 |
|---|---|
| `pio run` | SUCCESS，RAM 16.3% (53308/327680)，Flash 28.2% (1034457/3670016) |
| `test/test_param_pc.c` | 110 / 110 通过 |
| `test/test_pid_pc.c` | 21 / 21 通过 |
| `test/test_trim_pc.c` | 67 / 67 通过 |
| `test/bal_sim.py`（定轴） | RMS 13.3%，末段振幅 3.38° |
| `test/bal_sim.py --walk` | 目标 20/40/60/80 rpm 都不摔，倾角波动 3.14~3.56° |
| `node --check app/utils/control.js` | 通过 |
| **`test/test_speed_pid_pc.c`（新建）** | **14 / 14 通过**（死区软化 + 积分衰减） |
| `test/test_pid_pc.c` | **29 / 29 通过**（新增 8 条积分衰减断言） |
| `app/pages.json` JSON.parse | 通过 |
| 车端 vs App 参数表逐项比对 | 17 项全一致 |

---

## 4. 待办 / 待用户确认

### 🔴 最优先：真机验证今天修的两个问题
用户连提 4 次的问题，今天已定位根因并改完代码，**但还没上真机**。

**症状与对应修复**：

| 症状 | 根因 | 修复 | 位置 |
|---|---|---|---|
| 会一直往前跑 | 速度环死区在 PID 前归零 → 积分**冻结** → 残留积分永久非零 → 目标倾角恒定非零 | 新增 `pid_decay_integral()`，死区内积分按 `exp(-dt/3s)` 衰减到 0 | `user/pid/pid.{c,h}`、`user/pid/speed_pid.c` |
| 一直在高频抖动 | 死区**硬归零** → 实测轮速跨量化台阶 (0.90↔1.05 rpm) 时目标倾角每 300ms 跳一次 → 平衡环反复纠偏 | 死区加线性过渡带 `SPD_PID_ERR_SOFT_BAND = 0.5 rpm`，误差连续 | `user/pid/speed_pid.{c,h}` |

**验证状态**：固件编译 SUCCESS；离线测试 pid 29/29、speed_pid 14/14、
param 110/110、trim 67/67，全绿无回归。

**需要用户做的**：
- [ ] 烧录 0.31（`pio run --target upload`）
- [ ] **重新配平**（旧配平表作废）
- [ ] 定轴静置 30 秒，看**还往不往前跑**（修复 1 的直接判据）
- [ ] 静置时听电机声音，看**还有没有高频嗡嗡声**（修复 2 的直接判据）
- [ ] App 方向键走一段再松手，看停不停止（验证积分能退干净）

**如果还是往前跑**，可能的追加方向（按优先级）：
1. `SPD_PID_I_DECAY_TAU` 调小（3.0 → 1.5）
2. 死区 `SPD_PID_ERR_DEADBAND` 偏小，量化噪声进了积分 → 调大到 2.0
3. 编码器方向符号 `BAL_ENC_FWD_SIGN`（balance.h）反了 → 车速反馈符号错

**位置环：尚未接入**（用户明确要求过）
- 需要里程计 = **轮径参数** + 编码器位移积分 + 编码器零点复位接口
- 参数表里目前只有 `gear=30`，**没有轮径参数** → 需要问用户轮子实际直径
- 接入点在 `balance.c:657` `speed_pid_update()` 调用**之前**，
  位置环输出 (rpm) 加到 `s_tgt_avg`
- 里程累计要挂在 `balance.c:340-341` 的 `d1/d2` 窗口增量处（100ms 一次），
  不要挂在 rpm 换算后面（那样 300ms 才动一次）
- `user/pid/position_pid.c` 接口占位已就绪（`position_pid_update(dt, err)`）

**子代理怎么派（2026-10-27 起）**：
Cursor 内置 `Task` 报 `message_id` 错误时，用外部脚本（走本机 claude CLI）：
```bash
cat > /tmp/q.txt <<'EOF'
工程根: /Volumes/Moving/work/01_code/03_Personal/ESP32/esp_balance_robot
目标文件: 0.31/<具体路径>
要找什么: [明确的问题]
返回格式: 只回结论 + 文件:行号, 40 行以内, 不贴大段源码
EOF
scripts/subagent.sh /tmp/q.txt <任务名>
```
报告落在 `logs/subagents/<任务名>-<时间戳>.md`，主对话只回 20 行摘要。
⚠ 依赖 claude CLI 额度，今天下午遇到过 402 余额不足。

**`.cursor/hooks/` 装了三个强制钩子**（真实环境已验证生效）：
- `require_subagent.py`：自己查文件满 6 次就拦截，强制先派子代理
- `guard_reads.py`：拦重复读同一位置、拦大文件整读（>800 行）
- `limit_verbose_output.py`：单轮正文超 12000 字符时提醒精简
维护类命令（git/python/编译/写文件）不计数，不会被误拦。
紧急开关（子代理不可用时 hook 会把自己锁死）：
```bash
touch /tmp/esp_balance_robot_ctx_guard_off   # 全部放行
rm   /tmp/esp_balance_robot_ctx_guard_off   # 恢复正常拦截
```
⚠ **在独立的终端窗口执行**，不要在被拦的那次调用里执行 —— hook 已经
决定 deny，命令根本跑不到（2026-10-27 踩过）。

### 需要用户动手的（必须等硬件）
- [ ] 烧录 0.31 后**必须重新配平**（旧配平表 + 旧配平零点作废）
- [ ] 旧参数表也作废（会回到 `gear=30` / `move_rpm_max=100` / 速度环 0.06~0.01 的出厂值）
- [ ] 真机走一圈验证：定轴静止、推它不倒、App 方向键走路
- [ ] **轮子直径实际值是多少**（加位置环做里程计要用）—— 这是位置环的唯一硬阻塞

### 助手可以自己做的
- [x] `0.31/` 整个目录已进 Git（135 文件，提交 `700d122`）
- [ ] `0.30/` 的 18 个文件改动也还没提交
- [ ] 0.31 发布到 GitHub（tag `v0.31` + release + 源码包）
- [ ] 位置环接入（**等用户给轮径**）


### 已知的文档脱节（0.30 遗留，0.31 已修好，0.30 待修）
- `0.30/README.md` 断言数写的 89（实为 110）
- `0.30/test/README.md` 第 15 行写 `cd 0.30`（0.31 里已是 `cd 0.31`）
- `0.30/app/onenet_物模型.md` 写"其它 5 个服务都有 ok"（现共 10 个服务）

---

## 5. 关键约定（别再踩）

- **配平**：`trim_record` 存表 → `trim_save` 存 NVS → `trim_get` 读回
- **参数实时生效**：改参数不需要重启车
- **速度环**：`spd_kp` 建议 0.06~0.1；`move_rpm_max` = 100 约为最高转速的 30%，再往上目标倾角会顶到 4° 安全阀，表现为一直倾着追速度、左右猛摇
- **OneNet 物模型后台配置**：4 个配平服务需要在 [产品页面](https://open.iot.10086.cn/console/product/6YlUqr9lue/model) 配置，状态表见 `0.31/docs/物模型待补清单.md`
- **产品 ID** `6YlUqr9lue`，设备名 `balancing_dog`
- **`user/pid/` 各环接口统一**：`init` / `load_params` / `reset` / `update`；增益从参数表实时读；每周期调 `load_params()`（用 `pid_set_gains`，不清积分）
- **`balance.c` 现在只管**：传感器采样 / 状态机 / 摔倒检测 / 电机输出 / 对外 API
- **位置环接口已留但未接入** —— 等里程计 (轮径参数) 做好直接接

---

## 6. 上下文自测

对话较长时先跑：

```bash
python3 scripts/ctx_check.py          # 当前对话占用，判定红/黄/绿灯
python3 scripts/ctx_check.py --all    # 最近 10 个会话的历史规律
```

**实测结论**：触发"输入内容过长"的元凶是**助手自己的输出文字**（占上下文 75%~99%），
不是读进来的大文件（工具 IO 只占 1%~30%）。所以：

- 每轮输出控制在 2~3 段以内，推演过程不要全部留在主对话里
- 大范围比对/通读 → 派子代理，只带结论回来
- 黄灯时把结论写进 `logs/` 和本文件；红灯时提醒用户新开对话

规则全文见 `.cursor/rules/project-workflow.mdc` 第 14 条。
