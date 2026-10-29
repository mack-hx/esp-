# HANDOFF.md

## 当前状态
- 最新版本工程：`0.32/`（0.31 为历史版本，**不得修改**）。
- 本轮工作：**代码整洁度清理**（注释精简 + 文件头去日期戳），上一轮已删死代码与零调用 API。
- 编译状态：`pio run` 通过，**零 warning 零 error**（RAM 26.6% / 87052 B，Flash 28.4% / 1041357 B）。
- 未烧录验证：本机没接开发板（`/dev/cu.usbmodem*` 不存在），需用户接板后自行烧录。

## 注释规则（最新，用户拍板，后续照此执行）
- **只写三类内容**：① 变量含义 ② 接口契约 ③ 技术事实。
- **不要写**：理由/推导、改动历史（"xx 日修改补充"、"原来…改成…"）、日期戳、版本标记、长篇大论。
- 文件头**不再写 `@date`**（`@file` / `@brief` / `@author` 保留，`@param` / `@return` 永远保留）。
- 代码与注释都以**简洁**为先。
- 已落地到 `.cursor/rules/project-workflow.mdc` 第 10 条。

## 本轮（第三轮）改了什么
- **删掉全部 67 个文件头的 `@date` 行**（`@date 2026-10-xx`），文件头只留 `@file/@brief/@author`。
- **全工程注释再精简一轮**：只保留变量含义、接口契约、技术事实；删掉理由推导、长说明、大段 ASCII 图。
  - 头文件重点：`user/pid/*.h`、`user/control/balance.h`、`user/trim/trim.h`、`user/param/param.h`、
    `hardware/{servo,motor_driver,hc595,magnet,mpu9250,lcd,wifi_sta,tft_display}/*.h`。
  - 源文件重点：`user/control/balance.c`、`user/onenet_client/onenet_client.c`、
    `hardware/mpu9250/mpu9250.c`、`hardware/pcf8574/pcf8574.c`、`hardware/motor_driver/motor_driver.c`、
    `hardware/servo/servo.c`、`hardware/wifi_sta/wifi_sta.c`、`user/pid/*.c`、`main/main.c`、`user/ui/*.c`。
- **顺手修正一处注释与代码不符**：`magnet.h` 写"电磁铁1 = Q5"，实际代码是 `Q2`（电磁铁2 = Q1），已改正。

## 前两轮改了什么（只删无用代码，不改控制逻辑）
- **删掉 3 个零引用硬件组件目录**：`hardware/voice_module`、`hardware/i2c_scanner`、`hardware/k230_uart`
  （根 `CMakeLists.txt` 的 `EXTRA_COMPONENT_DIRS` 同步去掉）。
- **删掉 29 个零调用公共函数**（定义+声明）：`param_to_json`、`param_count`、`param_get_item`、
  `param_stale_write`、`trim_del_point`、`trim_erase_nvs`、`balance_get_arm_ms`、`balance_get_move`、
  `balance_reload_params`、`mqtt_client_post_property`、`encoder_get_and_clear`、`encoder_clear`、
  `encoder_get_revolutions`、`hc595_get_state`、`lcd_ui_get_current_page`、`led_deinit`、
  `led_color_to_brightness`、`motor_get_deadband`、`motor_set_direction`、`mpu9250_read_accel`、
  `mpu9250_read_mag`、`mpu9250_read_temperature`、`pcf8574_is_pressed`、`pcf8574_register_callback`,
  `pcf8574_enable_interrupt`、`tft_draw_pixel`、`tft_draw_string`、`wifi_sta_is_connected`、
  `wifi_sta_reprovision`；连带删 2 个孤儿：`wifi_sta.c:clear_credentials_in_nvs()`、
  `pcf8574.c:pcf8574_isr_handler()`。
- **删掉 26 个零引用宏**（`LED_BRIGHTNESS_*`、`SERVO_PERIOD_MS`、`LOOP_TICK_MS` 等）。
- **删掉 3 个只写不读字段**：`bal_data_t.speed_raw` / `.target_speed` / `.spd_out`。
- 重复 include 各删一份；`BAL_MOVE_TURN_BASE` 改为接上（`onenet_client.c` 里 1.0f/2.0f 换成宏，数值等价）。

## 待办 / 注意
- **需要用户烧录验证**：动过 `balance.c` 控制任务与 `onenet_client.c` 指令解码，建议接板跑一次起立 + 方向键。
- `app/unpackage/dist/` 是构建产物，未同步注释改动，重新打包会覆盖。
- 本轮备份：`/tmp/bak032d`（0.32 全量副本，除 `.pio`/`build`）；上一轮：`/tmp/bak032c`。
- 本轮有**另一个 Cursor 会话可能同时改同一批文件**，动文件前先确认当前内容。
- 详细过程见 `logs/2026-10-29.md`。

## 等价性核对结论
去注释后逐文件 diff（Python 剥注释比对，70 个文件）：**代码差异 0**，本轮只动注释。
前两轮的唯一"改写"是 `1.0f/2.0f` → `BAL_MOVE_TURN_BASE`（数值等价），控制逻辑零变化。
