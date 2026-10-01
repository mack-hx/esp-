# ESP32-S3 引脚配置说明

## 主控芯片
- **型号**: ESP32-S3R8N8
- **PSRAM**: 8MB Octal SPI
- **Flash**: 8MB

---

## 引脚分配表

### I2C 总线 1 (MPU9250)
| GPIO | 功能 | 连接设备 | 说明 |
|------|------|----------|------|
| GPIO1 | SCL1 | MPU9250 | I2C 时钟线 |
| GPIO2 | SDA1 | MPU9250 | I2C 数据线 |
| GPIO3 | INT | MPU9250 | 中断引脚 |

### SPI 移位寄存器 (74HC595D)
| GPIO | 功能 | 连接设备 | 说明 |
|------|------|----------|------|
| GPIO4 | SCLK | 74HC595D | 串行时钟 (Shift Clock) |
| GPIO5 | RCLK | 74HC595D | 锁存时钟 (Latch Clock) |
| GPIO6 | DS | 74HC595D | 串行数据输入 (Data Serial) |

### I2C 总线 2 (RGB TFT 屏幕)
| GPIO | 功能 | 连接设备 | 说明 |
|------|------|----------|------|
| GPIO7 | SCL2 | 128x160 RGB TFT | I2C 时钟线 |
| GPIO8 | SDA2 | 128x160 RGB TFT | I2C 数据线 |
| ~~GPIO9~~ | ~~RST~~ | ~~128x160 RGB TFT~~ | **已释放, RST 短接 VCC** |
| GPIO10 | DC | 128x160 RGB TFT | 数据/命令选择 |
| GPIO11 | CS | 128x160 RGB TFT | 片选信号 |
| GPIO12 | BLK | 128x160 RGB TFT | 背光控制 |

### SK6812 RGB LED (RMT 直驱)
| GPIO | 功能 | 连接设备 | 说明 |
|------|------|----------|------|
| **GPIO9** | **DIN** | **SK6812 LED** | **由 ESP32 RMT 外设驱动** (原 TFT RST 改用) |

### PWM 控制 (舵机)
| GPIO | 功能 | 连接设备 | 说明 |
|------|------|----------|------|
| GPIO13 | PWM1 | 舵机1 | PWM 控制信号 |
| GPIO14 | PWM2 | 舵机2 | PWM 控制信号 |

### PWM 控制 (TB6612 电机驱动)
| GPIO | 功能 | 连接设备 | 说明 |
|------|------|----------|------|
| GPIO15 | PWMA | TB6612 | 电机 A PWM 控制 |
| GPIO16 | PWMB | TB6612 | 电机 B PWM 控制 |

### I2C 总线 3 (PCF8574 IO 扩展)
| GPIO | 功能 | 连接设备 | 说明 |
|------|------|----------|------|
| GPIO17 | SCL3 | PCF8574 | I2C 时钟线 |
| GPIO18 | SDA3 | PCF8574 | I2C 数据线 |
| GPIO21 | INT2 | PCF8574 | 中断引脚 |

### UART1 (K230 通信)
| GPIO | 功能 | 连接设备 | 说明 |
|------|------|----------|------|
| GPIO38 | TXD1 | K230 | UART 发送 |
| GPIO47 | RXD1 | K230 | UART 接收 |

### UART2 (CI-03T 语音模块)
| GPIO | 功能 | 连接设备 | 说明 |
|------|------|----------|------|
| GPIO39 | RXD2 | CI-03T | UART 接收 |
| GPIO40 | TXD2 | CI-03T | UART 发送 |

### PCNT (脉冲计数器 - 编码器)
| GPIO | 功能 | 连接设备 | 说明 |
|------|------|----------|------|
| GPIO41 | B- | 编码器1 | B 相负 |
| GPIO42 | B+ | 编码器1 | B 相正 |
| GPIO45 | A- | 编码器2 | A 相负 |
| GPIO46 | A+ | 编码器2 | A 相正 |

---

## 74HC595D 输出引脚分配

74HC595D 是一个 8 位串行输入并行输出移位寄存器,通过 GPIO4/5/6 控制。

| 输出引脚 | 连接设备 | 功能说明 |
|----------|----------|----------|
| Q0 | ~~SK6812~~ (已断开) | **悬空, 保留为通用输出** |
| Q1 | TB6612 AIN1 | 电机 A 方向控制 1 |
| Q2 | TB6612 AIN2 | 电机 A 方向控制 2 |
| Q3 | TB6612 BIN1 | 电机 B 方向控制 1 |
| Q4 | TB6612 BIN2 | 电机 B 方向控制 2 |
| Q5 | 电磁铁1 IO1 | 电磁铁1 控制 |
| Q6 | 电磁铁2 IO2 | 电磁铁2 控制 |
| Q7 | (未使用) | 保留 |

---

## PCF8574 输出引脚分配

PCF8574 是一个 8 位 I2C IO 扩展芯片,通过 I2C3 (GPIO17/18) 控制。

| 输出引脚 | 连接设备 | 功能说明 |
|----------|----------|----------|
| P0 | K1 | 按键1 输入 |
| P1 | K2 | 按键2 输入 |
| P2 | K3 | 按键3 输入 |
| P3 | K4 | 按键4 输入 |
| P4 | K6 | 微动开关2 输入 |
| P5 | K5 | 微动开关1 输入 |
| P6 | (未使用) | 保留 |
| P7 | (未使用) | 保留 |

---

## 外设通信接口汇总

| 外设 | 通信方式 | 引脚 |
|------|----------|------|
| MPU9250 | I2C | GPIO1(SCL), GPIO2(SDA), GPIO3(INT) |
| 74HC595D | SPI(软件) | GPIO4(SCLK), GPIO5(RCLK), GPIO6(DS) |
| RGB TFT 屏 | SPI | GPIO7(SCL), GPIO8(SDA), GPIO10(DC), GPIO11(CS), GPIO12(BLK) |
| SK6812 LED | RMT | GPIO9(DIN) |
| 舵机1 | PWM | GPIO13 |
| 舵机2 | PWM | GPIO14 |
| TB6612 | PWM | GPIO15(PWMA), GPIO16(PWMB) |
| PCF8574 | I2C | GPIO17(SCL), GPIO18(SDA), GPIO21(INT) |
| K230 | UART | GPIO38(TX), GPIO47(RX) |
| CI-03T | UART | GPIO39(RX), GPIO40(TX) |
| 编码器 | PCNT | GPIO41/42(编码器1), GPIO45/46(编码器2) |

---

## 硬件变更记录

### 2026-10-01: SK6812 RGB 真实调光改造

**背景**：
SK6812 DIN 原接 74HC595 Q0。由于 74HC595 在 RCLK 锁存后 Q0 保持稳态，
物理上无法在 1.25µs 内产生 SK6812 归零码所需的多次高低翻转，SK6812
仅能跟随 Q0 开关而无法调色调光。

**硬件改动**：
1. **断开**：74HC595 Q0 到 SK6812 DIN 的连线（原 PCB 飞线或杜邦线）
2. **改接**：SK6812 DIN 改接至 ESP32 GPIO9（原 TFT RST 引脚）
3. **TFT RST 处理**：TFT 模块 RST 引脚短接 VCC（依赖模块内部 POR 上电自复位）

**GPIO 变化**：
- GPIO9: TFT RST → **SK6812 DIN (RMT 直驱)**
- TFT RST: GPIO9 控制 → **悬空，依赖 POR**

**软件改动**：
- `hardware/led/` 重写为 RMT 驱动
- `hardware/tft_display/` 移除 RST 相关代码
- `hardware/hc595/` 移除 `hc595_set_led()` 接口

---

## 注意事项

1. **I2C 总线**: 本项目使用了 3 组 I2C 总线,需要分别初始化
   - I2C1: MPU9250 惯性测量单元
   - I2C2: RGB TFT 显示屏
   - I2C3: PCF8574 IO 扩展芯片

2. **PWM 通道**: ESP32-S3 有 8 个 LEDC 通道,本项目使用:
   - 通道 0-1: 两个舵机
   - 通道 2-3: TB6612 电机驱动
   - 通道 4: TFT 背光

3. **RMT 通道**: ESP32-S3 有 4 个 RMT TX 通道,本项目使用:
   - 通道 0: SK6812 LED 驱动

4. **74HC595D 级联**: 如需扩展更多输出,可通过 Q7S 引脚级联多个 74HC595D

5. **编码器接口**: 使用 ESP32-S3 的 PCNT (脉冲计数器) 外设,支持正交编码器

6. **电源管理**: 舵机和电机驱动需要外部供电,注意共地

7. **引脚复用**: 部分引脚可能与 PSRAM/Flash 冲突,请参考 ESP32-S3 数据手册

---

## 参考资料
- [ESP32-S3 技术规格书](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_cn.pdf)
- [ESP32-S3 技术参考手册](https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_cn.pdf)
- 74HC595D 数据手册
- PCF8574 数据手册
- TB6612FNG 电机驱动数据手册
- SK6812 数据手册