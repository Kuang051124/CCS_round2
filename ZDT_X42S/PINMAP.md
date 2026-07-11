# MSPM0G3507 引脚占用清单 (LQFP-64)

> 本文件记录 Target 工程当前所有外设引脚分配，供移植/排错参考。
> 生成日期: 2026-07-04

## UART 串口

| 模块 | 变量名 | 硬件 | TX | RX | 波特率 | 说明 |
|---|---|---|---|---|---|---|
| UART_WIT | `UART_WIT_INST` | UART0 (Main) | - | **PA11** | 115200 | WIT 传感器 (仅 RX, DMA) |
| K230 | `K230_INST` | UART2 (Main) | **PB17** | **PB18** | 115200 | K230 视觉通信 |
| **UART_ZDT** | `UART_ZDT_INST` | **UART3 (Extend)** | **PA26** | **PA13** | 115200 | **电机 1** (ZDT_X42S) |
| **UART_ZDT2** | `UART_ZDT2_INST` | **UART1 (Main)** | **PB4** | **PB5** | 115200 | **电机 2** (ZDT_X42S) |

> ⚠️ UART3 是 **Extend** 类型，初始化必须用 `DL_UART_Extend_*` API；
>   UART0/1/2 是 **Main** 类型，用 `DL_UART_Main_*` API。

## I2C

| 模块 | 变量名 | SDA | SCL | 频率 |
|---|---|---|---|---|
| I2C_OLED | `I2C_OLED_INST` | **PB3** | **PB2** | 400kHz |

## PWM / 定时器

| 模块 | 变量名 | 硬件 | 通道 | 引脚 |
|---|---|---|---|---|
| PWM_MOTOR | `PWM_MOTOR_INST` | TIMA0 | CCP0~3 | PA21, PA22, PA15, PA25 |
| ~~SM_PUL_VER~~ | ~~SM_PUL_VER_INST~~ | ~~TIMG6~~ | — | ~~PA29~~ **已删除** |
| ~~SM_PUL_HOR~~ | ~~SM_PUL_HOR_INST~~ | ~~TIMA1~~ | — | ~~PA28~~ **已删除** |
| MOTOR_TIM_INST | `MOTOR_TIM_INST_INST` | TIMG0 | — | — | 10ms 周期定时 |

## GPIO - 电机驱动 (MOTOR)

| 名称 | 引脚 | 方向 |
|---|---|---|
| FL_IN1 | PA27 | OUT |
| FL_IN2 | PB27 | OUT |
| FR_IN1 | PB25 | OUT |
| FR_IN2 | PA24 | OUT |
| BL_IN1 | PB24 | OUT |
| BL_IN2 | PB23 | OUT |
| BR_IN1 | PB22 | OUT |
| BR_IN2 | PB20 | OUT |

## GPIO - 其他

| 名称 | 引脚 | 方向 | 说明 |
|---|---|---|---|
| LED RED | PB14 | OUT | 指示灯 |
| LASER PURPLE | PA8 | OUT | 激光 |
| BUZZER BEEP | PA30 | OUT | 蜂鸣器 (初始高) |
| ~~STEP_DIR_VER~~ | ~~PA26~~ | — | ~~已删除~~ → 现为 UART_ZDT TX |
| ~~STEP_DIR_HOR~~ | ~~PB26~~ | — | ~~已删除~~ (空闲) |

## GPIO - 键盘矩阵 (KBD_GRP) 3×3

| 名称 | 引脚 | 方向 | 说明 |
|---|---|---|---|
| R1 | PA17 | OUT | 行1 |
| R2 | PA9 | OUT | 行2 |
| R3 | PB13 | OUT | 行3 |
| C1 | PA16 | IN(PU) | 列1 |
| C2 | PA14 | IN(PU) | 列2 |
| **C3** | **PA12** | IN(PU) | 列3 (原 PA13, 已移让给 UART_ZDT RX) |

## GPIO - 循迹 (TRACE) 5路

| 名称 | 引脚 | 方向 |
|---|---|---|
| LAMP_0 | PB12 | IN(PU) |
| LAMP_1 | PB10 | IN(PU) |
| LAMP_2 | PB8 | IN(PU) |
| LAMP_3 | PB6 | IN(PU) |
| LAMP_4 | PB7 | IN(PU) |

## 调试接口

| 功能 | 引脚 |
|---|---|
| SWCLK | PA20 |
| SWDIO | PA19 |

## 空闲引脚 (可用扩展)

PA0, PA1 (LFXT晶振脚,勿用), PA2, PA12*已用C3*, PA18*, PA23, PA28, PA29,
PB0, PB1, PB11, PB15, PB16, PB19, PB21, PB26, PB31

---

## 本次改动摘要

1. **新增**: UART_ZDT (UART3/Extend, PA26-TX + PA13-RX) → 电机 1
2. **新增**: UART_ZDT2 (UART1/Main, PB4-TX + PB5-RX) → 电机 2
3. **删除**: STEP_DIR_VER (PA26) — 让位给 UART_ZDT TX
4. **删除**: STEP_DIR_HOR (PB26) — 已释放
5. **删除**: SM_PUL_VER (PWM, PA29) — 步进脉冲垂直
6. **删除**: SM_PUL_HOR (PWM, PA28) — 步进脉冲水平
7. **移位**: 键盘 C3: PA13 → PA12 (让 PA13 给 UART_ZDT RX)

> 原 DIR/PUL 步进电机控制已由 ZDT_X42S 串口电机驱动器接管，不再需要脉冲/方向信号。
