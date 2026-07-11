# 编码器测速 & PID 速度控制说明

## 一、文件结构

```
ENCODER/
├── encoder.h          # 编码器计数 + 测速接口
├── encoder.c          # 编码器中断 + 测速实现
├── speed_control.h    # PID 速度控制接口
├── speed_control.c    # PID 速度控制实现
└── README.md          # 本文件
```

---

## 二、encoder.c — 编码器计数 & 测速

### 变量

| 变量 | 类型 | 说明 |
|------|------|------|
| `g_leftEncoderCount` | `volatile int32_t` | 左轮编码器**累计脉冲数**。ISR 中实时更新，正转++，反转-- |
| `g_rightEncoderCount` | `volatile int32_t` | 右轮编码器累计脉冲数 |
| `g_lastLeftCount` | `static int32_t` | 上一次测速采样时的左轮计数值。用于计算差值 |
| `g_lastRightCount` | `static int32_t` | 同上，右轮 |
| `g_leftSpeed` | `static int32_t` | 左轮**当前速度**（counts/s）。每次测速后更新 |
| `g_rightSpeed` | `static int32_t` | 右轮当前速度 |
| `g_lastSampleTick` | `static uint32_t` | 上一次测速采样的 `tick_ms` 时间戳 |

### 函数

#### `ENCODER_Init()`
- 清零所有计数器和速度变量
- 使能 GPIOA、GPIOB 的 NVIC 中断（编码器引脚的中断入口）

#### `ENCODER_GetLeftCount()` / `ENCODER_GetRightCount()`
- 返回编码器**累计脉冲数**，用于调试显示

#### `ENCODER_ResetLeft()` / `ENCODER_ResetRight()`
- 清零累计脉冲数

#### `ENCODER_SpeedSample()`
- **周期性调用**（内部用 `tick_ms` 计时，每 10ms 自动采样一次）
- 计算公式：

```
speed = (当前计数值 - 上次计数值) × 1000 / 间隔时间(ms)
      = delta × 1000 / 10
      = 单位: counts/s
```

#### `ENCODER_GetLeftSpeed()` / `ENCODER_GetRightSpeed()`
- 返回最近一次采样的**速度值**（counts/s）

#### `GROUP1_IRQHandler()`
- GPIO GROUP1 中断服务函数
- **2 倍频**正交解码：A 相上升沿 + B 相下降沿各触发一次计数

```
A 相上升沿 → 读 B 相电平:
  B = HIGH → 正转, count++
  B = LOW  → 反转, count--

B 相下降沿 → 读 A 相电平:
  A = HIGH → 正转, count++
  A = LOW  → 反转, count--
```

---

## 三、speed_control.c — PID 速度控制

### 变量

| 变量 | 类型 | 说明 |
|------|------|------|
| `g_targetSpeed` | `static int32_t` | **目标速度**（counts/s），由 `SPEED_SetTarget()` 设置 |
| `g_leftIntegral` | `static int32_t` | 左轮 PID 的**积分累加值** |
| `g_rightIntegral` | `static int32_t` | 右轮 PID 的积分累加值 |
| `g_leftDuty` | `static int32_t` | 左轮 PWM 占空比（0~400，对应 0%~100%） |
| `g_rightDuty` | `static int32_t` | 右轮 PWM 占空比 |

### 电机参数标定 (MG310P20)

| 参数 | 值 | 说明 |
|------|-----|------|
| 编码器类型 | **13 PPR 霍尔** | 电机自带磁感应编码器 |
| 减速比 | **1:20.409** | 电机轴到输出轴 |
| 2 倍频分辨率 | **530 counts/圈** | 13 × 2 × 20.409（输出轴） |
| 额定速度 | **~173 counts/s** | 400 RPM 电机 → 19.6 RPM 输出 |
| 全速 | **~217 counts/s** | 500 RPM 电机 |
| PWM→速度斜率 | **~0.5 counts/s / PWM** | 满 PWM(400) 对应约 200 counts/s |
| 速度→PWM 斜率 | **~2.0 PWM / counts/s** | 每 1 count/s 误差约需调 2 个 PWM 单位 |

### PID 参数 (基于上述电机标定)

| 参数 | 值 | 说明 |
|------|-----|------|
| `KP` | 5 | 比例系数，实际 Kp = 5/100 = **0.05**。比原来精细 60 倍 |
| `KI` | 1 | 积分系数，实际 Ki = 1/100 = **0.01** |
| `PID_SCALE` | 100 | PID 除数。用 100 替代原来的 10，小误差不会因整数截断而丢失 |
| `MAX_INTEGRAL` | 10000 | 积分限幅（约对应 ±100 PWM 单位的积分调节范围） |
| `MAX_DUTY` | 390 | PWM 上限（97.5%），保护电机 |
| `MIN_DUTY` | 10 | PWM 下限（2.5%），避免堵转 |

### 新旧参数对比

| 场景 | 旧参数 (KP=3.0 ÷10) | 新参数 (KP=0.05 ÷100) |
|------|---------------------|----------------------|
| 误差=200 counts/s | 每次调 **600 PWM** → 立即打穿限幅 | 每次调 **10 PWM** → 平滑接近 |
| 误差=50 counts/s | 每次调 150 PWM | 每次调 **2.5 PWM** |
| 误差=10 counts/s | 每次调 30 PWM | 每次调 **0.5 PWM** → 积分补足 |
| 到达稳态 | 剧烈震荡、无法收敛 | 约 0.5~1 秒平滑收敛 |

### 函数

#### `SPEED_Init(base_duty)`
- 清零积分、设置初始占空比
- `base_duty`：初始 PWM 占空比（建议 100~200，对应 25%~50%）

#### `SPEED_SetTarget(target_speed)`
- 设定目标速度，单位 **counts/s**
- 实际值取决于编码器线数和减速比，需要实测标定

#### `SPEED_Update()`
- **核心函数，周期性调用**（配合 `ENCODER_SpeedSample()`，约 10ms 一次）
- 执行 PID 计算并更新 PWM 硬件寄存器

#### `SPEED_GetLeftDuty()` / `SPEED_GetRightDuty()`
- 返回当前 PWM 占空比值，调试用

---

## 四、PID 控制流程

```
每隔 10ms 执行一次:

┌─────────────────────────────────────────────────┐
│  1. ENCODER_SpeedSample()  测速                 │
│     leftSpeed  = 左轮速度 (counts/s)            │
│     rightSpeed = 右轮速度 (counts/s)            │
└──────────────────┬──────────────────────────────┘
                   ▼
┌─────────────────────────────────────────────────┐
│  2. SPEED_Update()  PID 计算                    │
│                                                 │
│  左轮 PID:                                      │
│    error  = 目标速度 - 左轮实际速度               │
│    integral += error          (累加积分)         │
│    adjust  = (Kp×error + Ki×integral) / 100      │
│    duty    = duty + adjust    (更新占空比)       │
│    duty    = clamp(duty, 10, 390)               │
│                                                 │
│  右轮 PID (跟踪左轮):                            │
│    error  = 左轮实际速度 - 右轮实际速度            │
│    integral += error                            │
│    adjust  = (Kp×error + Ki×integral) / 100      │
│    duty    = duty + adjust                      │
│    duty    = clamp(duty, 10, 390)               │
└──────────────────┬──────────────────────────────┘
                   ▼
┌─────────────────────────────────────────────────┐
│  3. 写入 PWM 硬件                                │
│    DL_TimerA_setCaptureCompareValue(CC0, left)  │
│    DL_TimerA_setCaptureCompareValue(CC1, right) │
└─────────────────────────────────────────────────┘
```

### 走直线策略说明

```
目标速度 ──→ [左轮 PID] ──→ PWM_CC0 驱动左轮
                │
     左轮实际速度 (作为右轮的目标)
                │
                ▼
             [右轮 PID] ──→ PWM_CC1 驱动右轮
```

- **左轮是主**，跟踪用户设定的目标速度
- **右轮是从**，跟踪左轮的实际速度
- 这样即使左右电机特性不同、地面摩擦力不同，右轮会自
动调节 PWM 追上左轮，最终两轮同速 → 走直线

### 为什么用左轮实际速度而不是用目标速度来引导右轮？

如果用同一个目标速度同时控制左右轮：

```
目标 ──→ [左轮 PID] ──→ 左轮实际 450
目标 ──→ [右轮 PID] ──→ 右轮实际 420
```

两轮各自追同一个目标，但实际输出可能不同（机械差异），
造成偏差累积。

用左轮实际速度引导右轮：

```
目标 500 ──→ [左轮 PID] ──→ 左轮实际 450 ──→ [右轮 PID] ──→ 右轮实际 450
```

右轮直接追踪左轮的真实状态，响应更快，同步更紧。

### PID 参数调优指南

| 现象 | 调整 |
|------|------|
| 速度响应慢、跟不上目标 | 增大 **KP** |
| 速度超调、来回震荡 | 减小 **KP**，增大 **KI** |
| 稳态有偏差（一直差一点） | 增大 **KI** |
| 积分项涨太高、恢复慢 | 减小 **KI**，或降低 **MAX_INTEGRAL** |
| 电机低速抖动 | 提高 **MIN_DUTY** |
