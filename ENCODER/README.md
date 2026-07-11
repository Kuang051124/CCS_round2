# ENCODER 编码器模块使用说明

## 硬件接线

| 编码器 | 信号线 | MSPM0 引脚 | 中断触发 |
|--------|--------|-----------|----------|
| 左轮 A 相 | LA | PA31 | 上升沿 → GPIOA IIDX_DIO31 |
| 左轮 B 相 | LB | PA28 | 下降沿 → GPIOA IIDX_DIO28 |
| 右轮 A 相 | RA | PA29 | 上升沿 → GPIOA IIDX_DIO29 |
| 右轮 B 相 | RB | PB27 | 下降沿 → GPIOB IIDX_DIO27 |

## 解码方式（2 倍频）

中断里对每个编码器做了 A/B 双相 2 倍频：

```
A 相上升沿 → 读 B 相电平: B=HIGH → 计数+1, B=LOW → 计数-1
B 相下降沿 → 读 A 相电平: A=HIGH → 计数+1, A=LOW → 计数-1
```

每周期产生 2 个计数脉冲，方向自动判定。

## API 速查

| 函数 | 返回 | 说明 |
|------|------|------|
| `ENCODER_Init()` | — | 初始化，清零所有计数器 |
| `ENCODER_GetLeftCount()` | `int32_t` | 左轮累计脉冲（正=前进） |
| `ENCODER_GetRightCount()` | `int32_t` | 右轮累计脉冲（正=前进） |
| `ENCODER_ResetLeft()` | — | 左轮计数清零 |
| `ENCODER_ResetRight()` | — | 右轮计数清零 |
| `ENCODER_SpeedSample()` | — | 测速采样，需在主循环周期调用 |
| `ENCODER_GetLeftSpeed()` | `int32_t` | 左轮速度（counts/s） |
| `ENCODER_GetRightSpeed()` | `int32_t` | 右轮速度（counts/s） |

## 使用步骤

### 1. 上电初始化（一次）

```c
#include "ENCODER/encoder.h"

void main(void) {
    // ... 系统初始化 ...
    Interrupt_Init();   // 使能 GPIO 中断（编码器中断在这里注册）
    ENCODER_Init();     // 清零计数
}
```

### 2. 主循环中周期调用测速

```c
while (1) {
    ENCODER_SpeedSample();  // 每 10ms 自动计算一次速度

    int32_t left_speed  = ENCODER_GetLeftSpeed();   // counts/s
    int32_t right_speed = ENCODER_GetRightSpeed();  // counts/s
    
    // ... 控制逻辑 ...
}
```

### 3. 距离/位置控制示例

```c
// 出发前清零
ENCODER_ResetLeft();
ENCODER_ResetRight();

// 等待走到指定脉冲数
while (ENCODER_GetLeftCount() < TARGET_COUNTS) {
    // 控制马达前进
    Motor_Set_Duty(&motor_bl, 0.3f);
    Motor_Set_Duty(&motor_br, 0.3f);
}
Motor_Stop(&motor_bl);
Motor_Stop(&motor_br);
```

### 4. 速度闭环控制示例

```c
// PID 速度控制
int32_t target_speed = 500;  // counts/s
ENCODER_SpeedSample();
int32_t actual = ENCODER_GetLeftSpeed();
int32_t error = target_speed - actual;
float duty = Kp * error;
Motor_Set_Duty(&motor_bl, duty);
```

### 5. 集成到 Task4 delay 段（替代延时）

```c
// 替代 mspm0_delay_ms 的编码器距离控制
ENCODER_ResetLeft();
ENCODER_ResetRight();

while (1) {
    ENCODER_SpeedSample();
    
    // 直走直到脉冲数达标
    int32_t dist = (ENCODER_GetLeftCount() + ENCODER_GetRightCount()) / 2;
    if (dist >= AO_TARGET_COUNTS) {
        break;  // 到达 O 点
    }
    
    tk_gyro_control(base_speed);  // 陀螺仪保持航向
}
```

## 注意事项

| 项 | 说明 |
|----|------|
| **方向** | 左轮硬件取反（面对面安装），`GetLeftCount/GetLeftSpeed` 已内部取反，正值=前进 |
| **采样间隔** | `SPEED_SAMPLE_INTERVAL_MS = 10ms`，`ENCODER_SpeedSample()` 两次调用间隔需 ≥10ms |
| **计数范围** | `int32_t`，约 ±21 亿脉冲，不会溢出 |
| **中断** | 编码器中断在 `Drivers/MSPM0/interrupt.c` 的 `GROUP1_IRQHandler` 中处理，无需用户干预 |
| **配对使用** | `ENCODER_SpeedSample()` 必须周期性调用（≥10ms 间隔），否则 `GetSpeed` 返回值为旧数据 |
| **左轮取反** | `GetLeftCount` 和 `GetLeftSpeed` 内部已对左轮取反（编码器面对面安装导致同向转动计数相反），对外统一：正值=前进 |
