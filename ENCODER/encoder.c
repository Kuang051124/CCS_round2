#include "encoder.h"
#include "Drivers/MSPM0/clock.h"

volatile int32_t g_leftEncoderCount  = 0;
volatile int32_t g_rightEncoderCount = 0;

/* ========== 测速相关 (TIMG0 10ms ISR 驱动) ========== */
#define SPEED_SAMPLE_INTERVAL_MS  10    /* 10ms, 与 MOTOR_TIM_INST 一致 */

int32_t  g_leftSpeed   = 0;
int32_t  g_rightSpeed  = 0;

static int32_t  g_lastLeftCount  = 0;
static int32_t  g_lastRightCount = 0;

void ENCODER_Init(void)
{
    g_leftEncoderCount  = 0;
    g_rightEncoderCount = 0;

    g_lastLeftCount  = 0;
    g_lastRightCount = 0;
    g_leftSpeed      = 0;
    g_rightSpeed     = 0;

    /* GPIO 中断使能已由 Interrupt_Init() 统一管理 */
}

/*
 *  由 MOTOR_TIM_INST (TIMG0) 10ms ZERO 中断调用。
 *  固定间隔, 不依赖 SysTick 轮询。
 *
 *  一阶低通滤波 (EMA): α=1/4, 时间常数 ~3 采样周期 (~30ms)
 *  消除编码器量化台阶 (100/200/300... → 平滑过渡)
 */
void ENCODER_SpeedSample(void)
{
    int32_t leftDelta  = g_leftEncoderCount  - g_lastLeftCount;
    int32_t rightDelta = g_rightEncoderCount - g_lastRightCount;

    /* 固定 10ms → 瞬时 counts/s = delta * 100 */
    int32_t leftRaw  = leftDelta  * 100;
    int32_t rightRaw = rightDelta * 100;

    /* EMA 低通滤波: filtered = (old * 3 + new) / 4  (α = 1/4) */
    g_leftSpeed  = (g_leftSpeed  * 3 + leftRaw*1)  / 4;
    g_rightSpeed = (g_rightSpeed * 3 + rightRaw*1) / 4;

    g_lastLeftCount  = g_leftEncoderCount;
    g_lastRightCount = g_rightEncoderCount;
}

/*
 *  编码器面对面安装，同向转动时计数相反。
 *  左轮取反，右轮保持原值。返回值: 正 = 前进, 负 = 后退。
 */
int32_t ENCODER_GetLeftCount(void)
{
    return -g_leftEncoderCount;
}

int32_t ENCODER_GetRightCount(void)
{
    return g_rightEncoderCount;
}

void ENCODER_ResetLeft(void)
{
    g_leftEncoderCount = 0;
    g_lastLeftCount    = 0;  /* 同步清零, 防止下次 SpeedSample 算出负速度尖峰 */
}

void ENCODER_ResetRight(void)
{
    g_rightEncoderCount = 0;
    g_lastRightCount    = 0;
}

int32_t ENCODER_GetLeftSpeed(void)
{
    return -g_leftSpeed;
}

int32_t ENCODER_GetRightSpeed(void)
{
    return g_rightSpeed;
}

/*
 *  编码器 GPIO 中断处理:
 *    位于 Drivers/MSPM0/interrupt.c 的 GROUP1_IRQHandler 中。
 *    LA=PA31 (RISE), LB=PA28 (FALL), RA=PA29 (RISE), RB=PB27 (FALL)
 *    2 倍频解码:
 *      A 相上升沿 → 读 B 相电平: B=HIGH → ++,  B=LOW → --
 *      B 相下降沿 → 读 A 相电平: A=HIGH → ++,  A=LOW → --
 */
