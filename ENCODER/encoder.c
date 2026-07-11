#include "encoder.h"
#include "Drivers/MSPM0/clock.h"

volatile int32_t g_leftEncoderCount  = 0;
volatile int32_t g_rightEncoderCount = 0;

/* ========== 测速相关 ========== */
#define SPEED_SAMPLE_INTERVAL_MS  10    /* 10ms 采样一次 */

static int32_t  g_lastLeftCount  = 0;
static int32_t  g_lastRightCount = 0;
static int32_t  g_leftSpeed      = 0;   /* counts/s */
static int32_t  g_rightSpeed     = 0;
static uint32_t g_lastSampleTick  = 0;

void ENCODER_Init(void)
{
    g_leftEncoderCount  = 0;
    g_rightEncoderCount = 0;

    g_lastLeftCount   = 0;
    g_lastRightCount  = 0;
    g_leftSpeed       = 0;
    g_rightSpeed      = 0;
    g_lastSampleTick  = tick_ms;

    /* 中断使能已统一由 Interrupt_Init() 管理，见 Drivers/MSPM0/interrupt.c */
}

/*
 *  周期性调用，每 SPEED_SAMPLE_INTERVAL_MS 计算一次速度
 */
void ENCODER_SpeedSample(void)
{
    uint32_t now = tick_ms;
    int32_t  elapsed;

    /* 防溢出：tick_ms 是 unsigned long，直接减法安全 */
    if (now < g_lastSampleTick) {
        g_lastSampleTick = now;
        return;
    }
    elapsed = now - g_lastSampleTick;
    if (elapsed < SPEED_SAMPLE_INTERVAL_MS)
        return;

    int32_t leftDelta  = g_leftEncoderCount  - g_lastLeftCount;
    int32_t rightDelta = g_rightEncoderCount - g_lastRightCount;

    g_leftSpeed  = leftDelta  * 1000 / elapsed;   /* counts/s */
    g_rightSpeed = rightDelta * 1000 / elapsed;

    g_lastLeftCount   = g_leftEncoderCount;
    g_lastRightCount  = g_rightEncoderCount;
    g_lastSampleTick  = now;
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
 *  编码器中断处理已移至 Drivers/MSPM0/interrupt.c 的 GROUP1_IRQHandler 中，
 *  通过 MSPM0 标准 IIDX 分派机制处理:
 *    LA=PA31 (RISE) → IIDX_DIO31
 *    LB=PA28 (FALL) → IIDX_DIO28
 *    RA=PA29 (RISE) → IIDX_DIO29
 *    RB=PB27 (FALL) → IIDX_DIO27
 *
 *  2 倍频解码:
 *    A 相上升沿 → 读 B 相电平: B=HIGH → ++,  B=LOW → --
 *    B 相下降沿 → 读 A 相电平: A=HIGH → ++,  A=LOW → --
 */

/*
 *  编码器面对面安装，同向转动时计数相反。
 *  左轮取反，右轮保持原值。统一方向：正值 = 前进，负值 = 后退。
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
}

void ENCODER_ResetRight(void)
{
    g_rightEncoderCount = 0;
}
