#ifndef ENCODER_ENCODER_H_
#define ENCODER_ENCODER_H_

#include "ti_msp_dl_config.h"

/* 编码器原始计数值 (GPIO ISR 实时更新) */
extern volatile int32_t g_leftEncoderCount;
extern volatile int32_t g_rightEncoderCount;

/* 编码器测速值 (counts/s, 由 TIMG0 10ms ISR 更新) */
extern int32_t g_leftSpeed;
extern int32_t g_rightSpeed;

void ENCODER_Init(void);

/* 原始计数读取 (SEG_STRAIGHT_DELAY 编码器到位判定用) */
int32_t ENCODER_GetLeftCount(void);
int32_t ENCODER_GetRightCount(void);
void ENCODER_ResetLeft(void);
void ENCODER_ResetRight(void);

/* 由 TIMG0 10ms ISR 调用, 计算左右轮速度 */
void ENCODER_SpeedSample(void);

/* speed_control 模块使用: 获取当前速度 */
int32_t ENCODER_GetLeftSpeed(void);
int32_t ENCODER_GetRightSpeed(void);

/* ========== PID 速度控制接口 (speed_control.c 实现) ========== */
#define SPEED_MAX  5000   /* 最大速度 (counts/s), 实测 96%占空比≈5000, 取整 */

#endif /* ENCODER_ENCODER_H_ */
