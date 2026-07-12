#ifndef ENCODER_ENCODER_H_
#define ENCODER_ENCODER_H_

#include "ti_msp_dl_config.h"

/* 编码器原始计数值 */
extern volatile int32_t g_leftEncoderCount;
extern volatile int32_t g_rightEncoderCount;

void ENCODER_Init(void);
int32_t ENCODER_GetLeftCount(void);
int32_t ENCODER_GetRightCount(void);
void ENCODER_ResetLeft(void);
void ENCODER_ResetRight(void);

/* 测速 (counts/s) */
void ENCODER_SpeedSample(void);
int32_t ENCODER_GetLeftSpeed(void);
int32_t ENCODER_GetRightSpeed(void);

/* ========== PID 速度控制接口 ========== */

/* 占空比 → 编码器速度 (counts/s) 转换系数
 * 估算依据: PULSE_PER_CYCLE≈265, 最高转速≈300RPM
 *           300/60*265 ≈ 1325 counts/s, 取整 1400 留余量
 * 需上实测速校准: 跑最高占空比, 读 ENCODER_GetLeftSpeed() 取值
 */
#define DUTY_TO_SPEED_MAX  1400   /* 占空比 1.0 → 编码器速度 (counts/s) */
#define SPEED_MAX          1330   /* 最大速度 (counts/s), 0.95 * DUTY_TO_SPEED_MAX */

void SPEED_PID_Init(void);                                     /* 初始化 PID 参数           */
void SPEED_SetTarget(int32_t left_speed, int32_t right_speed); /* 设定目标速度 (counts/s)    */
void SPEED_Stop(void);                                         /* 紧急停止, PWM 清零         */
void SPEED_PID_Tick(void);                                     /* 10ms 中断回调: 测速→PID→输出 */

#endif /* ENCODER_ENCODER_H_ */
