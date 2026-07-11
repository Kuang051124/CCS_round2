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

#endif /* ENCODER_ENCODER_H_ */
