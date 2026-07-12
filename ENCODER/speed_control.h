#ifndef ENCODER_SPEED_CONTROL_H_
#define ENCODER_SPEED_CONTROL_H_

#include "ti_msp_dl_config.h"

/**
 * @brief  初始化速度 PI 控制
 * @param  base_duty  初始 PWM 占空比 (0 ~ PWM_PERIOD)
 */
void SPEED_Init(int32_t base_duty);

/**
 * @brief  速度控制唯一入口 — 设定左右轮目标速度并执行 PI 控制
 *
 *         左右轮各自独立 PI, 分别跟踪 left_speed 和 right_speed。
 *         由 TIMG0 ISR 每 10ms 定时调用, 也由蓝牙/用户代码调用以改变目标。
 *
 * @param  left_speed   左轮目标速度 (counts/s), = 0 停止
 * @param  right_speed  右轮目标速度 (counts/s), = 0 停止
 *         (同时为 0 时急停, 清零积分和 PWM)
 */
void SPEED_SetTarget(int32_t left_speed, int32_t right_speed);

/* 紧急停止: 等价于 SPEED_SetTarget(0, 0) */
void SPEED_Stop(void);

/* 由 10ms ISR 独占调用: 测速 → 增量式 PID → Motor_Set_Duty
 * SPEED_SetTarget 只存目标, 不跑 PID; 此函数是 PID 的唯一执行者 */
void SPEED_PID_Tick(void);

/* PID 是否活跃: ISR 用此标志决定是否执行闭环控制
 *   1 = 有人设过非零目标 → ISR 每 10ms 跑 PID
 *   0 = 未设目标或已停止 → ISR 跳过, 不干扰手动 PWM 测试 */
uint8_t SPEED_IsActive(void);

/**
 * @brief  获取当前目标速度
 */
int32_t SPEED_GetTargetLeft(void);
int32_t SPEED_GetTargetRight(void);

/**
 * @brief  获取当前实际 PWM 占空比值
 */
int32_t SPEED_GetLeftDuty(void);
int32_t SPEED_GetRightDuty(void);

/* ========== PID 参数在线调整 (蓝牙 slider 指令) ========== */

void SPEED_SetKp(float kp);
void SPEED_SetKi(float ki);
void SPEED_SetKd(float kd);
float SPEED_GetKp(void);
float SPEED_GetKi(void);
float SPEED_GetKd(void);

#endif /* ENCODER_SPEED_CONTROL_H_ */
