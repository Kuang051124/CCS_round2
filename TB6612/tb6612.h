
#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "pid.h"
#include "stdlib.h"
#include "ti_msp_dl_config.h"

#define MOTOR_SPEED_RERATIO 20.409f // 电机减速比
#define PULSE_PRE_ROUND 13     // 电机每圈脉冲数
#define ENCODER_MULTIPLE 1.0   // 编码器倍频
#define PULSE_PER_CYCLE                                                        \
  (MOTOR_SPEED_RERATIO * PULSE_PRE_ROUND * ENCODER_MULTIPLE) // 根据每圈脉冲计数
// #define PULSE_PER_CYCLE 1061
#define RADIUS_OF_TYRE 3.3                     // 轮子直径 cm
#define LINE_SPEED_C RADIUS_OF_TYRE * 2 * 3.14 // 轮子周长 cm
#define SPEED_RECORD_NUM 1000                  // 速度记录值，用于均值滤波
#define MAX_PERIOD_CNT 4000

typedef struct {
  uint8_t direct;
  int count_num;  // 编码器计数
  int last_count; // 上一轮编码器计数
  // int cycle_num;
  // int last_cycle;
  float speed;
  uint8_t speed_count; // 在线计算速度用的变量
  float speed_recode[SPEED_RECORD_NUM];
} Encoder_t;

typedef struct {
  uint8_t direct;               // 方向
  uint8_t channel;              // PWM定时器通道
  int32_t pos_pulse;            // 位置环目标脉冲
  int32_t forward_GPIO_PIN;     // 前进GPIO引脚
  int32_t reverse_GPIO_PIN;     // 后退GPIO引脚
  float pwm_pulse;              // PWM占空比
  float speed;                  // 目标速度
  float pos;                    // 目标位置
  Encoder_t encoder;            // 编码器
  Pid_t pos_pid;                // 速度环pid
  Pid_t speed_pid;              // 位置环pid
  GPIO_Regs *forward_GPIO_PORT; // 前进GPIO端口
  GPIO_Regs *reverse_GPIO_PORT; // 后退GPIO端口
} Motor_t;

extern Motor_t motor_bl;
extern Motor_t motor_br;
extern Motor_t motor_fl;
extern Motor_t motor_fr;

void Motor_Init();
void Motor_Set_Pulse(Motor_t *motor);      /* 底层: 直接写 PWM 寄存器 + 方向引脚   */
void Motor_Set_Duty(Motor_t *motor, float duty); /* 中层: duty∈[-1,1] → Motor_Set_Pulse */
void Motor_Stop(Motor_t *motor);           /* 停止: PWM 清零 + 方向引脚拉低          */

/* ---- 以下为旧版接口 (已废弃, 由 ENCODER/encoder.c 取代) ----
void Speed_Low_Filter(float new_speed, Encoder_t *encoder);
float Speed_Filter(float new_speed, Encoder_t *encoder);
void Motor_Set_Speed(Motor_t *motor, float speed);
------------------------------------------------------------- */

#endif /* ti_msp_dl_config_h */