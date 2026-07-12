/*
 * Copyright (c) 2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "tb6612.h"
#include "clock.h"
#include "pid.h"
#include "ti_msp_dl_config.h"

Motor_t motor_bl;
Motor_t motor_br;
Motor_t motor_fl;
Motor_t motor_fr;

void Motor_Init() {
  /*电机*/
  DL_TimerG_startCounter(PWM_MOTOR_INST); // PWM 初始化

  /*编码器*/
  // NVIC_EnableIRQ(ENCODER_GPIOB_INT_IRQN);  // 编码器输入捕获中断
  // NVIC_EnableIRQ(ENCODER_GPIOA_INT_IRQN);  // 编码器输入捕获中断
  // NVIC_EnableIRQ(MOTOR_TIM_INST_INT_IRQN); // 电机速度定时器中断开启
  // DL_TimerA_startCounter(MOTOR_TIM_INST);  // 电机速度定时器初始化

  /*motor_b1 初始化*/
  motor_bl.pwm_pulse = 0;
  motor_bl.speed = 0;
  motor_bl.channel = DL_TIMER_CC_0_INDEX;
  motor_bl.forward_GPIO_PORT = MOTOR_BL_IN2_PORT;
  motor_bl.reverse_GPIO_PORT = MOTOR_BL_IN1_PORT;
  motor_bl.forward_GPIO_PIN = MOTOR_BL_IN2_PIN;
  motor_bl.reverse_GPIO_PIN = MOTOR_BL_IN1_PIN;

  motor_bl.speed_pid.kp = 0.9;
  motor_bl.speed_pid.ki = 0.05;
  motor_bl.speed_pid.kd = 0.8;
  motor_bl.speed_pid.max_integral = DL_Timer_getLoadValue(PWM_MOTOR_INST);
  motor_bl.speed_pid.max_output = DL_Timer_getLoadValue(PWM_MOTOR_INST);
  motor_bl.speed_pid.dead_zone = 0;

  motor_bl.pos_pid.kp = 1;
  motor_bl.pos_pid.ki = 1;
  motor_bl.pos_pid.kd = 1;
  motor_bl.pos_pid.max_integral = 1000;
  motor_bl.pos_pid.max_output = 1000;
  motor_bl.pos_pid.dead_zone = 0;

  motor_bl.encoder.count_num = 0;
  motor_bl.encoder.last_count = 0;
  motor_bl.encoder.speed_count = 0;

  /*motor_br 初始化*/
  motor_br.pwm_pulse = 0;
  motor_br.speed = 0;
  motor_br.channel = DL_TIMER_CC_1_INDEX;
  motor_br.forward_GPIO_PORT = MOTOR_BR_IN2_PORT;
  motor_br.reverse_GPIO_PORT = MOTOR_BR_IN1_PORT;
  motor_br.forward_GPIO_PIN = MOTOR_BR_IN2_PIN;
  motor_br.reverse_GPIO_PIN = MOTOR_BR_IN1_PIN;

  motor_br.speed_pid.kp = 0.9;
  motor_br.speed_pid.ki = 0.05;
  motor_br.speed_pid.kd = 0.01;
  motor_br.speed_pid.max_integral = DL_Timer_getLoadValue(PWM_MOTOR_INST);
  motor_br.speed_pid.max_output = DL_Timer_getLoadValue(PWM_MOTOR_INST);
  motor_br.speed_pid.dead_zone = 0;

  motor_br.pos_pid.kp = 1;
  motor_br.pos_pid.ki = 1;
  motor_br.pos_pid.kd = 1;
  motor_br.pos_pid.max_integral = 1000;
  motor_br.pos_pid.max_output = 1000;
  motor_br.pos_pid.dead_zone = 0;

  motor_br.encoder.count_num = 0;
  motor_br.encoder.last_count = 0;
  motor_br.encoder.speed_count = 0;

  /*motor_fl 初始化*/
  motor_fl.pwm_pulse = 0;
  motor_fl.speed = 0;
  motor_fl.channel = DL_TIMER_CC_2_INDEX;
  motor_fl.forward_GPIO_PORT = MOTOR_FL_IN1_PORT;
  motor_fl.reverse_GPIO_PORT = MOTOR_FL_IN2_PORT;
  motor_fl.forward_GPIO_PIN = MOTOR_FL_IN1_PIN;
  motor_fl.reverse_GPIO_PIN = MOTOR_FL_IN2_PIN;

  motor_fl.speed_pid.kp = 0.9;
  motor_fl.speed_pid.ki = 0.05;
  motor_fl.speed_pid.kd = 0.8;
  motor_fl.speed_pid.max_integral = DL_Timer_getLoadValue(PWM_MOTOR_INST);
  motor_fl.speed_pid.max_output = DL_Timer_getLoadValue(PWM_MOTOR_INST);
  motor_fl.speed_pid.dead_zone = 0;

  motor_fl.pos_pid.kp = 1;
  motor_fl.pos_pid.ki = 1;
  motor_fl.pos_pid.kd = 1;
  motor_fl.pos_pid.max_integral = 1000;
  motor_fl.pos_pid.max_output = 1000;
  motor_fl.pos_pid.dead_zone = 0;

  motor_fl.encoder.count_num = 0;
  motor_fl.encoder.last_count = 0;
  motor_fl.encoder.speed_count = 0;

  /*motor_fr 初始化*/
  motor_fr.pwm_pulse = 0;
  motor_fr.speed = 0;
  motor_fr.channel = DL_TIMER_CC_3_INDEX;
  motor_fr.forward_GPIO_PORT = MOTOR_FR_IN1_PORT;
  motor_fr.reverse_GPIO_PORT = MOTOR_FR_IN2_PORT;
  motor_fr.forward_GPIO_PIN = MOTOR_FR_IN1_PIN;
  motor_fr.reverse_GPIO_PIN = MOTOR_FR_IN2_PIN;

  motor_fr.speed_pid.kp = 0.9;
  motor_fr.speed_pid.ki = 0.05;
  motor_fr.speed_pid.kd = 0.8;
  motor_fr.speed_pid.max_integral = DL_Timer_getLoadValue(PWM_MOTOR_INST);
  motor_fr.speed_pid.max_output = DL_Timer_getLoadValue(PWM_MOTOR_INST);
  motor_fr.speed_pid.dead_zone = 0;

  motor_fr.pos_pid.kp = 1;
  motor_fr.pos_pid.ki = 1;
  motor_fr.pos_pid.kd = 1;
  motor_fr.pos_pid.max_integral = 1000;
  motor_fr.pos_pid.max_output = 1000;
  motor_fr.pos_pid.dead_zone = 0;

  motor_fr.encoder.count_num = 0;
  motor_fr.encoder.last_count = 0;
  motor_fr.encoder.speed_count = 0;
}

/* ========== 以下为旧版编码器/速度控制系统 (已废弃) ==========
 * 功能已被 ENCODER/encoder.c 取代:
 *   旧: 基于 Timer 周期 + Motor_t.encoder.count_num 测速
 *   新: 基于 GPIO 中断 + g_leftEncoderCount/g_rightEncoderCount 测速
 * Speed_Low_Filter / Speed_Filter / Encoder_Get_Speed 均无外部调用者,
 * 仅保留供参考, 后续可删除。
 * ================================================================ */

#if 0  /* ---- 旧版速度滤波器 ---- */

void Speed_Low_Filter(float new_speed, Encoder_t *encoder) {
  /*
  在对电机测速时，需要进行均值滤波，否则由于编码器测速的特性，电机测出来的速度只能是某一个值的整数倍。
  */
  (encoder->speed) = ((encoder->speed) * SPEED_RECORD_NUM -
                      encoder->speed_recode[encoder->speed_count] + new_speed) /
                     SPEED_RECORD_NUM;
  encoder->speed_recode[encoder->speed_count] = new_speed;
  ++(encoder->speed_count);
  if ((encoder->speed_count) >= (uint8_t)SPEED_RECORD_NUM) {
    (encoder->speed_count) = 0;
  }
}

float Speed_Filter(float new_speed, Encoder_t *encoder) {
  float sum = 0.0f;
  for (uint16_t i = SPEED_RECORD_NUM - 1; i > 0; --i) {
    encoder->speed_recode[i] = encoder->speed_recode[i - 1];
    sum += encoder->speed_recode[i - 1];
  }
  encoder->speed_recode[0] = new_speed;
  sum += new_speed;
  encoder->speed = sum / SPEED_RECORD_NUM;
  return encoder->speed;
}

/* BUG: 硬编码了 motor_bl, 传其他电机会算出错误速度 */
float Encoder_Get_Speed(Encoder_t *encoder, uint16_t freq) {
  // 计算 rpm，
  float speed = (motor_bl.encoder.count_num - motor_bl.encoder.last_count) * 60 *
                freq / PULSE_PER_CYCLE;
  Speed_Low_Filter(speed, encoder);
  encoder->last_count = encoder->count_num;
  return encoder->speed;
}

#endif /* ---- 旧版速度系统 END ---- */

void Motor_Set_Pulse(Motor_t *motor) {
  /*为了输出正的占空比*/
  if (motor->pwm_pulse >= 0) {
    /*正转*/
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, motor->pwm_pulse,
                                     motor->channel);
    DL_GPIO_setPins(motor->forward_GPIO_PORT, motor->forward_GPIO_PIN);
    DL_GPIO_clearPins(motor->reverse_GPIO_PORT, motor->reverse_GPIO_PIN);
  } else if (motor->pwm_pulse < 0) {
    /*反转*/
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, -motor->pwm_pulse,
                                     motor->channel);
    DL_GPIO_setPins(motor->reverse_GPIO_PORT, motor->reverse_GPIO_PIN);
    DL_GPIO_clearPins(motor->forward_GPIO_PORT, motor->forward_GPIO_PIN);
  }
}

void Motor_Set_Duty(Motor_t *motor, float duty) {
  motor->pwm_pulse = duty * motor->speed_pid.max_output;
  /*为了输出正的占空比*/
  if (motor->pwm_pulse >= 0) {
    /*正转*/
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, motor->pwm_pulse,
                                     motor->channel);
    DL_GPIO_setPins(motor->forward_GPIO_PORT, motor->forward_GPIO_PIN);
    DL_GPIO_clearPins(motor->reverse_GPIO_PORT, motor->reverse_GPIO_PIN);
  } else if (motor->pwm_pulse < 0) {
    /*反转*/
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, -motor->pwm_pulse,
                                     motor->channel);
    DL_GPIO_setPins(motor->reverse_GPIO_PORT, motor->reverse_GPIO_PIN);
    DL_GPIO_clearPins(motor->forward_GPIO_PORT, motor->forward_GPIO_PIN);
  }
}

/* ========== 旧版速度控制 (已废弃) ==========
 * Motor_Set_Speed: 只存目标值, 不执行 PID (PID 调用已被注释)
 * motor_modify_speed: 旧版增量式 PID, 已由 SPEED_PID_Tick() 取代
 * 保留供参考, 后续可删除。
 * ========================================== */

#if 0  /* ---- 旧版速度控制 ---- */

void Motor_Set_Speed(Motor_t *motor, float speed) {
  /*速度环*/
  motor->speed = speed;
  // motor->pwm_pulse =
  //     PID_Inc(&motor->speed_pid, motor->speed, motor->encoder.speed,
  //     interval);
  // Motor_Set_Duty(motor);
}

void motor_modify_speed(Motor_t *motor, float interval) {
  motor->pwm_pulse =
      PID_Inc(&motor->speed_pid, motor->speed, motor->encoder.speed, interval);
  Motor_Set_Pulse(motor);
}

#endif /* ---- 旧版速度控制 END ---- */

void Motor_Set_Position(Motor_t *motor) { ; }

void Motor_Stop(Motor_t *motor) {
  DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, 0, motor->channel);
  DL_GPIO_clearPins(motor->forward_GPIO_PORT, motor->forward_GPIO_PIN);
  DL_GPIO_clearPins(motor->reverse_GPIO_PORT, motor->reverse_GPIO_PIN);
}

/* ========== 100Hz 电机 PID 调速定时器中断 ==========
 * TODO Step 3: 取消注释 + 改为调用 SPEED_PID_Tick()
 *   1. Motor_Init() 中启动定时器: DL_TimerA_startCounter(MOTOR_TIM_INST);
 *   2. NVIC 使能:               NVIC_EnableIRQ(MOTOR_TIM_INST_INT_IRQN);
 *   3. ISR 改为:
 *        case DL_TIMER_IIDX_ZERO:
 *            SPEED_PID_Tick();   // 测速 → PID → Motor_Set_Duty()
 *            break;
 *
 * 旧版代码 (已由 ENCODER/encoder.c + SPEED_PID_Tick 取代):
 *
 *   Encoder_Get_Speed(&motor_bl.encoder, 100);
 *   motor_modify_speed(&motor_bl, 1);
 * ================================================================ */

// void MOTOR_TIM_INST_IRQHandler(void) {
//   switch (DL_TimerA_getPendingInterrupt(MOTOR_TIM_INST)) {
//   case DL_TIMER_IIDX_ZERO:
//     SPEED_PID_Tick();  /* 在 ENCODER/speed_control.c 中实现 */
//     break;
//   default:
//     break;
//   }
// }