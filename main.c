/*
 * Copyright (c) 2021, Texas Instruments Incorporated
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


#include "main.h"
#include "Test2/pages.h"
#include "Test2/mode1.h"
// #include "PID_zdt/pid_store.h"
// #include "PID_zdt/pid_zdt_speed.h"
#include "stdio.h"
#include "tb6600.h"
#include "tb6612.h"
#include "Keyboard/keyboard.h"
#include "ti_msp_dl_config.h"
#include "BLUETOOTH/bluetooth.h"
#include "ENCODER/encoder.h"
#include "ENCODER/speed_control.h"

uint8_t oled_buffer[32];
int CAR_ID=1;//1是主车，2为从车
/* ZDT_X42S 步进电机句柄 (全局, 供各页面调用) */
// ZDT_HandleTypeDef motor1, motor2;



int main(void) {
  SYSCFG_DL_init();
  SysTick_Init();

  // MPU6050_Init();
  OLED_Init();
  WIT_Init(); 

  /* Don't remove this! */
  Interrupt_Init();
  ENCODER_Init();
  Motor_Init();
  SPEED_Init(150);       /* 初始 PWM 占空比 150/500 = 30% */
  Keyboard_Init();
  Bluetooth_Init();

  /* 调试: 蓝牙初始化后亮一下 LED */
  DL_GPIO_setPins(LED_PORT, LED_RED_PIN);
  mspm0_delay_ms(200);
  DL_GPIO_clearPins(LED_PORT, LED_RED_PIN);

  // SMotor_Init();

  NVIC_EnableIRQ(K230_INST_INT_IRQN);

  // DL_GPIO_setPins(LASER_PORT, LASER_PURPLE_PIN);
  // 步进电机初始化
  // ZDT_Init(&motor1, UART_ZDT_INST, 1);    // PA26/PA13
  // ZDT_Init(&motor2, UART_ZDT2_INST, 1);   // PB4/PB5

  /* 从 Flash 加载已保存的 Task1/2 参数 */
  TKParam_Load();

  /* 速度环跟踪初始化 (与 CameraPID_Track 并存) */
  // CameraPID_SpeedTrack_Init();

  uint8_t prev_page = 0xFF;

  /* B 车无键盘，上电直接进入工作模式 */
  if (CAR_ID == 2) {
      OLED_Clear();
      OLED_ShowString(0, 0, (uint8_t *)"B-Car Ready", 8);
      OLED_ShowString(0, 2, (uint8_t *)"Wait Master...", 8);
  }

  for (;;) {
    /* 蓝牙延迟任务: ISR 内不阻塞, 由主循环发送 ACK 等 */
    Bluetooth_Poll();

    /* B 车: 永远跑 Page_Test2, 不经过菜单 */
    if (CAR_ID == 2) {
        Page_Test2();
        continue;
    }

    if (page_state != prev_page) {
      OLED_Clear();
      prev_page = page_state;
    }
    switch (page_state) {
    case 0:
      Page_Home();
      break;
    case 1:
      Page_Calib();
      break;
    case 2:
      Page_Debug();
      break;
    case 3:
      Page_Test();
      break;
    case 4:
      Page_Debug_Step();
      break;
    case 5:
      Page_Debug_Gyro();
      break;
    case 6:
      Page_Debug_Camera();
      break;
    case 7:
      Page_CurveDebug();
      break;
    case 8:
      Page_PWMTest();
      break;
    default:
      break;
    }
  }
}
