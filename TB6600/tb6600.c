#include "tb6600.h"
SMotor smotor_hor;
SMotor smotor_ver;

void SMotor_Init() {
  smotor_hor.PUL_INST = SM_PUL_HOR_INST;
  smotor_hor.PUL_channel = DL_TIMER_CC_0_INDEX;
  smotor_hor.DIR_GPIO_Port = STEP_DIR_HOR_PORT;
  smotor_hor.DIR_pin = STEP_DIR_HOR_PIN;
  smotor_hor.sub_div = 32;
  smotor_hor.step_angle = 1.8;
  smotor_hor.rot_cnt = 0;
  smotor_hor.rot_max = 0;
  smotor_hor.deg = 0;
  NVIC_EnableIRQ(SM_PUL_HOR_INST_INT_IRQN);
  SMotor_Set_Speed(&smotor_hor, 0.3);
  DL_TimerG_setCaptureCompareValue(smotor_hor.PUL_INST, 0,
                                   smotor_hor.PUL_channel);

  smotor_ver.PUL_INST = SM_PUL_VER_INST;
  smotor_ver.PUL_channel = DL_TIMER_CC_0_INDEX;
  smotor_ver.DIR_GPIO_Port = STEP_DIR_VER_PORT;
  smotor_ver.DIR_pin = STEP_DIR_VER_PIN;
  smotor_ver.sub_div = 32;
  smotor_ver.step_angle = 1.8;
  smotor_ver.rot_cnt = 0;
  smotor_ver.rot_max = 0;
  smotor_ver.deg = 0;
  NVIC_EnableIRQ(SM_PUL_VER_INST_INT_IRQN);
  SMotor_Set_Speed(&smotor_ver, 0.3);
  DL_TimerG_setCaptureCompareValue(smotor_ver.PUL_INST, 0,
                                   smotor_ver.PUL_channel);
}

void SMotor_Start(SMotor *motor) {
  DL_TimerG_startCounter(motor->PUL_INST); // PWM 初始化
  DL_TimerG_setCaptureCompareValue(motor->PUL_INST, motor->period_cnt / 2,
                                   motor->PUL_channel);
}

void SMotor_Set_Speed(SMotor *motor, float speed) {
  motor->speed = speed;
  uint32_t period_count =
      CPUCLK_FREQ * motor->step_angle / 8 / 360 / motor->sub_div / speed;
  // prescaler
  if (period_count > 65536 || period_count < 2) {
    motor->period_cnt = 0;
    return; // 速度超出范围
  }
  motor->period_cnt = period_count;
  DL_TimerG_stopCounter(motor->PUL_INST);
  DL_TimerG_setLoadValue(motor->PUL_INST, period_count);
  DL_TimerG_startCounter(motor->PUL_INST);
}

void SMotor_Stop(SMotor *motor) {
  motor->rot_max = 0;
  DL_TimerG_stopCounter(motor->PUL_INST);
  DL_TimerG_setCaptureCompareValue(motor->PUL_INST, 0, motor->PUL_channel);
}

void SMotor_Turn_Pulse(SMotor *motor, int16_t pulse) {
  if (pulse > 0) {
    motor->rot_cnt = 0;
    motor->rot_max = pulse;
    DL_GPIO_setPins(motor->DIR_GPIO_Port, motor->DIR_pin);
    SMotor_Start(motor);
  } else if (pulse < 0) {
    motor->rot_cnt = 0;
    motor->rot_max = -pulse;
    DL_GPIO_clearPins(motor->DIR_GPIO_Port, motor->DIR_pin);
    SMotor_Start(motor);
  }
}

void SMotor_Turn_Degree(SMotor *motor, float deg) {
  // 转多少角度
  motor->deg += deg;
  int16_t rot_max = deg * motor->sub_div / motor->step_angle;
  SMotor_Turn_Pulse(motor, rot_max);
  // SMotor_Set_Speed(motor, motor->speed);
}

void SMotor_Turn_To(SMotor *motor, float deg) {
  // 转到多少角度
  float delta_deg = deg - motor->deg;
  // if (fabs(delta_deg) < 0.5) {
  //   return;
  // }
  SMotor_Turn_Degree(motor, delta_deg);
}

// 这个函数其实没什么用感觉
// void SMotor_Turn_Cycle(SMotor *motor, int n) {
//   //   DL_TimerG_stopCounter(motor->PUL_INST);
//   //   deg_count_max = n * 360 * motor->step_angle / motor->sub_div;
//   //   deg_count = 0;
//   //   SMotor_Set_Speed(motor, motor->speed);
//   //   DL_TimerG_startCounter(motor->PUL_INST);
//   SMotor_Turn_Degree(motor, n * 360);
// }

void SM_PUL_HOR_INST_IRQHandler(void) {
  // 这里其实封装性已经被破坏了
  switch (DL_TimerG_getPendingInterrupt(SM_PUL_HOR_INST)) {
  case DL_TIMER_IIDX_LOAD:
    if (smotor_hor.rot_max) {
      ++smotor_hor.rot_cnt;
    } else {
      break;
    }
    if (smotor_hor.rot_cnt >= smotor_hor.rot_max) {
      SMotor_Stop(&smotor_hor);
    }
    break;
  default:
    break;
  }
}

void SM_PUL_VER_INST_IRQHandler(void) {
  // 这里其实封装性已经被破坏了
  switch (DL_TimerG_getPendingInterrupt(SM_PUL_VER_INST)) {
  case DL_TIMER_IIDX_LOAD:
    if (smotor_ver.rot_max) {
      ++smotor_ver.rot_cnt;
    } else {
      break;
    }
    if (smotor_ver.rot_cnt >= smotor_ver.rot_max) {
      SMotor_Stop(&smotor_ver);
    }
    break;
  default:
    break;
  }
}