#include "pid.h"
#include "clock.h"
#include "stdbool.h"
#include "stdio.h"
#include "string.h"
#include "ti_msp_dl_config.h"

char pid_rx_buffer[PID_RX_BUFFER_SIZE];
uint8_t pid_rx_state = 0;
bool new_pid_flag = false;

void parse_pid_rx_buffer(uint8_t tmp) {
  if (pid_rx_state == 0 && tmp == '$') {
    pid_rx_state = 1;
    new_pid_flag = false;
    for (int i = 0; i < PID_RX_BUFFER_SIZE; ++i) {
      pid_rx_buffer[i] = 0;
    }
  } else if (tmp == '#') {
    pid_rx_state = 0;
    new_pid_flag = true;
    // pid_rx_buffer[pid_rx_state - 1] = 0;
  } else if (pid_rx_state != 0 && !new_pid_flag) {
    pid_rx_buffer[(pid_rx_state++) - 1] = tmp;
  } else {
    new_pid_flag = false;
    pid_rx_state = 0;
  }
}

void parse_pid_coef(Pid_t *pid, char pid_rx_buffer[]) {
  sscanf(pid_rx_buffer, "%f,%f,%f", &pid->kp, &pid->ki, &pid->kd);
  sprintf(pid_rx_buffer, "kp:%f,ki:%f,kd:%f\n", pid->kp, pid->ki, pid->kd);
  uint8_t len = strlen(pid_rx_buffer);
  // for (int i = 0; i < len; ++i) {
  //   DL_UART_Main_transmitData(PC_INST, pid_rx_buffer[i]);
  //   mspm0_delay_ms(10);
  // }
}

void clear_pid_flag() {
  pid_rx_state = 0;
  new_pid_flag = false;
}

float Constrain_Float(float amt, float low, float high) {
  return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

void Pid_Clear(Pid_t *pid) {
  pid->err = 0;
  pid->out_p = 0;
  pid->out_i = 0;
  pid->out_d = 0;
  pid->output = 0;
  pid->last_err = 0;
  pid->last_derivative = 0;
  pid->integral = 0;
}

float PID_Inc(Pid_t *pid, float target, float feedback, float interval) {
  /*增量式 pid 计算*/
  pid->err = target - feedback; // 计算误差
  if (pid->err < pid->dead_zone && pid->err > -pid->dead_zone)
    pid->err = 0; // pid 死区

  pid->out_p = pid->kp * (pid->err - pid->last_err);
  pid->out_i = pid->ki * pid->err * interval;
  pid->out_d =
      pid->kd / interval * ((pid->err - pid->last_err) - pid->last_derivative);

  pid->last_derivative = pid->err - pid->last_err;
  pid->last_err = pid->err;

  pid->output += pid->out_p + pid->out_i + pid->out_d;

  pid->output = Constrain_Float(pid->output, -pid->max_output, pid->max_output);
  return pid->output;
}

// float PID_Loc(Pid_t *pid, float target, float feedback, float interval) {
//   /*位置式 pid 计算*/
//   pid->err = target - feedback; // 计算误差
//   if (pid->err < pid->dead_zone && pid->err > -pid->dead_zone)
//     pid->err = 0; // pid 死区

//   pid->integral += pid->err; // 累计误差

//   pid->out_i = Constrain_Float(pid->integral, -pid->max_integral,
//                                pid->max_integral); // 误差限幅

//   pid->out_p = pid->kp * pid->err;
//   pid->out_i = pid->ki * interval * pid->integral;
//   pid->out_d = pid->kd / interval * (pid->err - pid->last_err);

//   pid->last_err = pid->err;

//   /*输出限幅*/
//   pid->output = Constrain_Float(pid->output, -pid->max_output,
//   pid->max_output);

//   return pid->output;
// }

float PID_Loc(Pid_t *pid, float target, float feedback, float interval) {
  /*位置式 pid 计算*/
  pid->err = target - feedback; // 计算误差

  // 死区处理
  if (pid->err < pid->dead_zone && pid->err > -pid->dead_zone)
    pid->err = 0;

  // 积分累加（带限幅）
  pid->integral += pid->err;
  pid->integral =
      Constrain_Float(pid->integral, -pid->max_integral, pid->max_integral);

  // 计算比例项
  pid->out_p = pid->kp * pid->err;

  // 计算积分项（使用限幅后的积分值）
  pid->out_i = pid->ki * interval * pid->integral;

  // 计算微分项
  pid->out_d = pid->kd * (pid->err - pid->last_err) / interval;

  // 更新误差记录
  pid->last_err = pid->err;

  // 计算总输出（直接求和而非累加）
  pid->output = pid->out_p + pid->out_i + pid->out_d;

  // 输出限幅
  pid->output = Constrain_Float(pid->output, -pid->max_output, pid->max_output);

  return pid->output;
}

// void PC_INST_IRQHandler(void) {
//   switch (DL_UART_Main_getPendingInterrupt(PC_INST)) {
//   case DL_UART_MAIN_IIDX_RX:
//     parse_pid_rx_buffer(DL_UART_Main_receiveData(PC_INST));
//     // echo_data = DL_UART_Main_receiveDataBlocking(PC_INST);
//     // Pto_Data_Receive(echo_data);
//     break;
//   default:
//     break;
//   }
// }