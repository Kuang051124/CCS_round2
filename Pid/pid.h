#ifndef _PID_H_
#define _PID_H_
#include "ti_msp_dl_config.h"

#define PID_RX_BUFFER_SIZE 32

typedef struct {
  float kp, ki, kd;             // pid参数
  float err, last_err;          // 误差、上一次误差
  float last_derivative;        // 上次误差与上上次误差之差
  float integral, max_integral; // 积分值，积分限幅
  float output, max_output;     // 输出， 输出限幅
  float out_p, out_i, out_d;    // 比例，积分，微分输出
  float dead_zone;              // pid 死区
} Pid_t;

extern char pid_rx_buffer[PID_RX_BUFFER_SIZE];
extern uint8_t pid_rx_state;
extern bool new_pid_flag;

void parse_pid_coef(Pid_t *pid, char pid_rx_buffer[]);

void parse_pid_rx_buffer(uint8_t tmp);

void clear_pid_flag();

void Pid_Clear(Pid_t *pid);

float Constrain_Float(float amt, float low, float high);

float PID_Inc(Pid_t *pid, float target, float feedback, float interval);

float PID_Loc(Pid_t *pid, float target, float feedback, float interval);

#endif