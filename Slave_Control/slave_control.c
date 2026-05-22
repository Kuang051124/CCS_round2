#include "slave_control.h"
#include "clock.h"
#include "keyboard.h"
#include "mpu6050.h"
#include "oled_hardware_i2c.h"
#include "pid.h"
#include "queue.h"
#include "tb6600.h"
#include "tb6612.h"
#include "trace.h"
#include "wit.h"
#include "yb_protocol.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PI 3.1415926535f
#define EPSILON 1e-10

uint16_t len;
uint8_t page_state = 0;
uint8_t key = 0;
int recv_data[6];
uint8_t echo_data = 0;

uint8_t trace_turn_times = 0; // 循迹黑线状态

uint8_t state;        // 状态机变量
SeqQueue_t maze;      // 迷宫
uint8_t target_state; // 目标状态
Point_t gap_point;
Point_t end_point;
uint8_t center_distance = 4;

char oled_buffer[32];

typedef struct {
  float xdeg;
  float ydeg;
} Deg_t;

Point_t target_pos;
Point_t laser_pos;
Point_t last_pos;
Point_t temp_pos;

Deg_t circle_deg_pos;

Pid_t vert_laser_pid;
Pid_t hori_laser_pid;

uint16_t round_num = 0; // 记录拐弯数
uint8_t circle_num = 0; // 记录要转的圈数

#define PARAM_SIZE 5
float walk_strt_1_duty = 0.17;         // 直走参数
float turn_left_1_left_duty = -0.25;   // 直角拐弯左轮参数
float turn_left_1_right_duty = 0.5;    // 直角拐弯右轮参数
unsigned long turn_left_1_delay = 800; // 直角拐弯延时
uint8_t turn_left_1_succeed_times = 3; // 转弯角成功次数

uint8_t target_succeed_num = 0; // 中靶计数，以免转过
uint8_t target_succeed_max = 5;

uint8_t param_index = 0;

uint32_t last_time = 0; // 记录上一次时间
uint32_t curr_time = 0; // 记录现在时间

bool flag; // 状态机中只执行一次

float gyro_hori_deg = 0;
float gyro_vert_deg = 0;

float delta; // 第三问角度

float Distance(Point_t p, Point_t q) {
  return sqrtf(pow(p.x - q.x, 2) + pow(p.y - q.y, 2));
}

Deg_t Circle_Location(double x, double y, double r, uint16_t n,
                      uint16_t accuracy)
// 圆取点
/*
 * TODO 还有别的方式，先计算生成一个数组，让电机按这个数组里的点移动即可
 */
{
  double angle = 0;
  Deg_t tmp;
  angle = -PI + n * 2 * PI / accuracy;
  tmp.xdeg = r * cos(angle) + x;
  tmp.ydeg = r * sin(angle) + y;
  return tmp;
}

void Beep(uint16_t ms) {
  DL_GPIO_clearPins(BUZZER_PORT, BUZZER_BEEP_PIN);
  mspm0_delay_ms(ms);
  DL_GPIO_setPins(BUZZER_PORT, BUZZER_BEEP_PIN);
}

void Motor_Stop_All() {
  Motor_Stop(&motor_fl);
  Motor_Stop(&motor_fr);
  Motor_Stop(&motor_bl);
  Motor_Stop(&motor_br);
}

void Motor_Set_Left_Duty(float duty) {
  Motor_Set_Duty(&motor_fl, duty);
  Motor_Set_Duty(&motor_bl, duty);
}

void Motor_Set_Right_Duty(float duty) {
  Motor_Set_Duty(&motor_fr, duty);
  Motor_Set_Duty(&motor_br, duty);
}

//////////////// 第一版占空比 //////////////////////
// 全部动态
void Motor_Modify_Left_1(int trace_sum) {
  Motor_Set_Left_Duty(0.08 / trace_sum);
  Motor_Set_Right_Duty(0.2 + 0.1 * trace_sum);
}

void Motor_Modify_Right_1(int trace_sum) {
  Motor_Set_Left_Duty(0.2 + 0.1 * trace_sum);
  Motor_Set_Right_Duty(0.08 / trace_sum);
}

void Motor_Walk_Strt_1() {
  Motor_Set_Left_Duty(walk_strt_1_duty);
  Motor_Set_Right_Duty(walk_strt_1_duty);
}

void Motor_Turn_Left_1() {
  // Motor_Set_Left_Duty(0);
  Motor_Set_Left_Duty(turn_left_1_left_duty);
  Motor_Set_Right_Duty(turn_left_1_right_duty);
}
//////////////////////////////////////////////////

//////////////////////////////////////////////////
void Motor_Modify_Left(int trace_sum) {
  Motor_Set_Left_Duty(0.08 / trace_sum);
  Motor_Set_Right_Duty(0.2 + 0.1 * trace_sum);
}

void Motor_Modify_Right(int trace_sum) {
  Motor_Set_Left_Duty(0.2 + 0.1 * trace_sum);
  Motor_Set_Right_Duty(0.08 / trace_sum);
}

void Motor_Walk_Strt() {
  Motor_Set_Left_Duty(0.15);
  Motor_Set_Right_Duty(0.15);
}

void Motor_Turn_Left() {
  Motor_Set_Left_Duty(-0.2);
  Motor_Set_Right_Duty(0.4);
}
//////////////////////////////////////////////////

void Stop_Camera() {
  sprintf(oled_buffer, "$P#%%");
  len = strlen(oled_buffer);
  for (int i = 0; i < len; ++i) {
    DL_UART_Main_transmitData(K230_INST, oled_buffer[i]);
    mspm0_delay_ms(5);
  }
}

double Normalize_Angle_Positive(double angle_deg) {
  // 处理极端大值
  if (fabs(angle_deg) > 1e15) {
    angle_deg = fmod(angle_deg, 360.0);
  }

  // 规范化角度
  angle_deg = fmod(angle_deg, 360.0);
  if (angle_deg < 0) {
    angle_deg += 360.0;
  }

  // 处理边界情况
  if (fabs(angle_deg - 360.0) < EPSILON) {
    return 0.0;
  }
  return angle_deg;
}

// 计算两个角度之间的劣弧（最短角度差）
double Get_Shortest_Arc(double theta1_deg, double theta2_deg) {
  // 规范化输入角度
  theta1_deg = Normalize_Angle_Positive(theta1_deg);
  theta2_deg = Normalize_Angle_Positive(theta2_deg);

  // 计算原始差值
  double delta = theta2_deg - theta1_deg;

  // 使用模运算规范化到 [-180, 180] 范围
  double adjusted_delta = fmod(delta + 180.0, 360.0);
  if (adjusted_delta < 0) {
    adjusted_delta += 360.0;
  }
  adjusted_delta -= 180.0;

  // 处理浮点精度边界问题
  if (adjusted_delta <= -180.0) {
    adjusted_delta += 360.0;
  } else if (adjusted_delta > 180.0) {
    adjusted_delta -= 360.0;
  }

  // 处理 -180° 边界（统一表示为 180°）
  if (fabs(adjusted_delta + 180.0) < EPSILON) {
    return 180.0;
  }

  return adjusted_delta;
}

void Page_Home() {
  OLED_ShowString(0, 0, (uint8_t *)"Home Page", 8);
  OLED_ShowString(0, 1, (uint8_t *)"[1]Calib Page", 8);
  OLED_ShowString(0, 2, (uint8_t *)"[2]Debug Page", 8);
  OLED_ShowString(0, 3, (uint8_t *)"[3]Test Page", 8);
  key = Scan_Keyboard();
  switch (key) {
  case 1:
    while ((Scan_Keyboard()))
      ;

    OLED_Clear();
    page_state = 1;
    break;
  case 2:
    while ((Scan_Keyboard()))
      ;
    OLED_Clear();
    page_state = 2;
    break;
  case 3:
    while ((Scan_Keyboard()))
      ;
    OLED_Clear();
    page_state = 3;
    break;
  default:
    break;
  }
}

void Page_Calib() {
  uint8_t i = 0;
  sprintf(oled_buffer, "WkSt:%4.2f", walk_strt_1_duty);
  OLED_ShowString(6, i++, (uint8_t *)oled_buffer, 8);

  sprintf(oled_buffer, "LDuty:%4.2f", turn_left_1_left_duty);
  OLED_ShowString(6, i++, (uint8_t *)oled_buffer, 8);

  sprintf(oled_buffer, "RDuty:%4.2f", turn_left_1_right_duty);
  OLED_ShowString(6, i++, (uint8_t *)oled_buffer, 8);

  sprintf(oled_buffer, "Delay:%4d", turn_left_1_delay);
  OLED_ShowString(6, i++, (uint8_t *)oled_buffer, 8);

  sprintf(oled_buffer, "Sceds:%4d", turn_left_1_succeed_times);
  OLED_ShowString(6, i++, (uint8_t *)oled_buffer, 8);

  OLED_ShowChar(0, param_index, '>', 8);

  key = Scan_Keyboard();
  switch (key) {
  // 动态调参
  case 1: // 增加
    switch (param_index) {
    case 0:
      walk_strt_1_duty += 0.01;
      break;
    case 1:
      turn_left_1_left_duty += 0.01;
      break;
    case 2:
      turn_left_1_right_duty += 0.01;
      break;
    case 3:
      turn_left_1_delay += 1;
      break;
    case 4:
      turn_left_1_succeed_times += 1;
      break;
    default:
      break;
    }
    mspm0_delay_ms(100);
    break;

  case 2: // 减少
    switch (param_index) {
    case 0:
      walk_strt_1_duty -= 0.01;
      break;
    case 1:
      turn_left_1_left_duty -= 0.01;
      break;
    case 2:
      turn_left_1_right_duty -= 0.01;
      break;
    case 3:
      turn_left_1_delay -= 1;
      break;
    case 4:
      turn_left_1_succeed_times -= 1;
      break;
    default:
      break;
    }
    mspm0_delay_ms(100);

    break;
  case 3: // 调整
    OLED_Clear();
    param_index = (param_index + 1) % PARAM_SIZE;
    while (Scan_Keyboard())
      ;
    break;

  case 9:
    OLED_Clear();
    page_state = 0;
    break;

  default:
    break;
  }
}

void Page_Test() {
  uint8_t i = 0;
  OLED_ShowString(0, i++, (uint8_t *)"Test Page    ", 8);

  key = Scan_Keyboard();
  switch (key) {
  case 1:
    // 至少再给键盘接两根线吧
    while (Scan_Keyboard())
      ; // 堵塞键盘

    while (1) {
      key = Scan_Keyboard(); // 扫描键盘
      if (1 <= key && key <= 5) {
        circle_num = key;
        break;
      }
    }

    round_num = 0; // 拐弯数归零
    OLED_Clear();

    trace_turn_times = 0; // 拐弯滤波变量归零

    // 五路循迹 //////////////////////// 第一版阈值 /////////////////////////
    while (1) {
      i = 0;
      Trace_Read_Bit(); // 读取循迹状态

      // 显示
      sprintf(oled_buffer, "Tsum:%1d%1d%1d%1d%1d", trace_states[0],
              trace_states[1], trace_states[2], trace_states[3],
              trace_states[4]);
      OLED_ShowString(0, i++, (uint8_t *)oled_buffer, 8);

      // 得到循迹状态的加权和
      trace_sum = Get_Trace_Sum(trace_states);

      // 根据加权和进入不同状态
      if (trace_sum == STOP) {
        break;
      } else if (trace_sum == TURN_LEFT) {
        // mspm0_delay_ms(400);
        Motor_Turn_Left_1();
        ++round_num;
        mspm0_delay_ms(turn_left_1_delay);
        while (1) {
          Trace_Read_Bit();
          // mspm0_delay_ms(5);
          if (trace_states[2] == BLACK || trace_states[3] == BLACK) {
            ++trace_turn_times;
            if (trace_turn_times > turn_left_1_succeed_times) {
              trace_turn_times = 0;
              // Beep(100);
              break;
            }
            continue;
          }
          trace_turn_times = 0;
        }
      } else if (trace_sum < 0) {
        Motor_Modify_Left_1(abs(trace_sum));
      } else if (trace_sum > 0) {
        Motor_Modify_Right_1(abs(trace_sum));
      } else {
        Motor_Walk_Strt_1();
      }
      mspm0_delay_ms(50);
      if (round_num == 4 * circle_num) {
        break;
      }
    }
    // ////////////////////////////////////////////////////////////////////

    //////////////////////// 七路第一版阈值 /////////////////////////
    // while (1) {
    //   i = 0;
    //   Trace_Read_Bit(); // 读取循迹状态

    //   // 显示
    //   sprintf(oled_buffer, "Tsum:%1d%1d%1d%1d%1d", trace_states[0],
    //           trace_states[1], trace_states[2], trace_states[3],
    //           trace_states[4]);
    //   OLED_ShowString(0, i++, (uint8_t *)oled_buffer, 8);

    //   // 得到循迹状态的加权和
    //   trace_sum = Get_Trace_Sum(trace_states);

    //   // 根据加权和进入不同状态
    //   if (trace_sum == STOP) {
    //     break;
    //   } else if (trace_sum == TURN_LEFT) {
    //     Motor_Turn_Left_1();
    //     ++round_num;
    //     mspm0_delay_ms(800);
    //   } else if (trace_sum < 0) {
    //     Motor_Modify_Left_1(abs(trace_sum));
    //   } else if (trace_sum > 0) {
    //     Motor_Modify_Right_1(abs(trace_sum));
    //   } else {
    //     Motor_Walk_Strt_1();
    //   }
    //   mspm0_delay_ms(50);
    //   if (round_num == 4) {
    //     break;
    //   }
    // }
    ////////////////////////////////////////////////////////////////////

    // //////// 第二版占空比 ///////////////////////////////////////////////
    // while (1) {
    //   i = 0;
    //   Trace_Read_Bit(); // 读取循迹状态

    //   // 显示
    //   sprintf(oled_buffer, "Tsum:%1d%1d%1d%1d%1d", trace_states[0],
    //           trace_states[1], trace_states[2], trace_states[3],
    //           trace_states[4]);
    //   OLED_ShowString(0, i++, (uint8_t *)oled_buffer, 8);

    //   // 得到循迹状态的加权和
    //   trace_sum = Get_Trace_Sum(trace_states);

    //   // 根据加权和进入不同状态
    //   if (trace_sum == STOP) {
    //     break;
    //   } else if (trace_sum == TURN_LEFT) {
    //     Motor_Turn_Left();
    //     ++round_num;
    //     // mspm0_delay_ms(800);
    //     mspm0_delay_ms(1800);
    //   } else if (trace_sum < 0) {
    //     Motor_Modify_Left(abs(trace_sum));
    //   } else if (trace_sum > 0) {
    //     Motor_Modify_Right(abs(trace_sum));
    //   } else {
    //     Motor_Walk_Strt();
    //   }
    //   mspm0_delay_ms(50);
    //   if (round_num == 4) {
    //     break;
    //   }
    // }
    // ////////////////////////////////////////////////////////////////////

    Motor_Stop_All();
    Beep(300);
    OLED_Clear();
    break;

  case 2:
    OLED_Clear();
    // Pid_Clear(&vert_laser_pid);
    // Pid_Clear(&hori_laser_pid);
    // 接收 k230 进行步进电机 pid
    vert_laser_pid.kp = 0.07;
    vert_laser_pid.ki = 0.03;
    vert_laser_pid.kd = 0.01;
    vert_laser_pid.dead_zone = 0;
    vert_laser_pid.max_integral = 1000;
    vert_laser_pid.max_output = 20;
    vert_laser_pid.output = 0;
    vert_laser_pid.integral = 0;
    vert_laser_pid.last_err = 0;
    vert_laser_pid.last_derivative = 0;

    hori_laser_pid.kp = 0.07;
    hori_laser_pid.ki = 0.03;
    hori_laser_pid.kd = 0.01;
    hori_laser_pid.dead_zone = 0;
    hori_laser_pid.max_integral = 1000;
    hori_laser_pid.max_output = 20;
    hori_laser_pid.output = 0;
    hori_laser_pid.integral = 0;
    vert_laser_pid.last_err = 0;
    hori_laser_pid.last_derivative = 0;

    flag = true;

    mspm0_get_clock_ms(&last_time);

    // 激光笔坐标 + 目标坐标
    Pto_Clear_CMD_Flag();

    DL_UART_Main_getErrorStatus(K230_INST, DL_UART_ERROR_OVERRUN);

    for (uint8_t i = 0; i < 6; ++i) {
      recv_data[i] = 0;
    }

    for (;;) {
      i = 0;

      mspm0_get_clock_ms(&curr_time);
      if (curr_time - last_time > 2000) {
        break;
      }

      // 协议接受
      if (!New_CMD_flag) {
        continue;
      }

      Pto_Data_Parse(RxBuffer, New_CMD_length, recv_data);

      Pto_Clear_CMD_Flag();

      if (flag) { // 消除发送的第一个混乱数据
        flag = false;
        continue;
      }

      if (recv_data[0] == 0 || recv_data[1] == 0 || recv_data[2] == 0 ||
          recv_data[3] == 0) {
        continue;
      }

      laser_pos.x = recv_data[0];
      laser_pos.y = recv_data[1];

      target_pos.x = recv_data[2];
      target_pos.y = recv_data[3];

      if (Distance(target_pos, laser_pos) < 5) {
        break;
      }

      // 清除串口错误
      DL_UART_Main_getErrorStatus(K230_INST, DL_UART_ERROR_OVERRUN);

      sprintf(oled_buffer, "Tpos:%4d,%4d     ", target_pos.x, target_pos.y);
      OLED_ShowString(0, i++, (uint8_t *)oled_buffer, 8);

      sprintf(oled_buffer, "Lpos:%4d,%4d    ", laser_pos.x, laser_pos.y);
      OLED_ShowString(0, i++, (uint8_t *)oled_buffer, 8);

      // pid 调整
      SMotor_Turn_To(&smotor_hor,
                     PID_Loc(&hori_laser_pid, target_pos.x, laser_pos.x, 1));
      SMotor_Turn_To(&smotor_ver,
                     PID_Loc(&vert_laser_pid, target_pos.y, laser_pos.y, 1));

      sprintf(oled_buffer, "Sdeg:%4.2f,%4.2f    ", smotor_hor.deg,
              smotor_ver.deg);
      OLED_ShowString(0, i++, (uint8_t *)oled_buffer, 8);

      mspm0_delay_ms(50);
    }

    DL_GPIO_setPins(LASER_PORT, LASER_PURPLE_PIN);
    Beep(300);
    mspm0_delay_ms(3700);
    DL_GPIO_clearPins(LASER_PORT, LASER_PURPLE_PIN);
    break;

  case 3:
    OLED_Clear();
    gyro_hori_deg = wit_data.yaw; // 记录当前的朝向

    while (Scan_Keyboard()) // 堵塞按键
      ;

    OLED_ShowString(0, 0, (uint8_t *)"Enter Direction", 8);

    key = 0;
    while (!key) {
      key = Scan_Keyboard();
    } // 选择
    // 1 逆时针，2 顺时针。

    mspm0_get_clock_ms(&last_time);

    while (1) {
      delta = Get_Shortest_Arc(wit_data.yaw,
                               gyro_hori_deg); // delta 算的是逆时针的值
      i = 0;
      sprintf(oled_buffer, "Yaw :%-5.2f", wit_data.yaw);
      OLED_ShowString(0, i++, (uint8_t *)oled_buffer, 8);

      sprintf(oled_buffer, "Hori:%-5.2f", gyro_hori_deg);
      OLED_ShowString(0, i++, (uint8_t *)oled_buffer, 8);

      sprintf(oled_buffer, "Delta:%-5.2f", delta);
      OLED_ShowString(0, i++, (uint8_t *)oled_buffer, 8);

      if (fabs(delta) < 3) {
        break;
      }

      // Beep(300);
      // key = 1，逆时针转，key = 2，顺时针转，如果是其他按键，则退出程序
      if (key == 1) {    // 需要逆时转
        if (delta < 0) { // 说明角度是顺时针
          delta += 360;
        }
      } else if (key == 2) { // 需要顺时转
        if (delta > 0) {     // 说明角度是逆时针
          delta -= 360;
        }
      } else {
        break;
      }
      // 如果 delta 为负，则顺时针转
      // 如果 delta 为正，则逆时针转
      SMotor_Turn_Degree(&smotor_hor, -0.08 * delta); // 旋转指定角度

    }; // 进行旋转

    Beep(300); // 旋转结束

    smotor_hor.deg = 0;
    smotor_ver.deg = 0;

    // 初始化 PID
    vert_laser_pid.kp = 0.07;
    vert_laser_pid.ki = 0.03;
    vert_laser_pid.kd = 0.01;
    vert_laser_pid.dead_zone = 0;
    vert_laser_pid.max_integral = 1000;
    vert_laser_pid.max_output = 20;
    vert_laser_pid.output = 0;
    vert_laser_pid.integral = 0;
    vert_laser_pid.last_err = 0;
    vert_laser_pid.last_derivative = 0;

    hori_laser_pid.kp = 0.07;
    hori_laser_pid.ki = 0.03;
    hori_laser_pid.kd = 0.01;
    hori_laser_pid.dead_zone = 0;
    hori_laser_pid.max_integral = 1000;
    hori_laser_pid.max_output = 20;
    hori_laser_pid.output = 0;
    hori_laser_pid.integral = 0;
    vert_laser_pid.last_err = 0;
    hori_laser_pid.last_derivative = 0;

    Pto_Clear_CMD_Flag(); // 清楚中断标志位

    DL_UART_Main_getErrorStatus(K230_INST,
                                DL_UART_ERROR_OVERRUN); // 清除串口错误

    OLED_Clear();

    for (;;) {
      i = 0;

      mspm0_get_clock_ms(&curr_time);     // 获取当前时间
      if (curr_time - last_time > 4000) { // 判断超时
        break;
      }

      // 接收串口消息
      if (!New_CMD_flag) {
        continue;
      }

      Pto_Data_Parse(RxBuffer, New_CMD_length, recv_data); // 协议匹配

      Pto_Clear_CMD_Flag(); // 清除标志位

      if (flag) { // 消除发送的第一个混乱数据
        flag = false;
        continue;
      }

      if (recv_data[0] == 0 || recv_data[1] == 0 || recv_data[2] == 0 ||
          recv_data[3] == 0) {
        continue;
      }

      laser_pos.x = recv_data[0];
      laser_pos.y = recv_data[1];

      target_pos.x = recv_data[2];
      target_pos.y = recv_data[3];

      if (Distance(target_pos, laser_pos) < 5) {
        break;
      }

      // 清除串口错误
      DL_UART_Main_getErrorStatus(K230_INST, DL_UART_ERROR_OVERRUN);

      sprintf(oled_buffer, "Tpos:%4d,%4d     ", target_pos.x, target_pos.y);
      OLED_ShowString(0, i++, (uint8_t *)oled_buffer, 8);

      sprintf(oled_buffer, "Lpos:%4d,%4d    ", laser_pos.x, laser_pos.y);
      OLED_ShowString(0, i++, (uint8_t *)oled_buffer, 8);

      // pid 调整
      SMotor_Turn_To(&smotor_hor,
                     PID_Loc(&hori_laser_pid, target_pos.x, laser_pos.x, 1));
      SMotor_Turn_To(&smotor_ver,
                     PID_Loc(&vert_laser_pid, target_pos.y, laser_pos.y, 1));

      sprintf(oled_buffer, "Sdeg:%4.2f,%4.2f    ", smotor_hor.deg,
              smotor_ver.deg);
      OLED_ShowString(0, i++, (uint8_t *)oled_buffer, 8);

      mspm0_delay_ms(50);
    }

    DL_GPIO_setPins(LASER_PORT, LASER_PURPLE_PIN);
    Beep(300);
    mspm0_delay_ms(3700);
    DL_GPIO_clearPins(LASER_PORT, LASER_PURPLE_PIN);

    OLED_Clear();
    break;

  case 4:
    OLED_Clear();

    vert_laser_pid.kp = 0.07;
    vert_laser_pid.ki = 0.03;
    vert_laser_pid.kd = 0.01;
    vert_laser_pid.dead_zone = 0;
    vert_laser_pid.max_integral = 1000;
    vert_laser_pid.max_output = 25;
    vert_laser_pid.output = 0;
    vert_laser_pid.integral = 0;
    vert_laser_pid.last_err = 0;
    vert_laser_pid.last_derivative = 0;

    hori_laser_pid.kp = 0.07;
    hori_laser_pid.ki = 0.03;
    hori_laser_pid.kd = 0.01;
    hori_laser_pid.dead_zone = 0;
    hori_laser_pid.max_integral = 1000;
    hori_laser_pid.max_output = 25;
    hori_laser_pid.output = 0;
    hori_laser_pid.integral = 0;
    vert_laser_pid.last_err = 0;
    hori_laser_pid.last_derivative = 0;

    flag = true;

    mspm0_get_clock_ms(&last_time);

    // 激光笔坐标 + 目标坐标
    Pto_Clear_CMD_Flag();

    DL_UART_Main_getErrorStatus(K230_INST, DL_UART_ERROR_OVERRUN);

    target_succeed_num = 0; // 中靶计数归零

    for (uint8_t i = 0; i < 6; ++i) {
      recv_data[i] = 0;
    }

    for (;;) {
      i = 0;

      mspm0_get_clock_ms(&curr_time);
      if (curr_time - last_time > 4000) {
        break;
      }

      // 由于 recv_data 初始化为
      // 0，一旦收到靶消息后，就永远不会为零，前提是要保证对面不会发零
      if (recv_data[0] == 0 || recv_data[1] == 0 || recv_data[2] == 0 ||
          recv_data[3] == 0) {
        SMotor_Turn_Degree(&smotor_hor, -5); // 逆时针转，
        mspm0_delay_ms(60);                  // 加速
      }

      // 协议接受
      if (!New_CMD_flag) {
        continue;
      }

      Pto_Data_Parse(RxBuffer, New_CMD_length, recv_data);

      Pto_Clear_CMD_Flag();

      if (flag) { // 消除发送的第一个混乱数据
        smotor_hor.deg = 0;
        smotor_ver.deg = 0;
        flag = false;
        continue;
      }

      laser_pos.x = recv_data[0];
      laser_pos.y = recv_data[1];

      target_pos.x = recv_data[2];
      target_pos.y = recv_data[3];

      if (Distance(target_pos, laser_pos) < 2) {
        ++target_succeed_num;
        if (target_succeed_num > target_succeed_max) {
          target_succeed_num = 0;
          break;
        }
        mspm0_delay_ms(50);
        continue;
      }

      target_succeed_num = 0;
      // 别忘了改顺时针的

      // 清除串口错误
      DL_UART_Main_getErrorStatus(K230_INST, DL_UART_ERROR_OVERRUN);

      sprintf(oled_buffer, "Tpos:%4d,%4d     ", target_pos.x, target_pos.y);
      OLED_ShowString(0, i++, (uint8_t *)oled_buffer, 8);

      sprintf(oled_buffer, "Lpos:%4d,%4d    ", laser_pos.x, laser_pos.y);
      OLED_ShowString(0, i++, (uint8_t *)oled_buffer, 8);

      // pid 调整
      SMotor_Turn_To(&smotor_hor,
                     PID_Loc(&hori_laser_pid, target_pos.x, laser_pos.x, 1));
      SMotor_Turn_To(&smotor_ver,
                     PID_Loc(&vert_laser_pid, target_pos.y, laser_pos.y, 1));

      sprintf(oled_buffer, "Sdeg:%4.2f,%4.2f    ", smotor_hor.deg,
              smotor_ver.deg);
      OLED_ShowString(0, i++, (uint8_t *)oled_buffer, 8);

      mspm0_delay_ms(50);
    }

    DL_GPIO_setPins(LASER_PORT, LASER_PURPLE_PIN);
    Beep(300);
    mspm0_delay_ms(3700);
    DL_GPIO_clearPins(LASER_PORT, LASER_PURPLE_PIN);
    OLED_Clear();
    break;

  case 5: // 基本三顺时针
    OLED_Clear();

    vert_laser_pid.kp = 0.07;
    vert_laser_pid.ki = 0.03;
    vert_laser_pid.kd = 0.01;
    vert_laser_pid.dead_zone = 0;
    vert_laser_pid.max_integral = 1000;
    vert_laser_pid.max_output = 25;
    vert_laser_pid.output = 0;
    vert_laser_pid.integral = 0;
    vert_laser_pid.last_err = 0;
    vert_laser_pid.last_derivative = 0;

    hori_laser_pid.kp = 0.07;
    hori_laser_pid.ki = 0.03;
    hori_laser_pid.kd = 0.01;
    hori_laser_pid.dead_zone = 0;
    hori_laser_pid.max_integral = 1000;
    hori_laser_pid.max_output = 25;
    hori_laser_pid.output = 0;
    hori_laser_pid.integral = 0;
    vert_laser_pid.last_err = 0;
    hori_laser_pid.last_derivative = 0;

    flag = true;

    mspm0_get_clock_ms(&last_time);

    // 激光笔坐标 + 目标坐标
    Pto_Clear_CMD_Flag();

    DL_UART_Main_getErrorStatus(K230_INST, DL_UART_ERROR_OVERRUN);

    target_succeed_num = 0;

    for (uint8_t i = 0; i < 6; ++i) {
      recv_data[i] = 0;
    }

    for (;;) {
      i = 0;

      mspm0_get_clock_ms(&curr_time);
      if (curr_time - last_time > 4000) {
        break;
      }

      // 由于 recv_data 初始化为
      // 0，一旦收到靶消息后，就永远不会为零，前提是要保证对面不会发零
      if (recv_data[0] == 0 || recv_data[1] == 0 || recv_data[2] == 0 ||
          recv_data[3] == 0) {
        SMotor_Turn_Degree(&smotor_hor, 5); // 顺时针转
        mspm0_delay_ms(60);
      }

      // 协议接受
      if (!New_CMD_flag) {
        continue;
      }

      Pto_Data_Parse(RxBuffer, New_CMD_length, recv_data);

      Pto_Clear_CMD_Flag();

      if (flag) { // 消除发送的第一个混乱数据
        smotor_hor.deg = 0;
        smotor_ver.deg = 0;
        flag = false;
        continue;
      }

      laser_pos.x = recv_data[0];
      laser_pos.y = recv_data[1];

      target_pos.x = recv_data[2];
      target_pos.y = recv_data[3];

      if (Distance(target_pos, laser_pos) < 2) {
        ++target_succeed_num;
        if (target_succeed_num > target_succeed_max) {
          target_succeed_num = 0;
          break;
        }
        mspm0_delay_ms(50);
        continue;
      }

      target_succeed_num = 0;

      // 清除串口错误
      DL_UART_Main_getErrorStatus(K230_INST, DL_UART_ERROR_OVERRUN);

      sprintf(oled_buffer, "Tpos:%4d,%4d     ", target_pos.x, target_pos.y);
      OLED_ShowString(0, i++, (uint8_t *)oled_buffer, 8);

      sprintf(oled_buffer, "Lpos:%4d,%4d    ", laser_pos.x, laser_pos.y);
      OLED_ShowString(0, i++, (uint8_t *)oled_buffer, 8);

      // pid 调整
      SMotor_Turn_To(&smotor_hor,
                     PID_Loc(&hori_laser_pid, target_pos.x, laser_pos.x, 1));
      SMotor_Turn_To(&smotor_ver,
                     PID_Loc(&vert_laser_pid, target_pos.y, laser_pos.y, 1));

      sprintf(oled_buffer, "Sdeg:%4.2f,%4.2f    ", smotor_hor.deg,
              smotor_ver.deg);
      OLED_ShowString(0, i++, (uint8_t *)oled_buffer, 8);

      mspm0_delay_ms(50);
    }

    DL_GPIO_setPins(LASER_PORT, LASER_PURPLE_PIN);
    Beep(300);
    mspm0_delay_ms(3700);
    DL_GPIO_clearPins(LASER_PORT, LASER_PURPLE_PIN);
    OLED_Clear();
    break;

  case 6: // 发挥一
          /*
          思路 1：可能不会有思路 2 了。
          走直线用 pid。maybe预测？，转弯用陀螺仪。
          问题：1.转弯陀螺仪如何动？2.转弯如何保证扫到线？3.找不到靶。
          解决方案：1.保持同一个朝向。2.循迹调慢。3.关激光笔。
          */

    OLED_Clear();
    // 接收 k230 进行步进电机 pid
    Pid_Clear(&vert_laser_pid);
    Pid_Clear(&hori_laser_pid);
    vert_laser_pid.kp = 0.07;
    vert_laser_pid.ki = 0.03;
    vert_laser_pid.kd = 0.01;
    vert_laser_pid.dead_zone = 0;
    vert_laser_pid.max_integral = 1000;
    vert_laser_pid.max_output = 360;
    vert_laser_pid.output = 0;
    vert_laser_pid.integral = 0;
    vert_laser_pid.last_err = 0;
    vert_laser_pid.last_derivative = 0;

    hori_laser_pid.kp = 0.07;
    hori_laser_pid.ki = 0.03;
    hori_laser_pid.kd = 0.01;
    hori_laser_pid.dead_zone = 0;
    hori_laser_pid.max_integral = 1000;
    hori_laser_pid.max_output = 360;
    hori_laser_pid.output = 0;
    hori_laser_pid.integral = 0;
    vert_laser_pid.last_err = 0;
    hori_laser_pid.last_derivative = 0;

    flag = true; // 状态机中只更新一遍

    // 激光笔坐标 + 目标坐标
    Pto_Clear_CMD_Flag();

    DL_UART_Main_getErrorStatus(K230_INST, DL_UART_ERROR_OVERRUN);

    // 打开激光笔
    DL_GPIO_setPins(LASER_PORT, LASER_PURPLE_PIN);

    // mspm0_get_clock_ms(&last_time);

    round_num = 0;

    for (;;) {
      i = 0;
      mspm0_delay_ms(30); // 每轮延时

      // 循迹
      Trace_Read_Bit(); // 读取循迹状态

      // 显示
      sprintf(oled_buffer, "Tsum:%1d%1d%1d%1d%1d", trace_states[0],
              trace_states[1], trace_states[2], trace_states[3],
              trace_states[4]);
      OLED_ShowString(0, i++, (uint8_t *)oled_buffer, 8);

      // 得到循迹状态的加权和
      trace_sum = Get_Trace_Sum(trace_states);

      // 根据加权和进入不同状态
      if (trace_sum == STOP) {
        break;
      } else if (trace_sum == TURN_LEFT) {
        gyro_hori_deg = wit_data.yaw; // 获取陀螺仪 yaw 角
        Motor_Turn_Left_1();
        ++round_num;
        // 这里需要两个循环，合二为一
        mspm0_get_clock_ms(&last_time);
        while (1) {
          delta = Get_Shortest_Arc(wit_data.yaw, gyro_hori_deg);

          // 角落永远顺时针转，所以 delta 算出来为负
          // 角度多加几度，可能有用
          SMotor_Turn_Degree(&smotor_hor, -0.1 * delta + 0.1);

          mspm0_delay_ms(50);
          mspm0_get_clock_ms(&curr_time);
          if (curr_time - last_time > 800) {
            Trace_Read_Bit();
            if (trace_states[2] == BLACK || trace_states[3] == BLACK) {
              break;
            }
          }
        }
        break;
      } else if (trace_sum < 0) {
        Motor_Modify_Left(abs(trace_sum));
      } else if (trace_sum > 0) {
        Motor_Modify_Right(abs(trace_sum));
      } else {
        Motor_Walk_Strt();
      }

      // SMotor_Turn_Degree(&smotor_hor, 0.5);

      // 协议接收
      if (!New_CMD_flag) {
        continue;
      }

      Pto_Data_Parse(RxBuffer, New_CMD_length, recv_data);

      Pto_Clear_CMD_Flag();

      if (recv_data[0] == 0 || recv_data[1] == 0 || recv_data[2] == 0 ||
          recv_data[3] == 0) {
        continue;
      }

      laser_pos.x = recv_data[0];
      laser_pos.y = recv_data[1];

      target_pos.x = recv_data[2];
      target_pos.y = recv_data[3];

      if (flag) {
        flag = false;
        last_pos = target_pos;
      }

      // if (Distance(last_pos, target_pos) > 100) { // 防止误识别
      //   target_pos = last_pos;
      // }

      // 根据变化调整下一刻 PID 的值，可能有用
      temp_pos.x = target_pos.x + 0.9 * (target_pos.x - last_pos.x);

      last_pos = target_pos; // 保留上一个值

      target_pos.x = temp_pos.x;

      // 清除串口错误
      DL_UART_Main_getErrorStatus(K230_INST, DL_UART_ERROR_OVERRUN);

      // 考虑注释 OLED，防止未检测到拐点
      sprintf(oled_buffer, "Tpos:%4d,%4d     ", target_pos.x, target_pos.y);
      OLED_ShowString(0, i++, (uint8_t *)oled_buffer, 8);

      sprintf(oled_buffer, "Lpos:%4d,%4d    ", laser_pos.x, laser_pos.y);
      OLED_ShowString(0, i++, (uint8_t *)oled_buffer, 8);

      SMotor_Turn_Degree(&smotor_hor, 0.09 * (target_pos.x - laser_pos.x));

      // pid 调整
      // SMotor_Turn_To(&smotor_hor,
      //                PID_Loc(&hori_laser_pid, target_pos.x, laser_pos.x, 1));
      // SMotor_Turn_To(&smotor_ver,
      //                PID_Loc(&vert_laser_pid, target_pos.y, laser_pos.y, 1));

      sprintf(oled_buffer, "Sdeg:%4.2f,%4.2f    ", smotor_hor.deg,
              smotor_ver.deg);
      OLED_ShowString(0, i++, (uint8_t *)oled_buffer, 8);
    }

    DL_GPIO_clearPins(LASER_PORT, LASER_PURPLE_PIN);
    Motor_Stop_All();
    Beep(300);
    OLED_Clear();
    break;

  case 9:
    OLED_Clear();
    page_state = 0;
    break;
  default:
    break;
  }
}

void Page_Debug() {
  uint8_t i = 0;
  OLED_ShowString(0, i++, (uint8_t *)"Debug Page", 8);
  OLED_ShowString(0, i++, (uint8_t *)"[1]Step Calib", 8);
  OLED_ShowString(0, i++, (uint8_t *)"[2]Gyro Calib", 8);
  OLED_ShowString(0, i++, (uint8_t *)"[3]Toggle Laser", 8);

  key = Scan_Keyboard();
  switch (key) {
  case 1:
    OLED_Clear();
    page_state = 4;
    break;
  case 2:
    OLED_Clear();
    page_state = 5;
    break;
  case 3:
    DL_GPIO_togglePins(LASER_PORT, LASER_PURPLE_PIN);
    while (Scan_Keyboard())
      ;
    break;
  case 4:
    OLED_Clear();
    while (1) {
      i = 0;
      Trace_Read_Bit(); // 读取循迹状态

      // 显示
      sprintf(oled_buffer, "Tsum:%1d%1d%1d%1d%1d", trace_states[0],
              trace_states[1], trace_states[2], trace_states[3],
              trace_states[4]);
      OLED_ShowString(0, i++, (uint8_t *)oled_buffer, 8);

      key = Scan_Keyboard();
      if (key == 9) {
        while (Scan_Keyboard())
          ;
        break;
      }
    }
    break;
  case 9:
    OLED_Clear();
    page_state = 0;
    break;
  default:
    break;
  }

  // Pto_Clear_CMD_Flag();
  DL_UART_Main_getErrorStatus(K230_INST, DL_UART_ERROR_OVERRUN);
  if (New_CMD_flag) {
    // 清除串口错误
    Pto_Data_Parse(RxBuffer, New_CMD_length, recv_data);
    Pto_Clear_CMD_Flag();

    sprintf(oled_buffer, "Tpos:%4d,%4d     ", recv_data[2], recv_data[3]);
    OLED_ShowString(0, i++, (uint8_t *)oled_buffer, 8);

    sprintf(oled_buffer, "Lpos:%4d,%4d    ", recv_data[0], recv_data[1]);
    OLED_ShowString(0, i++, (uint8_t *)oled_buffer, 8);
  }
}

void Page_Debug_Step() {
  uint8_t i = 0;
  sprintf(oled_buffer, "Sdeg:%4.2f ,%4.2f    ", smotor_hor.deg, smotor_ver.deg);
  OLED_ShowString(0, i++, (uint8_t *)oled_buffer, 8);

  sprintf(oled_buffer, "Smax:%4d,%4d   ", smotor_hor.rot_max,
          smotor_ver.rot_max);
  OLED_ShowString(0, i++, (uint8_t *)oled_buffer, 8);

  sprintf(oled_buffer, "Scnt:%4d,%4d    ", smotor_hor.rot_cnt,
          smotor_ver.rot_cnt);
  OLED_ShowString(0, i++, (uint8_t *)oled_buffer, 8);

  key = Scan_Keyboard();
  switch (key) {
  // 测试水平电机
  case 1:
    SMotor_Turn_Degree(&smotor_hor, 0.1); // 顺
    break;
  case 2:
    SMotor_Turn_Degree(&smotor_hor, -0.1); // 逆
    break;
  case 3:
    SMotor_Turn_Degree(&smotor_ver, 0.1); // 下
    break;
  case 4:
    SMotor_Turn_Degree(&smotor_ver, -0.1); // 上
    break;
  case 5:
    smotor_hor.deg = 0;
    smotor_ver.deg = 0;
    break;
  case 9:
    while (Scan_Keyboard())
      ;
    OLED_Clear();
    page_state = 2;
    break;
  default:
    break;
  }
}

void Page_Debug_Gyro() {
  uint8_t i = 0;
  sprintf((char *)oled_buffer, "Pich:%-5.2f", wit_data.pitch);
  OLED_ShowString(0, i++, (uint8_t *)oled_buffer, 8);
  sprintf((char *)oled_buffer, "Roll:%-5.2f", wit_data.roll);
  OLED_ShowString(0, i++, (uint8_t *)oled_buffer, 8);
  sprintf(oled_buffer, "Yaw :%-5.2f", wit_data.yaw);
  OLED_ShowString(0, i++, (uint8_t *)oled_buffer, 8);

  sprintf((char *)oled_buffer, "Hori:%-5.2f", gyro_hori_deg);
  OLED_ShowString(0, i++, (uint8_t *)oled_buffer, 8);
  sprintf((char *)oled_buffer, "Vert:%-5.2f", gyro_vert_deg);
  OLED_ShowString(0, i++, (uint8_t *)oled_buffer, 8);

  key = Scan_Keyboard();
  switch (key) {
  case 1: // 校准
    gyro_hori_deg = wit_data.yaw;
    gyro_vert_deg = wit_data.roll;
    break;

  case 2: // 移动电机使陀螺仪到位
    OLED_Clear();
    while (Scan_Keyboard())
      ;

    vert_laser_pid.kp = 0.07;
    vert_laser_pid.ki = 0.03;
    vert_laser_pid.kd = 0.01;
    vert_laser_pid.dead_zone = 0;
    vert_laser_pid.max_integral = 1000;
    vert_laser_pid.max_output = 20;
    vert_laser_pid.output = 0;
    vert_laser_pid.integral = 0;
    vert_laser_pid.last_err = 0;
    vert_laser_pid.last_derivative = 0;

    hori_laser_pid.kp = 0.07;
    hori_laser_pid.ki = 0.03;
    hori_laser_pid.kd = 0.01;
    hori_laser_pid.dead_zone = 0;
    hori_laser_pid.max_integral = 1000;
    hori_laser_pid.max_output = 20;
    hori_laser_pid.output = 0;
    hori_laser_pid.integral = 0;
    vert_laser_pid.last_err = 0;
    hori_laser_pid.last_derivative = 0;

    if (gyro_hori_deg == 0 && gyro_vert_deg == 0) {
      Beep(500);
      break;
    }

    Beep(300);
    while (1) {
      delta = Get_Shortest_Arc(wit_data.yaw,
                               gyro_hori_deg); // delta 算的是逆时针的值
      i = 0;
      sprintf(oled_buffer, "Pich:%-5.2f", wit_data.pitch);
      OLED_ShowString(0, i++, (uint8_t *)oled_buffer, 8);
      sprintf(oled_buffer, "Roll:%-5.2f", wit_data.roll);
      OLED_ShowString(0, i++, (uint8_t *)oled_buffer, 8);
      sprintf(oled_buffer, "Yaw :%-5.2f", wit_data.yaw);
      OLED_ShowString(0, i++, (uint8_t *)oled_buffer, 8);

      sprintf(oled_buffer, "Hori:%-5.2f", gyro_hori_deg);
      OLED_ShowString(0, i++, (uint8_t *)oled_buffer, 8);
      sprintf(oled_buffer, "Vert:%-5.2f", gyro_vert_deg);
      OLED_ShowString(0, i++, (uint8_t *)oled_buffer, 8);
      sprintf(oled_buffer, "Delta:%-5.2f", delta);
      OLED_ShowString(0, i++, (uint8_t *)oled_buffer, 8);

      if (fabs(delta) < 3) {
        break;
      }

      key = 0;
      while (!key) {
        key = Scan_Keyboard();
      }

      // Beep(300);
      // key = 1，逆时针转，key = 2，顺时针转，如果是其他按键，则退出程序
      if (key == 1) {    // 需要逆时转
        if (delta < 0) { // 说明角度是顺时针
          delta += 360;
        }
      } else if (key == 2) { // 需要顺时转
        if (delta > 0) {     // 说明角度是逆时针
          delta -= 360;
        }
      } else {
        break;
      }
      SMotor_Turn_Degree(&smotor_hor, -0.05 * delta);
    };
    Beep(500);

    while (Scan_Keyboard())
      ;

    OLED_Clear();
    break;
  case 9:
    while (Scan_Keyboard())
      ;

    OLED_Clear();
    page_state = 2;
    break;
  default:
    break;
  }
}

void K230_INST_IRQHandler(void) {
  switch (DL_UART_Main_getPendingInterrupt(K230_INST)) {
  case DL_UART_MAIN_IIDX_RX:
    echo_data = DL_UART_Main_receiveDataBlocking(K230_INST);
    Pto_Data_Receive(echo_data);
    // DL_UART_Main_transmitDataBlocking(PC_INST, echo_data);
    break;
  default:
    break;
  }
}