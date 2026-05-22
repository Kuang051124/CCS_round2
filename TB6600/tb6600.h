/*
PUL+：脉冲信号输入正。( CP+ )
PUL-：脉冲信号输入负。( CP- )
DIR+：电机正、反转控制正。
DIR-：电机正、反转控制负。
EN+：电机脱机控制正。
EN-：电机脱机控制负。

共阳极接法：分别将PUL+，DIR+，EN+连接到控制系统的电源上，
如果此电源是+5V则可直接接入，如果此电源大于+5V，则须外部另加限流电阻R，保证给驱动器内部光藕提供8—15mA
的驱动电流。 共阴极接法：分别将
PUL-，DIR-，EN-连接到控制系统的地端；脉冲输入信号通过PUL+接入，方向信号通过DIR+接入，使能信号通过EN+接入。若需限流电阻，限流电阻R的接法取值与共阳极接法相同。
注：EN端可不接，EN有效时电机转子处于自由状态（脱机状态），这时可以手动转动电机转轴。
*/
#include "ti_msp_dl_config.h"
#ifndef _TB6600_MOTOR_
#define _TB6600_MOTOR_

// 共阴极接法
typedef struct Step_Motor {
  uint32_t DIR_pin;         // 方向pin
  GPIO_Regs *DIR_GPIO_Port; // 方向端口
  GPTIMER_Regs *PUL_INST;   // 脉冲
  uint8_t PUL_channel;      // PWM定时器通道
  uint8_t sub_div;          // 细分数
  float step_angle;         // 步距角
  float speed;              // 转每秒
  uint32_t period_cnt;      // 用于记录速度
  uint16_t rot_cnt;         // 旋转计数
  uint16_t rot_max;         // 旋转计数最大值
  float deg;                // 当前转过的角度，归零就把这个归零
} SMotor;
void SMotor_Init();
void SMotor_Set_Speed(SMotor *motor, float speed);
void SMotor_Stop(SMotor *motor);
void SMotor_Start(SMotor *motor);
void SMotor_Turn_Pulse(SMotor *motor, int16_t pulse);
void SMotor_Turn_Degree(SMotor *motor, float deg);
void SMotor_Turn_To(SMotor *motor, float deg);

// void SMotor_Turn_Degree(SMotor *motor, float deg);
// void SMotor_Turn_Cycle(SMotor *motor, int n);

extern SMotor smotor_hor;
extern SMotor smotor_ver;

#endif // !_TB6600_MOTOR_