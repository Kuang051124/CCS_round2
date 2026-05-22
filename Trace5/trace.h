#ifndef _TRACE_H_
#define _TRACE_H_
#include "ti_msp_dl_config.h"

#define BLACK 1       // 黑色是1
#define WHITE 0       // 白色是0
#define SENSOR_LEN 5  // 灰度传感器长度为8
#define OFFLINE_LIM 3 // 3帧未检测到则表示脱线
#define BRANCH_LIM 3  // 3帧存在岔路口,则表示真的岔路口

#define PATH_COLOR BLACK // 路径颜色为黑色
#define BACKGROUND WHITE // 背景色为白色

// 巡线状态
#define OFFLINE 100   // 脱线返回值
#define CROSS 101     // 检测到横向直线
#define TURN_LEFT 102 // 检测到左转弯
#define TURN_RIGHT 103 // 检测到右转弯
#define STOP 104      // 停止位的返回值
#define STRAIGHT 0    // 不偏

// 左偏为负，右偏为正
#define BRANCH_L -15 // 存在左岔路口
#define BRANCH_R 15  // 存在右岔路口

extern GPIO_Regs *trace_ports[SENSOR_LEN];
extern uint32_t trace_pins[SENSOR_LEN];

extern uint8_t bit;
extern uint8_t trace_states[SENSOR_LEN];

extern int trace_sum;
extern int gravitys[SENSOR_LEN];

uint8_t Trace_Read_Bit();
int Get_Trace_Sum(uint8_t trace_states[]);

#endif