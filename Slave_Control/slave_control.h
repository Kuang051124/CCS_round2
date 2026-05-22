#ifndef _SLAVE_CONTROL_H_
#define _SLAVE_CONTROL_H_
#include "ti_msp_dl_config.h"

extern uint8_t page_state;
extern uint8_t key;
// 记录拐弯数
extern uint16_t round_num;

void Page_Home();
void Page_Calib();
void Page_Debug();
void Page_Test();
void Page_Debug_Step();
void Page_Debug_Gyro();

#endif // "slave_control.h"