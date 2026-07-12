#ifndef _PAGES_H_
#define _PAGES_H_

#include "ti_msp_dl_config.h"
#include "ZDT_X42S/zdt_x42s.h"
#include "oled_hardware_i2c.h"

extern uint8_t page_state;
extern uint8_t key;
extern ZDT_HandleTypeDef motor1;
extern ZDT_HandleTypeDef motor2;

void Page_Home(void);
void Page_Calib(void);
void Page_Debug(void);
void Page_Test(void);
void Page_Test2(void);
void Page_Debug_Step(void);
void Page_Debug_Gyro(void);
void Page_Debug_Camera(void);
void Page_CurveDebug(void);

#endif /* _PAGES_H_ */
