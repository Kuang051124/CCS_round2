#include "ti_msp_dl_config.h"
#ifndef _KEYBOARD_H_
#define _KEYBOARD_H_

#define KBD_ROW_NUM 3
#define KBD_COL_NUM 3

volatile extern uint8_t KeyboardData[KBD_ROW_NUM][KBD_COL_NUM];
extern GPIO_Regs *row_ports[KBD_ROW_NUM];
extern GPIO_Regs *col_ports[KBD_COL_NUM];
volatile extern uint32_t row_pins[KBD_ROW_NUM];
volatile extern uint32_t col_pins[KBD_COL_NUM];

void Keyboard_Init();
uint8_t Scan_Keyboard();

#endif // _KEYBOARD_H_