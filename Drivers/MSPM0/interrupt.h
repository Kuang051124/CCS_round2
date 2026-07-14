#ifndef _INTERRUPT_H_
#define _INTERRUPT_H_
#include "ti_msp_dl_config.h"

extern uint8_t enable_group1_irq;

/* 外环同步标志: TIMG0 每 20ms (内环第 2 次中断) 置 1,
 * 主循环消费后清零。用于将外环锁定在 50Hz, 与内环 100Hz 保持 2:1 同步。 */
extern volatile uint8_t g_outer_loop_ready;

void Interrupt_Init(void);

#endif  /* #ifndef _INTERRUPT_H_ */