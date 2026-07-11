/*
 * sine_test.h
 *
 * 步进电机正弦波绘制测试
 * 进入 Test → Motor Test 后调用 SineTest_Tick(key)
 * 返回 1 表示用户请求返回菜单
 */

#ifndef BSP_TEST_SINE_TEST_H_
#define BSP_TEST_SINE_TEST_H_

#include "ti_msp_dl_config.h"

uint8_t SineTest_Tick(uint8_t key);  /* 返回 1=退出 0=继续 */

#endif
