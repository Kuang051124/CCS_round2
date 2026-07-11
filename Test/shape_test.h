/*
 * shape_test.h
 *
 * 简单图形绘制 (速度模式)
 * 进入 Test → Shape Draw 后调用 ShapeTest_Tick(key)
 * 返回 1 表示用户请求返回菜单
 */

#ifndef BSP_TEST_SHAPE_TEST_H_
#define BSP_TEST_SHAPE_TEST_H_

#include "ti_msp_dl_config.h"

uint8_t ShapeTest_Tick(uint8_t key);  /* 返回 1=退出 0=继续 */

#endif
