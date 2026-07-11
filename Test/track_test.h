/*
 * track_test.h
 *
 * 摄像头目标跟踪测试 (位置模式 & 速度模式)
 *
 * 模仿 demo3 任务4: 摄像头发送目标坐标 → PID闭环跟踪 → 到达后保持2秒
 *   [4] TrackPos   — CameraPID_Track()   位置脉冲模式
 *   [5] TrackSpeed — CameraPID_TrackSpeed() 速度环模式
 *
 * 调用: Page_Test → [4]/[5] → [5] 启动跟踪, [8] 退出
 */

#ifndef BSP_TEST_TRACK_TEST_H_
#define BSP_TEST_TRACK_TEST_H_

#include "ti_msp_dl_config.h"

uint8_t TrackPos_Tick(uint8_t key);    /* 位置模式跟踪, 返回 1=退出 */
uint8_t TrackSpeed_Tick(uint8_t key);  /* 速度模式跟踪, 返回 1=退出 */

#endif
