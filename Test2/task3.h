/**
 * @file  task3.h
 * @brief Task 3: 双车协同 — A车慢速走Task2路径, B车跟随+距离P控制
 */

#ifndef _TASK3_H_
#define _TASK3_H_

#include "ti_msp_dl_config.h"

/* ===================================================================
 * 距离 P 控制参数
 * =================================================================== */

#define T3_DIST_TARGET      30.0f   /* 目标距离 (cm)          */
#define T3_DIST_KP          0.02f  /* P 增益 (倍率/cm误差)   */
#define T3_SPEED_INIT       1.4f   /* 初始追赶倍率 (>1)      */
#define T3_SPEED_MIN        0.50f   /* 最小速度倍率           */
#define T3_SPEED_MAX        1.60f   /* 最大速度倍率           */

/* ===================================================================
 * API
 * =================================================================== */

/* Car A (WIT) */
void Task3A_Init(void);
uint8_t Task3A_Tick(uint8_t key);

/* Car B (MPU + 摄像头距离P控制) */
void Task3B_Init(void);
uint8_t Task3B_Tick(uint8_t key);

#endif
