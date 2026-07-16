/**
 * @file  task4.h
 * @brief Task 4: 双车相向超车 — 过中心O路线交叉方案
 *
 * A车: A→O(延时)→B→BC弧→C→D(直)→DA弧→A
 * B车: D→DA弧→A→B(直)→BC弧→C→O(延时)→D
 *
 * 超车原理:
 *   A→B段: A走A→O→B(绕O, 路程长+慢速), B走DA弧+A→B(直线, 路程短+快速) → B超A
 *   C→D段: B走C→O→D(绕O, 路程长+慢速), A走C→D+DA弧(直线, 路程短+快速) → A超B
 *
 * 延时段 (SEG_STRAIGHT_DELAY): A→O, O→B, C→O, O→D  编码器计数值判定终点 (T4_ENCODER_TARGET=2100)
 */

#ifndef _TASK4_H_
#define _TASK4_H_

#include "ti_msp_dl_config.h"
#include "mode1.h"

/* ===================================================================
 * 编码器到位参数
 * =================================================================== */

/* 编码器到位阈值: DELAY段独立可调 */
#define T4_ENC_AO           2100    /* A车 A→O 到位计数 */
#define T4_ENC_CO           1700    /* B车 C→O 到位计数 */
#define T4_ENC_DSTOP         300    /* B车 循迹到D后多走300计数停车 */
#define T4_STOP_MS           100     /* B车 C点停车等待时长 (ms) */

/* ===================================================================
 * Task4 每段独立参数 — 替代旧的 tk_speed_mult 间接调速
 * 字段: spd_straight, spd_arc, spd_arc_kp, gyro_kp, arc_ofs_cw, arc_ofs_ccw, arc_ofs_cw_t2, arc_ofs_ccw_t2
 * =================================================================== */

/* ---- A车 5段 ---- */
extern T1Param t4_AO;    /* A→O 慢速DELAY直 */
extern T1Param t4_OB;    /* O→B 慢速DELAY直 */
extern T1Param t4_ABC;   /* BC弧 正常巡线   */
extern T1Param t4_ACD;   /* C→D 快速直     */
extern T1Param t4_ADA;   /* DA弧 快速回A    */

/* ---- B车 5段 ---- */
extern T1Param t4_BDA;   /* DA弧 快速追A    */
extern T1Param t4_BAB;   /* A→B 快速超A    */
extern T1Param t4_BBC;   /* BC弧 正常巡线   */
extern T1Param t4_BCO;   /* C→O 慢速DELAY  */
extern T1Param t4_BOD;   /* O→D 慢速DELAY  */
extern T1Param t4_CSTOP; /* C点停车等待     */

/* ===================================================================
 * API
 * =================================================================== */

void Task4A_Init(void);
uint8_t Task4A_Tick(uint8_t key);

void Task4B_Init(void);
uint8_t Task4B_Tick(uint8_t key);

#endif
