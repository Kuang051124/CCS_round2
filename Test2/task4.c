/**
 * @file  task4.c
 * @brief Task 4: 双车相向超车 — 过中心O路线交叉方案
 *
 * 轨道示意:
 *          A ────→ O ────→ B
 *          ↑                 │
 *         DA弧             BC弧
 *          ↑                 ↓
 *          D ←──── O ←──── C
 *
 * A车路径: A→O(延时)→B→BC弧→C→D(直)→DA弧→A   (5段)
 * B车路径: D→DA弧→A→B(直)→BC弧→C→O(延时)→D   (5段)
 *
 * 超车原理:
 *   Phase 1 (A→B段, B超A):
 *     A: A→O→B  绕O, 2段直线, 慢速 (T4A_AO/OB_MULT)
 *     B: DA弧→A→B  1弧+1直, 快速 (T4B_DA/AB_MULT)
 *     → B路径更短+速度更快 → B超A
 *
 *   Phase 2 (BC弧):
 *     两车同向走BC弧, B在前A在后, 正常巡线 (T4A_BC_MULT / T4B_BC_MULT)
 *
 *   Phase 3 (C→D段, A超B):
 *     B: C→O→D  绕O, 2段直线, 慢速 (T4B_CO/OD_MULT)
 *     A: C→D→DA弧  1直+1弧, 快速 (T4A_CD/DA_MULT)
 *     → B路径更长+速度更慢 → A超B, A回到终点
 */

#include "task4.h"
#include "mode1.h"
#include "../Drivers/WIT/wit.h"
#include "../Drivers/MSPM0/clock.h"
#include "../Keyboard/keyboard.h"
#include "../BLUETOOTH/bluetooth.h"
#include "../yb_protocol/yb_protocol.h"
#include "../ENCODER/encoder.h"
#include "ti_msp_dl_config.h"
#include "pages.h"

#include <stdio.h>
#include <math.h>
#include <string.h>

/* ===================================================================
 * Task4 每段独立参数 (8组)
 * 字段: spd_straight, spd_arc, spd_arc_kp, gyro_kp, arc_ofs_cw, arc_ofs_ccw, arc_ofs_cw_t2, arc_ofs_ccw_t2
 *
 * 基准:
 *   t1 = { 0.28f, 0.22f, 0.05f, 0.01f, 0.05f, -0.05f, 0.20f, -0.20f }
 *   t3 = { 0.14f, 0.12f, 0.02f, 0.01f, 0.012f, -0.012f, 0.07f, -0.07f }
 * =================================================================== */

/* A车 — 5段 */
T1Param t4_AO  = { 0.10f, 0.10f, 0.02f, 0.005f, 0.012f, -0.012f, 0.07f, -0.07f };  /* A→O 慢速DELAY */
T1Param t4_OB  = { 0.10f, 0.10f, 0.02f, 0.005f, 0.012f, -0.012f, 0.07f, -0.07f };  /* O→B 慢速DELAY */
T1Param t4_ABC = { 0.14f, 0.12f, 0.02f, 0.01f, 0.012f, -0.012f, 0.07f, -0.07f };  /* BC弧 正常巡线 */
T1Param t4_ACD = { 0.28f, 0.22f, 0.05f, 0.01f, 0.05f, -0.05f, 0.2f, -0.2f };  /* C→D 快速直   */
T1Param t4_ADA = { 0.28f, 0.22f, 0.05f, 0.01f, 0.05f, -0.05f, 0.2f, -0.2f };  /* DA弧 快速回A  */

/* B车 — 5段 */
T1Param t4_BDA = { 0.28f, 0.22f, 0.05f, 0.01f, 0.05f, -0.05f, 0.2f, -0.2f };  /* DA弧 快速追A  */
T1Param t4_BAB = { 0.28f, 0.22f, 0.05f, 0.01f, 0.05f, -0.05f, 0.2f, -0.2f };  /* A→B 快速超A  */
T1Param t4_BBC = { 0.14f, 0.12f, 0.02f, 0.01f, 0.012f, -0.012f, 0.07f, -0.07f };  /* BC弧 正常巡线 */
T1Param t4_BCO = { 0.10f, 0.10f, 0.02f, 0.005f, 0.012f, -0.012f, 0.07f, -0.07f };  /* C→O 慢速DELAY */
T1Param t4_BOD = { 0.10f, 0.10f, 0.02f, 0.005f, 0.012f, -0.012f, 0.07f, -0.07f };  /* O→D 慢速DELAY */

/* ===================================================================
 * Car A — A→O→B→BC→C→D→DA→A
 *
 *   Seg 0 (A→O): SEG_STRAIGHT_DELAY, 慢速, 等B超车
 *   Seg 1 (O→B): SEG_STRAIGHT_DELAY, 慢速
 *   Seg 2 (BC弧): 正常巡线, B已在前
 *   Seg 3 (C→D): 追赶B (B绕O), 摄像头P控
 *   Seg 4 (DA弧): 继续追B/正常, 回到A终点
 * =================================================================== */

static uint8_t a_bt_sent = 0;

void Task4A_Init(void)
{
    tk_param        = &t4_AO;           /* A→O 段慢速等B超 */
    tk_gyro_src     = GYRO_SRC_WIT;
    tk_speed_mult   = 1.0f;             /* 不再使用倍率, 直接切换参数 */
    tk_heading_bias = 0.0f;
    a_bt_sent       = 0;
    task_init(task4a_path, TASK4A_SEG_COUNT);

    Pto_Clear_CMD_Flag();

    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Task4A: Car A", 8);
    OLED_ShowString(0, 1, (uint8_t *)"A-O-B-BC-C-D-DA-A", 8);
    OLED_ShowString(0, 2, (uint8_t *)"O-Cross Ovtk", 8);
    OLED_ShowString(0, 3, (uint8_t *)"[5] Start [9] Exit", 8);
    OLED_ShowString(0, 5, (uint8_t *)"A-O-B: slow T4A*", 8);
}

uint8_t Task4A_Tick(uint8_t key)
{
    uint8_t seg = tk_get_seg_index();

    /* 按5启动 → 蓝牙通知B车 */
    if (key == 5 && !a_bt_sent) {
        Bluetooth_SendString("TASK4\r\n");
        a_bt_sent = 1;
    }

    /* ==== 按段切换参数 (替代旧的 tk_speed_mult) ==== */

    if (seg == 0) {
        tk_param = &t4_AO;       /* A→O 慢速DELAY */
    } else if (seg == 1) {
        tk_param = &t4_OB;       /* O→B 慢速DELAY */
    } else if (seg == 2) {
        tk_param = &t4_ABC;      /* BC弧 正常巡线 */
    } else if (seg == 3) {
        tk_param = &t4_ACD;      /* C→D 快速直   */
    } else if (seg == 4) {
        tk_param = &t4_ADA;      /* DA弧 快速回A  */
    }

    /* ==== OLED 状态 ==== */
    {
        char buf[21];
        int32_t l   = ENCODER_GetLeftCount();
        int32_t r   = ENCODER_GetRightCount();
        int32_t avg = (l + r) / 2;

        sprintf(buf, "L:%+5ld R:%+5ld", (long)l, (long)r);
        OLED_ShowString(0, 3, (uint8_t *)buf, 8);
        sprintf(buf, "Avg:%5ld T:%d", (long)avg, T4_ENCODER_TARGET);
        OLED_ShowString(0, 4, (uint8_t *)buf, 8);
        sprintf(buf, "Seg:%d Str:%.2f Arc:%.2f", seg, tk_param->spd_straight, tk_param->spd_arc);
        OLED_ShowString(0, 6, (uint8_t *)buf, 8);
    }

    /* 任务完成 → 重置标志 */
    if (tk_is_done()) {
        a_bt_sent = 0;
        return 1;
    }

    return task_tick(key);
}

/* ===================================================================
 * Car B — D→DA→A→B→BC→C→O→D
 *
 *   Seg 0 (DA弧): 追赶A, 摄像头P控
 *   Seg 1 (A→B):  追赶A, 超车 (A绕O)
 *   Seg 2 (BC弧): B在前, 正常巡线
 *   Seg 3 (C→O): SEG_STRAIGHT_DELAY, 慢速, 等A超车
 *   Seg 4 (O→D): SEG_STRAIGHT_DELAY, 慢速, 到D终点
 * =================================================================== */

void Task4B_Init(void)
{
    tk_param        = &t4_BDA;          /* DA弧初始追赶 */
    tk_gyro_src     = GYRO_SRC_WIT;
    tk_speed_mult   = 1.0f;             /* 不再使用倍率 */
    tk_heading_bias = 0.0f;
    task_init(task4b_path, TASK4B_SEG_COUNT);

    Pto_Clear_CMD_Flag();

    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Task4B: Car B", 8);
    OLED_ShowString(0, 1, (uint8_t *)"D-DA-A-B-BC-C-O-D", 8);
    OLED_ShowString(0, 2, (uint8_t *)"O-Cross Ovtk", 8);
    OLED_ShowString(0, 3, (uint8_t *)"BT Start", 8);
    OLED_ShowString(0, 5, (uint8_t *)"Chasing A ...", 8);
}

uint8_t Task4B_Tick(uint8_t key)
{
    uint8_t seg = tk_get_seg_index();

    /* ==== 按段切换参数 (替代旧的 tk_speed_mult) ==== */

    if (seg == 0) {
        tk_param = &t4_BDA;      /* DA弧 快速追A */
    } else if (seg == 1) {
        tk_param = &t4_BAB;      /* A→B 快速超A */
    } else if (seg == 2) {
        tk_param = &t4_BBC;      /* BC弧 正常巡线 */
    } else if (seg == 3) {
        tk_param = &t4_BCO;      /* C→O 慢速DELAY */
    } else if (seg == 4) {
        tk_param = &t4_BOD;      /* O→D 慢速DELAY */
    }

    /* ==== OLED 状态 ==== */
    {
        char buf[21];
        int32_t l   = ENCODER_GetLeftCount();
        int32_t r   = ENCODER_GetRightCount();
        int32_t avg = (l + r) / 2;

        sprintf(buf, "L:%+5ld R:%+5ld", (long)l, (long)r);
        OLED_ShowString(0, 3, (uint8_t *)buf, 8);
        sprintf(buf, "Avg:%5ld T:%d", (long)avg, T4_ENCODER_TARGET);
        OLED_ShowString(0, 4, (uint8_t *)buf, 8);
        sprintf(buf, "Seg:%d Str:%.2f Arc:%.2f", seg, tk_param->spd_straight, tk_param->spd_arc);
        OLED_ShowString(0, 6, (uint8_t *)buf, 8);
    }

    /* 任务完成 → 重置标志 */
    if (tk_is_done()) {
        return 1;
    }

    return task_tick(key);
}
