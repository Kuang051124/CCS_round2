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
#include "ti_msp_dl_config.h"
#include "pages.h"

#include <stdio.h>
#include <math.h>
#include <string.h>

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
    tk_param        = &t3;
    tk_gyro_src     = GYRO_SRC_WIT;
    tk_speed_mult   = T4A_AO_MULT;      /* A→O 段慢速等B超 */
    tk_heading_bias = 0.0f;
    tk_seg_delay_ms = 0;
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

    /* ==== 按段设置速度倍率 + 延时 ==== */

    if (seg == 0) {
        /* A→O: 慢速, 等B超车 */
        tk_speed_mult   = T4A_AO_MULT;
        tk_seg_delay_ms = T4_AO_DELAY_MS;
    } else if (seg == 1) {
        /* O→B: 慢速, B已在前方 */
        tk_speed_mult   = T4A_OB_MULT;
        tk_seg_delay_ms = T4_OB_DELAY_MS;
    } else if (seg == 2) {
        /* BC弧: 正常巡线, B在前 */
        tk_speed_mult   = T4A_BC_MULT;
        tk_seg_delay_ms = 0;
    } else if (seg == 3) {
        /* C→D: 快速追赶B (B绕O走C→O→D) */
        tk_speed_mult   = T4A_CD_MULT;
        tk_seg_delay_ms = 0;
    } else if (seg == 4) {
        /* DA弧: 快速, 回A终点 */
        tk_speed_mult   = T4A_DA_MULT;
        tk_seg_delay_ms = 0;
    }

    /* ==== OLED 状态 ==== */
    {
        char buf[21];
        sprintf(buf, "Seg:%d Spd:%.2f", seg, tk_speed_mult);
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
    tk_param        = &t3;
    tk_gyro_src     = GYRO_SRC_WIT;
    tk_speed_mult   = T4B_DA_MULT;      /* DA弧初始追赶 */
    tk_heading_bias = 0.0f;
    tk_seg_delay_ms = 0;
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

    /* ==== 按段设置速度倍率 + 延时 ==== */

    if (seg == 0) {
        /* DA弧: 快速追A */
        tk_seg_delay_ms = 0;
        tk_speed_mult   = T4B_DA_MULT;
    } else if (seg == 1) {
        /* A→B: 快速超A (A绕O走A→O→B) */
        tk_seg_delay_ms = 0;
        tk_speed_mult   = T4B_AB_MULT;
    } else if (seg == 2) {
        /* BC弧: B在前, 正常巡线 */
        tk_speed_mult   = T4B_BC_MULT;
        tk_seg_delay_ms = 0;
    } else if (seg == 3) {
        /* C→O: 慢速绕O, 等A超车 */
        tk_speed_mult   = T4B_CO_MULT;
        tk_seg_delay_ms = T4_CO_DELAY_MS;
    } else if (seg == 4) {
        /* O→D: 慢速, 到D终点 */
        tk_speed_mult   = T4B_OD_MULT;
        tk_seg_delay_ms = T4_OD_DELAY_MS;
    }

    /* ==== OLED 状态 ==== */
    {
        char buf[21];
        sprintf(buf, "Seg:%d Spd:%.2f", seg, tk_speed_mult);
        OLED_ShowString(0, 6, (uint8_t *)buf, 8);
    }

    /* 任务完成 → 重置标志 */
    if (tk_is_done()) {
        return 1;
    }

    return task_tick(key);
}
