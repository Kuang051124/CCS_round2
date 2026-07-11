/**
 * @file  task3.c
 * @brief Task 3: 双车协同 — A车慢速领跑 + B车摄像头测距P控跟随
 *
 * 握手协议 (新增):
 *   1. A 按5 → 反复发 "TASK3" → 等 B 回复 "READY" → 启动
 *   2. B 收到 "TASK3" → 立即启动 → 发 "READY" × 5 次
 *   3. A 到终点 → 反复发 "STOP" (B 不回复)
 *
 * Car A (领头): Task2 路径 (A→C→CB→B→D→DA→A), 慢速参数 t3, WIT陀螺仪
 * Car B (跟随): 5段路径 D→DA→A→C→CB→B→D→DA2→(追A等STOP)
 *   → 摄像头 AprilTag 测距 → P 控 tk_speed_mult
 *   → 第5段DA弧永不结束 (yaw判定故意不触发), 等A车STOP
 */

#include "task3.h"
#include "mode1.h"
#include "../Drivers/WIT/wit.h"
#include "../Drivers/MSPM0/clock.h"
#include "../Keyboard/keyboard.h"
#include "../yb_protocol/yb_protocol.h"
#include "../BLUETOOTH/bluetooth.h"
#include "ti_msp_dl_config.h"
#include "pages.h"

#include <stdio.h>
#include <math.h>
#include <string.h>

/* ===================================================================
 * Car A — 握手 + 慢速 Task2
 *
 *   状态: IDLE → HANDSHAKE(发TASK3等READY) → RUNNING → STOPPING(发STOP)
 * =================================================================== */

static uint8_t  a_bt_sent       = 0;   /* 用户已按5, 进入握手阶段     */
static uint8_t  a_ready_rcvd    = 0;   /* 收到B的READY, 任务已启动    */
static uint8_t  a_stop_sent     = 0;   /* 已到达终点, 正在发STOP      */
static uint32_t a_last_send_ms  = 0;   /* 上次蓝牙发送时刻             */
static uint8_t  a_task3_send_cnt = 0;  /* "TASK3" 发送次数 (调试用)   */

void Task3A_Init(void)
{
    tk_param      = &t3;
    tk_gyro_src   = GYRO_SRC_WIT;
    tk_speed_mult = 1.0f;
    task_init(task2_path, TASK2_SEG_COUNT);

    a_bt_sent       = 0;
    a_ready_rcvd    = 0;
    a_stop_sent     = 0;
    a_last_send_ms  = 0;
    a_task3_send_cnt = 0;

    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Task3A: Car A", 8);
    OLED_ShowString(0, 1, (uint8_t *)"A-C-CB-B-D-DA-A", 8);
    OLED_ShowString(0, 2, (uint8_t *)"Slow + Lead", 8);
    OLED_ShowString(0, 3, (uint8_t *)"[5] Start [9] Exit", 8);
    OLED_ShowString(0, 5, (uint8_t *)"Wait B READY...", 8);
}

uint8_t Task3A_Tick(uint8_t key)
{
    /* ---- 按9退出 (任何阶段) ---- */
    if (key == 9) {
        a_bt_sent = 0; a_ready_rcvd = 0; a_stop_sent = 0;
        return 1;
    }

    /* ======== Phase 1: 按5 → 握手 ======== */
    if (key == 5 && !a_bt_sent) {
        a_bt_sent = 1;
        a_last_send_ms = 0;  /* 立即发第一条 */
    }

    if (a_bt_sent && !a_ready_rcvd) {
        /* 检查 B 的 READY */
        if (g_bt_rx_ready) {
            if (strncmp((const char *)g_bt_rx_buf, "READY", 5) == 0) {
                a_ready_rcvd = 1;
                task_tick(5);  /* 启动任务! */
            }
            g_bt_rx_ready = 0; g_bt_rx_len = 0;
        }

        /* 每 500ms 发一次 TASK3 */
        if (!a_ready_rcvd && tick_ms - a_last_send_ms > 500) {
            Bluetooth_SendString("TASK3\r\n");
            a_last_send_ms = tick_ms;
            a_task3_send_cnt++;
        }

        /* OLED: 握手状态 */
        if (!a_ready_rcvd) {
            char buf[21];
            sprintf(buf, "TASK3 sent x%d", a_task3_send_cnt);
            OLED_ShowString(0, 5, (uint8_t *)buf, 8);
            OLED_ShowString(0, 6, (uint8_t *)"Wait B READY...", 8);
            return 0;  /* 握手完成前不走 task_tick */
        }
    }

    /* ======== Phase 2: 运行中 ======== */
    uint8_t ret = task_tick(key);

    /* ======== Phase 3: 到终点 → 反复发 STOP ======== */
    if (tk_is_done() && !a_stop_sent) {
        a_stop_sent    = 1;
        a_last_send_ms = 0;  /* 立即发第一条 */
    }
    if (a_stop_sent) {
        if (tick_ms - a_last_send_ms > 500) {
            Bluetooth_SendString("STOP\r\n");
            a_last_send_ms = tick_ms;
        }
    }

    /* 用户按9退出 → 重置所有标志 */
    if (ret) {
        a_bt_sent = 0; a_ready_rcvd = 0; a_stop_sent = 0;
    }
    return ret;
}

/* ===================================================================
 * Car B — 收到 TASK3 → 启动 + 发 READY×5 → 跟随
 *
 *   pages.c 收到 "TASK3" → Task3B_Init() + task_tick(5) (自动启动)
 *   Task3B_Tick 负责发 READY×5 + 摄像头P控
 * =================================================================== */

static uint8_t  b_ready_cnt     = 0;
static uint32_t b_last_ready_ms = 0;

void Task3B_Init(void)
{
    tk_param      = &t3;
    tk_gyro_src   = GYRO_SRC_WIT;
    tk_speed_mult = T3_SPEED_INIT;  /* 初始加速追赶 A 车 */
    task_init(task3b_path, TASK3B_SEG_COUNT);

    b_ready_cnt     = 0;
    b_last_ready_ms = 0;

    /* 清摄像头标志位 */
    Pto_Clear_CMD_Flag();

    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Task3B: Car B", 8);
    OLED_ShowString(0, 1, (uint8_t *)"D-DA-A-C-CB-B-D-DA", 8);
    OLED_ShowString(0, 2, (uint8_t *)"Cam P-ctrl 30cm", 8);
    OLED_ShowString(0, 3, (uint8_t *)"BT Start [STOP]End", 8);
    OLED_ShowString(0, 5, (uint8_t *)"Init chase x1.50", 8);
}

uint8_t Task3B_Tick(uint8_t key)
{
    static uint32_t last_tag_ms = 0;   /* 上次收到AprilTag的时刻 */

    /* 不检查 tk_is_done(): 第5段DA弧永不结束, 等A车蓝牙STOP由pages.c截获退出 */

    /* ==== 启动后发 READY × 5 (间隔100ms) ==== */
    if (b_ready_cnt < 5 && tick_ms - b_last_ready_ms > 100) {
        Bluetooth_SendString("READY\r\n");
        b_ready_cnt++;
        b_last_ready_ms = tick_ms;
    }

    /* ==== 摄像头 AprilTag 测距 → P 控制速度 ==== */
    if (New_CMD_flag) {
        Pto_Frame_t frame;
        if (Pto_Parse_Frame(RxBuffer, New_CMD_length, &frame) == 0
            && frame.type == PTO_TYPE_APRILTAG) {
            float dist_cm = frame.data.apriltag.distance_mm / 10.0f;
            float error   = dist_cm - T3_DIST_TARGET;

            /* P 控制: 距离偏大 → 加速追赶, 距离偏小 → 减速 */
            tk_speed_mult = 1.0f + T3_DIST_KP * error;
            if (tk_speed_mult < T3_SPEED_MIN) tk_speed_mult = T3_SPEED_MIN;
            if (tk_speed_mult > T3_SPEED_MAX) tk_speed_mult = T3_SPEED_MAX;

            last_tag_ms = tick_ms;  /* 记录收到时刻 */

            /* OLED 显示距离+倍率 (行 5-6, 不刷全屏以免闪烁) */
            char buf[21];
            sprintf(buf, "Dist:%5.1fcm T:30", dist_cm);
            OLED_ShowString(0, 5, (uint8_t *)buf, 8);
            sprintf(buf, "SpdMul: %.2f", tk_speed_mult);
            OLED_ShowString(0, 6, (uint8_t *)buf, 8);
        }
        Pto_Clear_CMD_Flag();
    }

    /* ==== 丢失目标: >1s 未收到AprilTag → 正常速度 ==== */
    if (last_tag_ms != 0 && (tick_ms - last_tag_ms > 1000)) {
        tk_speed_mult = 1.0f;
        OLED_ShowString(0, 5, (uint8_t *)"LOST TAG! Chase..", 8);
        OLED_ShowString(0, 6, (uint8_t *)"SpdMul: 1.00", 8);
    }

    return task_tick(key);
}
