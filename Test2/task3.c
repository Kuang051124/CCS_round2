/**
 * @file  task3.c
 * @brief Task 3: 双车协同 — A车慢速领跑 + B车摄像头测距P控跟随
 *
 * 蓝牙协议:
 *   1. A 按5 → 发 "TASK3" × 5 → 启动
 *   2. B 收到 "TASK3" → pages.c 自动启动, 不回复
 *   3. A 到终点 → 发 "STOP" × 5 (B 不回复)
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
 * Car A — 发 TASK3×5 → 运行 → 发 STOP×5
 * =================================================================== */

static uint8_t  a_task3_cnt     = 0;   /* "TASK3" 已发送次数       */
static uint8_t  a_stop_cnt      = 0;   /* "STOP"  已发送次数       */
static uint32_t a_last_send_ms  = 0;   /* 上次蓝牙发送时刻          */

void Task3A_Init(void)
{
    tk_param      = &t3a;  /* Task3A 专用参数, 独立于 t3 */
    tk_gyro_src   = GYRO_SRC_WIT;
    tk_speed_mult = 1.0f;
    task_init(task3a_path, TASK3A_SEG_COUNT);  /* Task3A 专用路径, 航向独立可调 */

    a_task3_cnt    = 0;
    a_stop_cnt     = 0;
    a_last_send_ms = 0;

    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Task3A: Car A", 8);
    OLED_ShowString(0, 1, (uint8_t *)"A-C-CB-B-D-DA-A", 8);
    OLED_ShowString(0, 2, (uint8_t *)"Slow + Lead", 8);
    OLED_ShowString(0, 3, (uint8_t *)"[5] Start [9] Exit", 8);
}

uint8_t Task3A_Tick(uint8_t key)
{
    /* ---- 按9退出 ---- */
    if (key == 9) {
        a_task3_cnt = 0; a_stop_cnt = 0;
        return 1;
    }

    /* ======== Phase 1: 按5 → 发 TASK3 × 5 → 启动 ======== */
    if (a_task3_cnt == 0 && a_stop_cnt == 0 && !tk_is_done()) {
        if (key == 5) {
            a_task3_cnt = 1;
            Bluetooth_SendString("TASK3\r\n");
            a_last_send_ms = tick_ms;
        }
        /* 还没按5 → 等按键, 不走task_tick */
        if (a_task3_cnt == 0) return 0;
    }

    /* 发 TASK3 剩余 4 条 (每100ms一条) */
    if (a_task3_cnt > 0 && a_task3_cnt < 5) {
        if (tick_ms - a_last_send_ms > 100) {
            Bluetooth_SendString("TASK3\r\n");
            a_task3_cnt++;
            a_last_send_ms = tick_ms;

            char buf[21];
            sprintf(buf, "TASK3 sent: %d/5", a_task3_cnt);
            OLED_ShowString(0, 5, (uint8_t *)buf, 8);
        }
        return 0;  /* 发完前不走 task_tick */
    }

    /* 发完 TASK3 → 启动任务 (只触发一次) */
    if (a_task3_cnt == 5) {
        a_task3_cnt = 6;  /* 标记已启动, 防止重复触发 */
        task_tick(5);
    }

    /* ======== Phase 2: 运行中 ======== */
    uint8_t ret = task_tick(key);

    /* ======== Phase 3: 到终点 → 发 STOP × 5 ======== */
    if (tk_is_done() && a_stop_cnt == 0) {
        a_stop_cnt = 1;
        Bluetooth_SendString("STOP\r\n");
        a_last_send_ms = tick_ms;

        char buf[21];
        sprintf(buf, "STOP sent: 1/5");
        OLED_ShowString(0, 5, (uint8_t *)buf, 8);
    }
    if (a_stop_cnt > 0 && a_stop_cnt < 5) {
        if (tick_ms - a_last_send_ms > 100) {
            Bluetooth_SendString("STOP\r\n");
            a_stop_cnt++;
            a_last_send_ms = tick_ms;

            char buf[21];
            sprintf(buf, "STOP sent: %d/5", a_stop_cnt);
            OLED_ShowString(0, 5, (uint8_t *)buf, 8);
        }
    }

    /* 按9退出 → 重置 */
    if (ret) {
        a_task3_cnt = 0; a_stop_cnt = 0;
    }
    return ret;
}

/* ===================================================================
 * Car B — 收到 TASK3 → pages.c 自动启动 → 摄像头P控跟随
 *
 *   STOP 检测在此处完成, 停车后 OLED 持续显示原因
 *   强制停车: yaw差∈[-90°,-45°] 且 距离<30cm (蓝牙失效兜底)
 * =================================================================== */

uint8_t g_stop_reason = STOP_REASON_NONE;
static uint8_t b_stopped    = 0;  /* 0=运行, 1=BT, 2=force */
static uint8_t s_cam_seen   = 0;  /* 0=未识别, 1=已首次识别 (用于快速→慢速切换) */

void Task3B_Init(void)
{
    /* 用 Task3B 专用参数: 从快速追车参数开始, 减速段现场插值 */
    t3b_work = t3b_fast;         /* 结构体整体赋值, 初始 = 快 */
    tk_param      = &t3b_work;
    tk_gyro_src   = GYRO_SRC_WIT;
    tk_speed_mult = T3_SPEED_INIT;  /* = 1.0, 追车速度由 t3b_fast 基速决定 */
    task_init(task3b_path, TASK3B_SEG_COUNT);

    g_stop_reason = STOP_REASON_NONE;
    b_stopped     = 0;
    s_cam_seen    = 0;

    /* 清摄像头标志位 */
    Pto_Clear_CMD_Flag();

    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Task3B: Car B", 8);
    OLED_ShowString(0, 1, (uint8_t *)"D-DA-A-C-CB-B-D-DA", 8);
    OLED_ShowString(0, 2, (uint8_t *)"Cam P-ctrl 30cm", 8);
    OLED_ShowString(0, 3, (uint8_t *)"BT Start [STOP]End", 8);
    OLED_ShowString(0, 5, (uint8_t *)"Fast chase 1.0x", 8);
}

uint8_t Task3B_Tick(uint8_t key)
{
    static uint32_t last_tag_ms   = 0;    /* 上次收到AprilTag的时刻    */
    static float    last_dist_cm  = 0;    /* 最新距离 (cm), 停车用     */

    /* ==== 已停车 → OLED 持续显示原因, 等用户按 ModeB 退出 ==== */
    if (b_stopped) {
        OLED_ShowString(0, 0, (uint8_t *)"Task3B: STOPPED", 8);
        if (b_stopped == STOP_REASON_BT) {
            OLED_ShowString(0, 1, (uint8_t *)"STOP from A (BT)", 8);
            OLED_ShowString(0, 3, (uint8_t *)"Task3 Done", 8);
        } else {
            OLED_ShowString(0, 1, (uint8_t *)"FORCE STOP", 8);
            OLED_ShowString(0, 3, (uint8_t *)"Yaw+Dist trigger", 8);
        }
        return 0;
    }

    /* ==== 蓝牙 STOP 检测 (A车完成, b_parse_command 不会消费 STOP) ==== */
    if (g_bt_rx_ready) {
        if (strncmp((const char *)g_bt_rx_buf, "STOP", 4) == 0) {
            tk_abort();
            g_stop_reason = STOP_REASON_BT;
            b_stopped = STOP_REASON_BT;
            g_bt_rx_ready = 0; g_bt_rx_len = 0;
            OLED_Clear();
            return 0;
        }
        g_bt_rx_ready = 0; g_bt_rx_len = 0;
    }

    /* 不检查 tk_is_done(): 第5段DA弧永不结束 */

    /* ==== 摄像头 AprilTag 测距 → P 控制速度 ==== */
    if (New_CMD_flag) {
        Pto_Frame_t frame;
        if (Pto_Parse_Frame(RxBuffer, New_CMD_length, &frame) == 0
            && frame.type == PTO_TYPE_APRILTAG) {
            last_dist_cm = frame.data.apriltag.distance_mm / 10.0f;
            float error  = last_dist_cm - T3_DIST_TARGET;

            /* 首次识别到标签: 立刻切慢速参数, 此后摄像头 P 控基于慢速工作 */
            if (!s_cam_seen) {
                s_cam_seen = 1;
                t3b_work = t3b_slow;
                OLED_ShowString(0, 4, (uint8_t *)"CAM LOCK->SLOW", 8);
            }

            /* P 控制: 距离偏大 → 加速追赶, 距离偏小 → 减速 */
            tk_speed_mult = 1.0f + T3_DIST_KP * error;
            if (tk_speed_mult < T3_SPEED_MIN) tk_speed_mult = T3_SPEED_MIN;
            if (tk_speed_mult > T3_SPEED_MAX) tk_speed_mult = T3_SPEED_MAX;

            last_tag_ms = tick_ms;

            /* OLED 显示距离+倍率 */
            char buf[21];
            sprintf(buf, "Dist:%5.1fcm T:30", last_dist_cm);
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

    /* ==== 航向差计算 (强制停车 + 速度递减共用, 仅在DA弧段生效) ==== */
    uint8_t seg   = tk_get_seg_index();
    uint8_t on_da = (seg == 0 || seg == 4);  /* task3b: DA弧 = seg0, DA弧2 = seg4 */

    if (on_da) {
        float init_yaw = tk_get_initial_yaw();
        float yaw_d = wit_data.yaw - init_yaw;
        if (yaw_d >  180.0f) yaw_d -= 360.0f;
        if (yaw_d < -180.0f) yaw_d += 360.0f;

        /* DEBUG: 实时显示 yaw_d, 确认是否进入减速窗口 [-180, -120] */
        {
            char buf[21];
            sprintf(buf, "yaw_d:%+6.0f s%d", yaw_d, seg);
            OLED_ShowString(0, 7, (uint8_t *)buf, 8);
        }

        /* ==== 强制停车: 仅最后一段DA弧 (seg4), 蓝牙失效兜底 ==== */
        if (seg == 4
            && yaw_d >= T3_FORCE_YAW_MIN && yaw_d <= T3_FORCE_YAW_MAX
            && last_dist_cm > 0 && last_dist_cm < T3_FORCE_DIST_CM) {
            tk_abort();
            g_stop_reason = STOP_REASON_FORCE;
            b_stopped = STOP_REASON_FORCE;
            OLED_Clear();
            return 0;
        }

        /* ==== 弧线末段减速: 所有 8 个参数从 t3b_fast 线性插值到 t3b_slow ==== */
        /* 仅最后一段 DA 弧 (seg==4), 与摄像头 P 控 (tk_speed_mult) 完全解耦
         * 插值幂等: 每帧基于 yaw_d 从头计算, 无状态累积, 无漂移 */
        if (seg == 0
            && yaw_d <= T3_SLOW_YAW_START && yaw_d >= T3_SLOW_YAW_END) {
            float progress = (yaw_d - T3_SLOW_YAW_START)
                           / (T3_SLOW_YAW_END - T3_SLOW_YAW_START);
            if (progress < 0.0f) progress = 0.0f;
            if (progress > 1.0f) progress = 1.0f;
            t3b_interpolate(&t3b_work, &t3b_fast, &t3b_slow, progress);

            /* DEBUG: 确认减速生效 */
            char buf[21];
            sprintf(buf, "SLOW p=%.2f a=%d", progress, (int)tk_param->spd_arc);
            OLED_ShowString(0, 6, (uint8_t *)buf, 8);
        }
    }

    return task_tick(key);
}
