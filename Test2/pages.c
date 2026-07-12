#include "pages.h"
#include "wit.h"
#include "keyboard.h"
#include "mode1.h"
#include "task3.h"
#include "task4.h"
#include "clock.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "../yb_protocol/yb_protocol.h"
#include <math.h>
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include "BLUETOOTH/bluetooth.h"
#include "../TB6612/tb6612.h"
#include "ENCODER/encoder.h"
#include "ENCODER/speed_control.h"
#include "BLUETOOTH/bt_cmd_parser.h"

extern uint8_t oled_buffer[32];

uint8_t page_state = 0;
uint8_t key = 0;

/* ==================================================================
 * Page_Home - 主页（page_state = 0）
 * TODO: 补充主页逻辑
 * ================================================================== */
void Page_Home(void)
{

  /* 键盘导航: 按数字键进入对应页面 */
  uint8_t home_key = Scan_Keyboard();
  if (home_key != 0) {
    switch (home_key) {
    case 1: while (Scan_Keyboard()); page_state = 1; break;  /* Calib */
    case 2: while (Scan_Keyboard()); page_state = 2; break;  /* Debug */
    case 3: while (Scan_Keyboard()); page_state = 3; break;  /* Test  */
    case 4: while (Scan_Keyboard()); page_state = 4; break;  /* Step  */
    case 5: while (Scan_Keyboard()); page_state = 5; break;  /* Gyro  */
    case 6: while (Scan_Keyboard()); page_state = 6; break;  /* Camera */
    case 7: while (Scan_Keyboard()); page_state = 7; break;  /* Curve  */
    case 8: while (Scan_Keyboard()); page_state = 8; break;  /* PWM   */
    }
  }

  /* =========================================================== */

  OLED_ShowString(0, 0, (uint8_t *)"Home Page  9:Home", 8);
  OLED_ShowString(0, 1, (uint8_t *)"[1]Calib Page    ", 8);
  OLED_ShowString(0, 2, (uint8_t *)"[2]Debug Page    ", 8);
  OLED_ShowString(0, 3, (uint8_t *)"[3]Test Page     ", 8);
  OLED_ShowString(0, 4, (uint8_t *)"[4]Step Debug    ", 8);
  OLED_ShowString(0, 5, (uint8_t *)"[5]Gyro Debug    ", 8);
  OLED_ShowString(0, 6, (uint8_t *)"[6]Cam [7]Curve  ", 8);
  OLED_ShowString(0, 7, (uint8_t *)"[8]PWM   9:Home  ", 8);
}

/* ==================================================================
 * Page_Calib - Task1 参数调校 (page_state = 1)
 *
 * 键盘操作:
 *   1/2 = 上/下一个参数    3 = 恢复默认
 *   4/5 = 微调 -/+         8 = 粗调 +
 *   6   = 微调/粗调切换    9 = Home
 * ================================================================== */

/* ---- Task1 参数表 ---- */
typedef struct {
    const char *name;
    float      *ptr;          /* 指向 mode1.c 中的全局变量 */
    float       def_val;      /* 默认值 */
    float       fine_step;    /* 微调步长 */
    float       coarse_step;  /* 粗调步长 */
} T1_ParamEntry;

static T1_ParamEntry t1_param_table[] = {
    {"SpdStr",  &t1.spd_straight,   1400.0f,  50.0f,  200.0f},
    {"SpdArc",  &t1.spd_arc,        1200.0f,  50.0f,  200.0f},
    {"ArcKP",   &t1.spd_arc_kp,     200.0f,   20.0f,  100.0f},
    {"GyroKP",  &t1.gyro_kp,        50.0f,    10.0f,   50.0f},
    {"OfsCW",   &t1.arc_ofs_cw,     250.0f,   25.0f,  100.0f},
    {"OfsCCW",  &t1.arc_ofs_ccw,   -250.0f,   25.0f,  100.0f},
    {"CW_T2",   &t1.arc_ofs_cw_t2,  1000.0f,  50.0f,  200.0f},
    {"CCW_T2",  &t1.arc_ofs_ccw_t2,-1000.0f,  50.0f,  200.0f},
};
#define T1_PARAM_COUNT (sizeof(t1_param_table) / sizeof(t1_param_table[0]))

void Page_Calib(void)
{
    static uint8_t  idx      = 0;
    static uint8_t  coarse   = 0;
    static uint8_t  last_key = 0;
    static uint8_t  saved    = 0;
    static uint32_t save_tick = 0;

    uint8_t k = Scan_Keyboard();

    if (k != 0 && k != last_key) {
        T1_ParamEntry *e = &t1_param_table[idx];
        float step;
        switch (k) {
        case 1: /* 上一个参数 */
            idx = (idx == 0) ? T1_PARAM_COUNT - 1 : idx - 1;
            break;
        case 2: /* 下一个参数 */
            idx = (idx == T1_PARAM_COUNT - 1) ? 0 : idx + 1;
            break;
        case 3: /* 恢复默认 */
            *e->ptr = e->def_val;
            break;
        case 4: /* 微调 - */
            step   = coarse ? e->coarse_step : e->fine_step;
            *e->ptr -= step;
            break;
        case 5: /* 微调 + */
            step   = coarse ? e->coarse_step : e->fine_step;
            *e->ptr += step;
            break;
        case 6: /* 粗调/微调切换 */
            coarse = !coarse;
            break;
        case 7: /* 保存到 Flash */
            TKParam_Save();
            saved     = 1;
            save_tick = tick_ms;
            break;
        case 8: /* 粗调 + */
            *e->ptr += e->coarse_step;
            break;
        case 9: /* 返回 Home */
            while (Scan_Keyboard());
            page_state = 0;
            break;
        }
    }
    last_key = k;

    if (saved && (tick_ms - save_tick > 1000)) {
        saved = 0;
    }

    /* ========== OLED 显示 ========== */
    T1_ParamEntry *e = &t1_param_table[idx];

    sprintf((char *)oled_buffer, "T1:%-10s[%c]", e->name, coarse ? 'C' : 'F');
    OLED_ShowString(0, 0, oled_buffer, 8);

    sprintf((char *)oled_buffer, "Val:%10.3f", *e->ptr);
    OLED_ShowString(0, 1, oled_buffer, 8);

    OLED_ShowString(0, 2, (uint8_t *)"4:Fine- 5:Fine+", 8);
    OLED_ShowString(0, 3, (uint8_t *)"6:Coars 8:Cors+", 8);
    OLED_ShowString(0, 4, (uint8_t *)"1/2:Sel 3:Rst", 8);

    if (saved) {
        OLED_ShowString(0, 5, (uint8_t *)"[SAVED!]  9:Home", 8);
    } else {
        OLED_ShowString(0, 5, (uint8_t *)"7:SAVE FLASH 9:Home", 8);
    }
}

/* ==================================================================
 * Page_Debug - 编码器调试页（page_state = 2）
 *
 * 显示内容:
 *   Line 0: 标题 + 目标速度
 *   Line 1~2: 左/右轮编码器计数 + PWM 占空比
 *   Line 3~4: 左/右轮实时速度 vs 目标
 *   Line 5~6: 按键操作提示
 *   Line 7: 9:Home
 *
 * 按键:
 *   1 = 目标 +50   4 = 停止
 *   2 = 目标 -50   9 = 返回 Home
 *   3 = 目标 +100
 * ================================================================== */
void Page_Debug(void)
{
    uint8_t k = Scan_Keyboard();
    int32_t tgtL = SPEED_GetTargetLeft();
    int32_t tgtR = SPEED_GetTargetRight();

    /* 按键处理 */
    if (k == 9) {
        SPEED_SetTarget(0, 0);
        while (Scan_Keyboard());
        page_state = 0;
        return;
    }

    switch (k) {
    case 1: SPEED_SetTarget(tgtL + 50, tgtR);       break;  /* 左轮加速 */
    case 2: SPEED_SetTarget(tgtL - 50, tgtR);       break;  /* 左轮减速 */
    case 4: SPEED_SetTarget(tgtL, tgtR + 50);       break;  /* 右轮加速 */
    case 5: SPEED_SetTarget(tgtL, tgtR - 50);       break;  /* 右轮减速 */
    case 3: SPEED_SetTarget(tgtL + 100, tgtR + 100); break; /* 双轮加速 */
    case 8: SPEED_SetTarget(0, 0);                  break;  /* 停止 */
    }

    /* 定时器 ISR 每 10ms 调用, 此处兜底 (防 ISR 未配置) */
    {
        static uint32_t s_lastMs = 0;
        if (tick_ms - s_lastMs >= 10) {
            s_lastMs = tick_ms;
            ENCODER_SpeedSample();
        }
    }

    /* ========== OLED 显示 ========== */

    /* Line 0: 标题 + 目标速度 */
    sprintf((char *)oled_buffer, "EncDbg L:%-4d R:%-4d", (int)tgtL, (int)tgtR);
    OLED_ShowString(0, 0, oled_buffer, 8);

    /* Line 1: 左轮 计数 + 占空比 */
    sprintf((char *)oled_buffer, "L:%+6ld D:%-4d",
            (long)g_leftEncoderCount, (int)SPEED_GetLeftDuty());
    OLED_ShowString(0, 1, oled_buffer, 8);

    /* Line 2: 右轮 计数 + 占空比 */
    sprintf((char *)oled_buffer, "R:%+6ld D:%-4d",
            (long)g_rightEncoderCount, (int)SPEED_GetRightDuty());
    OLED_ShowString(0, 2, oled_buffer, 8);

    /* Line 3: 左轮速度 (当前/目标) */
    sprintf((char *)oled_buffer, "L:%+5d /%+5d/s",
            (int)ENCODER_GetLeftSpeed(), (int)tgtL);
    OLED_ShowString(0, 3, oled_buffer, 8);

    /* Line 4: 右轮速度 */
    sprintf((char *)oled_buffer, "R:%+5d /%+5d/s",
            (int)ENCODER_GetRightSpeed(), (int)tgtR);
    OLED_ShowString(0, 4, oled_buffer, 8);

    /* Line 5~6: 操作提示 */
    OLED_ShowString(0, 5, (uint8_t *)"1/2:L+/- 4/5:R+/-", 8);
    OLED_ShowString(0, 6, (uint8_t *)"3:+100 8:STOP 9:Home", 8);

    /* Line 7 */
    OLED_ShowString(0, 7, (uint8_t *)"9:Home", 8);
}

/* ==================================================================
 * Page_Test - 测试页（page_state = 3）
 * TODO: 补充测试逻辑
 * ================================================================== */
void Page_Test(void)
{
    static uint8_t test_mode  = 0;
    static uint8_t prev_mode  = 0xFF;

    uint8_t k = Scan_Keyboard();

    /* 子任务 delegate: test_mode > 0 时交给对应模块处理 */
    /* 必须放在最前面, 否则 Page_Test 自身的 if(k==9) 会截获按键 */
    /* Task1: [1]WIT  [5]Slow  [7]MPU */
    if (test_mode == 1) { if (Task1_Tick(k))      { test_mode = 0; prev_mode = 0xFF; } return; }
    if (test_mode == 5) { if (Task1_Slow_Tick(k)) { test_mode = 0; prev_mode = 0xFF; } return; }
    if (test_mode == 7) { if (Task1_MPU_Tick(k))  { test_mode = 0; prev_mode = 0xFF; } return; }
    /* Task2: [2]WIT  [6]Slow  [8]MPU */
    if (test_mode == 2) { if (Task2_Tick(k))      { test_mode = 0; prev_mode = 0xFF; } return; }
    if (test_mode == 6) { if (Task2_Slow_Tick(k)) { test_mode = 0; prev_mode = 0xFF; } return; }
    if (test_mode == 8) { if (Task2_MPU_Tick(k))  { test_mode = 0; prev_mode = 0xFF; } return; }
    /* Task3: [3]Follow */
    if (test_mode == 3) { if (Task3A_Tick(k)) { test_mode = 0; prev_mode = 0xFF; } return; }
    /* Task4: [4]Overtake */
    if (test_mode == 4) { if (Task4A_Tick(k)) { test_mode = 0; prev_mode = 0xFF; } return; }

    /* ---- 菜单模式 ---- */
    if (k == 9) { while (Scan_Keyboard()); page_state = 0; return; }

    if (k != 0) test_mode = k;

    if (test_mode != prev_mode) {
        OLED_Clear();
        prev_mode = test_mode;
        if (test_mode == 1) Task1_Init();
        if (test_mode == 5) Task1_Slow_Init();
        if (test_mode == 7) Task1_MPU_Init();
        if (test_mode == 2) Task2_Init();
        if (test_mode == 6) Task2_Slow_Init();
        if (test_mode == 8) Task2_MPU_Init();
        if (test_mode == 3) Task3A_Init();
        if (test_mode == 4) Task4A_Init();
    }

    if (test_mode == 0) {
        OLED_ShowString(0, 0, (uint8_t *)"Test Menu", 8);
        OLED_ShowString(0, 1, (uint8_t *)"[1]T1 [2]T2 [3]T3 [4]T4", 8);
        OLED_ShowString(0, 2, (uint8_t *)"[5]T1S [6]T2S", 8);
        OLED_ShowString(0, 3, (uint8_t *)"[7]T1M [8]T2M", 8);
        OLED_ShowString(0, 7, (uint8_t *)"9:Home", 8);
    }
}
/* ==================================================================
从车 (B车) 控制逻辑 — 蓝牙指令驱动, 无键盘
 * ================================================================== */

#define B_MODE_IDLE   0
#define B_MODE_BTASK1 1    /* ModeB按键: 调试Task1 */
#define B_MODE_BTASK2 2    /* ModeB按键: 调试Task2 */
#define B_MODE_TASK3  3
#define B_MODE_TASK4  4
#define B_MODE_SPEED  5    /* 蓝牙目标速度控制模式 */

/* ModeB 调试按键 (B车无键盘, 独立按钮) */
#define MODE_B_PORT   GPIOB
#define MODE_B_PIN    DL_GPIO_PIN_21   /* PB21, 按下=LOW, 外部上拉 */
#define MODE_B_IOMUX  (IOMUX_PINCM49)  /* PB21 → PINCM49 */

static char b_last_cmd[48] = "--";

/* 解析蓝牙指令 → 返回目标模式 (只消费 TASK3/TASK4, 其余留给 task tick 处理) */
static uint8_t b_parse_command(void)
{
    if (!g_bt_rx_ready) return 0;

    uint8_t len = g_bt_rx_len;
    if (len > sizeof(b_last_cmd) - 1) len = sizeof(b_last_cmd) - 1;
    memcpy(b_last_cmd, (const void *)g_bt_rx_buf, len);
    b_last_cmd[len] = '\0';
    g_bt_rx_ready = 0;
    g_bt_rx_len   = 0;

    if (strncmp(b_last_cmd, "TASK3", 5) == 0) return B_MODE_TASK3;
    if (strncmp(b_last_cmd, "TASK4", 5) == 0) return B_MODE_TASK4;
    if (strncmp(b_last_cmd, "STOP",  4) == 0) return B_MODE_IDLE;

    return 0;
}

/* ==================================================================
 * SpeedMode_Tick — B_MODE_SPEED 每帧调用
 *
 *  由 TIMG0 ISR (10ms) 自动执行 PI 控制, 本函数负责:
 *    1. 解析新收到的蓝牙指令 (slider 在线调参)
 *    2. 每 100ms 发送 [plot,...] 绘图数据到小程序
 *    3. OLED 实时显示: 目标速度 / 实际速度 / Kp Ki Kd / PWM 占空比
 *    4. 检测 STOP 指令退出
 *
 *  返回: 1=退出, 0=保持
 * ================================================================== */
static uint8_t SpeedMode_Tick(void)
{
    static uint32_t s_lastPlotMs = 0;

    /* ---- 处理新收到的蓝牙指令 ---- */
    if (g_bt_rx_ready) {
        uint8_t len = g_bt_rx_len;
        if (len > sizeof(b_last_cmd) - 1) len = sizeof(b_last_cmd) - 1;
        memcpy(b_last_cmd, (const void *)g_bt_rx_buf, len);
        b_last_cmd[len] = '\0';
        g_bt_rx_ready = 0;
        g_bt_rx_len   = 0;

        if (b_last_cmd[0] == '[') {
            int8_t ret = BT_ParseCommand(b_last_cmd, len);
            if (ret == -1) {
                SPEED_SetTarget(0, 0);
                return 1;
            }
        }
    }

    /* ---- 每 100ms 发送绘图数据 ---- */
    if (tick_ms - s_lastPlotMs >= 100) {
        s_lastPlotMs = tick_ms;
        BT_SendPlot(
            SPEED_GetTargetLeft(), SPEED_GetTargetRight(),
            ENCODER_GetLeftSpeed(), ENCODER_GetRightSpeed()
        );
    }

    /* ========== OLED 显示 (8行) ========== */
    char buf[21];

    /* Line 0: 在线状态 + 模式 */
    if (g_bt_state == BT_CONNECTED) {
        sprintf(buf, "B-Speed [Online]");
    } else {
        sprintf(buf, "B-Speed [Offline]");
    }
    OLED_ShowString(0, 0, (uint8_t *)buf, 8);

    /* Line 1: 左轮 — 实际 / 目标 */
    sprintf(buf, "L:%+5d /%+5d/s",
            (int)ENCODER_GetLeftSpeed(), (int)SPEED_GetTargetLeft());
    OLED_ShowString(0, 1, (uint8_t *)buf, 8);

    /* Line 2: 右轮 — 实际 / 目标 */
    sprintf(buf, "R:%+5d /%+5d/s",
            (int)ENCODER_GetRightSpeed(), (int)SPEED_GetTargetRight());
    OLED_ShowString(0, 2, (uint8_t *)buf, 8);

    /* Line 3: PWM 占空比 */
    sprintf(buf, "Duty L:%-4d R:%-4d",
            (int)SPEED_GetLeftDuty(), (int)SPEED_GetRightDuty());
    OLED_ShowString(0, 3, (uint8_t *)buf, 8);

    /* Line 4: Kp + Ki */
    sprintf(buf, "Kp:%-6.2f Ki:%.2f",
            SPEED_GetKp(), SPEED_GetKi());
    OLED_ShowString(0, 4, (uint8_t *)buf, 8);

    /* Line 5: Kd */
    sprintf(buf, "Kd:%-6.2f", SPEED_GetKd());
    OLED_ShowString(0, 5, (uint8_t *)buf, 8);

    /* Line 6: 编码器原始计数 */
    sprintf(buf, "Cnt L:%+6ld R:%+6ld",
            (long)g_leftEncoderCount, (long)g_rightEncoderCount);
    OLED_ShowString(0, 6, (uint8_t *)buf, 8);

    /* Line 7: 最近指令 (截断到 14 字符防止 buf 溢出) */
    sprintf(buf, "CMD:%.14s", b_last_cmd);
    OLED_ShowString(0, 7, (uint8_t *)buf, 8);

    return 0;
}

void Page_Test2(void)
{
    static uint8_t b_mode        = B_MODE_IDLE;
    static uint8_t b_prev_mode   = 0xFF;
    static uint8_t btn_debounce  = 0;
    static uint32_t btn_last_ms  = 0;
    static uint8_t btn_inited    = 0;

    /* 首次调用: 初始化ModeB按键GPIO */
    if (!btn_inited) {
        btn_inited = 1;
        DL_GPIO_initDigitalInputFeatures(MODE_B_IOMUX,
            DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
            DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    }

    /* ======== ModeB 按键: 循环切换模式 (IDLE→Task1→Task2→IDLE) ======== */
    if (!DL_GPIO_readPins(MODE_B_PORT, MODE_B_PIN)) {
        /* 按下: 消抖 50ms */
        if (btn_debounce == 0) {
            btn_last_ms = tick_ms;
            btn_debounce = 1;
        }
        if (btn_debounce == 1 && (tick_ms - btn_last_ms > 50)) {
            btn_debounce = 2;
            /* 循环切换: IDLE → BTASK1 → BTASK2 → IDLE */
            uint8_t old_mode = b_mode;
            if (b_mode == B_MODE_IDLE)       b_mode = B_MODE_SPEED;
            // else if (b_mode == B_MODE_BTASK1) b_mode = B_MODE_BTASK2;
            else                              b_mode = B_MODE_IDLE;
            if (old_mode != B_MODE_IDLE) {
                tk_abort();  /* 从运行模式切出时停电机 */
            }
            OLED_Clear();
        }
    } else {
        btn_debounce = 0;
    }

    /* 解析蓝牙指令 (可覆盖按键选择的模式) */
    uint8_t new_mode = b_parse_command();
    if (new_mode != 0 && new_mode != b_mode) {
        if (new_mode == B_MODE_IDLE && b_mode != B_MODE_IDLE) {
            tk_abort();
        }
        b_mode = new_mode;
        OLED_Clear();
    }

    /* ======== 模式切换初始化 (必须在 delegate 前面) ======== */
    if (b_mode != b_prev_mode) {
        OLED_Clear();
        b_prev_mode = b_mode;
        if (b_mode == B_MODE_BTASK1) {
            Task1_MPU_Init();
            task_tick(5);
        }
        if (b_mode == B_MODE_BTASK2) {
            Task2_MPU_Init();
            task_tick(5);
        }
        if (b_mode == B_MODE_TASK3) {
            Task3B_Init();
            task_tick(5);
        }
        if (b_mode == B_MODE_TASK4) {
            Task4B_Init();
            task_tick(5);
        }
        if (b_mode == B_MODE_SPEED) {
            BT_SendPlotClear();        /* 清空小程序绘图区 */
        }
    }

    /* ======== delegate: 任务运行中, 每帧 tick ======== */
    if (b_mode == B_MODE_BTASK1) {
        if (Task1_MPU_Tick(0)) { b_mode = B_MODE_IDLE; b_prev_mode = 0xFF; }
        return;
    }
    if (b_mode == B_MODE_BTASK2) {
        if (Task2_MPU_Tick(0)) { b_mode = B_MODE_IDLE; b_prev_mode = 0xFF; }
        return;
    }
    if (b_mode == B_MODE_TASK3) {
        /* STOP 检测 + 强制停车全部在 Task3B_Tick 内部处理 */
        if (Task3B_Tick(0)) { b_mode = B_MODE_IDLE; b_prev_mode = 0xFF; }
        return;
    }
    if (b_mode == B_MODE_TASK4) {
        if (Task4B_Tick(0)) { b_mode = B_MODE_IDLE; b_prev_mode = 0xFF; }
        return;
    }
    if (b_mode == B_MODE_SPEED) {
        if (SpeedMode_Tick()) { b_mode = B_MODE_IDLE; b_prev_mode = 0xFF; }
        return;
    }

    /* ======== OLED 显示 ======== */
    char buf[24];
    if (g_bt_state == BT_CONNECTED) {
        OLED_ShowString(0, 0, (uint8_t *)"B-Car  [Online] ", 8);
    } else {
        OLED_ShowString(0, 0, (uint8_t *)"B-Car  [Offline]", 8);
    }

    OLED_ShowString(0, 1, (uint8_t *)"Mode:", 8);
    if (b_mode == B_MODE_IDLE) {
        OLED_ShowString(40, 1, (uint8_t *)"IDLE", 8);
        OLED_ShowString(0, 2, (uint8_t *)"Btn:T1/T2 BT:T3/4", 8);
    } else if (b_mode == B_MODE_BTASK1) {
        OLED_ShowString(40, 1, (uint8_t *)"T1*", 8);
        OLED_ShowString(0, 2, (uint8_t *)"Debug Task1 (MPU)", 8);
    } else if (b_mode == B_MODE_BTASK2) {
        OLED_ShowString(40, 1, (uint8_t *)"T2*", 8);
        OLED_ShowString(0, 2, (uint8_t *)"Debug Task2 (MPU)", 8);
    } else if (b_mode == B_MODE_TASK3) {
        OLED_ShowString(40, 1, (uint8_t *)"TASK3*", 8);
        OLED_ShowString(0, 2, (uint8_t *)"Following A ...", 8);
    } else if (b_mode == B_MODE_TASK4) {
        OLED_ShowString(40, 1, (uint8_t *)"T4*", 8);
        OLED_ShowString(0, 2, (uint8_t *)"T4B: D-DA-A-B-BC-C-D", 8);
    }

    /* WIT yaw 实时显示 + 按键提示 */
    sprintf(buf, "Yaw:%6.1f", wit_data.yaw);
    OLED_ShowString(0, 3, (uint8_t *)buf, 8);

    sprintf(buf, "BT:%s Btn:PB21",
        (g_bt_state == BT_CONNECTED) ? "OK" : "NO");
    OLED_ShowString(0, 5, (uint8_t *)buf, 8);
    sprintf(buf, "CMD:%-14s", b_last_cmd);
    OLED_ShowString(0, 7, (uint8_t *)buf, 8);
}

/* ==================================================================
 * Page_Debug_Step - 步进调试页（page_state = 4）
 * ================================================================== */
void Page_Debug_Step(void)
{
      static uint8_t test_done = 0;
    if (Scan_Keyboard() == 9) { while (Scan_Keyboard()); page_state = 0; }
    OLED_ShowString(0, 0, (uint8_t *)"Step Debug", 8);
    OLED_ShowString(0, 7, (uint8_t *)"9:Home", 8);
      /* ========== 电机自检: 上电跑一次, 确认电机能转 ========== */
  if (!test_done) {


    test_done = 1;
      }
}

/* ==================================================================
 * Page_Debug_Gyro - 陀螺仪调试页（page_state = 5）
 * ================================================================== */
void Page_Debug_Gyro(void)
{
    if (Scan_Keyboard() == 9) { while (Scan_Keyboard()); page_state = 0; }
//WIT
    // OLED_ShowString(0, 0, (uint8_t *)"Gyro Debug", 8);
    // OLED_ShowString(0, 7, (uint8_t *)"9:Home", 8);
    
    // sprintf((char *)oled_buffer, "Pitch:%6.1f", wit_data.pitch);
    // OLED_ShowString(0, 2, oled_buffer, 8);

    // sprintf((char *)oled_buffer, "Roll :%6.1f", wit_data.roll);
    // OLED_ShowString(0, 3, oled_buffer, 8);

    // sprintf((char *)oled_buffer, "Yaw  :%6.1f", wit_data.yaw);
    // OLED_ShowString(0, 4, oled_buffer, 8);

    // sprintf((char *)oled_buffer, "Temp :%5.1fC", wit_data.temperature);
    // OLED_ShowString(0, 5, oled_buffer, 8);

    //MPU
    // /* ---- 轮询读取DMP数据 (无GPIO中断时用此方式) ---- */
    int mpu_ok = (Read_Quad() == 0);

    /* ---- 读取温度 (直接读MPU6050寄存器, 非DMP数据) ---- */
    long temp_raw;
    unsigned long ts;
    float temp_c = 0.0f;
    int temp_ok = (mpu_get_temperature(&temp_raw, &ts) == 0);
    if (temp_ok) {
        temp_c = (temp_raw / 340.0f) + 36.53f;  /* MPU6050 datasheet formula */
    }

    /* ========== OLED 显示 ========== */
    /* 标题行显示状态: OK 或 ERR */
    sprintf((char *)oled_buffer, "Gyro %s %s",
        mpu_ok ? "OK" : "ERR",
        temp_ok ? "  " : "T_ERR");
    OLED_ShowString(0, 0, (uint8_t *)oled_buffer, 8);

    /* 姿态角 (DMP四元数解算, 轮询更新) */
    sprintf((char *)oled_buffer, "Pitch:%6.1f", pitch);
    OLED_ShowString(0, 1, oled_buffer, 8);

    sprintf((char *)oled_buffer, "Roll :%6.1f", roll);
    OLED_ShowString(0, 2, oled_buffer, 8);

    sprintf((char *)oled_buffer, "Yaw  :%6.1f", yaw);
    OLED_ShowString(0, 3, oled_buffer, 8);

    /* 原始陀螺仪数据 */
    sprintf((char *)oled_buffer, "GX:%5d GY:%5d", gyro[0], gyro[1]);
    OLED_ShowString(0, 4, oled_buffer, 8);

    sprintf((char *)oled_buffer, "GZ:%5d", gyro[2]);
    OLED_ShowString(0, 5, oled_buffer, 8);

    /* 温度 */
    sprintf((char *)oled_buffer, "Temp:%5.1fC", temp_c);
    OLED_ShowString(0, 6, oled_buffer, 8);

    OLED_ShowString(0, 7, (uint8_t *)"9:Home", 8);
}

/* ==================================================================
 * Page_Debug_Camera — 串口摄像头数据调试页 (page_state = 6)
 *
 * 三层诊断:
 *   B>0   → ISR 正常工作 (UART 物理层 OK)
 *   RxF=1 → 正在收帧, 收到过 '$' 头
 *   Flag=1 → 完整帧已收完 (# 尾已到)
 *
 * OLED 布局 (8行×16列):
 *   Line 0: 字节计数 + 帧计数
 *   Line 1: RxFlag/RxIndex + UART 错误
 *   Line 2~6: 原始缓冲区 / 解析数据
 *   Line 7: 按键提示
 * ================================================================== */
void Page_Debug_Camera(void)
{
    static uint16_t frame_count = 0;
    static Pto_Frame_t last_frame;
    static uint8_t  has_data   = 0;

    uint8_t k = Scan_Keyboard();
    if (k == 9) {
        while (Scan_Keyboard());
        frame_count = 0;
        has_data    = 0;
        Pto_Raw_Byte_Count = 0;   /* 重置诊断计数器 */
        OLED_Clear();
        page_state = 0;
        return;
    }

    /* ---- 清除 UART 错误 (防止 overrun 卡死) ---- */
    DL_UART_Main_getErrorStatus(K230_INST, DL_UART_ERROR_OVERRUN);

    /* ---- 有新帧: 解析 ---- */
    if (New_CMD_flag) {
        Pto_Frame_t frame;
        int8_t ret = Pto_Parse_Frame(RxBuffer, New_CMD_length, &frame);
        Pto_Clear_Flag();

        if (ret == 0) {
            last_frame = frame;
            has_data   = 1;
            frame_count++;
        }
    }

    /* ========== OLED 显示 ========== */

    /* Line 0: 字节计数 + 帧计数 */
    sprintf((char *)oled_buffer, "B:%-5u F:%-4u", Pto_Raw_Byte_Count, frame_count);
    OLED_ShowString(0, 0, oled_buffer, 8);

    /* Line 1: 协议状态 + UART 错误 */
    {
        uint8_t err = DL_UART_Main_getErrorStatus(K230_INST,
                        DL_UART_ERROR_OVERRUN);
        sprintf((char *)oled_buffer, "RxF:%d Idx:%-2d E:%d",
                RxFlag, RxIndex, err);
        OLED_ShowString(0, 1, oled_buffer, 8);
    }

    if (!has_data) {
        OLED_ShowString(0, 2, (uint8_t *)"Waiting for", 8);
        OLED_ShowString(0, 3, (uint8_t *)"  camera data...", 8);
        OLED_ShowString(0, 7, (uint8_t *)"9:Home", 8);
        return;
    }

    /* ---- 第 2 层: 帧解析成功, 显示数据 ---- */
    /* Line 2: 帧类型 */
    sprintf((char *)oled_buffer, "T:%d %s",
            last_frame.type, Pto_Frame_Type_Name(last_frame.type));
    OLED_ShowString(0, 2, oled_buffer, 8);

    switch (last_frame.type) {

    case 0:  /* 目标位置 */
        sprintf((char *)oled_buffer, "Tgt X:%-4d", last_frame.data.position.x);
        OLED_ShowString(0, 3, oled_buffer, 8);
        sprintf((char *)oled_buffer, "Tgt Y:%-4d", last_frame.data.position.y);
        OLED_ShowString(0, 4, oled_buffer, 8);
        break;

    case 1:  /* 激光位置 */
        sprintf((char *)oled_buffer, "Lsr X:%-4d", last_frame.data.position.x);
        OLED_ShowString(0, 3, oled_buffer, 8);
        sprintf((char *)oled_buffer, "Lsr Y:%-4d", last_frame.data.position.y);
        OLED_ShowString(0, 4, oled_buffer, 8);
        break;

    case 2:  /* 数字 */
        sprintf((char *)oled_buffer, "X:%-4d Y:%-4d",
                last_frame.data.number.x, last_frame.data.number.y);
        OLED_ShowString(0, 3, oled_buffer, 8);
        sprintf((char *)oled_buffer, "Number: %d", last_frame.data.number.value);
        OLED_ShowString(0, 4, oled_buffer, 8);
        break;

    case 3:  /* 颜色形状 */
        sprintf((char *)oled_buffer, "X:%-4d Y:%-4d",
                last_frame.data.shape.x, last_frame.data.shape.y);
        OLED_ShowString(0, 3, oled_buffer, 8);
        sprintf((char *)oled_buffer, "%s %s C:%d S:%d",
                Pto_Color_Name(last_frame.data.shape.color),
                Pto_Shape_Name(last_frame.data.shape.shape),
                last_frame.data.shape.color, last_frame.data.shape.shape);
        OLED_ShowString(0, 4, oled_buffer, 8);
        break;

    case 4:  /* 终点 */
        sprintf((char *)oled_buffer, "End X:%-4d", last_frame.data.position.x);
        OLED_ShowString(0, 3, oled_buffer, 8);
        sprintf((char *)oled_buffer, "End Y:%-4d", last_frame.data.position.y);
        OLED_ShowString(0, 4, oled_buffer, 8);
        break;

    case 5:  /* AprilTag */
        sprintf((char *)oled_buffer, "Dist:%dmm Tag:%d",
                last_frame.data.apriltag.distance_mm,
                last_frame.data.apriltag.tag_id);
        OLED_ShowString(0, 3, oled_buffer, 8);
        {
            float ang = Pto_Apriltag_To_Degrees(
                last_frame.data.apriltag.yaw_raw,
                last_frame.data.apriltag.direction);
            sprintf((char *)oled_buffer, "Ang:%+5.1f %s",
                    ang,
                    last_frame.data.apriltag.direction == 0 ? "R" : "L");
            OLED_ShowString(0, 4, oled_buffer, 8);
        }
        break;

    default:
        OLED_ShowString(0, 3, (uint8_t *)"Unknown type", 8);
        break;
    }

    OLED_ShowString(0, 7, (uint8_t *)"9:Home", 8);
}

/* ==================================================================
 * Page_CurveDebug — 曲线调试页 (page_state = 7)
 *
 * 目的: 调试左右轮速度与差速对转弯半径的影响
 *   Left  = Base - Diff   (差速为正 → 左轮慢/右轮快 → 左转)
 *   Right = Base + Diff
 *
 * 按键:
 *   1/2 = Base速度 -/+      4/5 = 差速 -/+
 *   6   = 微调/粗调切换     7   = 启动/停止电机
 *   8   = 紧急停止          9   = 返回 Home
 *
 * 步长: 微调 Base±10 Diff±5  |  粗调 Base±50 Diff±25
 * ================================================================== */
void Page_CurveDebug(void)
{
    static int32_t base_speed   = 200;   /* 基础速度 (counts/s)       */
    static int32_t differential = 50;    /* 差速     (counts/s)       */
    static uint8_t coarse       = 0;     /* 0=微调  1=粗调            */
    static uint8_t motor_on     = 0;     /* 0=停止  1=SPEED_SetTarget */
    static uint8_t last_key     = 0;

    uint8_t k = Scan_Keyboard();

    /* ---- 9: 停电机 + 返回 Home ---- */
    if (k == 9) {
        SPEED_Stop();
        motor_on = 0;
        while (Scan_Keyboard());
        page_state = 0;
        return;
    }

    /* ---- 8: 紧急停止 (不退出页面) ---- */
    if (k == 8) {
        SPEED_Stop();
        motor_on = 0;
    }

    /* ---- 参数调节 (边沿触发) ---- */
    if (k != 0 && k != last_key) {
        int32_t b_step = coarse ? 50 : 10;
        int32_t d_step = coarse ? 25 : 5;

        switch (k) {
        case 1: base_speed   -= b_step; break;
        case 2: base_speed   += b_step; break;
        case 4: differential -= d_step; break;
        case 5: differential += d_step; break;
        case 6: coarse        = !coarse; break;
        case 7: /* 切换 启动/停止 */
            if (motor_on) { SPEED_Stop(); motor_on = 0; }
            else          { motor_on = 1;                   }
            break;
        }

        /* 限幅 */
        if (base_speed   < 0)              base_speed   = 0;
        if (base_speed   > SPEED_MAX)      base_speed   = SPEED_MAX;
        if (differential < -SPEED_MAX / 2) differential = -SPEED_MAX / 2;
        if (differential >  SPEED_MAX / 2) differential =  SPEED_MAX / 2;
    }
    last_key = k;

    /* ---- 运行中: 每帧刷新目标速度 ---- */
    if (motor_on) {
        int32_t L = base_speed - differential;
        int32_t R = base_speed + differential;

        if (L >  SPEED_MAX) L =  SPEED_MAX;
        if (L < -SPEED_MAX) L = -SPEED_MAX;
        if (R >  SPEED_MAX) R =  SPEED_MAX;
        if (R < -SPEED_MAX) R = -SPEED_MAX;

        SPEED_SetTarget(L, R);
    }

    /* ========== OLED 显示 (8行×16列) ========== */
    int32_t L = base_speed - differential;
    int32_t R = base_speed + differential;
    char buf[21];

    /* Line 0: 标题 + 粗/微调标志 */
    sprintf(buf, "CurveDebug [%s]", coarse ? "C" : "F");
    OLED_ShowString(0, 0, (uint8_t *)buf, 8);

    /* Line 1: 基础速度 */
    sprintf(buf, "Base:%+6d", (int)base_speed);
    OLED_ShowString(0, 1, (uint8_t *)buf, 8);

    /* Line 2: 差速 */
    sprintf(buf, "Diff:%+6d", (int)differential);
    OLED_ShowString(0, 2, (uint8_t *)buf, 8);

    /* Line 3: 左右轮实际目标速度 */
    sprintf(buf, "L:%+5d R:%+5d", (int)L, (int)R);
    OLED_ShowString(0, 3, (uint8_t *)buf, 8);

    /* Line 4~6: 按键提示 */
    OLED_ShowString(0, 4, (uint8_t *)"[1][2]Base [4][5]Diff", 8);
    OLED_ShowString(0, 5, (uint8_t *)"[6]Coarse [7]Run/Stop", 8);
    OLED_ShowString(0, 6, (uint8_t *)"[8]Stop motors", 8);

    /* Line 7: 状态 */
    sprintf(buf, "%s  9:Home", motor_on ? "**RUN**" : "STOP");
    OLED_ShowString(0, 7, (uint8_t *)buf, 8);
}

/* ==================================================================
 * Page_PWMTest — PWM→转速 校准页 (page_state = 8)
 *
 * 目的: 测试固定 PWM 占空比下电机的实际转速 (counts/s)
 *       直接调用 Motor_Set_Duty() 开环驱动, 不走 PID
 *       用于校准 PWM 与编码器速度的对应关系
 *
 * 显示:
 *   Line 0: 标题
 *   Line 1: 占空比 (0~100%)
 *   Line 2: 左右轮实时速度 (counts/s)
 *   Line 3: 编码器计数值
 *   Line 4: 校准系数 → 100% 占空比对应的预期速度
 *
 * 按键:
 *   1/2 = 占空比 -/+       4/5 = 快调 -/+
 *   6   = 微调/粗调切换    7   = 启动/停止电机
 *   8   = 紧急停止         9   = 返回 Home
 *
 * 步长: 微调 ±1%  粗调 ±5%  快调 ±10%
 * ================================================================== */
void Page_PWMTest(void)
{
    static int32_t pwm_pct     = 20;     /* PWM 占空比 (0~100%)         */
    static uint8_t coarse      = 0;      /* 0=微调  1=粗调              */
    static uint8_t motor_on    = 0;      /* 0=停止  1=电机运行          */
    static uint8_t last_key    = 0;

    uint8_t k = Scan_Keyboard();

    /* ---- 9: 停电机 + 返回 Home ---- */
    if (k == 9) {
        Motor_Stop(&motor_bl);
        Motor_Stop(&motor_br);
        motor_on = 0;
        while (Scan_Keyboard());
        page_state = 0;
        return;
    }

    /* ---- 8: 紧急停止 ---- */
    if (k == 8) {
        Motor_Stop(&motor_bl);
        Motor_Stop(&motor_br);
        motor_on = 0;
    }

    /* ---- 参数调节 (边沿触发) ---- */
    if (k != 0 && k != last_key) {
        int32_t step = coarse ? 5 : 1;   /* 微调 1%, 粗调 5% */

        switch (k) {
        case 1: pwm_pct -= step;    break;
        case 2: pwm_pct += step;    break;
        case 4: pwm_pct -= step*2;  break;  /* 快调: 2倍步长 */
        case 5: pwm_pct += step*2;  break;
        case 6: coarse = !coarse;   break;
        case 7: /* 切换 启动/停止 */
            if (motor_on) {
                Motor_Stop(&motor_bl);
                Motor_Stop(&motor_br);
                motor_on = 0;
            } else {
                motor_on = 1;
            }
            break;
        }

        if (pwm_pct < 0)    pwm_pct = 0;
        if (pwm_pct > 100)  pwm_pct = 100;
    }
    last_key = k;

    /* ---- 运行中: 每帧写入固定 PWM (Motor_Set_Duty 入参 0~1) ---- */
    if (motor_on) {
        float duty = (float)pwm_pct / 100.0f;
        Motor_Set_Duty(&motor_bl, duty);
        Motor_Set_Duty(&motor_br, duty);
    }

    /* ========== OLED 显示 (8行×16列) ========== */
    int32_t spdL = ENCODER_GetLeftSpeed();
    int32_t spdR = ENCODER_GetRightSpeed();
    int32_t cntL = (int32_t)(-g_leftEncoderCount);   /* 左轮取反, 正=前进 */
    int32_t cntR = (int32_t)g_rightEncoderCount;
    char buf[21];

    /* Line 0: 标题 */
    sprintf(buf, "PWM->Speed [%s]", coarse ? "C" : "F");
    OLED_ShowString(0, 0, (uint8_t *)buf, 8);

    /* Line 1: 占空比 + 对应的定时器原始值 */
    sprintf(buf, "Duty: %3d%% (%d/500)", (int)pwm_pct, (int)(pwm_pct * 5));
    OLED_ShowString(0, 1, (uint8_t *)buf, 8);

    /* Line 2: 实时速度 (counts/s) */
    sprintf(buf, "Spd L:%+5d R:%+5d", (int)spdL, (int)spdR);
    OLED_ShowString(0, 2, (uint8_t *)buf, 8);

    /* Line 3: 编码器计数值 */
    sprintf(buf, "Cnt L:%+5d R:%+5d", (int)cntL, (int)cntR);
    OLED_ShowString(0, 3, (uint8_t *)buf, 8);

    /* Line 4: 校准系数 = 100% 占空比对应的速度估算值
     *         公式: spdL × 100 / pwm_pct → 用于校准 SPEED_MAX */
    if (pwm_pct > 0 && spdL > 0) {
        sprintf(buf, "D2S:%4d ct/s@100%%", (int)(spdL * 100 / pwm_pct));
    } else {
        sprintf(buf, "D2S: ---          ");
    }
    OLED_ShowString(0, 4, (uint8_t *)buf, 8);

    /* Line 5~6: 按键提示 */
    OLED_ShowString(0, 5, (uint8_t *)"[1][2]Duty [4][5]Fast", 8);
    OLED_ShowString(0, 6, (uint8_t *)"[6]Coarse [7]Run/Stop", 8);

    /* Line 7: 状态 */
    sprintf(buf, "[8]Stop %s  9:Home", motor_on ? "**RUN**" : "STOP");
    OLED_ShowString(0, 7, (uint8_t *)buf, 8);
}
