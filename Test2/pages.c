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
#include "BLUETOOTH/bluetooth.h"

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
    }
  }

  /* =========================================================== */

  OLED_ShowString(0, 0, (uint8_t *)"Home Page  9:Home", 8);
  OLED_ShowString(0, 1, (uint8_t *)"[1]Calib Page    ", 8);
  OLED_ShowString(0, 2, (uint8_t *)"[2]Debug Page    ", 8);
  OLED_ShowString(0, 3, (uint8_t *)"[3]Test Page     ", 8);
  OLED_ShowString(0, 4, (uint8_t *)"[4]Step Debug    ", 8);
  OLED_ShowString(0, 5, (uint8_t *)"[5]Gyro Debug    ", 8);
  OLED_ShowString(0, 6, (uint8_t *)"[6]Camera Debug  ", 8);
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
    {"SpdStr",  &t1.spd_straight,   0.28f,  0.01f,  0.05f},
    {"SpdArc",  &t1.spd_arc,        0.22f,  0.01f,  0.05f},
    {"ArcKP",   &t1.spd_arc_kp,     0.05f,  0.005f, 0.02f},
    {"GyroKP",  &t1.gyro_kp,        0.01f,  0.005f, 0.02f},
    {"OfsCW",   &t1.arc_ofs_cw,     0.05f,  0.005f, 0.02f},
    {"OfsCCW",  &t1.arc_ofs_ccw,   -0.05f,  0.005f, 0.02f},
    {"CW_T2",   &t1.arc_ofs_cw_t2,  0.08f,  0.005f, 0.02f},
    {"CCW_T2",  &t1.arc_ofs_ccw_t2,-0.08f,  0.005f, 0.02f},
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
 * Page_Debug - 调试页（page_state = 2）
 * TODO: 补充调试逻辑
 * ================================================================== */
void Page_Debug(void)
{
    if (Scan_Keyboard() == 9) { while (Scan_Keyboard()); page_state = 0; }
    OLED_ShowString(0, 0, (uint8_t *)"Debug Page", 8);
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

/* ModeB 调试按键 (B车无键盘, 独立按钮) */
#define MODE_B_PORT   GPIOB
#define MODE_B_PIN    DL_GPIO_PIN_21   /* PB21, 按下=LOW, 外部上拉 */
#define MODE_B_IOMUX  (IOMUX_PINCM49)  /* PB21 → PINCM49 */

static char b_last_cmd[21] = "--";

/* 解析蓝牙指令 → 返回目标模式 (只消费 TASK3/TASK4, 其余留给 task tick 处理) */
static uint8_t b_parse_command(void)
{
    if (!g_bt_rx_ready) return 0;

    uint8_t len = g_bt_rx_len;
    if (len > 20) len = 20;
    memcpy(b_last_cmd, (const void *)g_bt_rx_buf, len);
    b_last_cmd[len] = '\0';

    if (strncmp(b_last_cmd, "TASK3", 5) == 0) {
        g_bt_rx_ready = 0; g_bt_rx_len = 0;
        return B_MODE_TASK3;
    }
    if (strncmp(b_last_cmd, "TASK4", 5) == 0) {
        g_bt_rx_ready = 0; g_bt_rx_len = 0;
        return B_MODE_TASK4;
    }

    /* 未识别 (如 STOP): 不清 buffer, 留给 Task3B_Tick 等消费 */
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
            if (b_mode == B_MODE_IDLE)       b_mode = B_MODE_BTASK1;
            else if (b_mode == B_MODE_BTASK1) b_mode = B_MODE_BTASK2;
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

    /* ======== OLED 显示 ======== */
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
    char buf[21];
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
