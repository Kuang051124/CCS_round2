#include "pages.h"
#include "wit.h"
#include "keyboard.h"
#include "PID_zdt/pid_store.h"
#include "Test/sine_test.h"
#include "Test/shape_test.h"
#include "Test/track_test.h"
#include "clock.h"
#include <math.h>
#include <stdio.h>
#include <stddef.h>

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
    }
  }

  /* =========================================================== */

  OLED_ShowString(0, 0, (uint8_t *)"Home Page  9:Home", 8);
  OLED_ShowString(0, 1, (uint8_t *)"[1]Calib Page    ", 8);
  OLED_ShowString(0, 2, (uint8_t *)"[2]Debug Page    ", 8);
  OLED_ShowString(0, 3, (uint8_t *)"[3]Test Page     ", 8);
  OLED_ShowString(0, 4, (uint8_t *)"[4]Step Debug    ", 8);
  OLED_ShowString(0, 5, (uint8_t *)"[5]Gyro Debug    ", 8);
}

/* ==================================================================
 * Page_Calib - 校准页 / PID 调参 (page_state = 1)
 *
 * 键盘操作:
 *   1/2 = 上/下一个参数    3 = 恢复默认
 *   4/5 = 微调 -/+         8 = 粗调 +
 *   6   = 微调/粗调切换    7 = 保存到 Flash   9 = Home
 * ================================================================== */

/* 参数类型 */
typedef enum { PIDT_FLOAT, PIDT_U16, PIDT_U8 } PID_ParamType;

/* 参数定义表 */
typedef struct {
    const char *name;       /* OLED 显示名 (≤10 chars) */
    PID_ParamType type;     /* 值类型 */
    uint8_t offset;         /* 在 PID_FlashConfig 中的偏移 */
    float    def_f;         /* float 默认值 */
    uint16_t def_u16;       /* u16 默认值 */
    uint8_t  def_u8;        /* u8 默认值 */
    float    fine_step;     /* 微调步长 */
    float    coarse_step;   /* 粗调步长 */
} PID_ParamEntry;

static const PID_ParamEntry pid_param_table[] = {
    /* name       type          offset                               default     fine_step   coarse_step */
    {"Kp_X",     PIDT_FLOAT, offsetof(PID_FlashConfig, kp_x),       .def_f=0.0f, .fine_step=0.01f, .coarse_step=0.1f  },
    {"Ki_X",     PIDT_FLOAT, offsetof(PID_FlashConfig, ki_x),       .def_f=0.0f, .fine_step=0.001f,.coarse_step=0.01f },
    {"Kd_X",     PIDT_FLOAT, offsetof(PID_FlashConfig, kd_x),       .def_f=0.0f, .fine_step=0.01f, .coarse_step=0.1f  },
    {"Kp_Y",     PIDT_FLOAT, offsetof(PID_FlashConfig, kp_y),       .def_f=0.0f, .fine_step=0.01f, .coarse_step=0.1f  },
    {"Ki_Y",     PIDT_FLOAT, offsetof(PID_FlashConfig, ki_y),       .def_f=0.0f, .fine_step=0.001f,.coarse_step=0.01f },
    {"Kd_Y",     PIDT_FLOAT, offsetof(PID_FlashConfig, kd_y),       .def_f=0.0f, .fine_step=0.01f, .coarse_step=0.1f  },
    {"Scale_X",  PIDT_FLOAT, offsetof(PID_FlashConfig, scale_x),    .def_f=10.0f,.fine_step=0.1f,  .coarse_step=1.0f  },
    {"Scale_Y",  PIDT_FLOAT, offsetof(PID_FlashConfig, scale_y),    .def_f=10.0f,.fine_step=0.1f,  .coarse_step=1.0f  },
    {"OutMax_X", PIDT_FLOAT, offsetof(PID_FlashConfig, out_max_x),  .def_f=0.0f, .fine_step=1.0f,  .coarse_step=10.0f },
    {"OutMax_Y", PIDT_FLOAT, offsetof(PID_FlashConfig, out_max_y),  .def_f=0.0f, .fine_step=1.0f,  .coarse_step=10.0f },
    {"Deadband", PIDT_U16,   offsetof(PID_FlashConfig, deadband),   .def_u16=3,  .fine_step=1.0f,  .coarse_step=5.0f  },
    {"Speed",    PIDT_U16,   offsetof(PID_FlashConfig, motor_speed),.def_u16=0,  .fine_step=10.0f, .coarse_step=100.0f},
    {"Acc",      PIDT_U8,    offsetof(PID_FlashConfig, motor_acc),  .def_u8=0,   .fine_step=1.0f,  .coarse_step=10.0f },
};
#define PID_PARAM_COUNT (sizeof(pid_param_table) / sizeof(pid_param_table[0]))

/* 读取参数当前值 (返回 float 便于统一处理) */
static float pid_get_value(const PID_ParamEntry *e)
{
    uint8_t *base = (uint8_t *)&g_pid_cfg;
    switch (e->type) {
    case PIDT_FLOAT: return *(float *)(base + e->offset);
    case PIDT_U16:   return (float)*(uint16_t *)(base + e->offset);
    case PIDT_U8:    return (float)*(uint8_t *)(base + e->offset);
    default:         return 0.0f;
    }
}

/* 写入参数新值 */
static void pid_set_value(const PID_ParamEntry *e, float val)
{
    uint8_t *base = (uint8_t *)&g_pid_cfg;
    switch (e->type) {
    case PIDT_FLOAT: *(float *)(base + e->offset)    = val; break;
    case PIDT_U16:   *(uint16_t *)(base + e->offset) = (uint16_t)val; break;
    case PIDT_U8:    *(uint8_t *)(base + e->offset)  = (uint8_t)val; break;
    }
}

/* 恢复当前参数为默认值 */
static void pid_restore_default(uint8_t idx, const PID_ParamEntry *e)
{
    switch (e->type) {
    case PIDT_FLOAT: pid_set_value(e, e->def_f);   break;
    case PIDT_U16:   pid_set_value(e, e->def_u16); break;
    case PIDT_U8:    pid_set_value(e, e->def_u8);  break;
    }
    PID_Store_Apply(&g_pid_cfg); /* 立刻生效 */
}

void Page_Calib(void)
{
    static uint8_t  idx       = 0;     /* 当前参数索引 */
    static uint8_t  coarse    = 0;     /* 0=微调 1=粗调 */
    static uint8_t  last_key  = 0;     /* 上一帧按键 (边沿检测) */
    static uint8_t  saved     = 0;     /* 保存成功提示标志 */
    static uint32_t save_tick = 0;     /* 保存时刻的 tick_ms */

    uint8_t k = Scan_Keyboard();

    /* 按键边沿检测: 仅当新按下时处理 */
    if (k != 0 && k != last_key) {
        const PID_ParamEntry *e = &pid_param_table[idx];
        float val, step;
        switch (k) {
        case 1: /* 上一个参数 */
            idx = (idx == 0) ? PID_PARAM_COUNT - 1 : idx - 1;
            break;
        case 2: /* 下一个参数 */
            idx = (idx == PID_PARAM_COUNT - 1) ? 0 : idx + 1;
            break;
        case 3: /* 恢复默认 */
            pid_restore_default(idx, e);
            break;
        case 4: /* 微调 - */
            step = coarse ? e->coarse_step : e->fine_step;
            val  = pid_get_value(e) - step;
            pid_set_value(e, val);
            PID_Store_Apply(&g_pid_cfg);
            break;
        case 5: /* 微调 + */
            step = coarse ? e->coarse_step : e->fine_step;
            val  = pid_get_value(e) + step;
            pid_set_value(e, val);
            PID_Store_Apply(&g_pid_cfg);
            break;
        case 6: /* 粗调/微调切换 */
            coarse = !coarse;
            break;
        case 7: /* 保存到 Flash */
            PID_Store_Save(&g_pid_cfg);
            saved     = 1;
            save_tick = tick_ms;
            break;
        case 8: /* 粗调 + */
            step = e->coarse_step;
            val  = pid_get_value(e) + step;
            pid_set_value(e, val);
            PID_Store_Apply(&g_pid_cfg);
            break;
        case 9: /* 返回 Home */
            while (Scan_Keyboard());
            page_state = 0;
            break;
        }
    }
    last_key = k;

    /* 保存提示约 1 秒后自动清除 */
    if (saved && (tick_ms - save_tick > 1000)) {
        saved = 0;
    }

    /* ========== OLED 显示 ========== */
    const PID_ParamEntry *e = &pid_param_table[idx];

    /* 行0: 参数名 + 模式 */
    sprintf((char *)oled_buffer, "PID:%-10s[%c]", e->name, coarse ? 'C' : 'F');
    OLED_ShowString(0, 0, oled_buffer, 8);

    /* 行1: 当前值 */
    switch (e->type) {
    case PIDT_FLOAT:
        sprintf((char *)oled_buffer, "Val:%10.3f", pid_get_value(e));
        break;
    case PIDT_U16:
        sprintf((char *)oled_buffer, "Val:%10d", (uint16_t)pid_get_value(e));
        break;
    case PIDT_U8:
        sprintf((char *)oled_buffer, "Val:%10d", (uint8_t)pid_get_value(e));
        break;
    }
    OLED_ShowString(0, 1, oled_buffer, 8);

    /* 行2-3: 操作提示 */
    OLED_ShowString(0, 2, (uint8_t *)"4:Fine- 5:Fine+", 8);
    OLED_ShowString(0, 3, (uint8_t *)"6:Coars 8:Cors+", 8);

    /* 行4: 导航/保存 */
    OLED_ShowString(0, 4, (uint8_t *)"1/2:Sel 3:Rst", 8);

    /* 行5: 保存状态 + Home */
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

    if (k == 9) { while (Scan_Keyboard()); page_state = 0; return; }

    /* 子任务 delegate: test_mode > 0 时交给对应模块处理 */
    // if (test_mode == 1) {
    //     if (SineTest_Tick(k)) { test_mode = 0; prev_mode = 0xFF; }
    //     return;
    // }
    // if (test_mode == 3) {
    //     if (ShapeTest_Tick(k)) { test_mode = 0; prev_mode = 0xFF; }
    //     return;
    // }
    // if (test_mode == 4) {
    //     if (TrackPos_Tick(k)) { test_mode = 0; prev_mode = 0xFF; }
    //     return;
    // }
    // if (test_mode == 5) {
    //     if (TrackSpeed_Tick(k)) { test_mode = 0; prev_mode = 0xFF; }
    //     return;
    // }

    /* ---- 菜单模式 ---- */
    if (k != 0) test_mode = k;

    if (test_mode != prev_mode) {
        OLED_Clear();
        prev_mode = test_mode;
    }

    OLED_ShowString(0, 0, (uint8_t *)"Test Menu", 8);
    OLED_ShowString(0, 1, (uint8_t *)"[1]Motor Test", 8);
    OLED_ShowString(0, 2, (uint8_t *)"[2]Sensor Tst", 8);
    OLED_ShowString(0, 3, (uint8_t *)"[3]Shape Draw", 8);
    OLED_ShowString(0, 4, (uint8_t *)"[4]Track Pos", 8);
    OLED_ShowString(0, 5, (uint8_t *)"[5]Track Speed", 8);
    OLED_ShowString(0, 7, (uint8_t *)"9:Home", 8);
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
    /* 使用标准位置模式 (FD命令), 协议格式: Addr FD Dir Speed Acc Pulses[4] Mode Sync 6B */
    /* 例: 01 FD 01 01 F4 50 00 00 0C 80 02 00 6B → CCW,500RPM,acc80,3200脉冲(1圈) */

    ZDT_PosCtrl(&motor1, ZDT_DIR_CW,  50, 80, 320, ZDT_MODE_REL_CURRENT);
    delay_ms(2000);

    ZDT_PosCtrl(&motor1, ZDT_DIR_CCW, 50, 80, 320, ZDT_MODE_REL_CURRENT);
    delay_ms(2000);

    ZDT_PosCtrl(&motor2, ZDT_DIR_CW,  50, 80, 320, ZDT_MODE_REL_CURRENT);
    delay_ms(2000);

    ZDT_PosCtrl(&motor2, ZDT_DIR_CCW, 50, 80, 320, ZDT_MODE_REL_CURRENT);
    delay_ms(2000);
//     ZDT_SpeedCtrl(&motor1, ZDT_DIR_CW, 1000, 120);
// ZDT_SpeedCtrl(&motor2, ZDT_DIR_CW, 1000, 120);

// ZDT_FastPosSetParams(&motor2, 800, 100, ZDT_MODE_REL_PREV);  // 800RPM, 加速100, 相对模式

// ZDT_FastPosMove(&motor2, 3200);    // 转 1 圈正转
// ZDT_FastPosSetParams(&motor1, 800, 100, ZDT_MODE_REL_PREV);  // 800RPM, 加速100, 相对模式

// ZDT_FastPosMove(&motor1, 3200);    // 转 1 圈正转

    test_done = 1;
      }
}

/* ==================================================================
 * Page_Debug_Gyro - 陀螺仪调试页（page_state = 5）
 * ================================================================== */
void Page_Debug_Gyro(void)
{
    if (Scan_Keyboard() == 9) { while (Scan_Keyboard()); page_state = 0; }
    OLED_ShowString(0, 0, (uint8_t *)"Gyro Debug", 8);
    OLED_ShowString(0, 7, (uint8_t *)"9:Home", 8);
    
    sprintf((char *)oled_buffer, "Pitch:%6.1f", wit_data.pitch);
    OLED_ShowString(0, 2, oled_buffer, 8);

    sprintf((char *)oled_buffer, "Roll :%6.1f", wit_data.roll);
    OLED_ShowString(0, 3, oled_buffer, 8);

    sprintf((char *)oled_buffer, "Yaw  :%6.1f", wit_data.yaw);
    OLED_ShowString(0, 4, oled_buffer, 8);

    sprintf((char *)oled_buffer, "Temp :%5.1fC", wit_data.temperature);
    OLED_ShowString(0, 5, oled_buffer, 8);
}
