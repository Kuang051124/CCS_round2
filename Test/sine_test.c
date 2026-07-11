/*
 * sine_test.c
 *
 * 步进电机正弦波绘制 (速度模式 / delay 阻塞 / 往复)
 *
 * 原理:
 *   motor1 (X轴) — 匀速运动, 每 2 周期后停顿 0.5s 再反向
 *   motor2 (Y轴) — 速度 = SINE_SPD_Y_MAX * cos(phase), 跟踪正弦曲线
 *
 * 时序:
 *   [正转 2 周期] → [停 0.5s] → [反转 2 周期] → [停 0.5s] → 循环
 *
 * 调用: Page_Test → [1]Motor Test → [5] 启动, 再按 [5] 停止, [8] 退出
 */

#include "sine_test.h"
#include "ZDT_X42S/zdt_x42s.h"
#include "oled_hardware_i2c.h"
#include "clock.h"
#include "keyboard.h"
#include <math.h>
#include <stdio.h>

extern uint8_t oled_buffer[32];
extern ZDT_HandleTypeDef motor1, motor2;

/* ========== 可调参数 ========== */
#define SINE_SPD_X      2    /* X轴匀速 RPM */
#define SINE_SPD_Y_MAX  4    /* Y轴最大转速 RPM */
#define SINE_ACC        100     /* 加速度 */
#define SINE_CYC        2       /* 每趟正弦周期数 */
#define PASS_TIME_MS    4000    /* 单趟扫描时间 ms */
#define UPDATE_MS       100     /* Y轴速度更新间隔 ms */
#define PAUSE_MS        500     /* 换向停顿 ms */

/* ========== 内部函数 ========== */

static void stop_both(void)
{
    ZDT_SpeedCtrl(&motor1, ZDT_DIR_CW, 0, SINE_ACC);
    ZDT_SpeedCtrl(&motor2, ZDT_DIR_CW, 0, SINE_ACC);
}

/*
 * 阻塞式单趟绘制 + 停顿
 * 返回: 0=用户按了停止, 1=用户按了退出
 */
static uint8_t sine_one_pass(uint8_t dir_x)
{
    uint32_t steps = PASS_TIME_MS / UPDATE_MS;
    uint8_t  k;

    /* 启动 motor1, 匀速 */
    ZDT_SpeedCtrl(&motor1, dir_x, SINE_SPD_X, SINE_ACC);

    /* 逐段更新 motor2 速度 */
    for (uint32_t i = 0; i < steps; i++) {
        /* 检查按键 */
        k = Scan_Keyboard();
        if (k == 8) { stop_both(); return 1; }
        if (k == 5) { stop_both(); return 0; }

        /* 更新 motor2: speed = Ymax * |cos(phase)|, dir = sign(cos) */
        float t_frac = (float)i / steps;
        float phase  = 2.0f * 3.14159f * SINE_CYC * t_frac;
        float cos_v  = cosf(phase);

        uint16_t speed_y = (uint16_t)(SINE_SPD_Y_MAX * fabsf(cos_v));
        uint8_t  dir_y   = (cos_v >= 0.0f) ? ZDT_DIR_CW : ZDT_DIR_CCW;

        if (speed_y > 0) {
            ZDT_SpeedCtrl(&motor2, dir_y, speed_y, SINE_ACC);
        } else {
            ZDT_SpeedCtrl(&motor2, ZDT_DIR_CW, 0, SINE_ACC);
        }

        delay_ms(UPDATE_MS);
    }

    /* 停顿 0.5s, 期间仍可响应按键 */
    stop_both();
    for (uint32_t i = 0; i < PAUSE_MS / 50; i++) {
        k = Scan_Keyboard();
        if (k == 8) return 1;
        if (k == 5) return 0;
        delay_ms(50);
    }

    return 2;  /* 正常完成, 继续下一趟 */
}

/* ========== 对外接口 ========== */

uint8_t SineTest_Tick(uint8_t key)
{
    /* ---- 退出 ---- */
    if (key == 8) {
        stop_both();
        return 1;
    }

    /* ---- 启动阻塞绘制 ---- */
    if (key == 5) {
        uint8_t dir_x = ZDT_DIR_CW;

        OLED_ShowString(0, 5, (uint8_t *)"Drawing...", 8);

        while (1) {
            uint8_t rc = sine_one_pass(dir_x);
            if (rc == 1) {          /* 用户按了 8 → 退出 */
                return 1;
            } else if (rc == 0) {   /* 用户按了 5 → 停止 */
                return 0;
            }
            /* rc == 2: 正常完成一趟, 换向继续 */
            dir_x = (dir_x == ZDT_DIR_CW) ? ZDT_DIR_CCW : ZDT_DIR_CW;
        }
    }

    /* ========== 空闲显示 ========== */
    OLED_ShowString(0, 0, (uint8_t *)"Sine Speed", 8);

    sprintf((char *)oled_buffer, "X:%dRPM Cyc:%d",
            SINE_SPD_X, SINE_CYC);
    OLED_ShowString(0, 1, oled_buffer, 8);

    sprintf((char *)oled_buffer, "Ymax:%d Acc:%d",
            SINE_SPD_Y_MAX, SINE_ACC);
    OLED_ShowString(0, 2, oled_buffer, 8);

    sprintf((char *)oled_buffer, "Pass:%dms Pause:%dms",
            PASS_TIME_MS, PAUSE_MS);
    OLED_ShowString(0, 3, oled_buffer, 8);

    OLED_ShowString(0, 5, (uint8_t *)"[5] Start Draw", 8);
    OLED_ShowString(0, 7, (uint8_t *)"8:Back", 8);

    return 0;
}
