/*
 * shape_test.c
 *
 * 简单图形绘制 (速度模式 / delay 阻塞)
 *
 * 图形:
 *   [1] Circle — motor1∝sin(phase), motor2∝cos(phase), 90° 相位差
 *   [2] Square — 四段直线: X+ → Y+ → X- → Y- → 循环
 *   [3] Zigzag — X 匀速前进, Y 周期性换向 → 锯齿波
 *   [4] Heart  — 爱心: x=16sin³(t), y=13cos(t)-5cos(2t)-...
 *
 * 调用: Page_Test → [3]Shape Draw → [1/2/3] 选图形
 *        [5] 停止当前图形  [8] 返回菜单
 */

#include "shape_test.h"
#include "ZDT_X42S/zdt_x42s.h"
#include "oled_hardware_i2c.h"
#include "clock.h"
#include "keyboard.h"
#include <math.h>
#include <stdio.h>

extern uint8_t oled_buffer[32];
extern ZDT_HandleTypeDef motor1, motor2;

/* ========== 可调参数 ========== */
#define SHAPE_SPD       5     /* 基础转速 RPM */
#define SHAPE_ACC       100     /* 加速度 */
#define SHAPE_PERIOD    3000    /* 一个完整图形周期 ms */
#define UPDATE_MS       100     /* 速度更新间隔 ms */
#define HEART_SCALE     5.0f    /* 爱心大小倍数 (越大心越大) */

/* ========== 内部辅助 ========== */

static void stop_both(void)
{
    ZDT_SpeedCtrl(&motor1, ZDT_DIR_CW, 0, SHAPE_ACC);
    ZDT_SpeedCtrl(&motor2, ZDT_DIR_CW, 0, SHAPE_ACC);
}

/*
 * 在 delay 间隙检查按键
 * 返回: 0=无操作  5=停止  8=退出
 */
static uint8_t wait_check(uint32_t ms)
{
    uint32_t chunks = ms / 50;
    for (uint32_t i = 0; i < chunks; i++) {
        uint8_t k = Scan_Keyboard();
        if (k == 5 || k == 8) return k;
        delay_ms(50);
    }
    return 0;
}

/* ================================================================
 * Circle: X∝sin(phase), Y∝cos(phase)
 * 两个电机速度都按正弦变化, 相位差 90° → 圆/椭圆轨迹
 * ================================================================ */
static uint8_t draw_circle(void)
{
    uint32_t steps = SHAPE_PERIOD / UPDATE_MS;

    while (1) {
        for (uint32_t i = 0; i < steps; i++) {
            float phase = 2.0f * 3.14159f * i / steps;
            float sin_v = sinf(phase);
            float cos_v = cosf(phase);

            /* motor1 ∝ sin: 过零点停, 正半周 CW, 负半周 CCW */
            uint16_t spd1 = (uint16_t)(SHAPE_SPD * fabsf(sin_v));
            uint8_t  dir1 = (sin_v >= 0.0f) ? ZDT_DIR_CW : ZDT_DIR_CCW;
            /* motor2 ∝ cos: 过零点停, 正半周 CW, 负半周 CCW */
            uint16_t spd2 = (uint16_t)(SHAPE_SPD * fabsf(cos_v));
            uint8_t  dir2 = (cos_v >= 0.0f) ? ZDT_DIR_CW : ZDT_DIR_CCW;

            ZDT_SpeedCtrl(&motor1, dir1, spd1, SHAPE_ACC);
            ZDT_SpeedCtrl(&motor2, dir2, spd2, SHAPE_ACC);

            uint8_t k = Scan_Keyboard();
            if (k == 8) { stop_both(); return 1; }
            if (k == 5) { stop_both(); return 0; }
            delay_ms(UPDATE_MS);
        }
    }
}

/* ================================================================
 * Square: X+(T) → Y+(T) → X-(T) → Y-(T) → 循环
 * 每段时长 period/4
 * ================================================================ */
static uint8_t draw_square(void)
{
    uint32_t side_ms = SHAPE_PERIOD / 4;
    uint8_t  rc;

    while (1) {
        /* 边1: X+  Y 停 */
        ZDT_SpeedCtrl(&motor1, ZDT_DIR_CW,  SHAPE_SPD, SHAPE_ACC);
        ZDT_SpeedCtrl(&motor2, ZDT_DIR_CW,  0,          SHAPE_ACC);
        rc = wait_check(side_ms);
        if (rc == 8) { stop_both(); return 1; }
        if (rc == 5) { stop_both(); return 0; }

        /* 边2: Y+  X 停 */
        ZDT_SpeedCtrl(&motor1, ZDT_DIR_CW,  0,          SHAPE_ACC);
        ZDT_SpeedCtrl(&motor2, ZDT_DIR_CW,  SHAPE_SPD, SHAPE_ACC);
        rc = wait_check(side_ms);
        if (rc == 8) { stop_both(); return 1; }
        if (rc == 5) { stop_both(); return 0; }

        /* 边3: X-  Y 停 */
        ZDT_SpeedCtrl(&motor1, ZDT_DIR_CCW, SHAPE_SPD, SHAPE_ACC);
        ZDT_SpeedCtrl(&motor2, ZDT_DIR_CW,  0,          SHAPE_ACC);
        rc = wait_check(side_ms);
        if (rc == 8) { stop_both(); return 1; }
        if (rc == 5) { stop_both(); return 0; }

        /* 边4: Y-  X 停 */
        ZDT_SpeedCtrl(&motor1, ZDT_DIR_CW,  0,          SHAPE_ACC);
        ZDT_SpeedCtrl(&motor2, ZDT_DIR_CCW, SHAPE_SPD, SHAPE_ACC);
        rc = wait_check(side_ms);
        if (rc == 8) { stop_both(); return 1; }
        if (rc == 5) { stop_both(); return 0; }
    }
}

/* ================================================================
 * Zigzag: X 匀速 CW, Y 周期换向
 * 前半 Y+, 后半 Y- → 锯齿/三角波
 * ================================================================ */
static uint8_t draw_zigzag(void)
{
    uint32_t half_ms = SHAPE_PERIOD / 2;
    uint8_t  rc;

    while (1) {
        /* 前半: Y+ */
        ZDT_SpeedCtrl(&motor1, ZDT_DIR_CW,  SHAPE_SPD, SHAPE_ACC);
        ZDT_SpeedCtrl(&motor2, ZDT_DIR_CW,  SHAPE_SPD, SHAPE_ACC);
        rc = wait_check(half_ms);
        if (rc == 8) { stop_both(); return 1; }
        if (rc == 5) { stop_both(); return 0; }

        /* 后半: Y- (X 不变) */
        ZDT_SpeedCtrl(&motor1, ZDT_DIR_CW,  SHAPE_SPD, SHAPE_ACC);
        ZDT_SpeedCtrl(&motor2, ZDT_DIR_CCW, SHAPE_SPD, SHAPE_ACC);
        rc = wait_check(half_ms);
        if (rc == 8) { stop_both(); return 1; }
        if (rc == 5) { stop_both(); return 0; }
    }
}

/* ================================================================
 * Heart: 爱心曲线
 *   x = 16·sin³(t)     → dx/dt = 48·sin²(t)·cos(t)
 *   y = 13·cos(t) - 5·cos(2t) - 2·cos(3t) - cos(4t)
 *     → dy/dt = -13·sin(t) + 10·sin(2t) + 6·sin(3t) + 4·sin(4t)
 * motor1 速度 ∝ |dx/dt|, motor2 速度 ∝ |dy/dt|
 * ================================================================ */
static uint8_t draw_heart(void)
{
    uint32_t steps = SHAPE_PERIOD / UPDATE_MS;

    while (1) {
        for (uint32_t i = 0; i < steps; i++) {
            float t     = 2.0f * 3.14159f * i / steps;
            float sin_t = sinf(t);
            float cos_t = cosf(t);

            /* dx/dt = 48·sin²(t)·cos(t) */
            float dx = 48.0f * sin_t * sin_t * cos_t;
            /* dy/dt = -13·sin(t) + 10·sin(2t) + 6·sin(3t) + 4·sin(4t) */
            float dy = -13.0f * sin_t
                     + 10.0f * sinf(2.0f * t)
                     +  6.0f * sinf(3.0f * t)
                     +  4.0f * sinf(4.0f * t);

            /* Scale to motor speed range, HEART_SCALE controls size */
            float scale = SHAPE_SPD / 48.0f * HEART_SCALE;
            int16_t spd1_raw = (int16_t)(fabsf(dx) * scale);
            int16_t spd2_raw = (int16_t)(fabsf(dy) * scale);

            uint16_t spd1 = (spd1_raw > 0) ? (uint16_t)spd1_raw : 0;
            uint16_t spd2 = (spd2_raw > 0) ? (uint16_t)spd2_raw : 0;
            uint8_t  dir1 = (dx >= 0.0f) ? ZDT_DIR_CW : ZDT_DIR_CCW;
            uint8_t  dir2 = (dy >= 0.0f) ? ZDT_DIR_CW : ZDT_DIR_CCW;

            ZDT_SpeedCtrl(&motor1, dir1, spd1, SHAPE_ACC);
            ZDT_SpeedCtrl(&motor2, dir2, spd2, SHAPE_ACC);

            uint8_t k = Scan_Keyboard();
            if (k == 8) { stop_both(); return 1; }
            if (k == 5) { stop_both(); return 0; }
            delay_ms(UPDATE_MS);
        }
    }
}

/* ================================================================
 * 对外接口
 * ================================================================ */
uint8_t ShapeTest_Tick(uint8_t key)
{
    /* 退出 */
    if (key == 8) {
        stop_both();
        return 1;
    }

    /* 选择图形 [1]=Circle [2]=Square [3]=Zigzag [4]=Heart */
    if (key >= 1 && key <= 4) {
        OLED_Clear();
        OLED_ShowString(0, 0, (uint8_t *)"Shape Draw", 8);
        OLED_ShowString(0, 7, (uint8_t *)"[5]Stop [8]Back", 8);

        uint8_t rc;
        switch (key) {
        case 1:
            OLED_ShowString(0, 2, (uint8_t *)"[1] Circle", 8);
            OLED_ShowString(0, 3, (uint8_t *)"X=sin Y=cos", 8);
            rc = draw_circle();
            break;
        case 2:
            OLED_ShowString(0, 2, (uint8_t *)"[2] Square", 8);
            OLED_ShowString(0, 3, (uint8_t *)"4-side loop", 8);
            rc = draw_square();
            break;
        case 3:
            OLED_ShowString(0, 2, (uint8_t *)"[3] Zigzag", 8);
            OLED_ShowString(0, 3, (uint8_t *)"X+ Y alt", 8);
            rc = draw_zigzag();
            break;
        case 4:
            OLED_ShowString(0, 2, (uint8_t *)"[4] Heart", 8);
            OLED_ShowString(0, 3, (uint8_t *)"<3 Love <3", 8);
            rc = draw_heart();
            break;
        default:
            rc = 0;
            break;
        }

        if (rc == 1) return 1;  /* 用户按了 8, 退出到菜单 */
        /* rc == 0: 用户按了 5, 回到形状选择界面 */
    }

    /* ---- 形状选择菜单 ---- */
    OLED_ShowString(0, 0, (uint8_t *)"Shape Menu", 8);
    OLED_ShowString(0, 1, (uint8_t *)"[1] Circle", 8);
    OLED_ShowString(0, 2, (uint8_t *)"[2] Square", 8);
    OLED_ShowString(0, 3, (uint8_t *)"[3] Zigzag", 8);
    OLED_ShowString(0, 4, (uint8_t *)"[4] Heart", 8);
    OLED_ShowString(0, 6, (uint8_t *)"Pick a shape", 8);
    OLED_ShowString(0, 7, (uint8_t *)"8:Back", 8);

    return 0;
}
