/*
 * track_pos.c
 *
 * 位置模式目标跟踪 (模仿 demo3 任务4)
 *
 * 流程:
 *   1. 开激光
 *   2. 循环调 CameraPID_Track() → PID 算脉冲 → ZDT_FastPosMove
 *   3. 检测激光是否到达目标 (两轴误差 ≤ deadband)
 *   4. 在目标上保持 2 秒 → 命中
 *   5. [5] 停止  [8] 退出
 */

#include "track_test.h"
#include "ZDT_X42S/zdt_x42s.h"
#include "PID_zdt/pid_zdtnew.h"
#include "oled_hardware_i2c.h"
#include "clock.h"
#include "keyboard.h"
#include <stdio.h>
#include <stdlib.h>

extern uint8_t oled_buffer[32];
extern ZDT_HandleTypeDef motor1, motor2;

#define HOLD_MS  2000   /* 目标上停留时间 ms */

static void stop_motors(void)
{
    ZDT_SpeedCtrl(&motor1, ZDT_DIR_CW, 0, 100);
    ZDT_SpeedCtrl(&motor2, ZDT_DIR_CW, 0, 100);
}

uint8_t TrackPos_Tick(uint8_t key)
{
    if (key == 8) {
        stop_motors();
        DL_GPIO_clearPins(LASER_PORT, LASER_PURPLE_PIN);
        return 1;
    }

    if (key == 5) {
        uint8_t  hit        = 0;
        uint32_t hold_start = 0;
        uint32_t last_disp  = 0;

        DL_GPIO_setPins(LASER_PORT, LASER_PURPLE_PIN);

        while (!hit) {
            /* 按键检测 */
            uint8_t k = Scan_Keyboard();
            if (k == 8) {
                stop_motors();
                DL_GPIO_clearPins(LASER_PORT, LASER_PURPLE_PIN);
                return 1;
            }
            if (k == 5) {
                stop_motors();
                DL_GPIO_clearPins(LASER_PORT, LASER_PURPLE_PIN);
                return 0;
            }

            /* 位置 PID 跟踪 (内部: Compute → FastPosMove) */
            CameraPID_Track();

            /* 到达判断 */
            if (g_pid.both_ready) {
                int16_t dx = abs((int16_t)g_pid.target_x - (int16_t)g_pid.laser_x);
                int16_t dy = abs((int16_t)g_pid.target_y - (int16_t)g_pid.laser_y);
                if (dx <= (int16_t)g_pid.deadband && dy <= (int16_t)g_pid.deadband) {
                    if (hold_start == 0) hold_start = tick_ms;
                    if (tick_ms - hold_start >= HOLD_MS) {
                        hit = 1;
                    }
                } else {
                    hold_start = 0;
                }
            }

            /* OLED 刷新 (~5Hz, 减少 I2C 开销) */
            if (tick_ms - last_disp >= 200) {
                OLED_ShowString(0, 0, (uint8_t *)"Track Pos", 8);

                sprintf((char *)oled_buffer, "T:%03d,%03d",
                        g_pid.target_x, g_pid.target_y);
                OLED_ShowString(0, 1, oled_buffer, 8);

                sprintf((char *)oled_buffer, "L:%03d,%03d",
                        g_pid.laser_x, g_pid.laser_y);
                OLED_ShowString(0, 2, oled_buffer, 8);

                if (hold_start > 0) {
                    sprintf((char *)oled_buffer, "Hold:%1ds/2s",
                            (int)((tick_ms - hold_start) / 1000));
                    OLED_ShowString(0, 3, oled_buffer, 8);
                } else if (g_pid.both_ready) {
                    int16_t dx = abs((int16_t)g_pid.target_x - (int16_t)g_pid.laser_x);
                    int16_t dy = abs((int16_t)g_pid.target_y - (int16_t)g_pid.laser_y);
                    sprintf((char *)oled_buffer, "Err:%d,%d", dx, dy);
                    OLED_ShowString(0, 3, oled_buffer, 8);
                }

                OLED_ShowString(0, 7, (uint8_t *)"5:Stop 8:Back", 8);
                last_disp = tick_ms;
            }

            delay_ms(5);  /* ~200Hz PID 周期 */
        }

        /* 命中! */
        DL_GPIO_clearPins(LASER_PORT, LASER_PURPLE_PIN);
        OLED_ShowString(0, 4, (uint8_t *)"** HIT! **", 8);
        delay_ms(1000);
        return 0;
    }

    /* ========== 空闲显示 ========== */
    OLED_ShowString(0, 0, (uint8_t *)"Track Pos", 8);
    OLED_ShowString(0, 1, (uint8_t *)"Position PID mode", 8);
    OLED_ShowString(0, 2, (uint8_t *)"ZDT_FastPosMove", 8);
    OLED_ShowString(0, 5, (uint8_t *)"[5] Start Track", 8);
    OLED_ShowString(0, 7, (uint8_t *)"8:Back", 8);

    return 0;
}
