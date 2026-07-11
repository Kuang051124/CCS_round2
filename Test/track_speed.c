/*
 * track_speed.c
 *
 * 速度模式目标跟踪 (模仿 demo3 任务4, 改用速度环)
 *
 * 流程:
 *   1. 开激光
 *   2. 循环调 CameraPID_TrackSpeed() → 误差→转速 → ZDT_SpeedCtrl
 *   3. 检测激光是否到达目标 (两轴误差 ≤ deadband)
 *   4. 在目标上保持 2 秒 → 命中
 *   5. [5] 停止  [8] 退出
 *
 * 与 track_pos.c 的唯一区别: 调用 CameraPID_TrackSpeed() 替代 CameraPID_Track()
 */

#include "track_test.h"
#include "ZDT_X42S/zdt_x42s.h"
#include "PID_zdt/pid_zdt_speed.h"
#include "oled_hardware_i2c.h"
#include "clock.h"
#include "keyboard.h"
#include <stdio.h>
#include <stdlib.h>

extern uint8_t oled_buffer[32];
extern ZDT_HandleTypeDef motor1, motor2;

#define HOLD_MS  2000

uint8_t TrackSpeed_Tick(uint8_t key)
{
    if (key == 8) {
        CameraPID_SpeedTrack_Stop();
        DL_GPIO_clearPins(LASER_PORT, LASER_PURPLE_PIN);
        return 1;
    }

    if (key == 5) {
        uint8_t  hit        = 0;
        uint32_t hold_start = 0;
        uint32_t last_disp  = 0;

        DL_GPIO_setPins(LASER_PORT, LASER_PURPLE_PIN);

        while (!hit) {
            uint8_t k = Scan_Keyboard();
            if (k == 8) {
                CameraPID_SpeedTrack_Stop();
                DL_GPIO_clearPins(LASER_PORT, LASER_PURPLE_PIN);
                return 1;
            }
            if (k == 5) {
                CameraPID_SpeedTrack_Stop();
                DL_GPIO_clearPins(LASER_PORT, LASER_PURPLE_PIN);
                return 0;
            }

            /* 速度环 PID 跟踪 (内部: 误差→转速 → ZDT_SpeedCtrl) */
            CameraPID_TrackSpeed();

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

            if (tick_ms - last_disp >= 200) {
                OLED_ShowString(0, 0, (uint8_t *)"Track Speed", 8);

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

            delay_ms(5);
        }

        DL_GPIO_clearPins(LASER_PORT, LASER_PURPLE_PIN);
        OLED_ShowString(0, 4, (uint8_t *)"** HIT! **", 8);
        delay_ms(1000);
        return 0;
    }

    /* ========== 空闲显示 ========== */
    OLED_ShowString(0, 0, (uint8_t *)"Track Speed", 8);
    OLED_ShowString(0, 1, (uint8_t *)"Speed Loop mode", 8);
    OLED_ShowString(0, 2, (uint8_t *)"ZDT_SpeedCtrl", 8);
    OLED_ShowString(0, 5, (uint8_t *)"[5] Start Track", 8);
    OLED_ShowString(0, 7, (uint8_t *)"8:Back", 8);

    return 0;
}
