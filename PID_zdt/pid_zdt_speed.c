/*
 * pid_zdt_speed.c
 *
 * 摄像头坐标速度环跟踪实现 (方案 A: 误差→转速)
 *
 * 与 CameraPID_Track() 并存, 不修改原有代码
 * 复用 g_pid 的坐标/标志/电机句柄, 中断帧解析保持不变
 */

#include "pid_zdt_speed.h"
#include "ZDT_X42S/zdt_x42s.h"
#include "clock.h"
#include <stdlib.h>

/* ========== 静态状态 (不侵入 CameraPID 结构体) ========== */
static SpeedTrackCfg s_cfg;
static uint32_t s_last_laser_tick = 0;  /* 上次收到激光的时刻 */
static uint8_t  s_was_stopped_x   = 1;  /* X 轴是否处于停止状态 (迟滞用) */
static uint8_t  s_was_stopped_y   = 1;  /* Y 轴是否处于停止状态 (迟滞用) */

/* ========== 内部: 单轴停止 ========== */
static void stop_x(void)
{
    if (g_pid.motor_x) {
        ZDT_SpeedCtrl(g_pid.motor_x, ZDT_DIR_CW, 0, s_cfg.motor_acc);
    }
    s_was_stopped_x = 1;
}

static void stop_y(void)
{
    if (g_pid.motor_y) {
        ZDT_SpeedCtrl(g_pid.motor_y, ZDT_DIR_CW, 0, s_cfg.motor_acc);
    }
    s_was_stopped_y = 1;
}

/* ========== 内部: 单轴速度控制 ========== */
static void drive_axis(ZDT_HandleTypeDef *motor, int16_t err,
                       uint8_t *was_stopped, uint8_t invert)
{
    /* 迟滞: 停止态需要更大的误差才能重启 */
    uint16_t threshold = *was_stopped
        ? s_cfg.deadband + s_cfg.hysteresis
        : s_cfg.deadband;

    if (abs(err) <= (int16_t)threshold) {
        /* 在死区/迟滞范围内 → 停 */
        if (motor) {
            ZDT_SpeedCtrl(motor, ZDT_DIR_CW, 0, s_cfg.motor_acc);
        }
        *was_stopped = 1;
        return;
    }

    /* 误差 → 转速 */
    uint16_t speed = (uint16_t)(s_cfg.kp_speed * abs(err));

    /* 限幅 */
    if (speed > s_cfg.max_rpm) speed = s_cfg.max_rpm;
    if (speed < s_cfg.min_rpm) {
        if (motor) {
            ZDT_SpeedCtrl(motor, ZDT_DIR_CW, 0, s_cfg.motor_acc);
        }
        *was_stopped = 1;
        return;
    }

    /* 方向: err>0 → CCW, invert 用 XOR 翻转 */
    uint8_t dir = ((err > 0) ? ZDT_DIR_CCW : ZDT_DIR_CW) ^ invert;

    if (motor) {
        ZDT_SpeedCtrl(motor, dir, speed, s_cfg.motor_acc);
    }
    *was_stopped = 0;
}

/* ========== 对外接口 ========== */

void CameraPID_SpeedTrack_Init(void)
{
    s_cfg.kp_speed         = 5.0f;
    s_cfg.max_rpm          = 500;
    s_cfg.min_rpm          = 20;
    s_cfg.deadband         = 3;
    s_cfg.hysteresis       = 2;
    s_cfg.laser_timeout_ms = 100;
    s_cfg.motor_acc        = 100;
    s_cfg.invert_x         = 0;
    s_cfg.invert_y         = 0;

    s_last_laser_tick = tick_ms;
    s_was_stopped_x   = 1;
    s_was_stopped_y   = 1;
}

void CameraPID_SpeedTrack_SetConfig(const SpeedTrackCfg *cfg)
{
    s_cfg = *cfg;
}

void CameraPID_SpeedTrack_Stop(void)
{
    stop_x();
    stop_y();
}

void CameraPID_TrackSpeed(void)
{
    /* ---- 0. 原子快照: 一次性读 ISR 共享字段, 防止中途被打断 ---- */
    uint8_t  lu = g_pid.laser_updated;
    uint8_t  tu = g_pid.target_updated;
    uint16_t tx = g_pid.target_x;
    uint16_t ty = g_pid.target_y;
    uint16_t lx = g_pid.laser_x;
    uint16_t ly = g_pid.laser_y;
    uint8_t  br = g_pid.both_ready;

    /* ---- 1. 激光坐标刷新 → 更新心跳 ---- */
    if (lu) {
        s_last_laser_tick = tick_ms;
    }

    /* ---- 2. 激光超时保护 ---- */
    if (tick_ms - s_last_laser_tick > s_cfg.laser_timeout_ms) {
        stop_x();
        stop_y();
        return;
    }

    /* ---- 3. 数据就绪检查 ---- */
    if (!br) return;
    if (!lu && !tu) return;

    /* ---- 4. 像素误差 (用快照值, 不重复读 g_pid) ---- */
    int16_t err_x = (int16_t)tx - (int16_t)lx;
    int16_t err_y = (int16_t)ty - (int16_t)ly;

    /* ---- 5. 逐轴速度控制 ---- */
    drive_axis(g_pid.motor_x, err_x, &s_was_stopped_x, s_cfg.invert_x);
    drive_axis(g_pid.motor_y, err_y, &s_was_stopped_y, s_cfg.invert_y);

    /* ---- 6. 清零标志, 等下一帧 ---- */
    g_pid.target_updated = 0;
    g_pid.laser_updated  = 0;
}
