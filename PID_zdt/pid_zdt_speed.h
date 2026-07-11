/*
 * pid_zdt_speed.h
 *
 * 摄像头坐标速度环跟踪 (方案 A: 误差→转速)
 *
 * 核心算法:
 *   err = target - laser (像素)
 *   speed = Kp_speed × |err|, 限幅 [min_rpm, max_rpm]
 *   dir = sign(err)
 *   → ZDT_SpeedCtrl(motor, dir, speed, acc)
 *
 * 安全机制:
 *   - 激光超时急停 (默认 100ms)
 *   - 死区 + 迟滞 防止边界抖动
 *   - 最低转速阈值 防止低速无效爬行
 *
 * 与 CameraPID_Track 并存, 不修改任何原有文件
 * 复用 g_pid 中的坐标、更新标志、电机句柄
 */

#ifndef BSP_PID_PID_ZDT_SPEED_H_
#define BSP_PID_PID_ZDT_SPEED_H_

#include "pid_zdtnew.h"
#include <stdint.h>

/* ========== 配置结构 ========== */
typedef struct {
    float    kp_speed;          /* 像素→RPM 增益 */
    uint16_t max_rpm;           /* 最大转速 */
    uint16_t min_rpm;           /* 最低有效转速 (低于此值停转) */
    uint16_t deadband;          /* 死区 (像素) */
    uint16_t hysteresis;        /* 迟滞 (像素, 停转后需超 deadband+hyst 才重启) */
    uint32_t laser_timeout_ms;  /* 激光超时 (ms, 超时急停) */
    uint8_t  motor_acc;         /* 加速度档位 */
    uint8_t  invert_x;          /* X轴方向反转 (0=正常, 1=反转) */
    uint8_t  invert_y;          /* Y轴方向反转 (0=正常, 1=反转) */
} SpeedTrackCfg;

/* ========== API ========== */

/* 加载默认参数并初始化状态 */
void CameraPID_SpeedTrack_Init(void);

/* 自定义参数 (可选, 不调用则用默认值) */
void CameraPID_SpeedTrack_SetConfig(const SpeedTrackCfg *cfg);

/* 主循环中调用, 替代 CameraPID_Track */
void CameraPID_TrackSpeed(void);

/* 紧急停止两轴 */
void CameraPID_SpeedTrack_Stop(void);

#endif /* BSP_PID_PID_ZDT_SPEED_H_ */
