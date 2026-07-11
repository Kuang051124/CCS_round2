/*
 * zdt_x42s.h
 *
 * ZDT_X42S 闭环步进电机驱动 (MSPM0G3507 移植版)
 * 协议: RS485/UART, 115200-8-N-1
 * 电机地址: 1-247 (默认1)
 *
 * 移植说明:
 *   原版基于 STM32 HAL, 现移植到 MSPM0 DriverLib
 *   - UART_HandleTypeDef* → UART_Regs*
 *   - HAL_UART_Transmit    → DL_UART_transmitDataBlocking (逐字节)
 *   - HAL_UART_Receive     → DL_UART_receiveDataBlocking  (逐字节)
 *
 * 电机1 UART 引脚配置 (UART_ZDT, 硬件 UART3 Extend):
 *   TX: PA26 (IOMUX PINCM59, UART3_TX)
 *   RX: PA13 (IOMUX PINCM35, UART3_RX)  ← 原键盘 C3, 已移至 PA12
 *   波特率: 115200, 8-N-1
 *
 * 电机2 UART 引脚配置 (UART_ZDT2, 硬件 UART1 Main):
 *   TX: PB4  (IOMUX PINCM17, UART1_TX)
 *   RX: PB5  (IOMUX PINCM18, UART1_RX)
 *   波特率: 115200, 8-N-1
 *
 * 使用示例:
 *   ZDT_Init(&motor1, UART_ZDT_INST, 1);   // 电机1, 地址1
 *   ZDT_Init(&motor2, UART_ZDT2_INST, 1);  // 电机2, 地址1 (独立串口)
 */

#ifndef BSP_ZDT_X42S_ZDT_X42S_H_
#define BSP_ZDT_X42S_ZDT_X42S_H_

#include "ti_msp_dl_config.h"

/* 脉冲换算: 1.8°步进电机, 16细分 */
#define ZDT_PULSES_PER_REV  3200    /* 3200 脉冲 = 360° */
#define ZDT_DEG_PER_PULSE   0.1125f /* 1 脉冲 = 0.1125° */

/* 运动模式 */
#define ZDT_MODE_REL_PREV    0x00   /* 相对上一目标位置 */
#define ZDT_MODE_ABS_ZERO    0x01   /* 相对坐标零点绝对位置 */
#define ZDT_MODE_REL_CURRENT 0x02   /* 相对当前实时位置 */

/* 方向 */
#define ZDT_DIR_CW  0x00  /* 顺时针 */
#define ZDT_DIR_CCW 0x01  /* 逆时针 */

/* 同步标志 */
#define ZDT_SYNC_IMMEDIATE 0x00  /* 立即执行 */
#define ZDT_SYNC_CACHE     0x01  /* 先缓存 */

/* 回零模式 */
#define ZDT_HOME_NEAREST    0x00  /* 单圈就近回零 */
#define ZDT_HOME_DIRECTION  0x01  /* 单圈方向回零 */
#define ZDT_HOME_COLLISION  0x02  /* 无限位碰撞回零 */
#define ZDT_HOME_LIMIT      0x03  /* 限位回零 */
#define ZDT_HOME_ABS_ZERO   0x04  /* 回到绝对位置零点 */
#define ZDT_HOME_LAST_POWER 0x05  /* 回到上次掉电位置 */

/* 电机句柄 */
typedef struct {
    UART_Regs *uart;    /* MSPM0 UART 外设基址 (如 UART3) */
    uint8_t addr;       /* 电机地址 (1-247) */
} ZDT_HandleTypeDef;

/* ========== 初始化 ========== */
void ZDT_Init(ZDT_HandleTypeDef *zdt, UART_Regs *uart, uint8_t addr);

/* ========== 快速位置模式 (推荐) ========== */
void ZDT_FastPosSetParams(ZDT_HandleTypeDef *zdt, uint16_t speed_rpm,
                          uint8_t acc, uint8_t mode);
void ZDT_FastPosMove(ZDT_HandleTypeDef *zdt, int32_t pulses);
void ZDT_FastPosMoveAngle(ZDT_HandleTypeDef *zdt, float angle_deg,
                          uint16_t speed_rpm, uint8_t acc, uint8_t mode);

/* ========== 标准位置模式 ========== */
void ZDT_PosCtrl(ZDT_HandleTypeDef *zdt, uint8_t dir, uint16_t speed_rpm,
                 uint8_t acc, int32_t pulses, uint8_t mode);

/* ========== 速度模式 ========== */
void ZDT_SpeedCtrl(ZDT_HandleTypeDef *zdt, uint8_t dir, uint16_t speed_rpm,
                   uint8_t acc);

/* ========== 回零 ========== */
void ZDT_SetHomeZero(ZDT_HandleTypeDef *zdt, uint8_t store);
void ZDT_TriggerHome(ZDT_HandleTypeDef *zdt, uint8_t mode);
void ZDT_AbortHome(ZDT_HandleTypeDef *zdt);
uint8_t ZDT_ReadHomeStatus(ZDT_HandleTypeDef *zdt);

/* ========== 参数设置 ========== */
void ZDT_SetMicrostep(ZDT_HandleTypeDef *zdt, uint8_t value, uint8_t store);

/* ========== 工具函数 ========== */
int32_t ZDT_AngleToPulses(float angle_deg);
float ZDT_PulsesToAngle(int32_t pulses);

#endif /* BSP_ZDT_X42S_ZDT_X42S_H_ */
