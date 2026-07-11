/*
 * pid_store.h
 *
 * PID 参数 Flash 存储 (MSPM0G3507)
 * 使用 Flash 扇区 127 (0x1FC00), 1KB
 * 存储 56 字节配置数据, magic+checksum 校验
 */

#ifndef BSP_PID_PID_STORE_H_
#define BSP_PID_PID_STORE_H_

#include "pid_zdtnew.h"
#include <stdint.h>

/* Flash 扇区 127 起始地址 (128KB Flash, 每扇区 1KB) */
#define PID_FLASH_ADDR    0x0001FC00

/* 幻数: "PIDC" = 0x50494443 */
#define PID_FLASH_MAGIC   0x50494443
#define PID_FLASH_VERSION 1

/* 存储结构: 56 字节 = 7 个 64-bit flash word */
typedef struct {
    /* 头部 (8 bytes) */
    uint32_t magic;          /* 0x50494443 */
    uint16_t version;        /* 1 */
    uint8_t  checksum;       /* 数据区 (offset 8..55) 的 XOR */
    uint8_t  header_pad;     /* 保留, 始终 0 */

    /* PID - X 轴 (12 bytes) */
    float kp_x;
    float ki_x;
    float kd_x;

    /* PID - Y 轴 (12 bytes) */
    float kp_y;
    float ki_y;
    float kd_y;

    /* 标定 (16 bytes) */
    float scale_x;
    float scale_y;
    float out_max_x;
    float out_max_y;

    /* 电机 + 死区 (8 bytes) */
    uint16_t deadband;
    uint16_t motor_speed;
    uint8_t  motor_acc;
    uint8_t  reserved[3];    /* 对齐到 8 字节边界 */
} PID_FlashConfig;

/* 工作副本, 在 pid_store.c 中定义 */
extern PID_FlashConfig g_pid_cfg;

/* ========== API ========== */

/* 从 Flash 加载配置到 cfg, 无效则填默认值 */
void PID_Store_Load(PID_FlashConfig *cfg);

/* 将 cfg 写入 Flash */
void PID_Store_Save(const PID_FlashConfig *cfg);

/* 校验 cfg 的 magic+checksum, 返回 1=有效 0=无效 */
uint8_t PID_Store_IsValid(const PID_FlashConfig *cfg);

/* 填充默认值 */
void PID_Store_SetDefaults(PID_FlashConfig *cfg);

/* 将 cfg 写入 g_pid (CameraPID 全局实例) */
void PID_Store_Apply(const PID_FlashConfig *cfg);

#endif /* BSP_PID_PID_STORE_H_ */
