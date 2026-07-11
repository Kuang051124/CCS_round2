/*
 * pid_store.c
 *
 * PID 参数 Flash 存储实现 (MSPM0G3507)
 * 扇区 127, 地址 0x1FC00, 56 字节配置数据
 */

#include "pid_store.h"
#include <string.h>

/* 工作副本 */
PID_FlashConfig g_pid_cfg;

/* 内部: 计算数据区 (offset 8..55) 的 XOR 校验和 */
static uint8_t pid_calc_checksum(const PID_FlashConfig *cfg)
{
    const uint8_t *p = (const uint8_t *)cfg;
    uint8_t sum = 0;
    for (int i = 8; i < (int)sizeof(PID_FlashConfig); i++) {
        sum ^= p[i];
    }
    return sum;
}

/* ========== 校验 ========== */

uint8_t PID_Store_IsValid(const PID_FlashConfig *cfg)
{
    if (cfg->magic != PID_FLASH_MAGIC)
        return 0;
    if (cfg->version != PID_FLASH_VERSION)
        return 0;
    if (cfg->checksum != pid_calc_checksum(cfg))
        return 0;
    return 1;
}

/* ========== 默认值 ========== */

void PID_Store_SetDefaults(PID_FlashConfig *cfg)
{
    memset(cfg, 0, sizeof(PID_FlashConfig));
    cfg->magic     = PID_FLASH_MAGIC;
    cfg->version   = PID_FLASH_VERSION;
    cfg->scale_x   = 10.0f;
    cfg->scale_y   = 10.0f;
    cfg->deadband  = 3;
    /* Kp/Ki/Kd, out_max, motor_speed, motor_acc 保持 0 (零 PID, 无输出) */
    cfg->checksum  = pid_calc_checksum(cfg);
}

/* ========== 从 Flash 加载 ========== */

void PID_Store_Load(PID_FlashConfig *cfg)
{
    /* Flash 是内存映射的, 直接读取 */
    const PID_FlashConfig *flash_cfg = (const PID_FlashConfig *)PID_FLASH_ADDR;

    if (PID_Store_IsValid(flash_cfg)) {
        memcpy(cfg, flash_cfg, sizeof(PID_FlashConfig));
    } else {
        /* 首次启动或数据损坏, 加载默认值 */
        PID_Store_SetDefaults(cfg);
    }
}

/* ========== 写入 Flash ========== */

void PID_Store_Save(const PID_FlashConfig *cfg)
{
    PID_FlashConfig buf;
    memcpy(&buf, cfg, sizeof(PID_FlashConfig));
    buf.checksum = pid_calc_checksum(&buf);

    /* 80MHz 下必须先清状态, 否则写保护可能误触发 */
    DL_FlashCTL_executeClearStatus(FLASHCTL);

    /* 禁中断: 擦除/写入期间 Flash 不可读, ISR 取指令会 fault */
    __disable_irq();

    /* 解除写保护 */
    DL_FlashCTL_unprotectSector(FLASHCTL, PID_FLASH_ADDR,
                                DL_FLASHCTL_REGION_SELECT_MAIN);

    /* 擦除扇区 */
    DL_FLASHCTL_COMMAND_STATUS status;
    status = DL_FlashCTL_eraseMemoryFromRAM(FLASHCTL, PID_FLASH_ADDR,
                                            DL_FLASHCTL_COMMAND_SIZE_SECTOR);

    if (status != DL_FLASHCTL_COMMAND_STATUS_PASSED) {
        /* 擦除失败, 再试一次 */
        DL_FlashCTL_executeClearStatus(FLASHCTL);
        DL_FlashCTL_unprotectSector(FLASHCTL, PID_FLASH_ADDR,
                                    DL_FLASHCTL_REGION_SELECT_MAIN);
        DL_FlashCTL_eraseMemoryFromRAM(FLASHCTL, PID_FLASH_ADDR,
                                       DL_FLASHCTL_COMMAND_SIZE_SECTOR);
    }

    /* 写入 56 字节 = 7 个 64-bit word
       每个 word 从 RAM 中取 2 个 uint32_t */
    const uint32_t *data = (const uint32_t *)&buf;
    for (int i = 0; i < 7; i++) {
        DL_FlashCTL_executeClearStatus(FLASHCTL);
        DL_FlashCTL_unprotectSector(FLASHCTL, PID_FLASH_ADDR,
                                    DL_FLASHCTL_REGION_SELECT_MAIN);
        DL_FlashCTL_programMemoryFromRAM64WithECCGenerated(
            FLASHCTL, PID_FLASH_ADDR + i * 8, &data[i * 2]);
    }

    __enable_irq();
}

/* ========== 应用到 CameraPID ========== */

void PID_Store_Apply(const PID_FlashConfig *cfg)
{
    CameraPID_SetTunings(cfg->kp_x, cfg->ki_x, cfg->kd_x,
                         cfg->kp_y, cfg->ki_y, cfg->kd_y);
    CameraPID_SetScale(cfg->scale_x, cfg->scale_y, cfg->out_max_x);
    /* SetScale 只接受一个 out_max 赋给两轴, Y 轴单独补写 */
    g_pid.pid_y.out_max = cfg->out_max_y;
    CameraPID_SetDeadband(cfg->deadband);
    CameraPID_SetMotorParams(cfg->motor_speed, cfg->motor_acc);
}
