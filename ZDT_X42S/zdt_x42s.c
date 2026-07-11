/*
 * zdt_x42s.c
 *
 * ZDT_X42S 闭环步进电机驱动实现 (MSPM0G3507 移植版)
 * 协议帧尾字节固定为 0x6B (非累加校验)
 *
 * 移植变更:
 *   HAL_UART_Transmit() → DL_UART_transmitDataBlocking() (逐字节循环)
 *   HAL_UART_Receive()  → DL_UART_receiveDataBlocking()  (逐字节循环)
 */

#include "zdt_x42s.h"

/* 协议固定尾字节 */
#define ZDT_TAIL  0x6B

/* ========== 内部辅助函数 ========== */

/**
 * @brief 逐字节阻塞发送 (替代 HAL_UART_Transmit)
 */
static void ZDT_SendCmd(ZDT_HandleTypeDef *zdt, uint8_t *data, uint8_t len)
{
    uint8_t i;
    for (i = 0; i < len; i++) {
        DL_UART_transmitDataBlocking(zdt->uart, data[i]);
    }
}

/**
 * @brief 逐字节阻塞接收 (替代 HAL_UART_Receive)
 * @return 成功接收的字节数 (始终为 len, 阻塞无限等待)
 */
static uint8_t ZDT_ReceiveData(ZDT_HandleTypeDef *zdt, uint8_t *rx, uint8_t len)
{
    uint8_t i;
    for (i = 0; i < len; i++) {
        rx[i] = DL_UART_receiveDataBlocking(zdt->uart);
    }
    return len;
}

/* ========== 初始化 ========== */

void ZDT_Init(ZDT_HandleTypeDef *zdt, UART_Regs *uart, uint8_t addr)
{
    zdt->uart = uart;
    zdt->addr = addr;
}

/* ========== 快速位置模式 ========== */

void ZDT_FastPosSetParams(ZDT_HandleTypeDef *zdt, uint16_t speed_rpm,
                          uint8_t acc, uint8_t mode)
{
    uint8_t cmd[8];
    cmd[0] = zdt->addr;
    cmd[1] = 0xF1;
    cmd[2] = (uint8_t)(speed_rpm >> 8);
    cmd[3] = (uint8_t)(speed_rpm & 0xFF);
    cmd[4] = acc;
    cmd[5] = mode;
    cmd[6] = 0x00;
    cmd[7] = ZDT_TAIL;
    ZDT_SendCmd(zdt, cmd, 8);
}

void ZDT_FastPosMove(ZDT_HandleTypeDef *zdt, int32_t pulses)
{
    uint8_t cmd[7];
    cmd[0] = zdt->addr;
    cmd[1] = 0xFC;
    cmd[2] = (uint8_t)((pulses >> 24) & 0xFF);
    cmd[3] = (uint8_t)((pulses >> 16) & 0xFF);
    cmd[4] = (uint8_t)((pulses >> 8) & 0xFF);
    cmd[5] = (uint8_t)(pulses & 0xFF);
    cmd[6] = ZDT_TAIL;
    ZDT_SendCmd(zdt, cmd, 7);
}

void ZDT_FastPosMoveAngle(ZDT_HandleTypeDef *zdt, float angle_deg,
                          uint16_t speed_rpm, uint8_t acc, uint8_t mode)
{
    ZDT_FastPosSetParams(zdt, speed_rpm, acc, mode);
    int32_t pulses = ZDT_AngleToPulses(angle_deg);
    ZDT_FastPosMove(zdt, pulses);
}

/* ========== 标准位置模式 ========== */

void ZDT_PosCtrl(ZDT_HandleTypeDef *zdt, uint8_t dir, uint16_t speed_rpm,
                 uint8_t acc, int32_t pulses, uint8_t mode)
{
    uint8_t cmd[13];
    cmd[0]  = zdt->addr;
    cmd[1]  = 0xFD;
    cmd[2]  = dir;
    cmd[3]  = (uint8_t)(speed_rpm >> 8);
    cmd[4]  = (uint8_t)(speed_rpm & 0xFF);
    cmd[5]  = acc;
    cmd[6]  = (uint8_t)((pulses >> 24) & 0xFF);
    cmd[7]  = (uint8_t)((pulses >> 16) & 0xFF);
    cmd[8]  = (uint8_t)((pulses >> 8) & 0xFF);
    cmd[9]  = (uint8_t)(pulses & 0xFF);
    cmd[10] = mode;
    cmd[11] = 0x00;
    cmd[12] = ZDT_TAIL;
    ZDT_SendCmd(zdt, cmd, 13);
}

/* ========== 速度模式 ========== */

void ZDT_SpeedCtrl(ZDT_HandleTypeDef *zdt, uint8_t dir, uint16_t speed_rpm,
                   uint8_t acc)
{
    uint8_t cmd[8];
    cmd[0] = zdt->addr;
    cmd[1] = 0xF6;
    cmd[2] = dir;
    cmd[3] = (uint8_t)(speed_rpm >> 8);
    cmd[4] = (uint8_t)(speed_rpm & 0xFF);
    cmd[5] = acc;
    cmd[6] = 0x00;
    cmd[7] = ZDT_TAIL;
    ZDT_SendCmd(zdt, cmd, 8);
}

/* ========== 回零 ========== */

/*
 * 回零典型流程:
 *   1. 上电后先手动把电机转到期望的零点位置
 *   2. ZDT_SetHomeZero(&motor, 1);      // 将当前位置标记为零点并存储
 *   3. ZDT_TriggerHome(&motor, ZDT_HOME_NEAREST);  // 触发回零
 *   4. 轮询 ZDT_ReadHomeStatus() 确认回零完成
 *   5. 之后每次上电调用 ZDT_TriggerHome() 即可自动找回零点
 */

/**
 * @brief 将电机当前角度设置为单圈回零的零点
 * @param store  0=不存储(掉电丢失), 1=存储(掉电记忆)
 *        调用后当前角度 = 单圈零点, 后续 TriggerHome 以此为基准
 */
void ZDT_SetHomeZero(ZDT_HandleTypeDef *zdt, uint8_t store)
{
    uint8_t cmd[5];
    cmd[0] = zdt->addr;
    cmd[1] = 0x93;
    cmd[2] = 0x88;
    cmd[3] = store;
    cmd[4] = ZDT_TAIL;
    ZDT_SendCmd(zdt, cmd, 5);
}

/**
 * @brief 触发回零, 电机自动运动到零点位置
 * @param mode  回零模式:
 *        0=ZDT_HOME_NEAREST    单圈就近回零 (最常用, 找最近零点)
 *        1=ZDT_HOME_DIRECTION  单圈方向回零 (指定方向绕一圈找零点)
 *        2=ZDT_HOME_COLLISION  碰撞回零 (走到堵转, 需配合硬件限位)
 *        3=ZDT_HOME_LIMIT      限位回零 (需外部限位开关)
 *        4=ZDT_HOME_ABS_ZERO   回到绝对位置坐标零点
 *        5=ZDT_HOME_LAST_POWER 回到上次掉电位置
 *        回零中可调用 ZDT_AbortHome() 强制中断
 */
void ZDT_TriggerHome(ZDT_HandleTypeDef *zdt, uint8_t mode)
{
    uint8_t cmd[5];
    cmd[0] = zdt->addr;
    cmd[1] = 0x9A;
    cmd[2] = mode;
    cmd[3] = 0x00;
    cmd[4] = ZDT_TAIL;
    ZDT_SendCmd(zdt, cmd, 5);
}

/**
 * @brief 强制中断并退出回零, 回零卡住/超时时使用
 */
void ZDT_AbortHome(ZDT_HandleTypeDef *zdt)
{
    uint8_t cmd[4];
    cmd[0] = zdt->addr;
    cmd[1] = 0x9C;
    cmd[2] = 0x48;
    cmd[3] = ZDT_TAIL;
    ZDT_SendCmd(zdt, cmd, 4);
}

/**
 * @brief 读取回零状态标志
 * @return 状态字节:
 *         bit0 Enc_Rdy  编码器就绪 (1=正常)
 *         bit1 Cal_Rdy  校准表就绪 (1=已校准)
 *         bit2 Org_SF   正在回零中 (1=进行中)
 *         bit3 Org_CF   回零失败   (1=失败)
 *         bit4 Otp_TF   过热保护   (1=触发)
 *         bit5 Ocp_TF   过流保护   (1=触发)
 *
 *         判断回零结果: (status & 0x0C)
 *           0x00 = 回零成功
 *           0x04 = 正在回零
 *           0x08 = 回零失败
 */
uint8_t ZDT_ReadHomeStatus(ZDT_HandleTypeDef *zdt)
{
    uint8_t cmd[3];
    uint8_t rx[4];

    cmd[0] = zdt->addr;
    cmd[1] = 0x3B;
    cmd[2] = ZDT_TAIL;
    ZDT_SendCmd(zdt, cmd, 3);

    /* 接收 4 字节应答, 阻塞等待 */
    if (ZDT_ReceiveData(zdt, rx, 4) == 4) {
        return rx[2];
    }
    return 0;
}

/* ========== 参数设置 ========== */

void ZDT_SetMicrostep(ZDT_HandleTypeDef *zdt, uint8_t value, uint8_t store)
{
    uint8_t cmd[6];
    cmd[0] = zdt->addr;
    cmd[1] = 0x84;
    cmd[2] = 0x8A;
    cmd[3] = store;   /* 0=不保存 1=保存至 EEPROM */
    cmd[4] = value;   /* 细分值 0-255 */
    cmd[5] = ZDT_TAIL;
    ZDT_SendCmd(zdt, cmd, 6);
}

/* ========== 工具函数 ========== */

int32_t ZDT_AngleToPulses(float angle_deg)
{
    return (int32_t)(angle_deg * (ZDT_PULSES_PER_REV / 360.0f));
}

float ZDT_PulsesToAngle(int32_t pulses)
{
    return (float)pulses * (360.0f / ZDT_PULSES_PER_REV);
}
