#ifndef BLUETOOTH_BLUETOOTH_H_
#define BLUETOOTH_BLUETOOTH_H_

#include "ti_msp_dl_config.h"

/* 蓝牙连接状态 */
#define BT_DISCONNECTED  0
#define BT_CONNECTING    1
#define BT_CONNECTED     2

extern volatile uint8_t g_bt_state;       /* 当前连接状态 */
extern volatile uint8_t g_bt_rx_ready;    /* 收到一帧数据标志 */

/* 接收缓冲区 (外部可读) */
#define BT_RX_BUF_SIZE 32
extern volatile char    g_bt_rx_buf[BT_RX_BUF_SIZE];
extern volatile uint8_t g_bt_rx_len;

/**
 * @brief  初始化蓝牙 UART (根据 CAR_ID 选 UART_BLA 或 UART_BLB)
 *         使能 RX 中断
 */
void Bluetooth_Init(void);

/**
 * @brief  A 车: 发起连接请求 → 发 "HELLO\r\n"
 *         非阻塞, 调用后需要在主循环中轮询 Bluetooth_CheckConnection()
 */
void Bluetooth_Connect(void);

/**
 * @brief  主循环中调用: 检查是否收到应答
 *         收到 ACK → g_bt_state = BT_CONNECTED
 * @return 0=等待中, 1=已连接, -1=超时
 */
int Bluetooth_CheckConnection(void);

/**
 * @brief  发送字符串 (非中断模式)
 */
void Bluetooth_SendString(const char *str);

/**
 * @brief  发送单字节
 */
void Bluetooth_SendByte(uint8_t data);

/**
 * @brief  主循环中调用: 处理 ISR 标记的延迟回复 (ACK 等)
 *         必须在 ISR 外发送, 避免阻塞中断
 */
void Bluetooth_Poll(void);

/**
 * @brief  UART RX 中断回调 (在 interrupt.c 的 ISR 中调用)
 */
void Bluetooth_RX_ISR(uint8_t byte);

#endif /* BLUETOOTH_BLUETOOTH_H_ */
