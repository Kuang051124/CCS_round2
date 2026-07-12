#include "bluetooth.h"
#include "bt_cmd_parser.h"
#include "../Drivers/MSPM0/clock.h"
#include <string.h>

/* ===================================================================
 * 全局状态
 * =================================================================== */
volatile uint8_t g_bt_state    = BT_DISCONNECTED;
volatile uint8_t g_bt_rx_ready = 0;

volatile char    g_bt_rx_buf[BT_RX_BUF_SIZE];
volatile uint8_t g_bt_rx_len  = 0;

/* ===================================================================
 * 连接握手相关
 * =================================================================== */
static uint32_t g_connect_start_ms = 0;
#define BT_CONNECT_TIMEOUT_MS  5000   /* 5 秒超时 */

/* ===================================================================
 * 外部变量: CAR_ID (在 main.c 中定义)
 * =================================================================== */
extern int CAR_ID;

/* ===================================================================
 * UART 实例: 根据 CAR_ID 选择
 * =================================================================== */
void Bluetooth_Init(void)
{
    g_bt_state    = BT_DISCONNECTED;
    g_bt_rx_ready = 0;
    g_bt_rx_len   = 0;
    memset((void *)g_bt_rx_buf, 0, BT_RX_BUF_SIZE);

    if (CAR_ID == 1) {
        DL_UART_enableInterrupt(UART_BLA_INST, DL_UART_INTERRUPT_RX);
        NVIC_EnableIRQ(UART_BLA_INST_INT_IRQN);
    } else {
        DL_UART_enableInterrupt(UART_BLB_INST, DL_UART_INTERRUPT_RX);
        NVIC_EnableIRQ(UART_BLB_INST_INT_IRQN);
    }
}

/* ===================================================================
 * A 车主动发起连接
 * =================================================================== */
void Bluetooth_Connect(void)
{
    g_bt_state = BT_CONNECTING;
    Bluetooth_SendString("HELLO\r\n");
    g_connect_start_ms = tick_ms;
}

/* ===================================================================
 * 检查连接结果 (主循环中轮询)
 * =================================================================== */
int Bluetooth_CheckConnection(void)
{
    if (g_bt_state != BT_CONNECTING) return 0;

    /* 超时 */
    if (tick_ms - g_connect_start_ms > BT_CONNECT_TIMEOUT_MS) {
        g_bt_state = BT_DISCONNECTED;
        return -1;
    }

    /* 收到完整一帧 */
    if (g_bt_rx_ready) {
        /* 判断是不是 ACK */
        if (strncmp((const char *)g_bt_rx_buf, "ACK", 3) == 0) {
            g_bt_state = BT_CONNECTED;
            g_bt_rx_ready = 0;
            g_bt_rx_len   = 0;
            return 1;
        }
        /* 不是 ACK → 清掉继续等 */
        g_bt_rx_ready = 0;
        g_bt_rx_len   = 0;
    }

    return 0;
}

/* ===================================================================
 * UART 发送
 * =================================================================== */
void Bluetooth_SendString(const char *str)
{
    while (*str) {
        Bluetooth_SendByte((uint8_t)*str);
        str++;
    }
}

void Bluetooth_SendByte(uint8_t data)
{
    if (CAR_ID == 1) {
        DL_UART_transmitDataBlocking(UART_BLA_INST, data);
    } else {
        DL_UART_transmitDataBlocking(UART_BLB_INST, data);
    }
}

/* ===================================================================
 * UART RX 中断回调 (在 interrupt.c 的 GROUP ISR 中调用)
 *
 * 简易帧解析: 以 \n 结尾作为一帧完成
 * =================================================================== */
volatile uint8_t g_bt_need_ack = 0;   /* B车需回复ACK标志 (ISR设, 主循环清) */

void Bluetooth_RX_ISR(uint8_t byte)
{
    /* 帧结束: 收到 \n 或缓冲区满 */
    if (byte == '\n' || g_bt_rx_len >= BT_RX_BUF_SIZE - 1) {
        g_bt_rx_buf[g_bt_rx_len] = '\0';
        g_bt_rx_ready = 1;

        /* B 车收到 HELLO → 标记需回复 ACK (ISR内绝不阻塞发送!) */
        if (CAR_ID == 2) {
            if (strncmp((const char *)g_bt_rx_buf, "HELLO", 5) == 0) {
                g_bt_need_ack = 1;
            }
        }

        return;
    }

    if (byte == '\r') return;

    g_bt_rx_buf[g_bt_rx_len++] = byte;
}

/* 主循环中调用: 处理 ISR 标记的延迟回复 + 命令解析 */
void Bluetooth_Poll(void)
{
    if (g_bt_need_ack) {
        g_bt_need_ack = 0;
        Bluetooth_SendString("ACK\r\n");
        g_bt_state = BT_CONNECTED;
    }

    /* 解析收到的蓝牙指令帧
     * BT_ParseCommand 返回值:
     *    1  = 速度指令已处理 → 消费帧
     *    0  = 参数调整等已处理 → 消费帧
     *   -1  = TASK3/TASK4 (需转发给 task 代码) → 不消费, 留给 task_tick */
    if (g_bt_rx_ready) {
        int8_t ret = BT_ParseCommand((const char *)g_bt_rx_buf, g_bt_rx_len);
        if (ret >= 0) {
            g_bt_rx_ready = 0;
            g_bt_rx_len   = 0;
        }
        /* ret == -1: 帧留给 task3.c / pages.c 处理, 不消费 */
    }
}
