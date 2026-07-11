# WIT — WitMotion IMU 驱动 (MSPM0G3507)

## 概述

WIT 模块驱动 **维特智能 (WitMotion)** 系列姿态传感器，通过 UART 接收角度/加速度/角速度数据。

该驱动基于 **DMA + UART 超时中断** 架构，数据在后台自动更新，用户只需读取全局结构体 `wit_data` 即可。

**支持的传感器型号:** WT901、WT931、HWT905 等使用 WitMotion 标准协议的型号。

---

## 协议格式

WitMotion 传感器以 **11 字节** 为最小数据包，连续发送：

```
Byte 0:   0x55 (帧头)
Byte 1:   数据包标识 (见下表)
Byte 2-3: 数据1 (低字节在前)
Byte 4-5: 数据2
Byte 6-7: 数据3
Byte 8-9: 温度/版本
Byte 10:  校验和 (前 10 字节的低字节累加)
```

### 数据包类型

| 标识 | 含义 | 数据1 | 数据2 | 数据3 | Byte 8-9 |
|------|------|-------|-------|-------|----------|
| `0x51` | 加速度 | ax | ay | az | 温度 |
| `0x52` | 角速度 | gx | gy | gz | 温度 |
| `0x53` | 欧拉角 | Roll | Pitch | Yaw | 版本号 |

### 单位换算

| 类型 | 原始值范围 | 换算公式 | 输出单位 |
|------|-----------|----------|----------|
| 加速度 | -32768 ~ 32767 | raw / 2.048 | mg (毫重力) |
| 角速度 | -32768 ~ 32767 | raw / 16.384 | °/s |
| 欧拉角 | -32768 ~ 32767 | raw / 32768 × 180 | ° (度) |
| 温度 | -32768 ~ 32767 | raw / 100 | °C |

---

## 数据结构

```c
typedef struct {
    float pitch;        // 俯仰角 (°)
    float roll;         // 横滚角 (°)
    float yaw;          // 偏航角 (°)
    float temperature;  // 温度 (°C)
    int16_t ax;         // X 轴加速度 (mg)
    int16_t ay;         // Y 轴加速度 (mg)
    int16_t az;         // Z 轴加速度 (mg)
    int16_t gx;         // X 轴角速度 (°/s × 16.384)
    int16_t gy;         // Y 轴角速度
    int16_t gz;         // Z 轴角速度
    int16_t version;    // 固件版本号
} WIT_Data_t;

extern WIT_Data_t wit_data;  // 全局实例，ISR 自动更新
```

---

## 硬件接线

| MSPM0G3507 引脚 | 信号 | 连接至传感器 |
|-----------------|------|-------------|
| **PA11** (IOMUX PINCM22) | UART0 RX | 传感器 TX |
| **GND** | 地 | 传感器 GND |
| **3.3V** | 电源 | 传感器 VCC (3.3V) |

> **注意:** 本驱动为 **RX only** 模式，仅接收传感器数据，不发送配置指令。如需修改传感器参数（波特率/回传内容/频率），请使用 WitMotion 上位机软件。

---

## SysConfig 配置

使用前需在 CCS SysConfig 中配置 UART 和 DMA，详细步骤见 `wit.h` 头部注释。本项目当前配置:

| 配置项 | 值 |
|--------|-----|
| 模块名 | `UART_WIT` |
| UART 实例 | **UART0** (Main 类型) |
| 波特率 | **115200** |
| RX 引脚 | PA11 |
| 时钟频率 | 40 MHz |
| 通信方向 | RX only |
| FIFO | 启用 |
| RX Timeout 中断计数 | 1 |
| 中断处理函数 | `UART0_IRQHandler` |
| DMA 通道名 | `DMA_WIT` |
| DMA 通道号 | 0 |
| DMA 触发源 | DMA_UART0_RX_TRIG |
| DMA 地址模式 | Fixed addr. to Block addr. |
| DMA 源/目标长度 | Byte |

---

## 快速开始

### 1. 初始化

```c
#include "wit.h"

int main(void) {
    SYSCFG_DL_init();   // SysConfig 生成的初始化
    WIT_Init();         // 启动 DMA 接收 + 使能 UART 中断

    while (1) {
        // wit_data 在后台由 DMA+ISR 自动更新，直接读即可
        printf("Yaw: %.2f\n", wit_data.yaw);
        delay_ms(100);
    }
}
```

### 2. 读取角度

```c
float pitch = wit_data.pitch;  // 俯仰角
float roll  = wit_data.roll;   // 横滚角
float yaw   = wit_data.yaw;    // 偏航角
```

### 3. 航向锁定 (典型应用)

```c
float target_heading;

// 记录当前朝向
void LockHeading(void) {
    target_heading = wit_data.yaw;
}

// 计算偏航修正量
float GetYawCorrection(void) {
    float delta = wit_data.yaw - target_heading;
    // 处理 0/360° 边界跳变
    if (delta >  180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;
    return delta;
}
```

---

## API 参考

### `void WIT_Init(void)`

初始化 DMA 通道并启用 UART 中断。DMA 从 `UART_WIT_INST->RXDATA` 持续搬运 32 字节到环形缓冲区，UART 超时中断触发后解析数据包。

**调用时机:** `SYSCFG_DL_init()` 之后，进入主循环之前。

**注意:** 调用前需确保 SysConfig 已正确配置 `UART_WIT` 和 `DMA_WIT`。

---

## 架构说明

```
┌──────────┐   UART     ┌──────────┐   DMA (32 bytes)   ┌──────────────┐
│ WIT 传感器 │ ────────→ │ UART_WIT │ ─────────────────→ │ wit_dmaBuffer │
│ (WT901等) │  连续发送   │  RX FIFO │   自动搬运到内存      │   (33 bytes)  │
└──────────┘           └──────────┘                     └──────┬───────┘
                                                               │
                                            RX Timeout 中断触发 → ISR 解析
                                                               │
                                                     ┌─────────┴─────────┐
                                                     │ 搜索 0x55 帧头     │
                                                     │ 校验 checksum      │
                                                     │ 根据标识 (0x51/    │
                                                     │   0x52/0x53) 写入   │
                                                     │ wit_data 对应字段   │
                                                     └───────────────────┘
```

**DMA 工作模式:**
- 配置为 32 字节环形接收
- DMA 源地址固定 (`UART_WIT_INST->RXDATA`)，目标地址递增
- UART RX Timeout 中断触发 → ISR 检查已接收字节数 → 解析完整包 → 重新配置 DMA

**为什么用 UART Timeout 而非 DMA 完成中断:**
- WitMotion 传感器是连续流式发送，没有固定帧间隔
- RX Timeout = 1 个字符时间无数据时触发，自然分隔数据包
- 32 字节缓冲区足够容纳 2 个完整数据包 (11 × 2 = 22 bytes)

---

## ISR 实现 (interrupt.c)

```c
void UART_WIT_INST_IRQHandler(void) {
    // 1. 暂停 DMA，读取已传输字节数
    DL_DMA_disableChannel(DMA, DMA_WIT_CHAN_ID);
    uint8_t rxSize = 32 - DL_DMA_getTransferSize(DMA, DMA_WIT_CHAN_ID);

    // 2. 如果 RX FIFO 还有数据，一并读取
    if (!DL_UART_isRXFIFOEmpty(UART_WIT_INST))
        wit_dmaBuffer[rxSize++] = DL_UART_receiveData(UART_WIT_INST);

    // 3. 从缓冲区中搜索 0x55 帧头，逐包解析
    uint8_t packCnt = 0;
    while (rxSize >= 11) {
        // 校验 checksum
        uint8_t checkSum = 0;
        for (int i = packCnt*11; i < (packCnt+1)*11 - 1; i++)
            checkSum += wit_dmaBuffer[i];

        if (wit_dmaBuffer[packCnt*11] == 0x55 &&
            checkSum == wit_dmaBuffer[packCnt*11 + 10]) {
            // 根据标识解析数据...
        }
        rxSize -= 11;
        packCnt++;
    }

    // 4. 排空 FIFO 残留，重新配置 DMA
    uint8_t dummy[4];
    DL_UART_drainRXFIFO(UART_WIT_INST, dummy, 4);
    DL_DMA_setTransferSize(DMA, DMA_WIT_CHAN_ID, 32);
    DL_DMA_enableChannel(DMA, DMA_WIT_CHAN_ID);
}
```

---

## 注意事项

1. **波特率必须匹配**: 传感器出厂默认通常为 115200 或 9600，可通过 WitMotion 上位机软件修改
2. **Z 轴朝向**: 传感器安装时 Yaw=0° 对应上电瞬间的朝向，若需绝对朝向需配合磁力计校准
3. **数据刷新率**: 取决于传感器输出频率 (通常 100Hz)，`wit_data` 在 ISR 中异步更新，主循环读取无需加锁
4. **Yaw 漂移**: 纯 6 轴 IMU (无磁力计) 的 Yaw 会随时间漂移，9 轴传感器使用磁力计融合后稳定
5. **Yaw 0/360° 跳变**: 传感器 Yaw 范围 0~360°，跨越 0° 时需自行处理角度差计算
