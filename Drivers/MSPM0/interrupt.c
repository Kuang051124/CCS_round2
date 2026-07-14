#include "ti_msp_dl_config.h"
#include "interrupt.h"
#include "clock.h"
#include "mpu6050.h"
#include "bno08x_uart_rvc.h"
#include "wit.h"
#include "vl53l0x.h"
#include "lsm6dsv16x.h"
#include "../../yb_protocol/yb_protocol.h"
#include "ENCODER/encoder.h"
#include "ENCODER/speed_control.h"
#include "BLUETOOTH/bluetooth.h"

uint8_t enable_group1_irq = 0;

/* 外环同步标志: TIMG0 10ms ISR 每 2 次 (20ms) 置 1,
 * pages.c 主循环消费后清零。实现内环 100Hz / 外环 50Hz 严格 2:1 同步。 */
volatile uint8_t g_outer_loop_ready = 0;

void Interrupt_Init(void)
{
    /* Group1 统一使能: GPIOA/GPIOB 中断用于编码器及其他传感器 */
    NVIC_EnableIRQ(GPIOA_INT_IRQn);

    /* MOTOR_TIM_INST (TIMG0) 10ms 定时器中断: 编码器测速 + PI 控制 */
    NVIC_EnableIRQ(MOTOR_TIM_INST_INST_INT_IRQN);
}

void SysTick_Handler(void)
{
    tick_ms++;
}

#if defined UART_BNO08X_INST_IRQHandler
void UART_BNO08X_INST_IRQHandler(void)
{
    uint8_t checkSum = 0;
    extern uint8_t bno08x_dmaBuffer[19];

    DL_DMA_disableChannel(DMA, DMA_BNO08X_CHAN_ID);
    uint8_t rxSize = 18 - DL_DMA_getTransferSize(DMA, DMA_BNO08X_CHAN_ID);

    if(DL_UART_isRXFIFOEmpty(UART_BNO08X_INST) == false)
        bno08x_dmaBuffer[rxSize++] = DL_UART_receiveData(UART_BNO08X_INST);

    for(int i=2; i<=14; i++)
        checkSum += bno08x_dmaBuffer[i];

    if((rxSize == 19) && (bno08x_dmaBuffer[0] == 0xAA) && (bno08x_dmaBuffer[1] == 0xAA) && (checkSum == bno08x_dmaBuffer[18]))
    {
        bno08x_data.index = bno08x_dmaBuffer[2];
        bno08x_data.yaw = (int16_t)((bno08x_dmaBuffer[4]<<8)|bno08x_dmaBuffer[3]) / 100.0;
        bno08x_data.pitch = (int16_t)((bno08x_dmaBuffer[6]<<8)|bno08x_dmaBuffer[5]) / 100.0;
        bno08x_data.roll = (int16_t)((bno08x_dmaBuffer[8]<<8)|bno08x_dmaBuffer[7]) / 100.0;
        bno08x_data.ax = (bno08x_dmaBuffer[10]<<8)|bno08x_dmaBuffer[9];
        bno08x_data.ay = (bno08x_dmaBuffer[12]<<8)|bno08x_dmaBuffer[11];
        bno08x_data.az = (bno08x_dmaBuffer[14]<<8)|bno08x_dmaBuffer[13];
    }
    
    uint8_t dummy[4];
    DL_UART_drainRXFIFO(UART_BNO08X_INST, dummy, 4);

    DL_DMA_setDestAddr(DMA, DMA_BNO08X_CHAN_ID, (uint32_t) &bno08x_dmaBuffer[0]);
    DL_DMA_setTransferSize(DMA, DMA_BNO08X_CHAN_ID, 18);
    DL_DMA_enableChannel(DMA, DMA_BNO08X_CHAN_ID);
}
#endif

#if defined UART_WIT_INST_IRQHandler
void UART_WIT_INST_IRQHandler(void)
{
    uint8_t checkSum, packCnt = 0;
    extern uint8_t wit_dmaBuffer[33];

    DL_DMA_disableChannel(DMA, DMA_WIT_CHAN_ID);
    uint8_t rxSize = 32 - DL_DMA_getTransferSize(DMA, DMA_WIT_CHAN_ID);

    if(DL_UART_isRXFIFOEmpty(UART_WIT_INST) == false)
        wit_dmaBuffer[rxSize++] = DL_UART_receiveData(UART_WIT_INST);

    while(rxSize >= 11)
    {
        checkSum=0;
        for(int i=packCnt*11; i<(packCnt+1)*11-1; i++)
            checkSum += wit_dmaBuffer[i];

        if((wit_dmaBuffer[packCnt*11] == 0x55) && (checkSum == wit_dmaBuffer[packCnt*11+10]))
        {
            if(wit_dmaBuffer[packCnt*11+1] == 0x51)
            {
                wit_data.ax = (int16_t)((wit_dmaBuffer[packCnt*11+3]<<8)|wit_dmaBuffer[packCnt*11+2]) / 2.048; //mg
                wit_data.ay = (int16_t)((wit_dmaBuffer[packCnt*11+5]<<8)|wit_dmaBuffer[packCnt*11+4]) / 2.048; //mg
                wit_data.az = (int16_t)((wit_dmaBuffer[packCnt*11+7]<<8)|wit_dmaBuffer[packCnt*11+6]) / 2.048; //mg
                wit_data.temperature =  (int16_t)((wit_dmaBuffer[packCnt*11+9]<<8)|wit_dmaBuffer[packCnt*11+8]) / 100.0; //°C
            }
            else if(wit_dmaBuffer[packCnt*11+1] == 0x52)
            {
                wit_data.gx = (int16_t)((wit_dmaBuffer[packCnt*11+3]<<8)|wit_dmaBuffer[packCnt*11+2]) / 16.384; //°/S
                wit_data.gy = (int16_t)((wit_dmaBuffer[packCnt*11+5]<<8)|wit_dmaBuffer[packCnt*11+4]) / 16.384; //°/S
                wit_data.gz = (int16_t)((wit_dmaBuffer[packCnt*11+7]<<8)|wit_dmaBuffer[packCnt*11+6]) / 16.384; //°/S
            }
            else if(wit_dmaBuffer[packCnt*11+1] == 0x53)
            {
                wit_data.roll  = (int16_t)((wit_dmaBuffer[packCnt*11+3]<<8)|wit_dmaBuffer[packCnt*11+2]) / 32768.0 * 180.0; //°
                wit_data.pitch = (int16_t)((wit_dmaBuffer[packCnt*11+5]<<8)|wit_dmaBuffer[packCnt*11+4]) / 32768.0 * 180.0; //°
                wit_data.yaw   = (int16_t)((wit_dmaBuffer[packCnt*11+7]<<8)|wit_dmaBuffer[packCnt*11+6]) / 32768.0 * 180.0; //°
                wit_data.version = (int16_t)((wit_dmaBuffer[packCnt*11+9]<<8)|wit_dmaBuffer[packCnt*11+8]);
            }
        }

        rxSize -= 11;
        packCnt++;
    }
    
    uint8_t dummy[4];
    DL_UART_drainRXFIFO(UART_WIT_INST, dummy, 4);

    DL_DMA_setDestAddr(DMA, DMA_WIT_CHAN_ID, (uint32_t) &wit_dmaBuffer[0]);
    DL_DMA_setTransferSize(DMA, DMA_WIT_CHAN_ID, 32);
    DL_DMA_enableChannel(DMA, DMA_WIT_CHAN_ID);
}
#endif

void GROUP1_IRQHandler(void)
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
        /* ====== GPIOA 中断分派 (传感器 + 编码器) ====== */
        #if defined GPIO_MULTIPLE_GPIOA_INT_IIDX
        case GPIO_MULTIPLE_GPIOA_INT_IIDX:
            switch (DL_GPIO_getPendingInterrupt(GPIOA))
            {
                #if (defined GPIO_MPU6050_PORT) && (GPIO_MPU6050_PORT == GPIOA)
                case GPIO_MPU6050_PIN_MPU6050_INT_IIDX:
                    Read_Quad();
                    break;
                #endif

                #if (defined GPIO_LSM6DSV16X_PORT) && (GPIO_LSM6DSV16X_PORT == GPIOA)
                case GPIO_LSM6DSV16X_PIN_LSM6DSV16X_INT_IIDX:
                    Read_LSM6DSV16X();
                    break;
                #endif

                #if (defined GPIO_VL53L0X_PIN_VL53L0X_GPIO1_PORT) && (GPIO_VL53L0X_PIN_VL53L0X_GPIO1_PORT == GPIOA)
                case GPIO_VL53L0X_PIN_VL53L0X_GPIO1_IIDX:
                    Read_VL53L0X();
                    break;
                #endif

                /* ====== 编码器 GPIOA 中断 (LA=PA31, LB=PA28, RA=PA29) ====== */
                case ENCODERs_LA_IIDX:
                    if (DL_GPIO_readPins(GPIOA, ENCODERs_LB_PIN)) {
                        g_leftEncoderCount++;
                    } else {
                        g_leftEncoderCount--;
                    }
                    DL_GPIO_clearInterruptStatus(GPIOA, ENCODERs_LA_PIN);
                    break;

                case ENCODERs_LB_IIDX:
                    if (DL_GPIO_readPins(GPIOA, ENCODERs_LA_PIN)) {
                        g_leftEncoderCount++;
                    } else {
                        g_leftEncoderCount--;
                    }
                    DL_GPIO_clearInterruptStatus(GPIOA, ENCODERs_LB_PIN);
                    break;

                case ENCODERs_RA_IIDX:
                    if (DL_GPIO_readPins(GPIOB, ENCODERs_RB_PIN)) {
                        g_rightEncoderCount++;
                    } else {
                        g_rightEncoderCount--;
                    }
                    DL_GPIO_clearInterruptStatus(GPIOA, ENCODERs_RA_PIN);
                    break;

                default:
                    break;
            }
            break;
        #elif defined ENCODERs_GPIOA_INT_IIDX
        /* 仅编码器使用 GPIOA 中断 (GPIO_MULTIPLE 未定义时) */
        case ENCODERs_GPIOA_INT_IIDX:
            switch (DL_GPIO_getPendingInterrupt(GPIOA))
            {
                case ENCODERs_LA_IIDX:
                    if (DL_GPIO_readPins(GPIOA, ENCODERs_LB_PIN)) {
                        g_leftEncoderCount++;
                    } else {
                        g_leftEncoderCount--;
                    }
                    DL_GPIO_clearInterruptStatus(GPIOA, ENCODERs_LA_PIN);
                    break;

                case ENCODERs_LB_IIDX:
                    if (DL_GPIO_readPins(GPIOA, ENCODERs_LA_PIN)) {
                        g_leftEncoderCount++;
                    } else {
                        g_leftEncoderCount--;
                    }
                    DL_GPIO_clearInterruptStatus(GPIOA, ENCODERs_LB_PIN);
                    break;

                case ENCODERs_RA_IIDX:
                    if (DL_GPIO_readPins(GPIOB, ENCODERs_RB_PIN)) {
                        g_rightEncoderCount++;
                    } else {
                        g_rightEncoderCount--;
                    }
                    DL_GPIO_clearInterruptStatus(GPIOA, ENCODERs_RA_PIN);
                    break;

                default:
                    break;
            }
            break;
        #endif

        /* ====== GPIOB 中断分派 (传感器 + 编码器) ====== */
        #if defined GPIO_MULTIPLE_GPIOB_INT_IIDX
        case GPIO_MULTIPLE_GPIOB_INT_IIDX:
            switch (DL_GPIO_getPendingInterrupt(GPIOB))
            {
                #if (defined GPIO_MPU6050_PORT) && (GPIO_MPU6050_PORT == GPIOB)
                case GPIO_MPU6050_PIN_MPU6050_INT_IIDX:
                    Read_Quad();
                    break;
                #endif

                #if (defined GPIO_LSM6DSV16X_PORT) && (GPIO_LSM6DSV16X_PORT == GPIOB)
                case GPIO_LSM6DSV16X_PIN_LSM6DSV16X_INT_IIDX:
                    Read_LSM6DSV16X();
                    break;
                #endif

                #if (defined GPIO_VL53L0X_PIN_VL53L0X_GPIO1_PORT) && (GPIO_VL53L0X_PIN_VL53L0X_GPIO1_PORT == GPIOB)
                case GPIO_VL53L0X_PIN_VL53L0X_GPIO1_IIDX:
                    Read_VL53L0X();
                    break;
                #endif

                /* ====== 编码器 GPIOB 中断 (RB=PB27) ====== */
                case ENCODERs_RB_IIDX:
                    if (DL_GPIO_readPins(GPIOA, ENCODERs_RA_PIN)) {
                        g_rightEncoderCount++;
                    } else {
                        g_rightEncoderCount--;
                    }
                    DL_GPIO_clearInterruptStatus(GPIOB, ENCODERs_RB_PIN);
                    break;

                default:
                    break;
            }
            break;
        #elif defined ENCODERs_GPIOB_INT_IIDX
        /* 仅编码器使用 GPIOB 中断 (GPIO_MULTIPLE 未定义时) */
        case ENCODERs_GPIOB_INT_IIDX:
            switch (DL_GPIO_getPendingInterrupt(GPIOB))
            {
                case ENCODERs_RB_IIDX:
                    if (DL_GPIO_readPins(GPIOA, ENCODERs_RA_PIN)) {
                        g_rightEncoderCount++;
                    } else {
                        g_rightEncoderCount--;
                    }
                    DL_GPIO_clearInterruptStatus(GPIOB, ENCODERs_RB_PIN);
                    break;

                default:
                    break;
            }
            break;
        #endif

        #if defined GPIO_MPU6050_INT_IIDX
            case GPIO_MPU6050_INT_IIDX:
                Read_Quad();
                break;
        #endif

        #if defined GPIO_LSM6DSV16X_INT_IIDX
            case GPIO_LSM6DSV16X_INT_IIDX:
                Read_LSM6DSV16X();
                break;
        #endif

        #if defined GPIO_VL53L0X_INT_IIDX
            case GPIO_VL53L0X_INT_IIDX:
                Read_VL53L0X();
                break;
        #endif
    }
}
#if defined K230_INST_IRQHandler
void K230_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(K230_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        /* 一次中断清空 FIFO, 115200bps 单字节读会挤死主循环 */
        while (!DL_UART_Main_isRXFIFOEmpty(K230_INST)) {
            Pto_Receive_Byte(DL_UART_Main_receiveData(K230_INST));
        }
        break;
    default:
        break;
    }
}
#endif

/* ====== 蓝牙 UART_BLA (A车) ====== */
#if defined UART_BLA_INST_IRQHandler
void UART_BLA_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(UART_BLA_INST)) {
    case DL_UART_IIDX_RX:
        /* 一次中断清空 FIFO, 防止高频中断挤死主循环 */
        while (!DL_UART_isRXFIFOEmpty(UART_BLA_INST)) {
            Bluetooth_RX_ISR(DL_UART_receiveData(UART_BLA_INST));
        }
        break;
    default:
        break;
    }
}
#endif

#if defined UART_BLB_INST_IRQHandler
/* ====== 蓝牙 UART_BLB (B车) ====== */
void UART_BLB_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(UART_BLB_INST)) {
    case DL_UART_IIDX_RX:
        while (!DL_UART_isRXFIFOEmpty(UART_BLB_INST)) {
            Bluetooth_RX_ISR(DL_UART_receiveData(UART_BLB_INST));
        }
        break;
    default:
        break;
    }
}
#endif

/* ====== 10ms 定时器: 编码器测速 + 速度闭环 ====== */
#if defined MOTOR_TIM_INST_INST_IRQHandler
void MOTOR_TIM_INST_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(MOTOR_TIM_INST_INST)) {
    case DL_TIMER_IIDX_ZERO:
        ENCODER_SpeedSample();        /* 计算左右轮速度 (counts/s) */
        SPEED_PID_Tick();             /* 测速→PID→输出 (内部自带防重入+活跃检查) */

        /* 外环 50Hz 同步: 每 2 次 10ms 中断置位一次 */
        {
            static uint8_t tick_div = 0;
            if (++tick_div >= 1) {
                tick_div = 0;
                g_outer_loop_ready = 1;
            }
        }
        break;
    default:
        break;
    }
}
#endif