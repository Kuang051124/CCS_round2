/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     80000000



/* Defines for PWM_MOTOR */
#define PWM_MOTOR_INST                                                     TIMA0
#define PWM_MOTOR_INST_IRQHandler                               TIMA0_IRQHandler
#define PWM_MOTOR_INST_INT_IRQN                                 (TIMA0_INT_IRQn)
#define PWM_MOTOR_INST_CLK_FREQ                                         40000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_MOTOR_C0_PORT                                             GPIOA
#define GPIO_PWM_MOTOR_C0_PIN                                     DL_GPIO_PIN_21
#define GPIO_PWM_MOTOR_C0_IOMUX                                  (IOMUX_PINCM46)
#define GPIO_PWM_MOTOR_C0_IOMUX_FUNC                 IOMUX_PINCM46_PF_TIMA0_CCP0
#define GPIO_PWM_MOTOR_C0_IDX                                DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_MOTOR_C1_PORT                                             GPIOA
#define GPIO_PWM_MOTOR_C1_PIN                                     DL_GPIO_PIN_22
#define GPIO_PWM_MOTOR_C1_IOMUX                                  (IOMUX_PINCM47)
#define GPIO_PWM_MOTOR_C1_IOMUX_FUNC                 IOMUX_PINCM47_PF_TIMA0_CCP1
#define GPIO_PWM_MOTOR_C1_IDX                                DL_TIMER_CC_1_INDEX
/* GPIO defines for channel 2 */
#define GPIO_PWM_MOTOR_C2_PORT                                             GPIOA
#define GPIO_PWM_MOTOR_C2_PIN                                     DL_GPIO_PIN_15
#define GPIO_PWM_MOTOR_C2_IOMUX                                  (IOMUX_PINCM37)
#define GPIO_PWM_MOTOR_C2_IOMUX_FUNC                 IOMUX_PINCM37_PF_TIMA0_CCP2
#define GPIO_PWM_MOTOR_C2_IDX                                DL_TIMER_CC_2_INDEX
/* GPIO defines for channel 3 */
#define GPIO_PWM_MOTOR_C3_PORT                                             GPIOA
#define GPIO_PWM_MOTOR_C3_PIN                                     DL_GPIO_PIN_25
#define GPIO_PWM_MOTOR_C3_IOMUX                                  (IOMUX_PINCM55)
#define GPIO_PWM_MOTOR_C3_IOMUX_FUNC                 IOMUX_PINCM55_PF_TIMA0_CCP3
#define GPIO_PWM_MOTOR_C3_IDX                                DL_TIMER_CC_3_INDEX

/* Defines for SM_PUL_VER */
#define SM_PUL_VER_INST                                                    TIMG6
#define SM_PUL_VER_INST_IRQHandler                              TIMG6_IRQHandler
#define SM_PUL_VER_INST_INT_IRQN                                (TIMG6_INT_IRQn)
#define SM_PUL_VER_INST_CLK_FREQ                                        10000000
/* GPIO defines for channel 0 */
#define GPIO_SM_PUL_VER_C0_PORT                                            GPIOA
#define GPIO_SM_PUL_VER_C0_PIN                                    DL_GPIO_PIN_29
#define GPIO_SM_PUL_VER_C0_IOMUX                                  (IOMUX_PINCM4)
#define GPIO_SM_PUL_VER_C0_IOMUX_FUNC                 IOMUX_PINCM4_PF_TIMG6_CCP0
#define GPIO_SM_PUL_VER_C0_IDX                               DL_TIMER_CC_0_INDEX

/* Defines for SM_PUL_HOR */
#define SM_PUL_HOR_INST                                                    TIMA1
#define SM_PUL_HOR_INST_IRQHandler                              TIMA1_IRQHandler
#define SM_PUL_HOR_INST_INT_IRQN                                (TIMA1_INT_IRQn)
#define SM_PUL_HOR_INST_CLK_FREQ                                        10000000
/* GPIO defines for channel 0 */
#define GPIO_SM_PUL_HOR_C0_PORT                                            GPIOA
#define GPIO_SM_PUL_HOR_C0_PIN                                    DL_GPIO_PIN_28
#define GPIO_SM_PUL_HOR_C0_IOMUX                                  (IOMUX_PINCM3)
#define GPIO_SM_PUL_HOR_C0_IOMUX_FUNC                 IOMUX_PINCM3_PF_TIMA1_CCP0
#define GPIO_SM_PUL_HOR_C0_IDX                               DL_TIMER_CC_0_INDEX



/* Defines for MOTOR_TIM_INST */
#define MOTOR_TIM_INST_INST                                              (TIMG0)
#define MOTOR_TIM_INST_INST_IRQHandler                          TIMG0_IRQHandler
#define MOTOR_TIM_INST_INST_INT_IRQN                            (TIMG0_INT_IRQn)
#define MOTOR_TIM_INST_INST_LOAD_VALUE                                   (1562U)




/* Defines for I2C_OLED */
#define I2C_OLED_INST                                                       I2C1
#define I2C_OLED_INST_IRQHandler                                 I2C1_IRQHandler
#define I2C_OLED_INST_INT_IRQN                                     I2C1_INT_IRQn
#define I2C_OLED_BUS_SPEED_HZ                                             400000
#define GPIO_I2C_OLED_SDA_PORT                                             GPIOB
#define GPIO_I2C_OLED_SDA_PIN                                      DL_GPIO_PIN_3
#define GPIO_I2C_OLED_IOMUX_SDA                                  (IOMUX_PINCM16)
#define GPIO_I2C_OLED_IOMUX_SDA_FUNC                   IOMUX_PINCM16_PF_I2C1_SDA
#define GPIO_I2C_OLED_SCL_PORT                                             GPIOB
#define GPIO_I2C_OLED_SCL_PIN                                      DL_GPIO_PIN_2
#define GPIO_I2C_OLED_IOMUX_SCL                                  (IOMUX_PINCM15)
#define GPIO_I2C_OLED_IOMUX_SCL_FUNC                   IOMUX_PINCM15_PF_I2C1_SCL


/* Defines for UART_WIT */
#define UART_WIT_INST                                                      UART0
#define UART_WIT_INST_FREQUENCY                                         40000000
#define UART_WIT_INST_IRQHandler                                UART0_IRQHandler
#define UART_WIT_INST_INT_IRQN                                    UART0_INT_IRQn
#define GPIO_UART_WIT_RX_PORT                                              GPIOA
#define GPIO_UART_WIT_RX_PIN                                      DL_GPIO_PIN_11
#define GPIO_UART_WIT_IOMUX_RX                                   (IOMUX_PINCM22)
#define GPIO_UART_WIT_IOMUX_RX_FUNC                    IOMUX_PINCM22_PF_UART0_RX
#define UART_WIT_BAUD_RATE                                              (115200)
#define UART_WIT_IBRD_40_MHZ_115200_BAUD                                    (21)
#define UART_WIT_FBRD_40_MHZ_115200_BAUD                                    (45)
/* Defines for K230 */
#define K230_INST                                                          UART2
#define K230_INST_FREQUENCY                                             40000000
#define K230_INST_IRQHandler                                    UART2_IRQHandler
#define K230_INST_INT_IRQN                                        UART2_INT_IRQn
#define GPIO_K230_RX_PORT                                                  GPIOB
#define GPIO_K230_TX_PORT                                                  GPIOB
#define GPIO_K230_RX_PIN                                          DL_GPIO_PIN_18
#define GPIO_K230_TX_PIN                                          DL_GPIO_PIN_17
#define GPIO_K230_IOMUX_RX                                       (IOMUX_PINCM44)
#define GPIO_K230_IOMUX_TX                                       (IOMUX_PINCM43)
#define GPIO_K230_IOMUX_RX_FUNC                        IOMUX_PINCM44_PF_UART2_RX
#define GPIO_K230_IOMUX_TX_FUNC                        IOMUX_PINCM43_PF_UART2_TX
#define K230_BAUD_RATE                                                  (115200)
#define K230_IBRD_40_MHZ_115200_BAUD                                        (21)
#define K230_FBRD_40_MHZ_115200_BAUD                                        (45)





/* Defines for DMA_WIT */
#define DMA_WIT_CHAN_ID                                                      (0)
#define UART_WIT_INST_DMA_TRIGGER                            (DMA_UART0_RX_TRIG)


/* Port definition for Pin Group LED */
#define LED_PORT                                                         (GPIOB)

/* Defines for RED: GPIOB.14 with pinCMx 31 on package pin 2 */
#define LED_RED_PIN                                             (DL_GPIO_PIN_14)
#define LED_RED_IOMUX                                            (IOMUX_PINCM31)
/* Port definition for Pin Group LASER */
#define LASER_PORT                                                       (GPIOA)

/* Defines for PURPLE: GPIOA.8 with pinCMx 19 on package pin 54 */
#define LASER_PURPLE_PIN                                         (DL_GPIO_PIN_8)
#define LASER_PURPLE_IOMUX                                       (IOMUX_PINCM19)
/* Port definition for Pin Group BUZZER */
#define BUZZER_PORT                                                      (GPIOA)

/* Defines for BEEP: GPIOA.30 with pinCMx 5 on package pin 37 */
#define BUZZER_BEEP_PIN                                         (DL_GPIO_PIN_30)
#define BUZZER_BEEP_IOMUX                                         (IOMUX_PINCM5)
/* Port definition for Pin Group TRACE */
#define TRACE_PORT                                                       (GPIOB)

/* Defines for LAMP_0: GPIOB.12 with pinCMx 29 on package pin 64 */
#define TRACE_LAMP_0_PIN                                        (DL_GPIO_PIN_12)
#define TRACE_LAMP_0_IOMUX                                       (IOMUX_PINCM29)
/* Defines for LAMP_1: GPIOB.10 with pinCMx 27 on package pin 62 */
#define TRACE_LAMP_1_PIN                                        (DL_GPIO_PIN_10)
#define TRACE_LAMP_1_IOMUX                                       (IOMUX_PINCM27)
/* Defines for LAMP_2: GPIOB.8 with pinCMx 25 on package pin 60 */
#define TRACE_LAMP_2_PIN                                         (DL_GPIO_PIN_8)
#define TRACE_LAMP_2_IOMUX                                       (IOMUX_PINCM25)
/* Defines for LAMP_3: GPIOB.6 with pinCMx 23 on package pin 58 */
#define TRACE_LAMP_3_PIN                                         (DL_GPIO_PIN_6)
#define TRACE_LAMP_3_IOMUX                                       (IOMUX_PINCM23)
/* Defines for LAMP_4: GPIOB.7 with pinCMx 24 on package pin 59 */
#define TRACE_LAMP_4_PIN                                         (DL_GPIO_PIN_7)
#define TRACE_LAMP_4_IOMUX                                       (IOMUX_PINCM24)
/* Defines for FL_IN1: GPIOA.27 with pinCMx 60 on package pin 31 */
#define MOTOR_FL_IN1_PORT                                                (GPIOA)
#define MOTOR_FL_IN1_PIN                                        (DL_GPIO_PIN_27)
#define MOTOR_FL_IN1_IOMUX                                       (IOMUX_PINCM60)
/* Defines for FL_IN2: GPIOB.27 with pinCMx 58 on package pin 29 */
#define MOTOR_FL_IN2_PORT                                                (GPIOB)
#define MOTOR_FL_IN2_PIN                                        (DL_GPIO_PIN_27)
#define MOTOR_FL_IN2_IOMUX                                       (IOMUX_PINCM58)
/* Defines for FR_IN1: GPIOB.25 with pinCMx 56 on package pin 27 */
#define MOTOR_FR_IN1_PORT                                                (GPIOB)
#define MOTOR_FR_IN1_PIN                                        (DL_GPIO_PIN_25)
#define MOTOR_FR_IN1_IOMUX                                       (IOMUX_PINCM56)
/* Defines for FR_IN2: GPIOA.24 with pinCMx 54 on package pin 25 */
#define MOTOR_FR_IN2_PORT                                                (GPIOA)
#define MOTOR_FR_IN2_PIN                                        (DL_GPIO_PIN_24)
#define MOTOR_FR_IN2_IOMUX                                       (IOMUX_PINCM54)
/* Defines for BL_IN1: GPIOB.24 with pinCMx 52 on package pin 23 */
#define MOTOR_BL_IN1_PORT                                                (GPIOB)
#define MOTOR_BL_IN1_PIN                                        (DL_GPIO_PIN_24)
#define MOTOR_BL_IN1_IOMUX                                       (IOMUX_PINCM52)
/* Defines for BL_IN2: GPIOB.23 with pinCMx 51 on package pin 22 */
#define MOTOR_BL_IN2_PORT                                                (GPIOB)
#define MOTOR_BL_IN2_PIN                                        (DL_GPIO_PIN_23)
#define MOTOR_BL_IN2_IOMUX                                       (IOMUX_PINCM51)
/* Defines for BR_IN1: GPIOB.22 with pinCMx 50 on package pin 21 */
#define MOTOR_BR_IN1_PORT                                                (GPIOB)
#define MOTOR_BR_IN1_PIN                                        (DL_GPIO_PIN_22)
#define MOTOR_BR_IN1_IOMUX                                       (IOMUX_PINCM50)
/* Defines for BR_IN2: GPIOB.20 with pinCMx 48 on package pin 19 */
#define MOTOR_BR_IN2_PORT                                                (GPIOB)
#define MOTOR_BR_IN2_PIN                                        (DL_GPIO_PIN_20)
#define MOTOR_BR_IN2_IOMUX                                       (IOMUX_PINCM48)
/* Defines for DIR_VER: GPIOA.26 with pinCMx 59 on package pin 30 */
#define STEP_DIR_VER_PORT                                                (GPIOA)
#define STEP_DIR_VER_PIN                                        (DL_GPIO_PIN_26)
#define STEP_DIR_VER_IOMUX                                       (IOMUX_PINCM59)
/* Defines for DIR_HOR: GPIOB.26 with pinCMx 57 on package pin 28 */
#define STEP_DIR_HOR_PORT                                                (GPIOB)
#define STEP_DIR_HOR_PIN                                        (DL_GPIO_PIN_26)
#define STEP_DIR_HOR_IOMUX                                       (IOMUX_PINCM57)
/* Defines for R1: GPIOA.17 with pinCMx 39 on package pin 10 */
#define KBD_GRP_R1_PORT                                                  (GPIOA)
#define KBD_GRP_R1_PIN                                          (DL_GPIO_PIN_17)
#define KBD_GRP_R1_IOMUX                                         (IOMUX_PINCM39)
/* Defines for R2: GPIOA.9 with pinCMx 20 on package pin 55 */
#define KBD_GRP_R2_PORT                                                  (GPIOA)
#define KBD_GRP_R2_PIN                                           (DL_GPIO_PIN_9)
#define KBD_GRP_R2_IOMUX                                         (IOMUX_PINCM20)
/* Defines for R3: GPIOB.13 with pinCMx 30 on package pin 1 */
#define KBD_GRP_R3_PORT                                                  (GPIOB)
#define KBD_GRP_R3_PIN                                          (DL_GPIO_PIN_13)
#define KBD_GRP_R3_IOMUX                                         (IOMUX_PINCM30)
/* Defines for C1: GPIOA.16 with pinCMx 38 on package pin 9 */
#define KBD_GRP_C1_PORT                                                  (GPIOA)
#define KBD_GRP_C1_PIN                                          (DL_GPIO_PIN_16)
#define KBD_GRP_C1_IOMUX                                         (IOMUX_PINCM38)
/* Defines for C2: GPIOA.14 with pinCMx 36 on package pin 7 */
#define KBD_GRP_C2_PORT                                                  (GPIOA)
#define KBD_GRP_C2_PIN                                          (DL_GPIO_PIN_14)
#define KBD_GRP_C2_IOMUX                                         (IOMUX_PINCM36)
/* Defines for C3: GPIOA.13 with pinCMx 35 on package pin 6 */
#define KBD_GRP_C3_PORT                                                  (GPIOA)
#define KBD_GRP_C3_PIN                                          (DL_GPIO_PIN_13)
#define KBD_GRP_C3_IOMUX                                         (IOMUX_PINCM35)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_MOTOR_init(void);
void SYSCFG_DL_SM_PUL_VER_init(void);
void SYSCFG_DL_SM_PUL_HOR_init(void);
void SYSCFG_DL_MOTOR_TIM_INST_init(void);
void SYSCFG_DL_I2C_OLED_init(void);
void SYSCFG_DL_UART_WIT_init(void);
void SYSCFG_DL_K230_init(void);
void SYSCFG_DL_DMA_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
