/*
 * Copyright (c) 2023, Texas Instruments Incorporated
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
 *  ============ ti_msp_dl_config.c =============
 *  Configured MSPM0 DriverLib module definitions
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */

#include "ti_msp_dl_config.h"

DL_TimerA_backupConfig gPWM_MOTORBackup;
DL_TimerG_backupConfig gSM_PUL_VERBackup;
DL_TimerA_backupConfig gSM_PUL_HORBackup;

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform any initialization needed before using any board APIs
 */
SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    /* Module-Specific Initializations*/
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_PWM_MOTOR_init();
    SYSCFG_DL_SM_PUL_VER_init();
    SYSCFG_DL_SM_PUL_HOR_init();
    SYSCFG_DL_MOTOR_TIM_INST_init();
    SYSCFG_DL_I2C_OLED_init();
    SYSCFG_DL_UART_WIT_init();
    SYSCFG_DL_K230_init();
    SYSCFG_DL_DMA_init();
    /* Ensure backup structures have no valid state */
	gPWM_MOTORBackup.backupRdy 	= false;
	gSM_PUL_VERBackup.backupRdy 	= false;
	gSM_PUL_HORBackup.backupRdy 	= false;



}
/*
 * User should take care to save and restore register configuration in application.
 * See Retention Configuration section for more details.
 */
SYSCONFIG_WEAK bool SYSCFG_DL_saveConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_saveConfiguration(PWM_MOTOR_INST, &gPWM_MOTORBackup);
	retStatus &= DL_TimerG_saveConfiguration(SM_PUL_VER_INST, &gSM_PUL_VERBackup);
	retStatus &= DL_TimerA_saveConfiguration(SM_PUL_HOR_INST, &gSM_PUL_HORBackup);

    return retStatus;
}


SYSCONFIG_WEAK bool SYSCFG_DL_restoreConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_restoreConfiguration(PWM_MOTOR_INST, &gPWM_MOTORBackup, false);
	retStatus &= DL_TimerG_restoreConfiguration(SM_PUL_VER_INST, &gSM_PUL_VERBackup, false);
	retStatus &= DL_TimerA_restoreConfiguration(SM_PUL_HOR_INST, &gSM_PUL_HORBackup, false);

    return retStatus;
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_TimerA_reset(PWM_MOTOR_INST);
    DL_TimerG_reset(SM_PUL_VER_INST);
    DL_TimerA_reset(SM_PUL_HOR_INST);
    DL_TimerG_reset(MOTOR_TIM_INST_INST);
    DL_I2C_reset(I2C_OLED_INST);
    DL_UART_Main_reset(UART_WIT_INST);
    DL_UART_Main_reset(K230_INST);


    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_TimerA_enablePower(PWM_MOTOR_INST);
    DL_TimerG_enablePower(SM_PUL_VER_INST);
    DL_TimerA_enablePower(SM_PUL_HOR_INST);
    DL_TimerG_enablePower(MOTOR_TIM_INST_INST);
    DL_I2C_enablePower(I2C_OLED_INST);
    DL_UART_Main_enablePower(UART_WIT_INST);
    DL_UART_Main_enablePower(K230_INST);

    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{

    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_MOTOR_C0_IOMUX,GPIO_PWM_MOTOR_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_MOTOR_C0_PORT, GPIO_PWM_MOTOR_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_MOTOR_C1_IOMUX,GPIO_PWM_MOTOR_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_MOTOR_C1_PORT, GPIO_PWM_MOTOR_C1_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_MOTOR_C2_IOMUX,GPIO_PWM_MOTOR_C2_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_MOTOR_C2_PORT, GPIO_PWM_MOTOR_C2_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_MOTOR_C3_IOMUX,GPIO_PWM_MOTOR_C3_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_MOTOR_C3_PORT, GPIO_PWM_MOTOR_C3_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_SM_PUL_VER_C0_IOMUX,GPIO_SM_PUL_VER_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_SM_PUL_VER_C0_PORT, GPIO_SM_PUL_VER_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_SM_PUL_HOR_C0_IOMUX,GPIO_SM_PUL_HOR_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_SM_PUL_HOR_C0_PORT, GPIO_SM_PUL_HOR_C0_PIN);

    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_OLED_IOMUX_SDA,
        GPIO_I2C_OLED_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_OLED_IOMUX_SCL,
        GPIO_I2C_OLED_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_I2C_OLED_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_I2C_OLED_IOMUX_SCL);

    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART_WIT_IOMUX_RX, GPIO_UART_WIT_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_K230_IOMUX_TX, GPIO_K230_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_K230_IOMUX_RX, GPIO_K230_IOMUX_RX_FUNC);

    DL_GPIO_initDigitalOutput(LED_RED_IOMUX);

    DL_GPIO_initDigitalOutput(LASER_PURPLE_IOMUX);

    DL_GPIO_initDigitalOutput(BUZZER_BEEP_IOMUX);

    DL_GPIO_initDigitalInputFeatures(TRACE_LAMP_0_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(TRACE_LAMP_1_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(TRACE_LAMP_2_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(TRACE_LAMP_3_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(TRACE_LAMP_4_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(MOTOR_FL_IN1_IOMUX);

    DL_GPIO_initDigitalOutput(MOTOR_FL_IN2_IOMUX);

    DL_GPIO_initDigitalOutput(MOTOR_FR_IN1_IOMUX);

    DL_GPIO_initDigitalOutput(MOTOR_FR_IN2_IOMUX);

    DL_GPIO_initDigitalOutput(MOTOR_BL_IN1_IOMUX);

    DL_GPIO_initDigitalOutput(MOTOR_BL_IN2_IOMUX);

    DL_GPIO_initDigitalOutput(MOTOR_BR_IN1_IOMUX);

    DL_GPIO_initDigitalOutput(MOTOR_BR_IN2_IOMUX);

    DL_GPIO_initDigitalOutput(STEP_DIR_VER_IOMUX);

    DL_GPIO_initDigitalOutput(STEP_DIR_HOR_IOMUX);

    DL_GPIO_initDigitalOutput(KBD_GRP_R1_IOMUX);

    DL_GPIO_initDigitalOutput(KBD_GRP_R2_IOMUX);

    DL_GPIO_initDigitalOutput(KBD_GRP_R3_IOMUX);

    DL_GPIO_initDigitalInputFeatures(KBD_GRP_C1_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(KBD_GRP_C2_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(KBD_GRP_C3_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_clearPins(GPIOA, LASER_PURPLE_PIN |
		MOTOR_FL_IN1_PIN |
		MOTOR_FR_IN2_PIN |
		STEP_DIR_VER_PIN |
		KBD_GRP_R1_PIN |
		KBD_GRP_R2_PIN);
    DL_GPIO_setPins(GPIOA, BUZZER_BEEP_PIN);
    DL_GPIO_enableOutput(GPIOA, LASER_PURPLE_PIN |
		BUZZER_BEEP_PIN |
		MOTOR_FL_IN1_PIN |
		MOTOR_FR_IN2_PIN |
		STEP_DIR_VER_PIN |
		KBD_GRP_R1_PIN |
		KBD_GRP_R2_PIN);
    DL_GPIO_clearPins(GPIOB, LED_RED_PIN |
		MOTOR_FL_IN2_PIN |
		MOTOR_FR_IN1_PIN |
		MOTOR_BL_IN1_PIN |
		MOTOR_BL_IN2_PIN |
		MOTOR_BR_IN1_PIN |
		MOTOR_BR_IN2_PIN |
		STEP_DIR_HOR_PIN |
		KBD_GRP_R3_PIN);
    DL_GPIO_enableOutput(GPIOB, LED_RED_PIN |
		MOTOR_FL_IN2_PIN |
		MOTOR_FR_IN1_PIN |
		MOTOR_BL_IN1_PIN |
		MOTOR_BL_IN2_PIN |
		MOTOR_BR_IN1_PIN |
		MOTOR_BR_IN2_PIN |
		STEP_DIR_HOR_PIN |
		KBD_GRP_R3_PIN);

}


static const DL_SYSCTL_SYSPLLConfig gSYSPLLConfig = {
    .inputFreq              = DL_SYSCTL_SYSPLL_INPUT_FREQ_16_32_MHZ,
	.rDivClk2x              = 1,
	.rDivClk1               = 0,
	.rDivClk0               = 0,
	.enableCLK2x            = DL_SYSCTL_SYSPLL_CLK2X_ENABLE,
	.enableCLK1             = DL_SYSCTL_SYSPLL_CLK1_DISABLE,
	.enableCLK0             = DL_SYSCTL_SYSPLL_CLK0_DISABLE,
	.sysPLLMCLK             = DL_SYSCTL_SYSPLL_MCLK_CLK2X,
	.sysPLLRef              = DL_SYSCTL_SYSPLL_REF_SYSOSC,
	.qDiv                   = 4,
	.pDiv                   = DL_SYSCTL_SYSPLL_PDIV_2
};
SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{

	//Low Power Mode is configured to be SLEEP0
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);
    DL_SYSCTL_setFlashWaitState(DL_SYSCTL_FLASH_WAIT_STATE_2);

    
	DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
	/* Set default configuration */
	DL_SYSCTL_disableHFXT();
	DL_SYSCTL_disableSYSPLL();
    DL_SYSCTL_configSYSPLL((DL_SYSCTL_SYSPLLConfig *) &gSYSPLLConfig);
    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_2);
    DL_SYSCTL_setMCLKSource(SYSOSC, HSCLK, DL_SYSCTL_HSCLK_SOURCE_SYSPLL);

}


/*
 * Timer clock configuration to be sourced by  / 1 (80000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   40000000 Hz = 80000000 Hz / (1 * (1 + 1))
 */
static const DL_TimerA_ClockConfig gPWM_MOTORClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 1U
};

static const DL_TimerA_PWMConfig gPWM_MOTORConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN_UP,
    .period = 500,
    .isTimerWithFourCC = true,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_PWM_MOTOR_init(void) {

    DL_TimerA_setClockConfig(
        PWM_MOTOR_INST, (DL_TimerA_ClockConfig *) &gPWM_MOTORClockConfig);

    DL_TimerA_initPWMMode(
        PWM_MOTOR_INST, (DL_TimerA_PWMConfig *) &gPWM_MOTORConfig);

    // Set Counter control to the smallest CC index being used
    DL_TimerA_setCounterControl(PWM_MOTOR_INST,DL_TIMER_CZC_CCCTL0_ZCOND,DL_TIMER_CAC_CCCTL0_ACOND,DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerA_setCaptureCompareOutCtl(PWM_MOTOR_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_0_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(PWM_MOTOR_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_0_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, 0, DL_TIMER_CC_0_INDEX);

    DL_TimerA_setCaptureCompareOutCtl(PWM_MOTOR_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_1_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(PWM_MOTOR_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_1_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, 0, DL_TIMER_CC_1_INDEX);

    DL_TimerA_setCaptureCompareOutCtl(PWM_MOTOR_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_2_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(PWM_MOTOR_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_2_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, 0, DL_TIMER_CC_2_INDEX);

    DL_TimerA_setCaptureCompareOutCtl(PWM_MOTOR_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_3_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(PWM_MOTOR_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_3_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, 0, DL_TIMER_CC_3_INDEX);

    DL_TimerA_enableClock(PWM_MOTOR_INST);


    
    DL_TimerA_setCCPDirection(PWM_MOTOR_INST , DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT | DL_TIMER_CC2_OUTPUT | DL_TIMER_CC3_OUTPUT );


}
/*
 * Timer clock configuration to be sourced by  / 1 (80000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   10000000 Hz = 80000000 Hz / (1 * (7 + 1))
 */
static const DL_TimerG_ClockConfig gSM_PUL_VERClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 7U
};

static const DL_TimerG_PWMConfig gSM_PUL_VERConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN_UP,
    .period = 5000,
    .isTimerWithFourCC = true,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_SM_PUL_VER_init(void) {

    DL_TimerG_setClockConfig(
        SM_PUL_VER_INST, (DL_TimerG_ClockConfig *) &gSM_PUL_VERClockConfig);

    DL_TimerG_initPWMMode(
        SM_PUL_VER_INST, (DL_TimerG_PWMConfig *) &gSM_PUL_VERConfig);

    // Set Counter control to the smallest CC index being used
    DL_TimerG_setCounterControl(SM_PUL_VER_INST,DL_TIMER_CZC_CCCTL0_ZCOND,DL_TIMER_CAC_CCCTL0_ACOND,DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerG_setCaptureCompareOutCtl(SM_PUL_VER_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_0_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(SM_PUL_VER_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptureCompareValue(SM_PUL_VER_INST, 0, DL_TIMER_CC_0_INDEX);

    DL_TimerG_enableClock(SM_PUL_VER_INST);


    DL_TimerG_enableInterrupt(SM_PUL_VER_INST , DL_TIMER_INTERRUPT_LOAD_EVENT);

    NVIC_SetPriority(SM_PUL_VER_INST_INT_IRQN, 1);
    DL_TimerG_setCCPDirection(SM_PUL_VER_INST , DL_TIMER_CC0_OUTPUT );


}
/*
 * Timer clock configuration to be sourced by  / 1 (80000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   10000000 Hz = 80000000 Hz / (1 * (7 + 1))
 */
static const DL_TimerA_ClockConfig gSM_PUL_HORClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 7U
};

static const DL_TimerA_PWMConfig gSM_PUL_HORConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN_UP,
    .period = 2000,
    .isTimerWithFourCC = true,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_SM_PUL_HOR_init(void) {

    DL_TimerA_setClockConfig(
        SM_PUL_HOR_INST, (DL_TimerA_ClockConfig *) &gSM_PUL_HORClockConfig);

    DL_TimerA_initPWMMode(
        SM_PUL_HOR_INST, (DL_TimerA_PWMConfig *) &gSM_PUL_HORConfig);

    // Set Counter control to the smallest CC index being used
    DL_TimerA_setCounterControl(SM_PUL_HOR_INST,DL_TIMER_CZC_CCCTL0_ZCOND,DL_TIMER_CAC_CCCTL0_ACOND,DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerA_setCaptureCompareOutCtl(SM_PUL_HOR_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_0_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(SM_PUL_HOR_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_0_INDEX);
    DL_TimerA_setCaptureCompareValue(SM_PUL_HOR_INST, 0, DL_TIMER_CC_0_INDEX);

    DL_TimerA_enableClock(SM_PUL_HOR_INST);


    DL_TimerA_enableInterrupt(SM_PUL_HOR_INST , DL_TIMER_INTERRUPT_LOAD_EVENT);

    NVIC_SetPriority(SM_PUL_HOR_INST_INT_IRQN, 1);
    DL_TimerA_setCCPDirection(SM_PUL_HOR_INST , DL_TIMER_CC0_OUTPUT );


}



/*
 * Timer clock configuration to be sourced by BUSCLK /  (40000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   156250 Hz = 40000000 Hz / (1 * (255 + 1))
 */
static const DL_TimerG_ClockConfig gMOTOR_TIM_INSTClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale    = 255U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * MOTOR_TIM_INST_INST_LOAD_VALUE = (10 ms * 156250 Hz) - 1
 */
static const DL_TimerG_TimerConfig gMOTOR_TIM_INSTTimerConfig = {
    .period     = MOTOR_TIM_INST_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_MOTOR_TIM_INST_init(void) {

    DL_TimerG_setClockConfig(MOTOR_TIM_INST_INST,
        (DL_TimerG_ClockConfig *) &gMOTOR_TIM_INSTClockConfig);

    DL_TimerG_initTimerMode(MOTOR_TIM_INST_INST,
        (DL_TimerG_TimerConfig *) &gMOTOR_TIM_INSTTimerConfig);
    DL_TimerG_enableClock(MOTOR_TIM_INST_INST);





}


static const DL_I2C_ClockConfig gI2C_OLEDClockConfig = {
    .clockSel = DL_I2C_CLOCK_BUSCLK,
    .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
};

SYSCONFIG_WEAK void SYSCFG_DL_I2C_OLED_init(void) {

    DL_I2C_setClockConfig(I2C_OLED_INST,
        (DL_I2C_ClockConfig *) &gI2C_OLEDClockConfig);
    DL_I2C_setAnalogGlitchFilterPulseWidth(I2C_OLED_INST,
        DL_I2C_ANALOG_GLITCH_FILTER_WIDTH_50NS);
    DL_I2C_enableAnalogGlitchFilter(I2C_OLED_INST);

    /* Configure Controller Mode */
    DL_I2C_resetControllerTransfer(I2C_OLED_INST);
    /* Set frequency to 400000 Hz*/
    DL_I2C_setTimerPeriod(I2C_OLED_INST, 9);
    DL_I2C_setControllerTXFIFOThreshold(I2C_OLED_INST, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setControllerRXFIFOThreshold(I2C_OLED_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(I2C_OLED_INST);


    /* Enable module */
    DL_I2C_enableController(I2C_OLED_INST);


}

static const DL_UART_Main_ClockConfig gUART_WITClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART_WITConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART_WIT_init(void)
{
    DL_UART_Main_setClockConfig(UART_WIT_INST, (DL_UART_Main_ClockConfig *) &gUART_WITClockConfig);

    DL_UART_Main_init(UART_WIT_INST, (DL_UART_Main_Config *) &gUART_WITConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115190.78
     */
    DL_UART_Main_setOversampling(UART_WIT_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_WIT_INST, UART_WIT_IBRD_40_MHZ_115200_BAUD, UART_WIT_FBRD_40_MHZ_115200_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(UART_WIT_INST,
                                 DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);
    /* Setting the Interrupt Priority */
    NVIC_SetPriority(UART_WIT_INST_INT_IRQN, 1);

    /* Configure DMA Receive Event */
    DL_UART_Main_enableDMAReceiveEvent(UART_WIT_INST, DL_UART_DMA_INTERRUPT_RX);
    /* Configure FIFOs */
    DL_UART_Main_enableFIFOs(UART_WIT_INST);
    DL_UART_Main_setRXFIFOThreshold(UART_WIT_INST, DL_UART_RX_FIFO_LEVEL_1_2_FULL);

    DL_UART_Main_setRXInterruptTimeout(UART_WIT_INST, 1);

    DL_UART_Main_enable(UART_WIT_INST);
}
static const DL_UART_Main_ClockConfig gK230ClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gK230Config = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_K230_init(void)
{
    DL_UART_Main_setClockConfig(K230_INST, (DL_UART_Main_ClockConfig *) &gK230ClockConfig);

    DL_UART_Main_init(K230_INST, (DL_UART_Main_Config *) &gK230Config);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115190.78
     */
    DL_UART_Main_setOversampling(K230_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(K230_INST, K230_IBRD_40_MHZ_115200_BAUD, K230_FBRD_40_MHZ_115200_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(K230_INST,
                                 DL_UART_MAIN_INTERRUPT_RX);
    /* Setting the Interrupt Priority */
    NVIC_SetPriority(K230_INST_INT_IRQN, 1);


    DL_UART_Main_enable(K230_INST);
}

static const DL_DMA_Config gDMA_WITConfig = {
    .transferMode   = DL_DMA_SINGLE_TRANSFER_MODE,
    .extendedMode   = DL_DMA_NORMAL_MODE,
    .destIncrement  = DL_DMA_ADDR_INCREMENT,
    .srcIncrement   = DL_DMA_ADDR_UNCHANGED,
    .destWidth      = DL_DMA_WIDTH_BYTE,
    .srcWidth       = DL_DMA_WIDTH_BYTE,
    .trigger        = UART_WIT_INST_DMA_TRIGGER,
    .triggerType    = DL_DMA_TRIGGER_TYPE_EXTERNAL,
};

SYSCONFIG_WEAK void SYSCFG_DL_DMA_WIT_init(void)
{
    DL_DMA_initChannel(DMA, DMA_WIT_CHAN_ID , (DL_DMA_Config *) &gDMA_WITConfig);
}
SYSCONFIG_WEAK void SYSCFG_DL_DMA_init(void){
    SYSCFG_DL_DMA_WIT_init();
}


