#include "speed_control.h"
#include "encoder.h"
#include "TB6612/tb6612.h"

/*
 *  PWM 周期 = timerCount = 500，占空比范围: 0 ~ 500
 *
 *  10ms 控制周期, 整数 PI 输出 (float 参数, 整数计算)
 *  左右轮独立 PI, 各自跟踪自己的目标速度
 *
 *  PWM 通道: CC0=左后(motor_bl), CC1=右后(motor_br)
 *  调用 TB6612 的 Motor_Set_Duty / Motor_Stop 控制电机
 */

/* PI 参数 (可通过蓝牙 slider 指令实时调整) */
static float g_kp = 0.05f;
static float g_ki = 0.01f;
static float g_kd = 0.0f;

#define MAX_INTEGRAL 3000
#define MAX_DUTY      480   /* 上限 96% */
#define MIN_DUTY       10   /* 下限 2%，避免堵转 */
#define PWM_PERIOD    500   /* PWM 周期, 用于 duty 计算 */

static int32_t g_targetLeft   = 0;
static int32_t g_targetRight  = 0;
static int32_t g_leftIntegral  = 0;
static int32_t g_rightIntegral = 0;
static int32_t g_leftDuty      = 150;
static int32_t g_rightDuty     = 150;

void SPEED_Init(int32_t base_duty)
{
    g_targetLeft     = 0;
    g_targetRight    = 0;
    g_leftIntegral   = 0;
    g_rightIntegral  = 0;
    g_leftDuty       = base_duty;
    g_rightDuty      = base_duty;

    if (g_leftDuty  > MAX_DUTY) g_leftDuty  = MAX_DUTY;
    if (g_leftDuty  < MIN_DUTY) g_leftDuty  = MIN_DUTY;
    if (g_rightDuty > MAX_DUTY) g_rightDuty = MAX_DUTY;
    if (g_rightDuty < MIN_DUTY) g_rightDuty = MIN_DUTY;

    /* 初始停止 */
    Motor_Stop(&motor_bl);
    Motor_Stop(&motor_br);
}

void SPEED_SetTarget(int32_t left_speed, int32_t right_speed)
{
    int32_t curLeft  = ENCODER_GetLeftSpeed();
    int32_t curRight = ENCODER_GetRightSpeed();

    g_targetLeft  = left_speed;
    g_targetRight = right_speed;

    /* 停止: 双轮目标均为 0 */
    if (left_speed == 0 && right_speed == 0) {
        g_leftIntegral  = 0;
        g_rightIntegral = 0;
        g_leftDuty      = 0;
        g_rightDuty     = 0;
        Motor_Stop(&motor_bl);
        Motor_Stop(&motor_br);
        return;
    }

    /* ---- 左轮 PI: 独立跟踪 left_speed ---- */
    if (left_speed != 0) {
        int32_t error = left_speed - curLeft;
        g_leftIntegral += error;
        if (g_leftIntegral >  MAX_INTEGRAL) g_leftIntegral =  MAX_INTEGRAL;
        if (g_leftIntegral < -MAX_INTEGRAL) g_leftIntegral = -MAX_INTEGRAL;

        float correction = g_kp * (float)error + g_ki * (float)g_leftIntegral;
        g_leftDuty += (int32_t)correction;
        if (g_leftDuty > MAX_DUTY) g_leftDuty = MAX_DUTY;
        if (g_leftDuty < 0)        g_leftDuty = 0;
    }

    /* ---- 右轮 PI: 独立跟踪 right_speed ---- */
    if (right_speed != 0) {
        int32_t error = right_speed - curRight;
        g_rightIntegral += error;
        if (g_rightIntegral >  MAX_INTEGRAL) g_rightIntegral =  MAX_INTEGRAL;
        if (g_rightIntegral < -MAX_INTEGRAL) g_rightIntegral = -MAX_INTEGRAL;

        float correction = g_kp * (float)error + g_ki * (float)g_rightIntegral;
        g_rightDuty += (int32_t)correction;
        if (g_rightDuty > MAX_DUTY) g_rightDuty = MAX_DUTY;
        if (g_rightDuty < 0)        g_rightDuty = 0;
    }

    /* 写入 PWM + 方向: duty = 占空比 / PWM周期 */
    Motor_Set_Duty(&motor_bl, (float)g_leftDuty   / (float)PWM_PERIOD);
    Motor_Set_Duty(&motor_br, (float)g_rightDuty  / (float)PWM_PERIOD);
}

int32_t SPEED_GetTargetLeft(void)    { return g_targetLeft; }
int32_t SPEED_GetTargetRight(void)   { return g_targetRight; }
int32_t SPEED_GetLeftDuty(void)      { return g_leftDuty; }
int32_t SPEED_GetRightDuty(void)     { return g_rightDuty; }

void SPEED_SetKp(float kp) { g_kp = kp; }
void SPEED_SetKi(float ki) { g_ki = ki; }
void SPEED_SetKd(float kd) { g_kd = kd; }
float SPEED_GetKp(void)    { return g_kp; }
float SPEED_GetKi(void)    { return g_ki; }
float SPEED_GetKd(void)    { return g_kd; }
