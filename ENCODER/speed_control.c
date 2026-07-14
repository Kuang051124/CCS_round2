#include "speed_control.h"
#include "encoder.h"
#include "TB6612/tb6612.h"
#include "Drivers/MSPM0/clock.h"   /* tick_ms, 用于防重入 */

/*
 *  PWM 周期 = 500，占空比范围: 0 ~ 500 (0% ~ 100%)
 *
 *  架构:
 *    SPEED_SetTarget()  → 只存目标, 不跑 PID  (主循环/task 调用)
 *    SPEED_PID_Tick()   → 测速→PID→输出      (10ms ISR 独占)
 *
 *  这样保证 PID 严格按 10ms 节拍执行, 不会被主循环的额外调用打乱。
 */

/* ---- PID 参数 (可通过 SPEED_SetKp/Ki/Kd 在线调整) ---- */
static float g_kp = 0.4f;
static float g_ki = 0.4f;
static float g_kd = 0.0005f;

#define SAMPLING_DT   0.01f   /* 控制周期 (秒), 固定 10ms */
#define MAX_DUTY      480     /* 上限 96% */
#define MIN_DUTY       10     /* 下限  2%, 避免堵转 */
#define PWM_PERIOD    500     /* PWM 周期 (ARR), 归一化用 */

/* ---- 目标速度 (counts/s) ---- */
static int32_t g_targetLeft   = 0;
static int32_t g_targetRight  = 0;
static uint8_t g_pidActive    = 0;     /* PID 活跃标志 */

/* ---- 增量式 PID 状态 ---- */
static int32_t g_lastErrorL  = 0;
static int32_t g_lastErrorR  = 0;
static int32_t g_prevDeltaL  = 0;
static int32_t g_prevDeltaR  = 0;
static int32_t g_outputL     = 0;
static int32_t g_outputR     = 0;
static int32_t g_leftDuty    = 0;
static int32_t g_rightDuty   = 0;

/* ---- 防重入: 距上次 PID 不足 8ms 则跳过 ---- */
static uint32_t g_lastTickMs = 0;

/* ===================================================================
 *  SPEED_Init — 上电初始化
 * =================================================================== */
void SPEED_Init(int32_t base_duty)
{
    g_pidActive    = 0;
    g_targetLeft   = 0;
    g_targetRight  = 0;
    g_lastErrorL   = 0;  g_lastErrorR   = 0;
    g_prevDeltaL   = 0;  g_prevDeltaR   = 0;
    g_outputL      = 0;  g_outputR      = 0;
    g_leftDuty     = base_duty;
    g_rightDuty    = base_duty;
    g_lastTickMs   = 0;

    if (g_leftDuty  > MAX_DUTY) g_leftDuty  = MAX_DUTY;
    if (g_leftDuty  < MIN_DUTY) g_leftDuty  = MIN_DUTY;
    if (g_rightDuty > MAX_DUTY) g_rightDuty = MAX_DUTY;
    if (g_rightDuty < MIN_DUTY) g_rightDuty = MIN_DUTY;

    Motor_Stop(&motor_bl);
    Motor_Stop(&motor_br);
}

/* ===================================================================
 *  SPEED_SetTarget — 设定目标速度 (不跑 PID)
 *
 *  主循环 / task / 蓝牙 调用此函数只存目标值。
 *  实际的 PID 计算由 10ms ISR 中的 SPEED_PID_Tick() 独占执行。
 * =================================================================== */
void SPEED_SetTarget(int32_t left_speed, int32_t right_speed)
{
    g_targetLeft  = left_speed;
    g_targetRight = right_speed;

    /* ---- 停止: 双轮目标均为 0 → 急停 + 全清零 ---- */
    if (left_speed == 0 && right_speed == 0) {
        g_pidActive   = 0;
        g_lastErrorL  = 0;  g_lastErrorR  = 0;
        g_prevDeltaL  = 0;  g_prevDeltaR  = 0;
        g_outputL     = 0;  g_outputR     = 0;
        g_leftDuty    = 0;  g_rightDuty   = 0;
        g_lastTickMs  = 0;
        Motor_Stop(&motor_bl);
        Motor_Stop(&motor_br);
        return;
    }

    g_pidActive = 1;  /* 通知 ISR: 有非零目标, 可以跑 PID */
}

/* ===================================================================
 *  SPEED_PID_Tick — 执行一轮增量式 PID (由 10ms ISR 独占调用)
 *
 *  公式 (增量式):
 *    Δu = Kp*[e(k)-e(k-1)] + Ki*e(k)*dt + (Kd/dt)*[e(k)-2e(k-1)+e(k-2)]
 *    u(k) = u(k-1) + Δu
 *
 *  速度前馈:
 *    外环目标大幅跳变 (>100 counts/s) 时, 先按线性模型预估占空比直接加载,
 *    PID 从零误差开始微调。避免积分项从 0 慢慢爬坡, 消除起步/变速滞后。
 *
 *  防重入: 距上次执行 < 8ms 则跳过 (防止 ISR + 主循环双重触发)
 * =================================================================== */
void SPEED_PID_Tick(void)
{
    uint32_t now = tick_ms;
    if (now - g_lastTickMs < 8) return;   /* 距上次不足 8ms, 跳过 */
    g_lastTickMs = now;

    if (!g_pidActive) return;

    int32_t curLeft  = ENCODER_GetLeftSpeed();
    int32_t curRight = ENCODER_GetRightSpeed();

    /* ================================================================
     * 速度前馈: 目标大幅跳变时, 在当前占空比基础上加 P×jump
     *
     *   线性关系: 1 count/s ≈ 480/5000 = 0.096 占空比
     *   增量修正: g_output += FF_KP × (新目标 - 旧目标)
     *
     *   优点:
     *     - 不重置 PID 历史, 增量式 delta 自然衔接
     *     - 自适应当前工作点 (电池电压/负载已体现在 g_output 基值中)
     *
     *   触发条件: 首次激活 / 目标跳变 >200 / 停车后重启 (输出=0但目标≠0)
     * ================================================================ */
    // {
    //     static int32_t prev_target_l = 0, prev_target_r = 0;
    //     static uint8_t  ff_inited    = 0;

    //     #define FF_KP  0.5f   /* duty/count: 线性模型 480/5000 */

    //     int32_t jump_l = g_targetLeft  - prev_target_l;
    //     int32_t jump_r = g_targetRight - prev_target_r;

    //     if (!ff_inited
    //         || abs(jump_l) > 5000
    //         || (g_outputL == 0 && g_targetLeft != 0)) {
    //         g_outputL += (int32_t)(FF_KP * (float)jump_l);
    //         if (g_outputL > MAX_DUTY) g_outputL = MAX_DUTY;
    //         if (g_outputL < 0)        g_outputL = 0;
    //     }
    //     if (!ff_inited
    //         || abs(jump_r) > 5000
    //         || (g_outputR == 0 && g_targetRight != 0)) {
    //         g_outputR += (int32_t)(FF_KP * (float)jump_r);
    //         if (g_outputR > MAX_DUTY) g_outputR = MAX_DUTY;
    //         if (g_outputR < 0)        g_outputR = 0;
    //     }

    //     prev_target_l = g_targetLeft;
    //     prev_target_r = g_targetRight;
    //     ff_inited = 1;
    // }

    /* ======== 左轮 增量式 PID ======== */
    {
        int32_t error = g_targetLeft - curLeft;
        int32_t delta = error - g_lastErrorL;
        int32_t d2   = delta - g_prevDeltaL;

        float p_term = g_kp * (float)delta;
        float i_term = g_ki * (float)error * SAMPLING_DT;
        float d_term = g_kd * (float)d2 / SAMPLING_DT;

        g_prevDeltaL = delta;
        g_lastErrorL = error;

        g_outputL += (int32_t)(p_term + i_term + d_term);

        if (g_outputL > MAX_DUTY) { g_outputL = MAX_DUTY; }
        if (g_outputL < 0)        { g_outputL = 0;        }

        g_leftDuty = g_outputL;
    }

    /* ======== 右轮 增量式 PID ======== */
    {
        int32_t error = g_targetRight - curRight;
        int32_t delta = error - g_lastErrorR;
        int32_t d2   = delta - g_prevDeltaR;

        float p_term = g_kp * (float)delta;
        float i_term = g_ki * (float)error * SAMPLING_DT;
        float d_term = g_kd * (float)d2 / SAMPLING_DT;

        g_prevDeltaR = delta;
        g_lastErrorR = error;

        g_outputR += (int32_t)(p_term + i_term + d_term);

        if (g_outputR > MAX_DUTY) { g_outputR = MAX_DUTY; }
        if (g_outputR < 0)        { g_outputR = 0;        }

        g_rightDuty = g_outputR;
    }

    /* ---- 写入硬件 ---- */
    Motor_Set_Duty(&motor_bl, (float)g_leftDuty  / (float)PWM_PERIOD);
    Motor_Set_Duty(&motor_br, (float)g_rightDuty / (float)PWM_PERIOD);
}

/* ===================================================================
 *  SPEED_Stop — 紧急停止
 * =================================================================== */
void SPEED_Stop(void)
{
    SPEED_SetTarget(0, 0);
    /* 立刻停 PWM, 不等 ISR */
    Motor_Stop(&motor_bl);
    Motor_Stop(&motor_br);
}

/* ---- Getter ---- */
uint8_t SPEED_IsActive(void)        { return g_pidActive; }
int32_t SPEED_GetTargetLeft(void)   { return g_targetLeft; }
int32_t SPEED_GetTargetRight(void)  { return g_targetRight; }
int32_t SPEED_GetLeftDuty(void)     { return g_leftDuty; }
int32_t SPEED_GetRightDuty(void)    { return g_rightDuty; }

/* ---- PID 参数在线调整 ---- */
void  SPEED_SetKp(float kp) { g_kp = kp; }
void  SPEED_SetKi(float ki) { g_ki = ki; }
void  SPEED_SetKd(float kd) { g_kd = kd; }
float SPEED_GetKp(void)      { return g_kp; }
float SPEED_GetKi(void)      { return g_ki; }
float SPEED_GetKd(void)      { return g_kd; }
