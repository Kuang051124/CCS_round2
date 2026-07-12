/**
 * @file  mode1.c
 * @brief 小车行驶控制 — Task1 & Task2 共享实现
 *
 * Task 1: A→B(直)→BC弧(CW)→C→D(直)→DA弧(CW)→A      ≤30s
 * Task 2: A→C(直)→CB弧(CCW)→B→D(直)→DA弧(CW)→A     ≤40s
 *
 * 架构:
 *   路径 = PathSeg 数组, 状态机逐段执行
 *   Task1/2 差异 = 不同路径数组 + 直线段航向偏转
 *
 * 后轮双电机 (motor_bl/motor_br), 编码器 + 巡线传感器
 */

#include "mode1.h"
#include "task4.h"
#include "../Trace5/trace.h"
#include "../TB6612/tb6612.h"
#include "../Drivers/WIT/wit.h"
#include "../Drivers/MPU6050/mpu6050.h"
#include "../Drivers/MSPM0/clock.h"
#include "../Keyboard/keyboard.h"
#include "../ENCODER/encoder.h"
#include "../ENCODER/speed_control.h"
#include "ti_msp_dl_config.h"
#include "pages.h"

#include <math.h>
#include <stdio.h>

/* ===================================================================
 * 陀螺仪源选择
 * =================================================================== */
#define GYRO_SRC_WIT  0   /* WIT 串口陀螺仪 (默认) */
#define GYRO_SRC_MPU  1   /* MPU6050 I2C + DMP      */
uint8_t tk_gyro_src = GYRO_SRC_WIT;

/* ===================================================================
 * 后轮双电机便捷宏
 * =================================================================== */

/* 电机控制宏: 统一改用 PID 速度闭环
 * 占空比 → 编码器目标速度 (counts/s), 由 10ms ISR 中的 SPEED_PID_Tick() 闭环执行
 */
#define MOTOR_ALL_STOP()  SPEED_Stop()

/* ===================================================================
 * 动态参数 (Page_Calib 可调)
 * =================================================================== */

// T1Param t1 = { 1400.0f, 1200.0f, 100.0f, 20.0f, 175.0f, -175.0f, 600.0f, -600.0f };
//                        SpdStr SpdArc  ArcKP  GyroKP  OfsCW  OfsCCW OfsCW2 OfsCCW2  (counts/s)
T1Param t1 = { 700.0f, 600.0f, 80.0f, 10.0f, 80.0f, -80.0f, 300.0f, -300.0f  };

/* Task3 慢速参数 (功能同 Task1, 速度偏小) */
T1Param t3 = { 500.0f, 400.0f, 80.0f, 10.0f, 80.0f, -80.0f, 300.0f, -300.0f };

/* 当前使用的参数指针: Task1/2 → &t1, Task3/4 → &t3 */
T1Param *tk_param = &t1;

/* 全局速度倍率: Car B 距离 P 控制用, 1.0 = 不变 */
float tk_speed_mult = 1.0f;

/* 超车航向偏置: 叠加到目标航向, 超车状态机实时修改 */
float tk_heading_bias = 0.0f;

/* ===================================================================
 * 参数 Flash 持久化 (扇区 126, 地址 0x1F800, 32 字节)
 * =================================================================== */

#define TKPARAM_FLASH_ADDR  0x0001F800u
#define TKPARAM_MAGIC       0x544B5032u  /* "TKP2" — 8 floats */

typedef struct {
    uint32_t magic;
    uint8_t  checksum;
    uint8_t  reserved[3];
    T1Param  p;              /* 嵌入参数结构体, 32 bytes total */
} TKParamStore;

static uint8_t tkparam_calc_checksum(const TKParamStore *s)
{
    const uint8_t *p = (const uint8_t *)&s->p;
    uint8_t sum = 0;
    for (int i = 0; i < (int)sizeof(T1Param); i++)
        sum ^= p[i];
    return sum;
}

void TKParam_Load(void)
{
    const TKParamStore *flash = (const TKParamStore *)TKPARAM_FLASH_ADDR;
    if (flash->magic == TKPARAM_MAGIC &&
        flash->checksum == tkparam_calc_checksum(flash)) {
        t1 = flash->p;      /* 整体赋值, 编译器保证不重叠 */
    }
}

void TKParam_Save(void)
{
    TKParamStore buf;
    buf.magic       = TKPARAM_MAGIC;
    buf.reserved[0] = 0;
    buf.reserved[1] = 0;
    buf.reserved[2] = 0;
    buf.p           = t1;   /* 整体复制 */
    buf.checksum    = tkparam_calc_checksum(&buf);

    DL_FlashCTL_executeClearStatus(FLASHCTL);

    __disable_irq();

    DL_FlashCTL_unprotectSector(FLASHCTL, TKPARAM_FLASH_ADDR,
                                DL_FLASHCTL_REGION_SELECT_MAIN);

    DL_FLASHCTL_COMMAND_STATUS status;
    status = DL_FlashCTL_eraseMemoryFromRAM(FLASHCTL, TKPARAM_FLASH_ADDR,
                                            DL_FLASHCTL_COMMAND_SIZE_SECTOR);
    if (status != DL_FLASHCTL_COMMAND_STATUS_PASSED) {
        DL_FlashCTL_executeClearStatus(FLASHCTL);
        DL_FlashCTL_unprotectSector(FLASHCTL, TKPARAM_FLASH_ADDR,
                                    DL_FLASHCTL_REGION_SELECT_MAIN);
        DL_FlashCTL_eraseMemoryFromRAM(FLASHCTL, TKPARAM_FLASH_ADDR,
                                       DL_FLASHCTL_COMMAND_SIZE_SECTOR);
    }

    /* 写入 5 个 64-bit ECC words (40 bytes) */
    const uint32_t *data = (const uint32_t *)&buf;
    for (int i = 0; i < 5; i++) {
        DL_FlashCTL_executeClearStatus(FLASHCTL);
        DL_FlashCTL_unprotectSector(FLASHCTL, TKPARAM_FLASH_ADDR,
                                    DL_FLASHCTL_REGION_SELECT_MAIN);
        DL_FlashCTL_programMemoryFromRAM64WithECCGenerated(
            FLASHCTL, TKPARAM_FLASH_ADDR + i * 8, &data[i * 2]);
    }

    __enable_irq();
}

/* ===================================================================
 * 路径定义
 * =================================================================== */

/*
 * Task 1: A→B(直,航向=当前) → BC弧(CW,~180°) → C→D(直,初始+180°) → DA弧(CW,~180°) → A
 *
 * A→B 采样当前航向 (放车时自然朝向 B)
 * C→D = 初始航向 + 180° (A→B 的反方向, BC弧转了半圈后正好沿此方向)
 */
const PathSeg task1_path[TASK1_SEG_COUNT] = {
    {SEG_STRAIGHT, T1_HEADING_AB, "A->B Str", "B", 0},
    {SEG_ARC_CW,   0,             "BC Arc",   "C", 0},
    {SEG_STRAIGHT, T1_HEADING_CD, "C->D Str", "D", 0},
    {SEG_ARC_CW,   0,             "DA Arc",   "A", 1},
};

/*
 * Task 2: A→C(直,init-atan0.8) → CB弧(CCW) → B→D(直,init+180-atan0.8) → DA弧(CW) → A
 *
 * 右转 = yaw 减小, 小车水平摆放初始航向=0°
 *   A→C: 目标 = init - atan0.8 = -38.66° (右偏)
 *   CB弧: CCW绕半圈 yaw 增大 ~180°
 *   B→D: 目标 = init - 180° + atan0.8  (反向偏回)
 */
const PathSeg task2_path[TASK2_SEG_COUNT] = {
    {SEG_STRAIGHT, T2_HEADING_AC,  "A->C Str", "C", 0},
    {SEG_ARC_CCW,  0,              "CB Arc",   "B", 0},
    {SEG_STRAIGHT, T2_HEADING_BD,  "B->D Str", "D", 0},
    {SEG_ARC_CW,   0,              "DA Arc",   "A", 1},
};

/*
 * B车 Task3 路径: D → DA弧(CW) → A → A→C(直) → C → CB弧(CCW) → B → B→D(直) → D → DA弧(CW) → (追A,等STOP)
 *
 * 右转=yaw减小, 初始yaw在D点记录
 *   DA弧:   CW绕半圈 yaw减小 ~180° 到达A                       is_return=0 (远离)
 *   A→C:    目标 = init - 180° - atan0.8  (经过DA弧后在A点右偏)
 *   CB弧:    CCW绕半圈 yaw增大 ~180° 到达B                      is_return=0 (远离)
 *   B→D:    目标 = init + atan0.8         (返回D方向)
 *   DA弧2:  CW绕半圈 yaw减小 ~180°, is_return=1 (接近) + line372拦截永不触发, 等STOP
 */
const PathSeg task3b_path[TASK3B_SEG_COUNT] = {
    {SEG_ARC_CW,   0,              "DA Arc",   "A", 0},
    {SEG_STRAIGHT, TB_HEADING_AC,  "A->C Str", "C", 0},
    {SEG_ARC_CCW,  0,              "CB Arc",   "B", 1},
    {SEG_STRAIGHT, TB_HEADING_BD,  "B->D Str", "D", 0},
    {SEG_ARC_CW,   0,              "DA Arc2",  "A", 0},
};

/*
 * A车 Task4 路径: A → A→O(直) → O → O→B(直) → B → BC弧(CW) → C → C→D(直) → D → DA弧(CW) → A
 *
 * A→O: T2_HEADING_AC 方向 (经过中心O)
 * O→B: T2_HEADING_BD+180° 方向
 * 过O点完成路线交叉, 替代旧的 heading_bias 超车方案
 */
const PathSeg task4a_path[TASK4A_SEG_COUNT] = {
    {SEG_STRAIGHT_DELAY, TA_HEADING_AO,  "A->O Dly", "O", 0},
    {SEG_STRAIGHT_DELAY, TA_HEADING_OB,  "O->B Dly", "B", 0},
    {SEG_ARC_CW,         0,              "BC Arc",   "C", 0},
    {SEG_STRAIGHT,       T1_HEADING_CD,  "C->D Str", "D", 0},
    {SEG_ARC_CW,         0,              "DA Arc",   "A", 1},
};

/*
 * B车 Task4 路径: D → DA弧(CW) → A → A→B(直) → B → BC弧(CW) → C → C→O(直) → O → O→D(直) → D
 *
 * C→O: TB_HEADING_AC+180° 方向 (经过中心O)
 * O→D: TB_HEADING_BD 方向
 */
const PathSeg task4b_path[TASK4B_SEG_COUNT] = {
    {SEG_ARC_CW,         0,              "DA Arc",   "A", 0},
    {SEG_STRAIGHT,       TB_HEADING_AB,  "A->B Str", "B", 0},
    {SEG_ARC_CW,         0,              "BC Arc",   "C", 1},
    {SEG_STRAIGHT_DELAY, TB_HEADING_CO,  "C->O Dly", "O", 0},
    {SEG_STRAIGHT_DELAY, TB_HEADING_BD,  "O->D Dly", "D", 0},
};

/* ===================================================================
 * 通用状态机 (Task1/2 共用)
 * =================================================================== */

typedef enum {
    TASK_IDLE = 0,
    TASK_SEG_STRAIGHT,    /* 直线段: 陀螺仪保持 */
    TASK_SEG_ARC,         /* 弧线段: 巡线 */
    TASK_DONE,            /* 完成 */
    TASK_EMERGENCY_STOP,  /* 超时 */
} task_state_t;

/* ===================================================================
 * 运行时变量
 * =================================================================== */

static task_state_t  tk_state = TASK_IDLE;

/* 当前路径 */
static const PathSeg *tk_path       = NULL;
static uint8_t        tk_seg_count  = 0;
static uint8_t        tk_seg_index  = 0;   /* 当前段在 path[] 中的下标 */

/* 陀螺仪 */
static float tk_initial_yaw  = 0.0f;   /* 任务启动时的初始航向 */
static float tk_target_yaw   = 0.0f;   /* 当前直线段目标航向 */
static float tk_current_yaw  = 0.0f;   /* 当前 yaw (显示用) */
static float tk_last_yaw_err = 0.0f;

/* 弧线偏移 (当前段) */
static float tk_arc_offset = 0.0f;

/* 弧线终点: 连续全白计数器 */
static uint8_t tk_white_cnt    = 0;

/* 计时 */
static uint32_t tk_blind_start_ms = 0;
static uint32_t tk_total_start_ms = 0;
static uint32_t tk_elapsed_ms     = 0;
static uint32_t tk_yaw_disp_tick  = 0;

/* 计数 */
static uint8_t tk_points_passed = 0;

/* 非阻塞声光: BEEP_TIM 100ms 单次定时器 tick 计数 (>0=蜂鸣中) */
static volatile uint8_t beep_ticks = 0;

/* ===================================================================
 * 声光提示
 * =================================================================== */

static void tk_beep(uint32_t ms)
{
    static uint8_t irq_inited = 0;
    if (!irq_inited) {
        NVIC_EnableIRQ(BEEP_TIM_INST_INT_IRQN);  /* syscfg不会自动开NVIC, 此处补开 */
        irq_inited = 1;
    }

    DL_GPIO_clearPins(BUZZER_PORT, BUZZER_BEEP_PIN);  /* LOW = 蜂鸣器响 */
    DL_GPIO_setPins(LED_PORT, LED_RED_PIN);           /* HIGH = LED亮   */

    beep_ticks = (uint8_t)((ms + 99) / 100);  /* 向上取整到100ms单位 */
    if (beep_ticks == 0) beep_ticks = 1;      /* 至少1个tick        */

    DL_TimerA_startCounter(BEEP_TIM_INST);     /* 启动100ms单次定时器  */
}

/* BEEP_TIM 中断: 非阻塞声光 tick 计数 */
void BEEP_TIM_INST_IRQHandler(void)
{
    switch (DL_TimerA_getPendingInterrupt(BEEP_TIM_INST)) {
    case DL_TIMERA_IIDX_ZERO:
        if (beep_ticks > 1) {
            beep_ticks--;
            DL_TimerA_startCounter(BEEP_TIM_INST);   /* 还有剩余tick, 重启定时器 */
        } else {
            /* 时间到, 关闭声光 */
            DL_GPIO_setPins(BUZZER_PORT, BUZZER_BEEP_PIN);    /* HIGH = 关 */
            DL_GPIO_clearPins(LED_PORT, LED_RED_PIN);         /* LOW = LED灭 */
            beep_ticks = 0;
        }
        break;
    }
}

/* ===================================================================
 * 途经顶点 — 停车+声光+OLED
 * =================================================================== */

static void tk_arrive_at_point(const char *name, uint32_t beep_ms)
{
    /* 不 STOP 马达, 不清 OLED (下一帧 tk_enter_segment 会刷新) */
    tk_points_passed++;
    tk_beep(beep_ms);   /* 非阻塞声光 */
}

/* ===================================================================
 * 巡线传感器辅助
 * =================================================================== */

static uint8_t tk_black_count(void)
{
    Trace_Read_Bit();
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < SENSOR_LEN; i++)
        if (trace_states[i] == BLACK) cnt++;
    return cnt;
}

/* 读取当前 yaw — MPU 模式下自动调用 Read_Quad() */
static float tk_get_yaw(void)
{
    if (tk_gyro_src == GYRO_SRC_MPU) {
        Read_Quad();
        return yaw;
    }
    return wit_data.yaw;
}

/* 直线→弧线: 盲区后首次黑线; 或编码器到位终点 (Task4 delay段) */
static uint8_t tk_detect_straight_end(void)
{
    if (tick_ms - tk_blind_start_ms < T1_STRAIGHT_BLIND_MS) return 0;

    /* 编码器到位终点: Task4 delay段 (A→O, O→B, C→O, O→D) */
    if (tk_path[tk_seg_index].type == SEG_STRAIGHT_DELAY) {
        int32_t avg = (ENCODER_GetLeftCount() + ENCODER_GetRightCount()) / 2;
        return (avg >= T4_ENCODER_TARGET);
    }

    uint8_t cnt = tk_black_count();
    return (cnt >= T1_BLACK_THRESHOLD && cnt < SENSOR_LEN);
}

/* 弧线→直线: 连续N帧全白 AND 与起点航向相差 > 阈值 */
static uint8_t tk_detect_arc_end(void)
{
    uint8_t all_white = (tk_black_count() == 0);

    if (all_white) {
        tk_white_cnt++;
    } else {
        tk_white_cnt = 0;
    }

    /* 用初始航向判断弧线终点 (处理 0/360 跳变) */
    float delta = tk_get_yaw() - tk_initial_yaw;
    if (delta >  180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;

    uint8_t yaw_ok = 0;

    /* Task3 B车最后一段弧永不结束: 等蓝牙STOP, 不走弧线终点判定 */
    if (tk_path == task3b_path && tk_seg_index == tk_seg_count - 1) {
        return 0;
    }

    /* 弧线段用 is_return 判断: 0=远离起点(delta需大), 1=接近起点(delta需小) */
    if (!tk_path[tk_seg_index].is_return) {
        yaw_ok = (fabs(delta) >= T1_ARC_YAW_DELTA);
    } else {
        yaw_ok = (fabs(delta) <= (180.0f - T1_ARC_YAW_DELTA));
    }

    if (tk_white_cnt >= T1_ARC_END_FRAMES && yaw_ok) {
        tk_white_cnt = 0;
        return 1;
    }
    return 0;
}

/* ===================================================================
 * 陀螺仪航向保持 (根据 tk_gyro_src 自动切换 WIT / MPU6050)
 * =================================================================== */

/* 根据 heading_ofs 设置目标航向 */
static void tk_set_heading(float heading_ofs)
{
    if (heading_ofs == 0.0f) {
        /* ofs=0: 沿初始朝向直行 (与启动时采样一致) */
        tk_target_yaw   = tk_initial_yaw;
        tk_last_yaw_err = 0.0f;
    } else {
        /* ofs≠0: 基于初始航向 + 偏移 (Task2 直线段) */
        tk_target_yaw   = tk_initial_yaw + heading_ofs;
        tk_last_yaw_err = 0.0f;
    }
}

static float tk_yaw_error(void)
{
    float current = tk_get_yaw();
    float err = current - tk_target_yaw;
    if (err >  180.0f) err -= 360.0f;
    if (err < -180.0f) err += 360.0f;
    return err;
}

/* 陀螺仪航向 P 控制 */
static void tk_gyro_control(float base_speed)
{
    base_speed *= tk_speed_mult;
    tk_current_yaw  = tk_get_yaw();      /* 一次读取, 避免二次调用消耗 */
    float yaw_err   = tk_current_yaw - tk_target_yaw - tk_heading_bias;
    if (yaw_err >  180.0f) yaw_err -= 360.0f;
    if (yaw_err < -180.0f) yaw_err += 360.0f;
    tk_last_yaw_err = yaw_err;
    float steering  = -tk_param->gyro_kp * yaw_err;  /* 负号: 纠正陀螺仪角度变化方向 */

    /* 限幅: 防止航向偏差大时单侧电机过快 */
    float steer_max = base_speed * T1_GYRO_STEER_MAX;
    if (steering >  steer_max) steering =  steer_max;
    if (steering < -steer_max) steering = -steer_max;

    float left  = base_speed - steering;
    float right = base_speed + steering;
    if (left  >  SPEED_MAX) left  =  SPEED_MAX;
    if (left  < -SPEED_MAX) left  = -SPEED_MAX;
    if (right >  SPEED_MAX) right =  SPEED_MAX;
    if (right < -SPEED_MAX) right = -SPEED_MAX;

    /* PID 速度闭环: 参数已为编码器目标速度 (counts/s) */
    SPEED_SetTarget((int32_t)left, (int32_t)right);
}

/* ===================================================================
 * 巡线控制 (比例公式 + 弧线偏置)
 * =================================================================== */

static void tk_trace_control(float base_speed)
{
    base_speed *= tk_speed_mult;
    Trace_Read_Bit();
    int sum = Get_Trace_Sum(trace_states);

    tk_current_yaw = tk_get_yaw();   /* 同步 yaw (显示用) */

    /* --- 计算偏置是否生效 (航向判断弧线进度) --- */
    float yaw_d = tk_current_yaw - tk_initial_yaw;
    if (yaw_d >  180.0f) yaw_d -= 360.0f;
    if (yaw_d < -180.0f) yaw_d += 360.0f;

    uint8_t apply = 1;
    // if (tk_seg_index < (tk_seg_count / 2)) {
    //     /* 路径前半段的弧远离起点 */
    //     apply = (fabs(yaw_d) < T1_ARC_YAW_DELTA);
    // } else {
    //     /* 路径后半段的弧接近起点 */
    //     apply = (fabs(yaw_d) > (180.0f - T1_ARC_YAW_DELTA));
    // }

    /* --- 舵量 = 偏置(兜底) + 巡线P(叠加) --- */
    float steering = 0.0f;

    /* 偏置始终作为基础转向 (近顶点时关闭, 防止过冲) */
    if (apply) {
        float bias = tk_arc_offset;

        /* Task2 入口需要大偏置咬线, 随弧线进度衰减到 T1 正常值 */
        if (tk_arc_offset != 0.0f
            && (tk_path == task2_path || tk_path == task4a_path || tk_path == task4b_path)) {
            float bias_t1 = (tk_arc_offset > 0)
                          ? tk_param->arc_ofs_cw : tk_param->arc_ofs_ccw;
            float bias_t2 = (tk_arc_offset > 0)
                          ? tk_param->arc_ofs_cw_t2 : tk_param->arc_ofs_ccw_t2;
            float progress;
            if(yaw_d<0&&yaw_d>-40)
                progress =(40.0f+yaw_d)/40.0f;
            else if(yaw_d<-140&&yaw_d>-180)            
                progress =(-140.0f-yaw_d)/40.0f;
            else progress = 1.0f;
            bias = bias_t2 + (bias_t1 - bias_t2) * progress;   /* T2→T1 线性过渡 */
        }
         /* Task3B 入口需要大偏置咬线, 随弧线进度衰减到 T1 正常值 */       
        if (tk_arc_offset != 0.0f && tk_path == task3b_path) {
            float bias_t1 = (tk_arc_offset > 0)
                          ? tk_param->arc_ofs_cw : tk_param->arc_ofs_ccw;
            float bias_t2 = (tk_arc_offset > 0)
                          ? tk_param->arc_ofs_cw_t2 : tk_param->arc_ofs_ccw_t2;
            float progress;
            if(yaw_d>0&&yaw_d<40)
                progress =(40.0f-yaw_d)/40.0f;
            else if(yaw_d>140&&yaw_d<180)            
                progress =(-140.0f+yaw_d)/40.0f;
            else progress = 1.0f;
            bias = bias_t2 + (bias_t1 - bias_t2) * progress;   /* T2→T1 线性过渡 */
        }
        /* Task4A和4B 入口需要大偏置咬线, 随弧线进度衰减到 T1 正常值 */
        if (tk_arc_offset != 0.0f && (tk_path == task4a_path || tk_path == task4b_path)) {            float bias_t1 = (tk_arc_offset > 0)
                          ? tk_param->arc_ofs_cw : tk_param->arc_ofs_ccw;
            float bias_t2 = (tk_arc_offset > 0)
                          ? tk_param->arc_ofs_cw_t2 : tk_param->arc_ofs_ccw_t2;
            float progress;
            if(yaw_d>0&&yaw_d<40)
                progress =(40.0f-yaw_d)/40.0f;
            else if(yaw_d>140&&yaw_d<180)            
                progress =(-140.0f+yaw_d)/40.0f;
            else progress = 1.0f;
            bias = bias_t2 + (bias_t1 - bias_t2) * progress;   /* T2→T1 线性过渡 */
        }                         
        steering = bias * tk_speed_mult;
    }

    /* 巡线 P 叠加在偏置之上 (有黑线时叠加微调, 无黑线时退化为纯偏置) */
    static int last_sum = 0;
    if (sum != 0) {
        steering += sum * tk_param->spd_arc_kp * tk_speed_mult;
        last_sum = sum;
    } else if (last_sum != 0) {
        /* 丢线恢复: 用上次偏差方向 ×1.0 倍力度扫回去找线 */
        steering += (last_sum > 0 ? 1.0f : -1.0f)
                  * tk_param->spd_arc_kp * 1.0f * tk_speed_mult;
    }

    float left  = base_speed + steering;
    float right = base_speed - steering;

    if (left  >  SPEED_MAX) left  =  SPEED_MAX;
    if (left  < -SPEED_MAX) left  = -SPEED_MAX;
    if (right >  SPEED_MAX) right =  SPEED_MAX;
    if (right < -SPEED_MAX) right = -SPEED_MAX;

    /* PID 速度闭环: 参数已为编码器目标速度 (counts/s) */
    SPEED_SetTarget((int32_t)left, (int32_t)right);
}

/* ===================================================================
 * Yaw 实时显示 (直线段每 200ms 刷新)
 * =================================================================== */

static void tk_update_yaw_display(void)
{
    if (tick_ms - tk_yaw_disp_tick < 200) return;
    /* DELAY段留给 Task4 显示编码器, 不刷 yaw */
    if (tk_path[tk_seg_index].type == SEG_STRAIGHT_DELAY) return;
    tk_yaw_disp_tick = tick_ms;

    char buf[21];
    sprintf(buf, "Yaw:%6.1f", tk_current_yaw);
    OLED_ShowString(0, 3, (uint8_t *)buf, 8);

    if (tk_state == TASK_SEG_STRAIGHT) {
        /* 直线段: 显示目标航向 + 误差 */
        sprintf(buf, "Ref:%6.1f err:%+.1f", tk_target_yaw, tk_yaw_error());
    } else {
        /* 弧线段: 显示当前与初始航向差 */
        float d = tk_get_yaw() - tk_initial_yaw;
        if (d >  180.0f) d -= 360.0f;
        if (d < -180.0f) d += 360.0f;
        sprintf(buf, "Arc: %+.0f/%3.0f deg", d, (double)T1_ARC_YAW_DELTA);
    }
    OLED_ShowString(0, 4, (uint8_t *)buf, 8);
}

/* ===================================================================
 * 进入下一段
 * =================================================================== */

static void tk_enter_segment(const PathSeg *seg)
{
    tk_white_cnt = 0;
    tk_yaw_disp_tick = 0;

    /* OLED: 段名 + 目标点 */
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Task: E", 8);
    OLED_ShowString(0, 1, (uint8_t *)seg->name, 8);
    char buf[21];
    sprintf(buf, "-> %s", seg->target);
    OLED_ShowString(0, 2, (uint8_t *)buf, 8);
    sprintf(buf, "Seg:%d/%d P:%d", tk_seg_index + 1, tk_seg_count, tk_points_passed);
    OLED_ShowString(0, 7, (uint8_t *)buf, 8);

    if (seg->type == SEG_STRAIGHT || seg->type == SEG_STRAIGHT_DELAY) {
        /* 直线段: 设置目标航向, 启动盲区计时 */
        tk_state = TASK_SEG_STRAIGHT;
        tk_arc_offset = 0.0f;
        tk_set_heading(seg->heading_ofs);
        tk_blind_start_ms = tick_ms;
        /* SEG_STRAIGHT_DELAY: 清零编码器, 后续用计数值判定终点 */
        if (seg->type == SEG_STRAIGHT_DELAY) {
            ENCODER_ResetLeft();
            ENCODER_ResetRight();
        }
    } else {
        /* 弧线段: 设置弧线偏置方向 + 记录起始航向 */
        tk_state = TASK_SEG_ARC;
        if (tk_path == task2_path || tk_path == task3b_path|| tk_path == task4a_path || tk_path == task4b_path) {
            /* Task 2 弧线偏移更大 (直线→弧线角度变化大) */
            tk_arc_offset = (seg->type == SEG_ARC_CW)
                          ? tk_param->arc_ofs_cw_t2 : tk_param->arc_ofs_ccw_t2;
        } else {
            tk_arc_offset = (seg->type == SEG_ARC_CW)
                          ? tk_param->arc_ofs_cw : tk_param->arc_ofs_ccw;
        }

    }
}

/* ===================================================================
 * 对外 API — Task1/2 初始化 & Tick
 * =================================================================== */

/* ---- 通用初始化 ---- */
void task_init(const PathSeg *path, uint8_t seg_count)
{
    tk_state         = TASK_IDLE;
    tk_path          = path;
    tk_seg_count     = seg_count;
    tk_seg_index     = 0;
    tk_points_passed = 0;
    tk_last_yaw_err  = 0.0f;
    tk_white_cnt     = 0;
}

/* ---- 通用 Tick ---- */
uint8_t task_tick(uint8_t key)
{
    uint32_t now = tick_ms;
    const PathSeg *seg = &tk_path[tk_seg_index];  /* 当前段 */

    /* ======== 全局退出 ======== */
    if (key == 9) {
        MOTOR_ALL_STOP();
        DL_GPIO_setPins(BUZZER_PORT, BUZZER_BEEP_PIN);
        DL_GPIO_clearPins(LED_PORT, LED_RED_PIN);
        OLED_Clear();
        OLED_ShowString(0, 1, (uint8_t *)"Task Aborted", 8);
        OLED_ShowString(0, 7, (uint8_t *)"Returning...", 8);
        while (Scan_Keyboard());
        return 1;
    }

    switch (tk_state) {

    /* ---- 空闲, 按键 5 启动 ---- */
    case TASK_IDLE:
        if (key == 5) {
            while (Scan_Keyboard());

            /* 采样20次取平均航向作为基准 (消除上电瞬态) */
            {
                float sum = 0.0f;
                for (int i = 0; i < T1_GYRO_SAMPLES; i++) {
                    sum += tk_get_yaw();
                    mspm0_delay_ms(T1_GYRO_SAMPLE_MS);
                }
                tk_initial_yaw = sum / T1_GYRO_SAMPLES;
            }
            /* 倒计时 */
            for (int i = 1; i > 0; i--) {
                OLED_Clear();
                char buf[21];
                sprintf(buf, "Starting in %d...", i);
                OLED_ShowString(0, 2, (uint8_t *)buf, 8);
                tk_beep(100);
            }
            tk_total_start_ms = tick_ms;
            tk_seg_index = 0;
            tk_points_passed = 0;
            tk_enter_segment(&tk_path[0]);   /* 进入第一段 */
        }
        break;

    /* ---- 直线段 ---- */
    case TASK_SEG_STRAIGHT:
        if ((now - tk_total_start_ms) > T1_MAX_TIME_SEC * 1000) {
            MOTOR_ALL_STOP(); tk_state = TASK_EMERGENCY_STOP; break;
        }
        tk_gyro_control(tk_param->spd_straight);
        tk_update_yaw_display();

        if (tk_detect_straight_end()) {
            tk_arrive_at_point(seg->target, T1_BEEP_MS);
            tk_seg_index++;

            if (tk_seg_index >= tk_seg_count) {
                /* 最后一段是直线 → 任务完成 */
                tk_elapsed_ms = now - tk_total_start_ms;
                tk_points_passed++;
                MOTOR_ALL_STOP();
                OLED_Clear();
                char buf[21];
                sprintf(buf, "Task Complete!");
                OLED_ShowString(0, 0, (uint8_t *)buf, 8);
                sprintf(buf, "Time: %4.1f s", tk_elapsed_ms / 1000.0f);
                OLED_ShowString(0, 2, (uint8_t *)buf, 8);
                OLED_ShowString(0, 4, (uint8_t *)"Arrived!", 8);
                OLED_ShowString(0, 7, (uint8_t *)"9:Return", 8);
                tk_beep(T1_STOP_BEEP_MS);
                tk_state = TASK_DONE;
            } else {
                tk_enter_segment(&tk_path[tk_seg_index]);
            }
        }
        break;

    /* ---- 弧线段 ---- */
    case TASK_SEG_ARC:
        if ((now - tk_total_start_ms) > T1_MAX_TIME_SEC * 1000) {
            MOTOR_ALL_STOP(); tk_state = TASK_EMERGENCY_STOP; break;
        }
        tk_trace_control(tk_param->spd_arc);
        tk_update_yaw_display();

        if (tk_detect_arc_end()) {
            tk_seg_index++;

            if (tk_seg_index >= tk_seg_count) {
                /* 最后一段弧结束 = 回到起点 A, 任务完成 */
                tk_elapsed_ms = now - tk_total_start_ms;
                tk_points_passed++;
                MOTOR_ALL_STOP();

                OLED_Clear();
                char buf[21];
                sprintf(buf, "Task Complete!");
                OLED_ShowString(0, 0, (uint8_t *)buf, 8);
                sprintf(buf, "Time: %4.1f s", tk_elapsed_ms / 1000.0f);
                OLED_ShowString(0, 2, (uint8_t *)buf, 8);
                OLED_ShowString(0, 4, (uint8_t *)"A <-- Arrived!", 8);
                OLED_ShowString(0, 7, (uint8_t *)"9:Return", 8);
                tk_beep(T1_STOP_BEEP_MS);
                tk_state = TASK_DONE;
            } else {
                /* 进入下一段 (直线) */
                tk_arrive_at_point(seg->target, T1_BEEP_MS);
                tk_enter_segment(&tk_path[tk_seg_index]);
            }
        }
        break;

    /* ---- 完成 ---- */
    case TASK_DONE:
        if (key == 9) {
            while (Scan_Keyboard());
            OLED_Clear();
            return 1;
        }
        {
            char buf[21];
            sprintf(buf, "Task Complete!");
            OLED_ShowString(0, 0, (uint8_t *)buf, 8);
            sprintf(buf, "Time: %4.1f s", tk_elapsed_ms / 1000.0f);
            OLED_ShowString(0, 2, (uint8_t *)buf, 8);
            OLED_ShowString(0, 7, (uint8_t *)"9:Return", 8);
        }
        break;

    /* ---- 超时 ---- */
    case TASK_EMERGENCY_STOP:
        MOTOR_ALL_STOP();
        DL_GPIO_setPins(BUZZER_PORT, BUZZER_BEEP_PIN);
        DL_GPIO_clearPins(LED_PORT, LED_RED_PIN);
        OLED_Clear();
        OLED_ShowString(0, 1, (uint8_t *)"EMERGENCY STOP", 8);
        OLED_ShowString(0, 3, (uint8_t *)"Timeout!", 8);
        OLED_ShowString(0, 7, (uint8_t *)"9:Return", 8);
        if (key == 9) { while (Scan_Keyboard()); OLED_Clear(); return 1; }
        break;

    default:
        break;
    }

    return 0;
}

/* 强制停止任务 (B车蓝牙 STOP 用, 不阻塞键盘) */
void tk_abort(void)
{
    MOTOR_ALL_STOP();
    DL_GPIO_setPins(BUZZER_PORT, BUZZER_BEEP_PIN);
    DL_GPIO_clearPins(LED_PORT, LED_RED_PIN);
    tk_state = TASK_IDLE;
}

uint8_t tk_is_done(void) { return (tk_state == TASK_DONE); }
uint8_t tk_get_seg_index(void)  { return tk_seg_index; }
float   tk_get_initial_yaw(void) { return tk_initial_yaw; }

/* ---- 对外 Task1/2 Tick 入口 (WIT 陀螺仪) ---- */
uint8_t Task1_Tick(uint8_t key) { return task_tick(key); }
uint8_t Task2_Tick(uint8_t key) { return task_tick(key); }

/* ---- 对外 Task1/2 Tick 入口 (MPU6050 陀螺仪) ---- */
uint8_t Task1_MPU_Tick(uint8_t key) { return task_tick(key); }
uint8_t Task2_MPU_Tick(uint8_t key) { return task_tick(key); }

/* ---- Task1 初始化 (WIT) ---- */
void Task1_Init(void)
{
    tk_param    = &t1;
    tk_gyro_src = GYRO_SRC_WIT;
    task_init(task1_path, TASK1_SEG_COUNT);
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Task 1: E-Track", 8);
    OLED_ShowString(0, 1, (uint8_t *)"A-B-BC-C-D-DA-A", 8);
    OLED_ShowString(0, 2, (uint8_t *)"[5] Start", 8);
    OLED_ShowString(0, 3, (uint8_t *)"[9] Back to Menu", 8);
    OLED_ShowString(0, 5, (uint8_t *)"Str: Gyro(WIT)", 8);
    OLED_ShowString(0, 6, (uint8_t *)"Arc: Trace", 8);
}

/* ---- Task2 初始化 (WIT) ---- */
void Task2_Init(void)
{
    tk_param    = &t1;
    tk_gyro_src = GYRO_SRC_WIT;
    task_init(task2_path, TASK2_SEG_COUNT);
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Task 2: E-Track", 8);
    OLED_ShowString(0, 1, (uint8_t *)"A-C-CB-B-D-DA-A", 8);
    OLED_ShowString(0, 2, (uint8_t *)"[5] Start", 8);
    OLED_ShowString(0, 3, (uint8_t *)"[9] Back to Menu", 8);
    OLED_ShowString(0, 5, (uint8_t *)"Hdg: WIT init+ofs", 8);
    OLED_ShowString(0, 6, (uint8_t *)"Arc: Trace", 8);
}

/* ---- Task1 初始化 (MPU6050) ---- */
void Task1_MPU_Init(void)
{
    tk_param    = &t1;
    tk_gyro_src = GYRO_SRC_MPU;
    task_init(task1_path, TASK1_SEG_COUNT);
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Task 1: E-Track", 8);
    OLED_ShowString(0, 1, (uint8_t *)"A-B-BC-C-D-DA-A", 8);
    OLED_ShowString(0, 2, (uint8_t *)"[5] Start", 8);
    OLED_ShowString(0, 3, (uint8_t *)"[9] Back to Menu", 8);
    OLED_ShowString(0, 5, (uint8_t *)"Str: MPU6050(P)", 8);
    OLED_ShowString(0, 6, (uint8_t *)"Arc: Trace", 8);
}

/* ---- Task2 初始化 (MPU6050) ---- */
void Task2_MPU_Init(void)
{
    tk_param    = &t1;
    tk_gyro_src = GYRO_SRC_MPU;
    task_init(task2_path, TASK2_SEG_COUNT);
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Task 2: E-Track", 8);
    OLED_ShowString(0, 1, (uint8_t *)"A-C-CB-B-D-DA-A", 8);
    OLED_ShowString(0, 2, (uint8_t *)"[5] Start", 8);
    OLED_ShowString(0, 3, (uint8_t *)"[9] Back to Menu", 8);
    OLED_ShowString(0, 5, (uint8_t *)"Hdg: MPU init+ofs", 8);
    OLED_ShowString(0, 6, (uint8_t *)"Arc: Trace", 8);
}

/* ---- Task1 慢速版 (WIT, 供 Task3 等复用) ---- */
void Task1_Slow_Init(void)
{
    tk_param    = &t3;
    tk_gyro_src = GYRO_SRC_WIT;
    task_init(task1_path, TASK1_SEG_COUNT);
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"T1 Slow (WIT)", 8);
    OLED_ShowString(0, 1, (uint8_t *)"A-B-BC-C-D-DA-A", 8);
    OLED_ShowString(0, 2, (uint8_t *)"[5] Start", 8);
    OLED_ShowString(0, 3, (uint8_t *)"[9] Back to Menu", 8);
    OLED_ShowString(0, 5, (uint8_t *)"Str: WIT Slow", 8);
    OLED_ShowString(0, 6, (uint8_t *)"Arc: Trace Slow", 8);
}
uint8_t Task1_Slow_Tick(uint8_t key) { return task_tick(key); }

/* ---- Task1 慢速版 (MPU6050, 供 Task3 等复用) ---- */
void Task1_Slow_MPU_Init(void)
{
    tk_param    = &t3;
    tk_gyro_src = GYRO_SRC_MPU;
    task_init(task1_path, TASK1_SEG_COUNT);
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"T1 Slow (MPU)", 8);
    OLED_ShowString(0, 1, (uint8_t *)"A-B-BC-C-D-DA-A", 8);
    OLED_ShowString(0, 2, (uint8_t *)"[5] Start", 8);
    OLED_ShowString(0, 3, (uint8_t *)"[9] Back to Menu", 8);
    OLED_ShowString(0, 5, (uint8_t *)"Str: MPU Slow", 8);
    OLED_ShowString(0, 6, (uint8_t *)"Arc: Trace Slow", 8);
}
uint8_t Task1_Slow_MPU_Tick(uint8_t key) { return task_tick(key); }

/* ---- Task2 慢速版 (WIT, 供 Task4 等复用) ---- */
void Task2_Slow_Init(void)
{
    tk_param    = &t3;
    tk_gyro_src = GYRO_SRC_WIT;
    task_init(task2_path, TASK2_SEG_COUNT);
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"T2 Slow (WIT)", 8);
    OLED_ShowString(0, 1, (uint8_t *)"A-C-CB-B-D-DA-A", 8);
    OLED_ShowString(0, 2, (uint8_t *)"[5] Start", 8);
    OLED_ShowString(0, 3, (uint8_t *)"[9] Back to Menu", 8);
    OLED_ShowString(0, 5, (uint8_t *)"Str: WIT Slow", 8);
    OLED_ShowString(0, 6, (uint8_t *)"Arc: Trace Slow", 8);
}
uint8_t Task2_Slow_Tick(uint8_t key) { return task_tick(key); }

/* ---- Task2 慢速版 (MPU6050, 供 Task4 等复用) ---- */
void Task2_Slow_MPU_Init(void)
{
    tk_param    = &t3;
    tk_gyro_src = GYRO_SRC_MPU;
    task_init(task2_path, TASK2_SEG_COUNT);
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"T2 Slow (MPU)", 8);
    OLED_ShowString(0, 1, (uint8_t *)"A-C-CB-B-D-DA-A", 8);
    OLED_ShowString(0, 2, (uint8_t *)"[5] Start", 8);
    OLED_ShowString(0, 3, (uint8_t *)"[9] Back to Menu", 8);
    OLED_ShowString(0, 5, (uint8_t *)"Str: MPU Slow", 8);
    OLED_ShowString(0, 6, (uint8_t *)"Arc: Trace Slow", 8);
}
uint8_t Task2_Slow_MPU_Tick(uint8_t key) { return task_tick(key); }
