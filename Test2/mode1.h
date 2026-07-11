/**
 * @file  mode1.h
 * @brief 小车行驶控制 — 共享参数、API、路径定义
 *
 * Task 1: A→B(直)→BC弧(CW)→C→D(直)→DA弧(CW)→A      ≤30s
 * Task 2: A→C(直)→CB弧(CCW)→B→D(直)→DA弧(CW)→A     ≤40s
 *
 * 直线段: 陀螺仪航向保持 (P 控制)
 * 弧线段: 巡线比例公式 + 弧线偏置
 * 顶点检测: 巡线传感器 (无编码器)
 */

#ifndef _MODE1_H_
#define _MODE1_H_

#include "ti_msp_dl_config.h"

/* ===================================================================
 * 静态参数 (编译期常量)
 * =================================================================== */

#define T1_BLACK_THRESHOLD   1      /* 黑线判定: 至少N个传感器检测到黑色 (排除悬空全黑)   */
#define T1_STRAIGHT_BLIND_MS 800    /* 直线段盲区: 启动后N ms内忽略传感器 (防止误触发弧线) */
#define T1_ARC_END_FRAMES    10     /* 弧线结束判定: 连续N帧全白 + 角度变化达标           */
#define T1_ARC_YAW_DELTA     160.0f /* 弧线结束判定: 累积航向变化 > 此值(°) 才对弧线终点   */
#define T1_ARC_FADE_DEG      30.0f  /* 弧线偏置衰减角: Task2 大偏置→T1正常偏置 过渡角度 (°) */
#define T1_GYRO_STEER_MAX    0.40f  /* 陀螺仪转向限幅: 差速不超过 base_speed 的 N%      */
#define T1_GYRO_SAMPLES      20     /* 航向采样次数: 直线段启动时采样N次取平均              */
#define T1_GYRO_SAMPLE_MS    10     /* 航向采样间隔: 每次采样间隔N ms                      */
#define T1_BEEP_MS           1000   /* 途经顶点蜂鸣时长 (ms)                               */
#define T1_STOP_BEEP_MS      800    /* 任务完成蜂鸣时长 (ms)                               */
#define T1_MAX_TIME_SEC      100     /* 任务超时上限 (秒), Task1≤30s / Task2≤40s 共用此值   */

/* ===================================================================
 * 几何常量
 * =================================================================== */

/* ===================================================================
 * 直线段航向偏移 (所有路径统一命名, 方便修改)
 * =================================================================== */

/* arctan(0.8) ≈ 38.66° (根据实际情况调整) */
#define ATAN08                35.0f

/* ---- Task1: A→B直 → BC弧 → C→D直 → DA弧 ---- */
#define T1_HEADING_AB         0.0f                  /* A→B: 采样当前航向 */
#define T1_HEADING_CD         -178.0f               /* C→D: init+180° (实际-178微调) */

/* ---- Task2: A→C直 → CB弧 → B→D直 → DA弧 ---- */
#define T2_HEADING_AC         (-ATAN08)             /* A→C: init - atan0.8 (右转) */
#define T2_HEADING_BD         (-180.0f + ATAN08)    /* B→D: init + 180 - atan0.8 */

/* ---- B车 Task3: D→DA弧→A→C直→CB弧→B→D直 (从D出发) ---- */
#define TB_HEADING_AC         (-180.0f - ATAN08)    /* A→C: 经DA弧后, init-180-atan0.8 */
#define TB_HEADING_BD         (ATAN08)              /* B→D: 经CB弧后, init+atan0.8     */

/* ---- B车 Task3 / Task4 通用 ---- */
#define TB_HEADING_AB         -180.0f                  /* A→B: 采样当前航向 */

/* ---- Task4 新路径 (过中心O) ---- */
/* A车: A→O = T2_HEADING_AC, O→B = T2_HEADING_BD+180° */
#define TA_HEADING_AO         T2_HEADING_AC             /* A→O: -ATAN08                 */
#define TA_HEADING_OB         (T2_HEADING_BD + 180.0f)  /* O→B: +ATAN08                 */
/* B车: C→O = TB_HEADING_AC+180°, O→D = TB_HEADING_BD */
#define TB_HEADING_CO         (TB_HEADING_AC + 180.0f)  /* C→O: -ATAN08                 */

/* ===================================================================
 * 路径段类型
 * =================================================================== */

#define SEG_STRAIGHT       0    /* 直线: 陀螺仪保持, 巡线终点检测      */
#define SEG_ARC_CW         1    /* 顺时针弧: 巡线 + CW偏移            */
#define SEG_ARC_CCW        2    /* 逆时针弧: 巡线 + CCW偏移           */
#define SEG_STRAIGHT_DELAY 3    /* 直线: 陀螺仪保持, 延时终点 (Task4用) */

#define GYRO_SRC_WIT    0    /* WIT 串口陀螺仪 */
#define GYRO_SRC_MPU    1    /* MPU6050 I2C + DMP  */
extern uint8_t tk_gyro_src;

/* ===================================================================
 * 动态参数 (运行时变量, Page_Calib 可调)
 * 结构体封装防止链接器 COMMON 段合并导致变量地址重叠
 * =================================================================== */

typedef struct {
    float spd_straight;     /* 直线段基础速度      (默认 0.28) */
    float spd_arc;          /* 弧线段基础速度      (默认 0.22) */
    float spd_arc_kp;       /* 弧线段差速 P        (默认 0.05) */
    float gyro_kp;          /* 航向 P 增益         (默认 0.02) */
    float arc_ofs_cw;       /* T1 顺时针弧偏移     (默认 0.06) */
    float arc_ofs_ccw;      /* T1 逆时针弧偏移     (默认 -0.06) */
    float arc_ofs_cw_t2;    /* T2 顺时针弧偏移     (默认 0.10) */
    float arc_ofs_ccw_t2;   /* T2 逆时针弧偏移     (默认 -0.10) */
} T1Param;

extern T1Param t1;          /* Task1/2 参数 */
extern T1Param t3;          /* Task3 慢速参数 */
extern T1Param *tk_param;   /* 当前参数指针 */

/* 全局速度倍率: Car B 距离 P 控制用, 默认 1.0 */
extern float tk_speed_mult;

/* 超车航向偏置: 叠加到目标航向, 0=无偏置/正常行驶 */
extern float tk_heading_bias;

/* 直线段延时终点: >0=用延时(ms)替代巡线检测, 0=正常巡线 (Task4 delay段用) */
extern uint32_t tk_seg_delay_ms;

/* ===================================================================
 * 路径段描述
 * =================================================================== */

typedef struct {
    uint8_t     type;           /* SEG_STRAIGHT / SEG_ARC_CW / SEG_ARC_CCW */
    float       heading_ofs;    /* 直线段: 目标航向 = 初始航向 + offset(°),
                                   0=采样当前航向 */
    const char *name;           /* OLED 显示段名 */
    const char *target;         /* 目标点 */
} PathSeg;

/* ===================================================================
 * 路径定义 (在 mode1.c 中)
 * =================================================================== */

extern const PathSeg task1_path[];
#define TASK1_SEG_COUNT 4

extern const PathSeg task2_path[];
#define TASK2_SEG_COUNT 4

/* B车 Task3 路径: D→DA弧(CW)→A→C(直)→CB弧(CCW)→B→D(直)→DA弧(CW)→(追A,等STOP) */
extern const PathSeg task3b_path[];
#define TASK3B_SEG_COUNT 5

/* B车 Task4 路径: D→DA弧(CW)→A→B(直)→BC弧(CW)→C→O(直)→O→D(直) */
extern const PathSeg task4a_path[];
#define TASK4A_SEG_COUNT 5

extern const PathSeg task4b_path[];
#define TASK4B_SEG_COUNT 5

/* ===================================================================
 * 共享内部 API (Task3 复用)
 * =================================================================== */
void task_init(const PathSeg *path, uint8_t seg_count);
uint8_t task_tick(uint8_t key);
void tk_abort(void);   /* 强制停止任务 (B车用, 不阻塞键盘) */
uint8_t tk_is_done(void); /* 任务是否已完成 (TASK_DONE状态) */
uint8_t tk_get_seg_index(void);    /* 获取当前段下标           */
float   tk_get_initial_yaw(void);   /* 获取任务启动时的初始航向  */

/* ===================================================================
 * 参数持久化 (Flash 扇区 126)
 * =================================================================== */

void TKParam_Load(void);   /* 上电时调用, 从 Flash 恢复参数 */
void TKParam_Save(void);   /* 调参后调用, 写入 Flash 保存   */

/* ===================================================================
 * API
 * =================================================================== */

/* Task 1 (WIT 陀螺仪) */
void Task1_Init(void);
uint8_t Task1_Tick(uint8_t key);

/* Task 2 (WIT 陀螺仪) */
void Task2_Init(void);
uint8_t Task2_Tick(uint8_t key);

/* Task 1 (MPU6050 陀螺仪) */
void Task1_MPU_Init(void);
uint8_t Task1_MPU_Tick(uint8_t key);

/* Task 2 (MPU6050 陀螺仪) */
void Task2_MPU_Init(void);
uint8_t Task2_MPU_Tick(uint8_t key);

/* Task1 慢速版 (WIT 陀螺仪, 供 Task3 等复用) */
void Task1_Slow_Init(void);
uint8_t Task1_Slow_Tick(uint8_t key);

/* Task1 慢速版 (MPU6050 陀螺仪, 供 Task3 等复用) */
void Task1_Slow_MPU_Init(void);
uint8_t Task1_Slow_MPU_Tick(uint8_t key);

/* Task2 慢速版 (WIT 陀螺仪, 供 Task4 等复用) */
void Task2_Slow_Init(void);
uint8_t Task2_Slow_Tick(uint8_t key);

/* Task2 慢速版 (MPU6050 陀螺仪, 供 Task4 等复用) */
void Task2_Slow_MPU_Init(void);
uint8_t Task2_Slow_MPU_Tick(uint8_t key);

#endif /* _MODE1_H_ */
