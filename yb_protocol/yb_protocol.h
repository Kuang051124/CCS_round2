#ifndef _YB_PROTOCOL_H_
#define _YB_PROTOCOL_H_
#include "stdint.h"

/* ========== 缓冲区与帧边界 ========== */
#define PTO_BUF_LEN_MAX           (50)
#define PTO_ATAG_LEN_MAX          (32)
#define PTO_HEAD                  (0x24)   /* '$' */
#define PTO_TAIL                  (0x23)   /* '#' */
#define PTO_FUNC_ID               (4)      /* 协议功能ID */

/* ========== 帧类型 (第5字段) ========== */
#define PTO_TYPE_TARGET           (0)      /* 目标位置             */
#define PTO_TYPE_LASER            (1)      /* 激光位置             */
#define PTO_TYPE_NUMBER           (2)      /* 数字识别             */
#define PTO_TYPE_COLOR_SHAPE      (3)      /* 颜色形状识别          */
#define PTO_TYPE_ENDPOINT         (4)      /* 终点目标             */
#define PTO_TYPE_APRILTAG         (5)      /* AprilTag 距离+角度   */

/* ========== 颜色编码 ========== */
#define PTO_COLOR_RED             (1)      /* 红色   */
#define PTO_COLOR_YELLOW          (2)      /* 黄色   */
#define PTO_COLOR_GREEN           (3)      /* 绿色   */
#define PTO_COLOR_BLUE            (4)      /* 蓝色   */

/* ========== 形状编码 ========== */
#define PTO_SHAPE_TRIANGLE        (1)      /* 三角形  */
#define PTO_SHAPE_SQUARE          (2)      /* 方形   */
#define PTO_SHAPE_CIRCLE          (3)      /* 圆形   */

/* ========== AprilTag 角度方向 ========== */
#define PTO_APRILTAG_DIR_RIGHT    (0)      /* 偏右 */
#define PTO_APRILTAG_DIR_LEFT     (1)      /* 偏左 */

/* ========== 全局状态变量 (接收用) ========== */
extern uint8_t RxBuffer[PTO_BUF_LEN_MAX];  /* 接收缓冲区      */
extern uint8_t RxIndex;                    /* 缓冲区写入下标   */
extern uint8_t RxFlag;                     /* 接收状态机状态   */
extern uint8_t New_CMD_flag;               /* 新帧就绪标志    */
extern uint8_t New_CMD_length;             /* 新帧数据长度    */
extern uint8_t rx_temp;
extern uint16_t Pto_Raw_Byte_Count;     /* ISR 收到的原始字节总数 (诊断用) */                    /* 临时接收字节    */

/* ==================================================================
 * 帧数据结构 (按类型使用不同子结构)
 * ================================================================== */

/** @brief 目标/激光/终点位置 (type=0,1,4) */
typedef struct {
    int16_t x;              /* X 坐标 (像素) */
    int16_t y;              /* Y 坐标 (像素) */
} Pto_Position_t;

/** @brief 数字识别 (type=2) */
typedef struct {
    int16_t x;              /* X 坐标 (像素) */
    int16_t y;              /* Y 坐标 (像素) */
    int16_t value;          /* 识别的数字值  */
} Pto_Number_t;

/** @brief 颜色形状识别 (type=3) */
typedef struct {
    int16_t x;              /* X 坐标 (像素) */
    int16_t y;              /* Y 坐标 (像素) */
    uint8_t color;          /* 颜色: 1=红,2=黄,3=绿,4=蓝 */
    uint8_t shape;          /* 形状: 1=三角形,2=方形,3=圆形 */
} Pto_ColorShape_t;

/** @brief AprilTag 距离与角度 (type=5)
 *  @note 帧格式: $31,04,DDD,HHH,5,III,AAA,SSS#
 *         DDD=距离mm低3位, HHH=距离mm千位(最大9999), III=Tag编号
 *         AAA=偏航角×10(如015=1.5°), SSS=方向(0=右 1=左)
 */
typedef struct {
    uint16_t distance_mm;   /* 距离 (mm) = DDD + HHH * 1000  */
    uint8_t  tag_id;        /* Tag 编号 (III)                 */
    int16_t  yaw_raw;       /* 偏航角原始值 (AAA, ×10)         */
    uint8_t  direction;     /* 偏航方向: 0=偏右, 1=偏左 (SSS)  */
} Pto_Apriltag_t;

/** @brief 统一帧数据 (解析后根据 type 读取对应子结构) */
typedef struct {
    uint8_t type;           /* 帧类型: 0~5 */
    union {
        Pto_Position_t  position;   /* type=0,1,4 时有效 */
        Pto_Number_t    number;     /* type=2 时有效     */
        Pto_ColorShape_t shape;     /* type=3 时有效     */
        Pto_Apriltag_t  apriltag;   /* type=5 时有效     */
    } data;
} Pto_Frame_t;

/* ==================================================================
 * 基础接收 API
 * ================================================================== */

/**
 * @brief 逐字节接收状态机，在 UART 中断中调用
 * @param byte  收到的单字节
 */
void Pto_Receive_Byte(uint8_t byte);

/**
 * @brief 清除帧就绪标志 (处理完一帧后调用)
 */
void Pto_Clear_Flag(void);

/* ==================================================================
 * 帧解析 API (按 通讯.txt 格式)
 * ================================================================== */

/**
 * @brief 解析任意类型帧，自动识别 type 并填充对应子结构
 *
 * 帧格式统一为: $<len>,4,<data1>,<data2>,<type>,<data3>,<data4>,<data5>#
 *
 * 各类型字段映射:
 *   type=0 目标位置:   data1=x,  data2=y
 *   type=1 激光位置:   data1=x,  data2=y
 *   type=2 数字识别:   data1=x,  data2=y,  data3=数字值
 *   type=3 颜色形状:   data1=x,  data2=y,  data3=颜色, data4=形状
 *   type=4 终点目标:   data1=x,  data2=y
 *   type=5 AprilTag:   data1=距离低3位(DDD), data2=距离千位(HHH), data3=Tag编号(III),
 *                       data4=偏航角×10(AAA), data5=方向(SSS)
 *
 * @param buf   接收缓冲区 (通常传 RxBuffer)
 * @param len   帧长度     (通常传 New_CMD_length)
 * @param frame 输出: 解析后的帧数据 (frame.type 标识类型)
 * @retval  0=成功
 *         -1=帧头 '$' 或帧尾 '#' 校验失败
 *         -2=帧长度校验失败
 *         -3=协议ID校验失败 (不是4)
 */
int8_t Pto_Parse_Frame(uint8_t *buf, uint8_t len, Pto_Frame_t *frame);

/* ==================================================================
 * 工具/转换函数
 * ================================================================== */

/**
 * @brief 获取帧类型的可读名称
 * @param type  帧类型 (0~5)
 * @return 中文字符串: "目标位置"/"激光位置"/"数字"/"颜色形状"/"终点"/"AprilTag"
 */
const char* Pto_Frame_Type_Name(uint8_t type);

/**
 * @brief 颜色编码转可读名称
 * @param code  颜色编码 (1~4)
 * @return "红"/"黄"/"绿"/"蓝" 或 "未知"
 */
const char* Pto_Color_Name(uint8_t code);

/**
 * @brief 形状编码转可读名称
 * @param code  形状编码 (1~3)
 * @return "三角形"/"方形"/"圆形" 或 "未知"
 */
const char* Pto_Shape_Name(uint8_t code);

/**
 * @brief AprilTag 偏航角原始值 → 带符号的实际度数
 * @param yaw_raw    偏航角原始值 (AAA, ×10, 如 015 → 1.5°)
 * @param direction  方向 (PTO_APRILTAG_DIR_RIGHT=偏右为正, LEFT=偏左为负)
 * @return 带符号的实际角度, 如 +1.5 或 -1.5
 */
float Pto_Apriltag_To_Degrees(int16_t yaw_raw, uint8_t direction);

/* ==================================================================
 * 以下为旧版 API, 保留向后兼容, 新代码请使用 Pto_Parse_Frame()
 * ================================================================== */
void Pto_Data_Receive(uint8_t Rx_Temp);
void Pto_Data_Parse(uint8_t *data_buf, uint8_t num, int *recv_data);
void Pto_Clear_CMD_Flag(void);

#endif  /* _YB_PROTOCOL_H_ */
