#include "yb_protocol.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

/* ========== 全局变量 ========== */
uint8_t RxBuffer[PTO_BUF_LEN_MAX];
uint8_t RxIndex = 0;
uint8_t RxFlag = 0;
uint8_t New_CMD_flag = 0;
uint8_t New_CMD_length = 0;
uint8_t rx_temp = 0;
uint16_t Pto_Raw_Byte_Count = 0;   /* ISR 收到的原始字节总数 */

/* ==================================================================
 * 基础接收
 * ================================================================== */

/** @brief 逐字节接收状态机，在 UART 中断中调用 */
void Pto_Receive_Byte(uint8_t byte)
{
    Pto_Raw_Byte_Count++;  /* 诊断计数 */
    switch (RxFlag) {
    case 0:  /* 等待帧头 '$' */
        if (byte == PTO_HEAD) {
            RxBuffer[0] = PTO_HEAD;
            RxFlag = 1;
            RxIndex = 1;
        }
        break;

    case 1:  /* 接收数据直到帧尾 '#' */
        RxBuffer[RxIndex] = byte;
        RxIndex++;
        if (byte == PTO_TAIL) {
            New_CMD_flag   = 1;
            New_CMD_length = RxIndex;
            RxFlag  = 0;
            RxIndex = 0;
        } else if (RxIndex >= PTO_BUF_LEN_MAX) {
            /* 缓冲区溢出: 丢弃本帧 */
            New_CMD_flag   = 0;
            New_CMD_length = 0;
            RxFlag  = 0;
            RxIndex = 0;
        }
        break;

    default:
        break;
    }
}

/** @brief 清除帧就绪标志 */
void Pto_Clear_Flag(void)
{
    New_CMD_flag   = 0;
    New_CMD_length = 0;
}

/* ==================================================================
 * 内部辅助
 * ================================================================== */

/** @brief 将逗号分隔的帧字符串转为整数数组 */
static int Pto_Char_To_Int(char *data)
{
    return atoi(data);
}

/**
 * @brief 扫描逗号并提取所有整数字段
 * @param buf       原始帧缓冲区 (会被修改: 逗号 → '\0')
 * @param len       帧长度
 * @param values    输出: 整数字段数组
 * @param pto_len   输出: 帧长度字段值
 * @param pto_id    输出: 协议ID字段值
 * @return 提取到的字段个数; 0=帧头尾校验失败
 */
static uint8_t Pto_Extract_Fields(uint8_t *buf, uint8_t len,
                                   int *values, uint8_t *pto_len, uint8_t *pto_id)
{
    /* 校验帧头尾 */
    if (buf[0] != PTO_HEAD || buf[len - 1] != PTO_TAIL) {
        return 0;
    }

    uint8_t field_index[PTO_BUF_LEN_MAX] = {0};
    uint8_t data_index = 1;
    int i;

    /* 扫描逗号位置并替换为 '\0' */
    for (i = 1; i < len - 1; i++) {
        if (buf[i] == ',') {
            buf[i] = 0;
            field_index[data_index] = i;
            data_index++;
        }
    }

    /* 逐个字段转整数 */
    for (i = 0; i < data_index; i++) {
        values[i] = Pto_Char_To_Int((char *)buf + field_index[i] + 1);
    }

    *pto_len = (uint8_t)values[0];
    *pto_id  = (uint8_t)values[1];

    return data_index;
}

/* ==================================================================
 * 统一帧解析
 * ================================================================== */

/**
 * @brief 解析任意类型帧
 *
 * 帧格式 (通讯.txt):
 *   $31,04,<data1>,<data2>,<type>,<data3>,<data4>,<data5>#
 *
 *   type=0 目标位置:  data1=x, data2=y, data3/4/5=000
 *   type=1 激光位置:  data1=x, data2=y, data3/4/5=000
 *   type=2 数字识别:  data1=x, data2=y, data3=数字值, data4/5=000
 *   type=3 颜色形状:  data1=x, data2=y, data3=颜色, data4=形状, data5=000
 *   type=4 终点目标:  data1=x, data2=y, data3/4/5=000
 *   type=5 AprilTag:  data1=距离低3位(DDD), data2=距离千位(HHH), data3=Tag编号(III),
 *                       data4=偏航角×10(AAA), data5=方向(SSS)
 */
int8_t Pto_Parse_Frame(uint8_t *buf, uint8_t len, Pto_Frame_t *frame)
{
    int values[PTO_BUF_LEN_MAX] = {0};
    uint8_t pto_len, pto_id;

    uint8_t field_count = Pto_Extract_Fields(buf, len, values, &pto_len, &pto_id);
    if (field_count == 0) {
        return -1;   /* 帧头尾 '$' / '#' 校验失败 */
    }
    /* 注意: 新协议中第一个字段(如31)是固定占位符, 不代表实际帧长, 不做长度校验 */
    if (pto_id != PTO_FUNC_ID) {
        return -2;   /* 协议ID校验失败 (不是4) */
    }

    uint8_t type = (uint8_t)values[4];
    frame->type = type;

    switch (type) {

    case PTO_TYPE_TARGET:    /* 0: 目标位置 */
    case PTO_TYPE_LASER:     /* 1: 激光位置 */
    case PTO_TYPE_ENDPOINT:  /* 4: 终点目标 */
        frame->data.position.x = (int16_t)values[2];
        frame->data.position.y = (int16_t)values[3];
        break;

    case PTO_TYPE_NUMBER:    /* 2: 数字识别 */
        frame->data.number.x     = (int16_t)values[2];
        frame->data.number.y     = (int16_t)values[3];
        frame->data.number.value = (int16_t)values[5];
        break;

    case PTO_TYPE_COLOR_SHAPE:  /* 3: 颜色形状 */
        frame->data.shape.x     = (int16_t)values[2];
        frame->data.shape.y     = (int16_t)values[3];
        frame->data.shape.color = (uint8_t)values[6];
        frame->data.shape.shape = (uint8_t)values[7];
        break;

    case PTO_TYPE_APRILTAG:    /* 5: AprilTag */
        /* DDD + HHH×1000 = 距离mm (最大9999) */
        frame->data.apriltag.distance_mm = (uint16_t)(values[2] + values[3] * 1000);
        frame->data.apriltag.tag_id      = (uint8_t)values[5];   /* III */
        frame->data.apriltag.yaw_raw     = (int16_t)values[6];   /* AAA, ×10 */
        frame->data.apriltag.direction   = (uint8_t)values[7];   /* SSS */
        break;

    default:
        /* 未知类型: 按位置类型兜底 */
        frame->data.position.x = (int16_t)values[2];
        frame->data.position.y = (int16_t)values[3];
        break;
    }

    return 0;
}

/* ==================================================================
 * 工具/转换函数
 * ================================================================== */

/** @brief 帧类型 → 可读名称 */
const char* Pto_Frame_Type_Name(uint8_t type)
{
    switch (type) {
    case PTO_TYPE_TARGET:      return "目标位置";
    case PTO_TYPE_LASER:       return "激光位置";
    case PTO_TYPE_NUMBER:      return "数字";
    case PTO_TYPE_COLOR_SHAPE: return "颜色形状";
    case PTO_TYPE_ENDPOINT:    return "终点";
    case PTO_TYPE_APRILTAG:    return "AprilTag";
    default:                   return "未知";
    }
}

/** @brief 颜色编码 → 名称 */
const char* Pto_Color_Name(uint8_t code)
{
    switch (code) {
    case PTO_COLOR_RED:    return "红";
    case PTO_COLOR_YELLOW: return "黄";
    case PTO_COLOR_GREEN:  return "绿";
    case PTO_COLOR_BLUE:   return "蓝";
    default:               return "?";
    }
}

/** @brief 形状编码 → 名称 */
const char* Pto_Shape_Name(uint8_t code)
{
    switch (code) {
    case PTO_SHAPE_TRIANGLE: return "三角形";
    case PTO_SHAPE_SQUARE:   return "方形";
    case PTO_SHAPE_CIRCLE:   return "圆形";
    default:                 return "?";
    }
}

/** @brief AprilTag 偏航角原始值(AAA, ×10) → 带符号度数 */
float Pto_Apriltag_To_Degrees(int16_t yaw_raw, uint8_t direction)
{
    float deg = (float)yaw_raw / 10.0f;   /* 015 → 1.5° */
    if (direction == PTO_APRILTAG_DIR_LEFT) {
        deg = -deg;
    }
    return deg;
}

/* ==================================================================
 * 以下为旧版 API (向后兼容, 新代码请用 Pto_Receive_Byte / Pto_Parse_Frame)
 * ================================================================== */

void Pto_Data_Receive(uint8_t Rx_Temp)
{
    Pto_Receive_Byte(Rx_Temp);
}

void Pto_Data_Parse(uint8_t *data_buf, uint8_t num, int *recv_data)
{
    Pto_Frame_t frame;
    int8_t ret = Pto_Parse_Frame(data_buf, num, &frame);
    if (ret < 0) return;

    /* 映射到旧格式 int 数组:
     * recv_data[0]=x, [1]=y, [2]=w(=type), [3]=h, [4]=id, [5]=degrees */
    recv_data[0] = frame.data.position.x;
    recv_data[1] = frame.data.position.y;

    switch (frame.type) {
    case PTO_TYPE_NUMBER:
        recv_data[2] = frame.type;
        recv_data[3] = frame.data.number.value;
        recv_data[4] = 0;
        recv_data[5] = 0;
        break;
    case PTO_TYPE_COLOR_SHAPE:
        recv_data[2] = frame.type;
        recv_data[3] = frame.data.shape.color;
        recv_data[4] = frame.data.shape.shape;
        recv_data[5] = 0;
        break;
    case PTO_TYPE_APRILTAG:
        recv_data[2] = frame.type;
        recv_data[3] = frame.data.apriltag.distance_mm;
        recv_data[4] = frame.data.apriltag.yaw_raw;
        recv_data[5] = frame.data.apriltag.direction;
        break;
    default:
        recv_data[2] = frame.type;
        recv_data[3] = 0;
        recv_data[4] = 0;
        recv_data[5] = 0;
        break;
    }
}

void Pto_Clear_CMD_Flag(void)
{
    Pto_Clear_Flag();
}
