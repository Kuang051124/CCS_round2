#include "bt_cmd_parser.h"
#include "ENCODER/speed_control.h"
#include "bluetooth.h"
#include <stdio.h>
#include <string.h>

/* 内部状态: 速度模式是否激活 */
static uint8_t s_speedModeActive = 0;

/*
 *  手动解析整数 (嵌入式环境避免依赖 sscanf/libc scanf)
 */
static int32_t parse_int(const char *s, const char **end)
{
    int32_t val = 0;
    int32_t sign = 1;
    if (*s == '-') { sign = -1; s++; }
    while (*s >= '0' && *s <= '9') {
        val = val * 10 + (int32_t)(*s - '0');
        s++;
    }
    if (end) *end = s;
    return sign * val;
}

/*
 *  手动解析浮点数
 */
static float parse_float(const char *s, const char **end)
{
    float val = 0.0f;
    float frac = 1.0f;
    int32_t sign = 1;

    if (*s == '-') { sign = -1; s++; }

    /* 整数部分 */
    while (*s >= '0' && *s <= '9') {
        val = val * 10.0f + (float)(*s - '0');
        s++;
    }

    /* 小数部分 */
    if (*s == '.') {
        s++;
        while (*s >= '0' && *s <= '9') {
            frac *= 0.1f;
            val += frac * (float)(*s - '0');
            s++;
        }
    }

    if (end) *end = s;
    return sign * val;
}

/*
 *  解析蓝牙指令帧 (手动解析, 不依赖 sscanf)。
 *
 *  返回:  1 = 速度模式激活
 *          0 = 参数调整 (保持当前模式)
 *         -1 = 停止
 */
int8_t BT_ParseCommand(const char *buf, uint8_t len)
{
    const char *p;

    /* ====== [leftspeed, L, rightspeed, R] ====== */
    if (strncmp(buf, "[leftspeed,", 11) == 0) {
        p = buf + 11;                       /* 跳过前缀 */
        int32_t L = parse_int(p, &p);       /* 解析左轮目标 */
        if (*p == ',') p++;                 /* 跳过 , (leftspeed,1000,rightspeed 中的第二个逗号) */
        /* 现在 p 指向 "rightspeed,..." 或已经是数字?
           格式是 [leftspeed,1000,rightspeed,1000]
           parse_int 停在 ',' → p 指向 ','
           下一个 , 跳过 → p 指向 'r' of "rightspeed"
           需要跳过 "rightspeed," */
        if (strncmp(p, "rightspeed,", 11) == 0) {
            p += 11;                        /* 跳过 "rightspeed," */
        }
        int32_t R = parse_int(p, NULL);     /* 解析右轮目标 */

        SPEED_SetTarget(L, R);
        s_speedModeActive = 1;
        return 1;
    }

    /* ====== [slider, Kp/Ki/Kd, value] ====== */
    if (strncmp(buf, "[slider,", 8) == 0) {
        p = buf + 8;                        /* 跳过 "[slider," */

        /* 提取参数类型 (到下一个逗号) */
        char key[8];
        uint8_t ki = 0;
        while (*p && *p != ',' && ki < sizeof(key) - 1) {
            key[ki++] = *p++;
        }
        key[ki] = '\0';
        if (*p == ',') p++;                 /* 跳过逗号 */

        /* 解析参数值 */
        float fval = parse_float(p, NULL);

        if (strncmp(key, "Kp", 2) == 0) {
            SPEED_SetKp(fval);
        } else if (strncmp(key, "Ki", 2) == 0) {
            SPEED_SetKi(fval);
        } else if (strncmp(key, "Kd", 2) == 0) {
            SPEED_SetKd(fval);
        }
        return 0;
    }

    /* ====== [plot-clear] ====== */
    if (strncmp(buf, "[plot-clear]", 12) == 0) {
        return 0;
    }

    /* ====== 原有指令透传 ====== */
    if (strncmp(buf, "STOP", 4) == 0) {
        SPEED_SetTarget(0, 0);
        s_speedModeActive = 0;
        return -1;  /* 不消费帧: 留给 task3.c 做 tk_abort() + 状态切换 */
    }
    if (strncmp(buf, "TASK3", 5) == 0) return -1;
    if (strncmp(buf, "TASK4", 5) == 0) return -1;

    return 0;
}

/*
 *  发送实时绘图数据
 */
void BT_SendPlot(int32_t tgtL, int32_t tgtR, int32_t actL, int32_t actR)
{
    char buf[64];
    sprintf(buf, "[plot,%d,%d,%d,%d]\n",
            (int)tgtL, (int)tgtR, (int)actL, (int)actR);
    Bluetooth_SendString(buf);
}

void BT_SendPlotClear(void)
{
    Bluetooth_SendString("[plot-clear]\n");
}
