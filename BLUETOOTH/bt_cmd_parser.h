#ifndef BLUETOOTH_BT_CMD_PARSER_H_
#define BLUETOOTH_BT_CMD_PARSER_H_

#include "ti_msp_dl_config.h"

/**
 * @brief  解析蓝牙接收到的指令帧, 直接调用对应功能函数
 *
 *         支持格式 (以 \n 结尾):
 *           [leftspeed, L, rightspeed, R] → SPEED_SetTarget(L,R)
 *           [slider, Kp, val]            → SPEED_SetKp(val)
 *           [slider, Ki, val]            → SPEED_SetKi(val)
 *           [slider, Kd, val]            → SPEED_SetKd(val)
 *           [plot-clear]                 → (内部处理)
 *
 * @param  buf   已以 '\0' 结尾的帧缓冲区
 * @param  len   帧长度
 * @return 1=速度模式激活, 0=非速度指令, -1=停止
 */
int8_t BT_ParseCommand(const char *buf, uint8_t len);

/**
 * @brief  发送实时绘图数据到小程序
 *         [plot, 左目标, 右目标, 左实际, 右实际]\n
 *
 * @param  tgtL  左轮目标速度 (counts/s)
 * @param  tgtR  右轮目标速度 (counts/s)
 * @param  actL  左轮实际速度 (counts/s)
 * @param  actR  右轮实际速度 (counts/s)
 */
void BT_SendPlot(int32_t tgtL, int32_t tgtR, int32_t actL, int32_t actR);

/**
 * @brief  发送清空绘图区指令 [plot-clear]\n
 */
void BT_SendPlotClear(void);

#endif /* BLUETOOTH_BT_CMD_PARSER_H_ */
