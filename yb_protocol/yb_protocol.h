#ifndef _YB_PROTOCOL_H_
#define _YB_PROTOCOL_H_
#include "stdint.h"

#define PTO_BUF_LEN_MAX           (50)

#define PTO_ATAG_LEN_MAX          (32)

#define PTO_HEAD                  (0x24)
#define PTO_TAIL                  (0x23)

extern uint8_t RxBuffer[PTO_BUF_LEN_MAX];
/* 接收数据下标 */
extern uint8_t RxIndex;
/* 接收状态机 */
extern uint8_t RxFlag;
/* 新命令接收标志 */
extern uint8_t New_CMD_flag;
/* 新命令数据长度 */
extern uint8_t New_CMD_length;

extern uint8_t rx_temp;

void Pto_Data_Receive(uint8_t Rx_Temp);
void Pto_Data_Parse(uint8_t *data_buf, uint8_t num, int *recv_data);
void Pto_Clear_CMD_Flag(void);

#endif
