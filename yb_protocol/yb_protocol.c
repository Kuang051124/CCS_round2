#include "yb_protocol.h"

#include "stdio.h"
#include "stdlib.h"
#include "string.h"


#define PTO_FUNC_ID        4


uint8_t RxBuffer[PTO_BUF_LEN_MAX];
/* 接收数据下标 */
uint8_t RxIndex = 0;
/* 接收状态机 */
uint8_t RxFlag = 0;
/* 新命令接收标志 */
uint8_t New_CMD_flag = 0;
/* 新命令数据长度 */
uint8_t New_CMD_length = 0;

uint8_t rx_temp = 0;


// 清除命令数据和相关标志
void Pto_Clear_CMD_Flag(void)
{

	New_CMD_length = 0;
	New_CMD_flag = 0;
}


// 接收数据进入协议缓存
void Pto_Data_Receive(uint8_t Rx_Temp)
{
	switch (RxFlag)
	{
	case 0:
		if (Rx_Temp == PTO_HEAD)
		{
			RxBuffer[0] = PTO_HEAD;
			RxFlag = 1;
            RxIndex = 1;
		}
		break;

	case 1:
		RxBuffer[RxIndex] = Rx_Temp;
		RxIndex++;
		if (Rx_Temp == PTO_TAIL)
		{
			New_CMD_flag = 1;
            New_CMD_length = RxIndex;
			RxFlag = 0;
			RxIndex = 0;
		}
        else if (RxIndex >= PTO_BUF_LEN_MAX)
        {
            New_CMD_flag = 0;
            New_CMD_length = 0;
			RxFlag = 0;
			RxIndex = 0;
            Pto_Clear_CMD_Flag();
        }
		break;

	default:
		break;
	}
}

// 将字符串数字转成数字。示例："12"->12
int Pto_Char_To_Int(char* data)
{
    return atoi(data);
}

/**
 * @Brief: 数据分析
 * @Note: 
 * @Parm: 传入接受到的一个数据帧和长度
 * @Retval: 
 */
void Pto_Data_Parse(uint8_t *data_buf, uint8_t num, int* recv_data)
{
    uint8_t pto_head = data_buf[0];
	uint8_t pto_tail = data_buf[num-1];
    if (!(pto_head == PTO_HEAD && pto_tail == PTO_TAIL))
    {
	    return;
    }
    uint8_t data_index = 1;
    uint8_t field_index[PTO_BUF_LEN_MAX] = {0};
    int i = 0;
    int values[PTO_BUF_LEN_MAX] = {0};
    for (i = 1; i < num-1; i++)
    {
        if (data_buf[i] == ',')
        {
            data_buf[i] = 0;
            field_index[data_index] = i;
            data_index++;
        }
    }
    
    for (i = 0; i < data_index; i++)
    {
        values[i] = Pto_Char_To_Int((char*)data_buf+field_index[i]+1);
    }
    
    uint8_t pto_len = values[0];
    
    if (pto_len != num)
    {
        return;
    }
    uint8_t pto_id = values[1];
    if (pto_id != PTO_FUNC_ID)
    {
        return;
    }
    recv_data[0] = values[2];  // x
    recv_data[1] = values[3];  // y
    recv_data[2] = values[4];  // w
    recv_data[3] = values[5];  // h
    recv_data[4] = values[6];  // id
    recv_data[5] = values[7];  // degrees
}
