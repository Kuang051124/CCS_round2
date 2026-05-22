#include "trace.h"
GPIO_Regs *trace_ports[SENSOR_LEN] = {TRACE_PORT, TRACE_PORT, TRACE_PORT,
                                      TRACE_PORT, TRACE_PORT, TRACE_PORT,
                                      TRACE_PORT, TRACE_PORT};

uint32_t trace_pins[SENSOR_LEN] = {
    TRACE_LAMP_0_PIN, TRACE_LAMP_1_PIN, TRACE_LAMP_2_PIN, TRACE_LAMP_3_PIN,
    TRACE_LAMP_4_PIN, TRACE_LAMP_5_PIN, TRACE_LAMP_6_PIN, TRACE_LAMP_7_PIN};

uint8_t trace_state[8];
uint8_t bit;

// 循迹权重
int gravitys[SENSOR_LEN] = {-4, -3, -2, -1, 1, 2, 3, 4};
int trace_sum;

// 获取循迹状态
uint8_t Trace_Read_Bit() {
  bit = 0;
  for (uint8_t i = 0; i < SENSOR_LEN; ++i) {
    if (DL_GPIO_readPins(trace_ports[i], trace_pins[i])) {
      trace_state[i] = 1;
      bit |= 1 << (SENSOR_LEN - i - i);
    } else {
      trace_state[i] = 0;
    }
  }
  return bit;
}

// uint8_t Get_Trace_State(uint8_t trace_state[]){
//     uint8_t i = 0;
//     for (uint8_t i = 0; i < )
// }

// 获取加权和
int Get_Trace_Sum(uint8_t trace_state[]) {
  trace_sum = 0;
  uint8_t black_num = 0;
  for (uint8_t i = 0; i < SENSOR_LEN; ++i) {
    if (trace_state[i] == 0) {
      trace_sum += gravitys[i];
      ++black_num;
    }
  }
  if (black_num >= STOP_LEN) {
    return STOP;
  }
  return trace_sum;
}