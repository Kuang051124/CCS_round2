#include "trace.h"
GPIO_Regs *trace_ports[SENSOR_LEN] = {TRACE_PORT, TRACE_PORT, TRACE_PORT,
                                      TRACE_PORT, TRACE_PORT};

uint32_t trace_pins[SENSOR_LEN] = {TRACE_LAMP_0_PIN, TRACE_LAMP_1_PIN,
                                   TRACE_LAMP_2_PIN, TRACE_LAMP_3_PIN,
                                   TRACE_LAMP_4_PIN};

uint8_t trace_states[SENSOR_LEN];
uint8_t bit;

// 循迹权重
// 七路权重
// int gravitys[SENSOR_LEN] = {-3, -1, 0, 1, 3};

// 五路第一版权重
int gravitys[SENSOR_LEN] = {-4, -1, 0, 1, 4};

// 五路第二版权重
// int gravitys[SENSOR_LEN] = {-3, -1, 0, 1, 3};
int trace_sum;

// 获取循迹状态
// 黑色是 1， 白色是 0
uint8_t Trace_Read_Bit() {
  bit = 0;
  for (uint8_t i = 0; i < SENSOR_LEN; ++i) {
    if (DL_GPIO_readPins(trace_ports[i], trace_pins[i])) {
      trace_states[i] = 1;
      bit |= 1 << (SENSOR_LEN - i - i);
    } else {
      trace_states[i] = 0;
    }
  }
  return bit;
}

// uint8_t Get_Trace_State(uint8_t trace_state[]){
//     uint8_t i = 0;
//     for (uint8_t i = 0; i < )
// }

// 获取加权和
int Get_Trace_Sum(uint8_t trace_states[]) {
  trace_sum = 0;
  uint8_t black_num = 0;
  for (uint8_t i = 0; i < SENSOR_LEN; ++i) {
    if (trace_states[i] == 1) {
      trace_sum += gravitys[i];
      ++black_num;
    }
  }
  if (black_num == SENSOR_LEN) {
    return STOP;
  }
  // 直角转弯
  if (trace_states[0] == BLACK && trace_states[1] == BLACK) {
    return TURN_LEFT;
  }
  // 返回正常加权和
  return trace_sum;
}