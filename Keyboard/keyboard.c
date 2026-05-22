#include "keyboard.h"
#include "clock.h"

volatile uint8_t KeyboardData[KBD_ROW_NUM][KBD_COL_NUM] = {1, 2, 3, 4, 5, 6, 7, 8, 9};

GPIO_Regs *row_ports[KBD_ROW_NUM] = {KBD_GRP_R1_PORT, KBD_GRP_R2_PORT,
                                     KBD_GRP_R3_PORT};

GPIO_Regs *col_ports[KBD_COL_NUM] = {KBD_GRP_C1_PORT, KBD_GRP_C2_PORT,
                                     KBD_GRP_C3_PORT};

volatile uint32_t row_pins[KBD_ROW_NUM] = {KBD_GRP_R1_PIN, KBD_GRP_R2_PIN,
                                           KBD_GRP_R3_PIN};

volatile uint32_t col_pins[KBD_COL_NUM] = {KBD_GRP_C1_PIN, KBD_GRP_C2_PIN,
                                           KBD_GRP_C3_PIN};

void Keyboard_Init() {
  for (int i = 0; i < KBD_ROW_NUM; ++i) {
    DL_GPIO_setPins(row_ports[i], row_pins[i]);
  }
}

uint8_t Scan_Keyboard() {
  uint8_t key = 0, row = 0, col = 0;
  for (row = 0; row < KBD_ROW_NUM; ++row) {
    DL_GPIO_clearPins(row_ports[row], row_pins[row]);
    for (col = 0; col < KBD_COL_NUM; ++col) {
      if (!DL_GPIO_readPins(col_ports[col], col_pins[col])) {
        mspm0_delay_ms(10);
        if (!DL_GPIO_readPins(col_ports[col], col_pins[col])) {
          key = KeyboardData[row][col];
        }
      }
    }
    DL_GPIO_setPins(row_ports[row], row_pins[row]);
  }
  return key;
}