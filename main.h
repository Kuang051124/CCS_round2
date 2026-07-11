#ifndef _MAIN_H_
#define _MAIN_H_

#include "clock.h"
#include "interrupt.h"

#include "mpu6050.h"
#include "oled_software_i2c.h"
#include "oled_hardware_i2c.h"
#include "oled_software_spi.h"
#include "oled_hardware_spi.h"
#include "ultrasonic_capture.h"
#include "ultrasonic_gpio.h"
#include "bno08x_uart_rvc.h"
#include "wit.h"
#include "vl53l0x.h"
#include "lsm6dsv16x.h"
#include "ZDT_X42S/zdt_x42s.h"

extern ZDT_HandleTypeDef motor1;
extern ZDT_HandleTypeDef motor2;

#endif  /* #ifndef _MAIN_H_ */
