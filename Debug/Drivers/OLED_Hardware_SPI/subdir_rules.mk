################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
Drivers/OLED_Hardware_SPI/%.o: ../Drivers/OLED_Hardware_SPI/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs2010/ccs/tools/compiler/ti-cgt-armllvm_4.0.3.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"D:/Program/chip/M0_Project/Target/Drivers/LSM6DSV16X" -I"D:/Program/chip/M0_Project/Target/Drivers/VL53L0X" -I"D:/Program/chip/M0_Project/Target/Drivers/WIT" -I"D:/Program/chip/M0_Project/Target/Drivers/BNO08X_UART_RVC" -I"D:/Program/chip/M0_Project/Target/Drivers/Ultrasonic_GPIO" -I"D:/Program/chip/M0_Project/Target/Drivers/Ultrasonic_Capture" -I"D:/Program/chip/M0_Project/Target/Drivers/OLED_Hardware_I2C" -I"D:/Program/chip/M0_Project/Target/Drivers/OLED_Hardware_SPI" -I"D:/Program/chip/M0_Project/Target/Drivers/OLED_Software_I2C" -I"D:/Program/chip/M0_Project/Target/Drivers/OLED_Software_SPI" -I"D:/Program/chip/M0_Project/Target/Drivers/MPU6050" -I"D:/Program/chip/M0_Project/Target/TB6612" -I"D:/Program/chip/M0_Project/Target/TB6600" -I"D:/Program/chip/M0_Project/Target/Pid" -I"D:/Program/chip/M0_Project/Target/yb_protocol" -I"D:/Program/chip/M0_Project/Target/Trace5" -I"D:/Program/chip/M0_Project/Target/Trace8" -I"D:/Program/chip/M0_Project/Target" -I"D:/Program/chip/M0_Project/Target/Debug" -I"C:/ti/mspm0_sdk_2_05_00_05/source/third_party/CMSIS/Core/Include" -I"C:/ti/mspm0_sdk_2_05_00_05/source" -I"D:/Program/chip/M0_Project/Target/Drivers/MSPM0" -I"D:/Program/chip/M0_Project/Target/Keyboard" -I"D:/Program/chip/M0_Project/Target/Slave_Control" -DMOTION_DRIVER_TARGET_MSPM0 -DMPU6050 -D__MSPM0G3507__ -gdwarf-3 -MMD -MP -MF"Drivers/OLED_Hardware_SPI/$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


