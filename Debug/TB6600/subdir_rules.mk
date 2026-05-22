################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
TB6600/%.o: ../TB6600/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"D:/Ti/CCS/install/ccs/tools/compiler/ti-cgt-armllvm_4.0.3.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"D:/Ti/chip/M0_Project/Target copy/Drivers/LSM6DSV16X" -I"D:/Ti/chip/M0_Project/Target copy/Drivers/VL53L0X" -I"D:/Ti/chip/M0_Project/Target copy/Drivers/WIT" -I"D:/Ti/chip/M0_Project/Target copy/Drivers/BNO08X_UART_RVC" -I"D:/Ti/chip/M0_Project/Target copy/Drivers/Ultrasonic_GPIO" -I"D:/Ti/chip/M0_Project/Target copy/Drivers/Ultrasonic_Capture" -I"D:/Ti/chip/M0_Project/Target copy/Drivers/OLED_Hardware_I2C" -I"D:/Ti/chip/M0_Project/Target copy/Drivers/OLED_Hardware_SPI" -I"D:/Ti/chip/M0_Project/Target copy/Drivers/OLED_Software_I2C" -I"D:/Ti/chip/M0_Project/Target copy/Drivers/OLED_Software_SPI" -I"D:/Ti/chip/M0_Project/Target copy/Drivers/MPU6050" -I"D:/Ti/chip/M0_Project/Target copy/TB6612" -I"D:/Ti/chip/M0_Project/Target copy/TB6600" -I"D:/Ti/chip/M0_Project/Target copy/Pid" -I"D:/Ti/chip/M0_Project/Target copy/yb_protocol" -I"D:/Ti/chip/M0_Project/Target copy/Trace5" -I"D:/Ti/chip/M0_Project/Target copy/Trace8" -I"D:/Ti/chip/M0_Project/Target copy" -I"D:/Ti/chip/M0_Project/Target copy/Debug" -I"D:/Ti/CCS/install/mspm0_sdk_2_05_00_05/source/third_party/CMSIS/Core/Include" -I"D:/Ti/CCS/install/mspm0_sdk_2_05_00_05/source" -I"D:/Ti/chip/M0_Project/Target copy/Drivers/MSPM0" -I"D:/Ti/chip/M0_Project/Target copy/Keyboard" -I"D:/Ti/chip/M0_Project/Target copy/Slave_Control" -DMOTION_DRIVER_TARGET_MSPM0 -DMPU6050 -D__MSPM0G3507__ -gdwarf-3 -MMD -MP -MF"TB6600/$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


