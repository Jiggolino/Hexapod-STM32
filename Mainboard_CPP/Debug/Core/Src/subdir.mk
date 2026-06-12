################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/VL53L1X_api.cpp \
../Core/Src/VL53L1X_calibration.cpp \
../Core/Src/app_main.cpp \
../Core/Src/battery.cpp \
../Core/Src/hexapod_ik.cpp \
../Core/Src/i2c.cpp \
../Core/Src/imu.cpp \
../Core/Src/loko_gait.cpp \
../Core/Src/loko_input.cpp \
../Core/Src/loko_legs.cpp \
../Core/Src/loko_servo.cpp \
../Core/Src/loko_stabilizer.cpp \
../Core/Src/loko_states.cpp \
../Core/Src/loko_tilt.cpp \
../Core/Src/loko_transitions.cpp \
../Core/Src/lokomotion.cpp \
../Core/Src/lsm6dso_reg.cpp \
../Core/Src/pca9685.cpp \
../Core/Src/tof.cpp \
../Core/Src/trajectory_calculator.cpp \
../Core/Src/uart_dma_tx.cpp \
../Core/Src/uart_protocol.cpp \
../Core/Src/vl53l1_platform.cpp \
../Core/Src/ws2812b.cpp 

C_SRCS += \
../Core/Src/adc.c \
../Core/Src/dma.c \
../Core/Src/gpio.c \
../Core/Src/i2c_msp.c \
../Core/Src/main.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h7xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c 

C_DEPS += \
./Core/Src/adc.d \
./Core/Src/dma.d \
./Core/Src/gpio.d \
./Core/Src/i2c_msp.d \
./Core/Src/main.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h7xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d 

OBJS += \
./Core/Src/VL53L1X_api.o \
./Core/Src/VL53L1X_calibration.o \
./Core/Src/adc.o \
./Core/Src/app_main.o \
./Core/Src/battery.o \
./Core/Src/dma.o \
./Core/Src/gpio.o \
./Core/Src/hexapod_ik.o \
./Core/Src/i2c.o \
./Core/Src/i2c_msp.o \
./Core/Src/imu.o \
./Core/Src/loko_gait.o \
./Core/Src/loko_input.o \
./Core/Src/loko_legs.o \
./Core/Src/loko_servo.o \
./Core/Src/loko_stabilizer.o \
./Core/Src/loko_states.o \
./Core/Src/loko_tilt.o \
./Core/Src/loko_transitions.o \
./Core/Src/lokomotion.o \
./Core/Src/lsm6dso_reg.o \
./Core/Src/main.o \
./Core/Src/pca9685.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h7xx.o \
./Core/Src/tim.o \
./Core/Src/tof.o \
./Core/Src/trajectory_calculator.o \
./Core/Src/uart_dma_tx.o \
./Core/Src/uart_protocol.o \
./Core/Src/usart.o \
./Core/Src/vl53l1_platform.o \
./Core/Src/ws2812b.o 

CPP_DEPS += \
./Core/Src/VL53L1X_api.d \
./Core/Src/VL53L1X_calibration.d \
./Core/Src/app_main.d \
./Core/Src/battery.d \
./Core/Src/hexapod_ik.d \
./Core/Src/i2c.d \
./Core/Src/imu.d \
./Core/Src/loko_gait.d \
./Core/Src/loko_input.d \
./Core/Src/loko_legs.d \
./Core/Src/loko_servo.d \
./Core/Src/loko_stabilizer.d \
./Core/Src/loko_states.d \
./Core/Src/loko_tilt.d \
./Core/Src/loko_transitions.d \
./Core/Src/lokomotion.d \
./Core/Src/lsm6dso_reg.d \
./Core/Src/pca9685.d \
./Core/Src/tof.d \
./Core/Src/trajectory_calculator.d \
./Core/Src/uart_dma_tx.d \
./Core/Src/uart_protocol.d \
./Core/Src/vl53l1_platform.d \
./Core/Src/ws2812b.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.cpp Core/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Drivers/BSP/Components/lsm6dso16is -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Drivers/BSP/Components/lsm6dso16is -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/VL53L1X_api.cyclo ./Core/Src/VL53L1X_api.d ./Core/Src/VL53L1X_api.o ./Core/Src/VL53L1X_api.su ./Core/Src/VL53L1X_calibration.cyclo ./Core/Src/VL53L1X_calibration.d ./Core/Src/VL53L1X_calibration.o ./Core/Src/VL53L1X_calibration.su ./Core/Src/adc.cyclo ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/adc.su ./Core/Src/app_main.cyclo ./Core/Src/app_main.d ./Core/Src/app_main.o ./Core/Src/app_main.su ./Core/Src/battery.cyclo ./Core/Src/battery.d ./Core/Src/battery.o ./Core/Src/battery.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/hexapod_ik.cyclo ./Core/Src/hexapod_ik.d ./Core/Src/hexapod_ik.o ./Core/Src/hexapod_ik.su ./Core/Src/i2c.cyclo ./Core/Src/i2c.d ./Core/Src/i2c.o ./Core/Src/i2c.su ./Core/Src/i2c_msp.cyclo ./Core/Src/i2c_msp.d ./Core/Src/i2c_msp.o ./Core/Src/i2c_msp.su ./Core/Src/imu.cyclo ./Core/Src/imu.d ./Core/Src/imu.o ./Core/Src/imu.su ./Core/Src/loko_gait.cyclo ./Core/Src/loko_gait.d ./Core/Src/loko_gait.o ./Core/Src/loko_gait.su ./Core/Src/loko_input.cyclo ./Core/Src/loko_input.d ./Core/Src/loko_input.o ./Core/Src/loko_input.su ./Core/Src/loko_legs.cyclo ./Core/Src/loko_legs.d ./Core/Src/loko_legs.o ./Core/Src/loko_legs.su ./Core/Src/loko_servo.cyclo ./Core/Src/loko_servo.d ./Core/Src/loko_servo.o ./Core/Src/loko_servo.su ./Core/Src/loko_stabilizer.cyclo ./Core/Src/loko_stabilizer.d ./Core/Src/loko_stabilizer.o ./Core/Src/loko_stabilizer.su ./Core/Src/loko_states.cyclo ./Core/Src/loko_states.d ./Core/Src/loko_states.o ./Core/Src/loko_states.su ./Core/Src/loko_tilt.cyclo ./Core/Src/loko_tilt.d ./Core/Src/loko_tilt.o ./Core/Src/loko_tilt.su ./Core/Src/loko_transitions.cyclo ./Core/Src/loko_transitions.d ./Core/Src/loko_transitions.o ./Core/Src/loko_transitions.su ./Core/Src/lokomotion.cyclo ./Core/Src/lokomotion.d ./Core/Src/lokomotion.o ./Core/Src/lokomotion.su ./Core/Src/lsm6dso_reg.cyclo ./Core/Src/lsm6dso_reg.d ./Core/Src/lsm6dso_reg.o ./Core/Src/lsm6dso_reg.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/pca9685.cyclo ./Core/Src/pca9685.d ./Core/Src/pca9685.o ./Core/Src/pca9685.su ./Core/Src/stm32h7xx_hal_msp.cyclo ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_hal_msp.su ./Core/Src/stm32h7xx_it.cyclo ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h7xx.cyclo ./Core/Src/system_stm32h7xx.d ./Core/Src/system_stm32h7xx.o ./Core/Src/system_stm32h7xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/tof.cyclo ./Core/Src/tof.d ./Core/Src/tof.o ./Core/Src/tof.su ./Core/Src/trajectory_calculator.cyclo ./Core/Src/trajectory_calculator.d ./Core/Src/trajectory_calculator.o ./Core/Src/trajectory_calculator.su ./Core/Src/uart_dma_tx.cyclo ./Core/Src/uart_dma_tx.d ./Core/Src/uart_dma_tx.o ./Core/Src/uart_dma_tx.su ./Core/Src/uart_protocol.cyclo ./Core/Src/uart_protocol.d ./Core/Src/uart_protocol.o ./Core/Src/uart_protocol.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su ./Core/Src/vl53l1_platform.cyclo ./Core/Src/vl53l1_platform.d ./Core/Src/vl53l1_platform.o ./Core/Src/vl53l1_platform.su ./Core/Src/ws2812b.cyclo ./Core/Src/ws2812b.d ./Core/Src/ws2812b.o ./Core/Src/ws2812b.su

.PHONY: clean-Core-2f-Src

