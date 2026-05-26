################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/hexapod/BatteryMonitor.cpp \
../Core/Src/hexapod/HexapodRobot.cpp \
../Core/Src/hexapod/IMUSensor.cpp \
../Core/Src/hexapod/LEDController.cpp \
../Core/Src/hexapod/Locomotion.cpp \
../Core/Src/hexapod/ServoDriver.cpp \
../Core/Src/hexapod/ToFSensor.cpp \
../Core/Src/hexapod/UARTInterface.cpp 

OBJS += \
./Core/Src/hexapod/BatteryMonitor.o \
./Core/Src/hexapod/HexapodRobot.o \
./Core/Src/hexapod/IMUSensor.o \
./Core/Src/hexapod/LEDController.o \
./Core/Src/hexapod/Locomotion.o \
./Core/Src/hexapod/ServoDriver.o \
./Core/Src/hexapod/ToFSensor.o \
./Core/Src/hexapod/UARTInterface.o 

CPP_DEPS += \
./Core/Src/hexapod/BatteryMonitor.d \
./Core/Src/hexapod/HexapodRobot.d \
./Core/Src/hexapod/IMUSensor.d \
./Core/Src/hexapod/LEDController.d \
./Core/Src/hexapod/Locomotion.d \
./Core/Src/hexapod/ServoDriver.d \
./Core/Src/hexapod/ToFSensor.d \
./Core/Src/hexapod/UARTInterface.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/hexapod/%.o Core/Src/hexapod/%.su Core/Src/hexapod/%.cyclo: ../Core/Src/hexapod/%.cpp Core/Src/hexapod/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Drivers/BSP/Components/lsm6dso16is -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-hexapod

clean-Core-2f-Src-2f-hexapod:
	-$(RM) ./Core/Src/hexapod/BatteryMonitor.cyclo ./Core/Src/hexapod/BatteryMonitor.d ./Core/Src/hexapod/BatteryMonitor.o ./Core/Src/hexapod/BatteryMonitor.su ./Core/Src/hexapod/HexapodRobot.cyclo ./Core/Src/hexapod/HexapodRobot.d ./Core/Src/hexapod/HexapodRobot.o ./Core/Src/hexapod/HexapodRobot.su ./Core/Src/hexapod/IMUSensor.cyclo ./Core/Src/hexapod/IMUSensor.d ./Core/Src/hexapod/IMUSensor.o ./Core/Src/hexapod/IMUSensor.su ./Core/Src/hexapod/LEDController.cyclo ./Core/Src/hexapod/LEDController.d ./Core/Src/hexapod/LEDController.o ./Core/Src/hexapod/LEDController.su ./Core/Src/hexapod/Locomotion.cyclo ./Core/Src/hexapod/Locomotion.d ./Core/Src/hexapod/Locomotion.o ./Core/Src/hexapod/Locomotion.su ./Core/Src/hexapod/ServoDriver.cyclo ./Core/Src/hexapod/ServoDriver.d ./Core/Src/hexapod/ServoDriver.o ./Core/Src/hexapod/ServoDriver.su ./Core/Src/hexapod/ToFSensor.cyclo ./Core/Src/hexapod/ToFSensor.d ./Core/Src/hexapod/ToFSensor.o ./Core/Src/hexapod/ToFSensor.su ./Core/Src/hexapod/UARTInterface.cyclo ./Core/Src/hexapod/UARTInterface.d ./Core/Src/hexapod/UARTInterface.o ./Core/Src/hexapod/UARTInterface.su

.PHONY: clean-Core-2f-Src-2f-hexapod

