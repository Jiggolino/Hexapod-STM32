################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/lsm6dso16is/lsm6dso16is.c \
../Drivers/BSP/Components/lsm6dso16is/lsm6dso16is_reg.c 

C_DEPS += \
./Drivers/BSP/Components/lsm6dso16is/lsm6dso16is.d \
./Drivers/BSP/Components/lsm6dso16is/lsm6dso16is_reg.d 

OBJS += \
./Drivers/BSP/Components/lsm6dso16is/lsm6dso16is.o \
./Drivers/BSP/Components/lsm6dso16is/lsm6dso16is_reg.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/lsm6dso16is/%.o Drivers/BSP/Components/lsm6dso16is/%.su Drivers/BSP/Components/lsm6dso16is/%.cyclo: ../Drivers/BSP/Components/lsm6dso16is/%.c Drivers/BSP/Components/lsm6dso16is/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Drivers/BSP/Components/lsm6dso16is -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-lsm6dso16is

clean-Drivers-2f-BSP-2f-Components-2f-lsm6dso16is:
	-$(RM) ./Drivers/BSP/Components/lsm6dso16is/lsm6dso16is.cyclo ./Drivers/BSP/Components/lsm6dso16is/lsm6dso16is.d ./Drivers/BSP/Components/lsm6dso16is/lsm6dso16is.o ./Drivers/BSP/Components/lsm6dso16is/lsm6dso16is.su ./Drivers/BSP/Components/lsm6dso16is/lsm6dso16is_reg.cyclo ./Drivers/BSP/Components/lsm6dso16is/lsm6dso16is_reg.d ./Drivers/BSP/Components/lsm6dso16is/lsm6dso16is_reg.o ./Drivers/BSP/Components/lsm6dso16is/lsm6dso16is_reg.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-lsm6dso16is

