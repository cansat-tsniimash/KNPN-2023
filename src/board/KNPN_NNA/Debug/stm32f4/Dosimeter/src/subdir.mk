################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/CanSat/2022-2023/KNPN-2023/lib-tsniimash/stm32f4/Dosimeter/src/dosimeter.c 

OBJS += \
./stm32f4/Dosimeter/src/dosimeter.o 

C_DEPS += \
./stm32f4/Dosimeter/src/dosimeter.d 


# Each subdirectory must supply rules for building sources it contributes
stm32f4/Dosimeter/src/dosimeter.o: D:/CanSat/2022-2023/KNPN-2023/lib-tsniimash/stm32f4/Dosimeter/src/dosimeter.c stm32f4/Dosimeter/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/CanSat/2022-2023/KNPN-2023/lib-tsniimash/stm32f4" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-stm32f4-2f-Dosimeter-2f-src

clean-stm32f4-2f-Dosimeter-2f-src:
	-$(RM) ./stm32f4/Dosimeter/src/dosimeter.d ./stm32f4/Dosimeter/src/dosimeter.o

.PHONY: clean-stm32f4-2f-Dosimeter-2f-src

