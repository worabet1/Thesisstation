################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/Autofox_INA226.c \
../Core/Inc/VL6180X.c \
../Core/Inc/hx711.c \
../Core/Inc/lcd.c 

OBJS += \
./Core/Inc/Autofox_INA226.o \
./Core/Inc/VL6180X.o \
./Core/Inc/hx711.o \
./Core/Inc/lcd.o 

C_DEPS += \
./Core/Inc/Autofox_INA226.d \
./Core/Inc/VL6180X.d \
./Core/Inc/hx711.d \
./Core/Inc/lcd.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/%.o Core/Inc/%.su: ../Core/Inc/%.c Core/Inc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc

clean-Core-2f-Inc:
	-$(RM) ./Core/Inc/Autofox_INA226.d ./Core/Inc/Autofox_INA226.o ./Core/Inc/Autofox_INA226.su ./Core/Inc/VL6180X.d ./Core/Inc/VL6180X.o ./Core/Inc/VL6180X.su ./Core/Inc/hx711.d ./Core/Inc/hx711.o ./Core/Inc/hx711.su ./Core/Inc/lcd.d ./Core/Inc/lcd.o ./Core/Inc/lcd.su

.PHONY: clean-Core-2f-Inc

