################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/LoRa_Drivers/Modules/DataLinkLayer/Layer2.c 

OBJS += \
./Drivers/LoRa_Drivers/Modules/DataLinkLayer/Layer2.o 

C_DEPS += \
./Drivers/LoRa_Drivers/Modules/DataLinkLayer/Layer2.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/LoRa_Drivers/Modules/DataLinkLayer/%.o: ../Drivers/LoRa_Drivers/Modules/DataLinkLayer/%.c Drivers/LoRa_Drivers/Modules/DataLinkLayer/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

