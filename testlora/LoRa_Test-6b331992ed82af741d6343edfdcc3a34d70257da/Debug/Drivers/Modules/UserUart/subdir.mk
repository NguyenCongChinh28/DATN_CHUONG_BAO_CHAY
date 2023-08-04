################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Modules/UserUart/User_Uart.c \
../Drivers/Modules/UserUart/User_Uart_Ipw.c 

OBJS += \
./Drivers/Modules/UserUart/User_Uart.o \
./Drivers/Modules/UserUart/User_Uart_Ipw.o 

C_DEPS += \
./Drivers/Modules/UserUart/User_Uart.d \
./Drivers/Modules/UserUart/User_Uart_Ipw.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Modules/UserUart/%.o: ../Drivers/Modules/UserUart/%.c Drivers/Modules/UserUart/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Drivers/LoRa_Drivers/Aloha_Protocol -I../Drivers/LoRa_Drivers/Devices/SX126X/inc -I../Drivers/LoRa_Drivers/Devices/SX126X/src -I../Drivers/LoRa_Drivers/Devices/SX126X/delay -I../Drivers/Modules/UserUart/cfg -I../Drivers/Modules/UserUart/ip/usart -I../Drivers/Modules/UserUart -I../Drivers/Modules/UserUart/protocol/slip -I../Drivers/Mid/base -I../Core/App_Test -I../Drivers/Modules/Button -I../Drivers/LoRa_Drivers/stm32/app_debug -I../Drivers/LoRa_Drivers/stm32/SEGGER_RTT_V698b -I../Drivers/LoRa_Drivers/stm32/SEGGER_RTT_V698b/RTT -I../Drivers/LoRa_Drivers/stm32/SEGGER_RTT_V698b/Syscalls -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

