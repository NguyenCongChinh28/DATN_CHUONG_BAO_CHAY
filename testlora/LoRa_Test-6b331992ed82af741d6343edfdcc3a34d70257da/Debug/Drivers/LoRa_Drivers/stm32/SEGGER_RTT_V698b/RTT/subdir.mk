################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/LoRa_Drivers/stm32/SEGGER_RTT_V698b/RTT/SEGGER_RTT.c \
../Drivers/LoRa_Drivers/stm32/SEGGER_RTT_V698b/RTT/SEGGER_RTT_printf.c 

S_UPPER_SRCS += \
../Drivers/LoRa_Drivers/stm32/SEGGER_RTT_V698b/RTT/SEGGER_RTT_ASM_ARMv7M.S 

OBJS += \
./Drivers/LoRa_Drivers/stm32/SEGGER_RTT_V698b/RTT/SEGGER_RTT.o \
./Drivers/LoRa_Drivers/stm32/SEGGER_RTT_V698b/RTT/SEGGER_RTT_ASM_ARMv7M.o \
./Drivers/LoRa_Drivers/stm32/SEGGER_RTT_V698b/RTT/SEGGER_RTT_printf.o 

S_UPPER_DEPS += \
./Drivers/LoRa_Drivers/stm32/SEGGER_RTT_V698b/RTT/SEGGER_RTT_ASM_ARMv7M.d 

C_DEPS += \
./Drivers/LoRa_Drivers/stm32/SEGGER_RTT_V698b/RTT/SEGGER_RTT.d \
./Drivers/LoRa_Drivers/stm32/SEGGER_RTT_V698b/RTT/SEGGER_RTT_printf.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/LoRa_Drivers/stm32/SEGGER_RTT_V698b/RTT/%.o: ../Drivers/LoRa_Drivers/stm32/SEGGER_RTT_V698b/RTT/%.c Drivers/LoRa_Drivers/stm32/SEGGER_RTT_V698b/RTT/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Drivers/LoRa_Drivers/Aloha_Protocol -I../Drivers/LoRa_Drivers/Devices/SX126X/inc -I../Drivers/LoRa_Drivers/Devices/SX126X/src -I../Drivers/LoRa_Drivers/Devices/SX126X/delay -I../Drivers/Modules/UserUart/cfg -I../Drivers/Modules/UserUart/ip/usart -I../Drivers/Modules/UserUart -I../Drivers/Modules/UserUart/protocol/slip -I../Drivers/Mid/base -I../Core/App_Test -I../Drivers/Modules/Button -I../Drivers/LoRa_Drivers/stm32/app_debug -I../Drivers/LoRa_Drivers/stm32/SEGGER_RTT_V698b -I../Drivers/LoRa_Drivers/stm32/SEGGER_RTT_V698b/RTT -I../Drivers/LoRa_Drivers/stm32/SEGGER_RTT_V698b/Syscalls -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/LoRa_Drivers/stm32/SEGGER_RTT_V698b/RTT/%.o: ../Drivers/LoRa_Drivers/stm32/SEGGER_RTT_V698b/RTT/%.S Drivers/LoRa_Drivers/stm32/SEGGER_RTT_V698b/RTT/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m3 -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

