#include <stdio.h>
#include <stdint.h>

#include <stdbool.h> 
#include "board_hw.h"
#include "stm32f1xx_hal.h"
#include "usart.h"





uint32_t Get_MS(void)
{
     return uwTick;
}
void board_hw_debug_uart_send(const void *data,uint32_t size)
{
   uart_send(data,size);
}
void board_hw_lora_reset_pin(bool on)
{
    if(on)
    {
        HAL_GPIO_WritePin(LORA_RESET_PIN_GPIO_Port,LORA_RESET_PIN_Pin,GPIO_PIN_SET);
        
    }
    else
    {
        HAL_GPIO_WritePin(LORA_RESET_PIN_GPIO_Port,LORA_RESET_PIN_Pin,GPIO_PIN_RESET);
    }
}

void board_hw_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}


void board_hw_control_remote_priority_io(bool level)
{
    
}
void SX126xDelayMs(uint32_t ms)
{
    board_hw_delay_ms(ms);
}

uint32_t board_hw_get_lora_busy_pin(void)
{
    return HAL_GPIO_ReadPin(LORA_BUSY_GPIO_Port, LORA_BUSY_Pin) ? 1 : 0;
}
uint8_t board_hw_get_battery_percent(void)
{
    return 100;
}




