#include "board_hw.h"
#include "systime.h"
#include "main.h"
#include "usart.h"
#include "lwrb/lwrb.h"

static lwrb_t m_ringbuffer_cli;
static uint8_t m_uart_rx_buffer[64];
                                    
void board_hw_internal_poll(void)
{
    
}
void board_hw_internal_timer_start(void)
{
    
}

uint32_t board_hw_button_read(uint32_t pin)
{
    if (pin == BOARD_HW_BUTTON_0)       // pair
    {
        return HAL_GPIO_ReadPin(BT_PAIR_GPIO_Port, BT_PAIR_Pin) ? 1 : 0;
    }
    else if (pin == BOARD_HW_BUTTON_1)  // on/off
    {
        return HAL_GPIO_ReadPin(BT_ALARM_GPIO_Port, BT_ALARM_Pin) ? 1 : 0;
    }

    return 0;
}



void board_hw_set_relay(board_hw_relay_t relay, bool on)
{
   
}


void board_hw_lora_write_lora_reset_pin(bool on)
{
    if (!on)
    {
        HAL_GPIO_WritePin(LORA_RESET_PIN_GPIO_Port, LORA_RESET_PIN_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(LORA_RESET_PIN_GPIO_Port, LORA_RESET_PIN_Pin, GPIO_PIN_SET);
    }
}

void board_hw_init_gpio(void)
{
    
}


void board_hw_reset(void)
{
    NVIC_SystemReset();
}

void board_hw_uart_debug_initialize(void)
{
    
}

uint32_t board_hw_get_reset_reason(void)
{
    static uint32_t reason = 0;
//    if (reason == 0)
//    {   
//        reason = RCU_RSTSCK;
//        rcu_all_reset_flag_clear();
//    }
    return reason;
}

void board_hw_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

void board_hw_debug_uart_send(const void *buffer, uint32_t size)
{
    usart_send(buffer, size);
}

uint32_t board_hw_get_lora_busy_pin(void)
{
    return HAL_GPIO_ReadPin(LORA_BUSY_GPIO_Port, LORA_BUSY_Pin) ? 1 : 0;
}

uint32_t board_hw_get_ms(void)
{
    return uwTick;
}

static uint8_t tmp_uart;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    lwrb_write(&m_ringbuffer_cli, &tmp_uart, 1);
    HAL_UART_Receive_IT(&huart1, &tmp_uart, 1);
}    

uint32_t board_hw_uart_poll(uint8_t *ch)
{
    if (m_ringbuffer_cli.buff == NULL)
    {
        lwrb_init(&m_ringbuffer_cli, m_uart_rx_buffer, sizeof(m_uart_rx_buffer));
        HAL_UART_Receive_IT(&huart1, &tmp_uart, 1);
    }
    return lwrb_read(&m_ringbuffer_cli, ch, 1);
}

void board_hw_set_buzzer(bool on)
{
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void SX126xDelayMs(uint32_t ms)
{
    board_hw_delay_ms(ms);
}

void board_hw_control_remote_priority_io(bool level)
{
    
}

uint8_t board_hw_get_battery_percent(void)
{
    return 100;
}

