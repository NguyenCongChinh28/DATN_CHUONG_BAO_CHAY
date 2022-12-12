#include "board_hw.h"
#include "systime.h"
#include "main.h"
#include "usart.h"
#include "lwrb/lwrb.h"

#define AC_LINE_DETECT_TIMEOUT_MS       (45)

static systime_t m_timer_phase_monitor;
static systime_t m_timer_contactor_monitor;

typedef union
{
    struct
    {
        uint8_t phase_lost : 1;
        uint8_t contactor_detect : 1;
        uint8_t reserve : 6;
    } __attribute__((packed)) error;
    uint8_t value;
} __attribute__((packed)) board_hw_err_t;

//static bool m_hw_error.error.phase_lost = 0;
//static bool m_is_contactor_on = false;
static bool m_relay_change = false;
static volatile board_hw_err_t m_hw_error = {    \
                                    .value = 0, \
                                    };

static lwrb_t m_ringbuffer_cli;
static uint8_t m_uart_rx_buffer[64];
                                    
void board_hw_internal_poll(void)
{
    if (systime_is_timer_elapse(&m_timer_phase_monitor, AC_LINE_DETECT_TIMEOUT_MS))
    {
        m_hw_error.error.phase_lost = 1;
        systime_stop(&m_timer_phase_monitor);
        systime_create(&m_timer_phase_monitor);
    }
//    m_hw_error.error.phase_lost = 1;        // phase never lost, because device power source is comming from phase lost detector
//                                    // =>> phase lost =>> device power off


    if (systime_is_timer_elapse(&m_timer_contactor_monitor, AC_LINE_DETECT_TIMEOUT_MS))
    {
        m_hw_error.error.contactor_detect = 0;
        systime_stop(&m_timer_contactor_monitor);
        systime_create(&m_timer_contactor_monitor);
    }
}

//bool board_hw_read_latch_feedback(board_hw_relay_t relay)
//{
//    if (relay != BOARD_HW_RELAY_0)
//    {
//        return gpio_input_bit_get(GPIOB, GPIO_PIN_4);
//    }
//    return gpio_input_bit_get(GPIOA, GPIO_PIN_15);
//}

void board_hw_internal_timer_start(void)
{
    systime_create(&m_timer_contactor_monitor);
    systime_create(&m_timer_phase_monitor);
}

//void board_hardware_on_phase_io_int_callback(void)
//{
//    m_hw_error.error.phase_lost = 0;
//    systime_create(&m_timer_phase_monitor);
//}

//void board_hardware_on_contactor_io_int_callback(void)
//{
//    m_hw_error.error.contactor_detect = 1;
//    systime_create(&m_timer_contactor_monitor);
//}

bool board_hw_is_phase_lost()
{
    return m_hw_error.error.phase_lost ? true : false;
}

bool board_hw_is_contactor_detect()
{
    return m_hw_error.error.contactor_detect;
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
//    if (relay == BOARD_HW_RELAY_0)
//    {
//        gpio_bit_write(GPIOF, GPIO_PIN_6, on ? 1 : 0);
//    }
//    else if(relay == BOARD_HW_RELAY_1)
//    {
//        gpio_bit_write(GPIOF, GPIO_PIN_7, on ? 1 : 0);
//    }
//    
//    // Latch output enable
//    gpio_bit_set(GPIOB, GPIO_PIN_3);
//    volatile uint8_t delay = 0x2F;      // Delay for latch pin
//    while (delay--);
//    // Disable latch output
//    gpio_bit_reset(GPIOB, GPIO_PIN_3);
//    delay = 0x2F;      // Delay for latch pin
//    while (delay--);
//    APP_DBG_ERR("Set relay[%d] level %d\r\n", relay == BOARD_HW_RELAY_0 ? 0 : 1, on ? 1 : 0);
    
    m_relay_change = true;
}

bool board_hw_relay_status_change(void)
{
    return m_relay_change;
}

void board_hw_relay_status_clear_change(void)
{
    m_relay_change = false;
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

///**
// * @brief               Read DIO gpio pin 
// * @retval              gpio level (0-1)
// */
//uint32_t board_hw_read_dio(void)
//{
//    return gpio_input_bit_get(GPIOB, GPIO_PIN_0);
//}

void board_hw_reset_phase_lost_timeout(void)
{
    m_hw_error.error.phase_lost = 0;
    systime_create(&m_timer_phase_monitor);
}

void board_hw_reset_contactor_monitor_timeout(void)
{
    m_hw_error.error.contactor_detect = 1;
    systime_create(&m_timer_contactor_monitor);
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

static uint32_t m_led_val[2] = {0, 0};
uint32_t board_hw_led_get(uint32_t pin)
{
    if (pin == 0)
    {
        return m_led_val[0];
    }
    return m_led_val[1];
}

void board_hw_led_set(uint32_t pin, uint32_t value)
{
    if (pin == 0)
    {
        m_led_val[0] = value ? 1 : 0;
    }
    m_led_val[1] = value ? 1 : 0;
}

void board_hw_led_toggle(uint32_t pin)
{
    if (pin == 0)
    {
        m_led_val[0] = m_led_val[0] ? 0 : 1;
    }
    m_led_val[1] = m_led_val[1] ? 0 : 1;
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
