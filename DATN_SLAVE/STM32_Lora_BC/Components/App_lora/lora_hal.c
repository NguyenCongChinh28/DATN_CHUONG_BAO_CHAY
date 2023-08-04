#include "lora_hal.h"
#include <string.h>
#include "app_debug.h"
#include "systime.h"
#include "radio.h"


#define LORA_HAL_TX_STATE_IS_IDLE() (c_packet_fsm == LORA_HAL_PACKET_FSM_IDLE)

typedef enum
{
    LORA_PREPARE_OFF,
    LORA_POWER_OFF,
    LORA_POWER_ON,
    LORA_POWER_ALWAYS_OFF
} lora_power_state_t;

static lora_hal_cfg_t c_hal_config;

static systime_t c_timer_transmit_timeout, c_timer_power_monitor;
static systime_t c_timer_channel_busy;


static lora_hal_packet_fsm_t c_packet_fsm = LORA_HAL_PACKET_FSM_INVALID;

static bool c_lora_on_bus = false;

static lora_power_state_t c_lora_power_state = LORA_PREPARE_OFF;






void lora_hal_init(lora_hal_cfg_t *hal_config)
{
    memcpy(&c_hal_config,hal_config,sizeof(c_hal_config));
    lora_hal_reset(); 
}

void lora_hal_reset(void)
{
    APP_DBG_INF("Lora hal_reset\r\n");
    c_hal_config.hard_reset(0);
    //LORA_HAL_CHANGE_TX_STATE(
    //c_lora_power_state = LORA_PREPARE_OFF;
    //c_lora_on_bus = false;
    systime_stop(&c_timer_power_monitor);
    systime_stop(&c_timer_transmit_timeout);
    APP_DBG_INF("Clear busy flag \r\n");
    lora_hal_clear_channel_busy_flag();
}
void lora_hal_clear_channel_busy_flag()
{
    systime_stop(&c_timer_channel_busy);
}

void lora_hal_set_temporary_config(uint32_t freq, uint32_t bw, uint32_t sf)
{
    c_hal_config.freq = freq;
    c_hal_config.bw = bw;
    c_hal_config.sf = sf;
}
bool lora_hal_is_packet_transmitting(void)
{
    return !LORA_HAL_TX_STATE_IS_IDLE();
}
bool lora_hal_is_device_on_bus(void)
{
    return c_lora_on_bus;
}
bool lora_hal_power_ready(void)
{
    return (c_lora_power_state == LORA_POWER_ON) ? true : false;
}
lora_hal_packet_fsm_t *lora_hal_get_packet_fsm(void)
{
    return &c_packet_fsm;
}
bool lora_hal_send_packet(uint8_t *buf, uint32_t size, uint32_t timeout_ms, lora_hal_transmit_cb_t cb)
{
    bool retval = false;

    if (LORA_HAL_TX_STATE_IS_IDLE())
    {
//        APP_DBG_INF("LoRa hal send\r\n");
        /*
        * Transfer data to radio.
        */
        c_lora_tx_cb = cb;
#if 0
        lora_hal_enter_standby();
        lora_write_reg(REG_FIFO_ADDR_PTR, 0);

        for (int i = 0; i < size; i++)
            lora_write_reg(REG_FIFO, *buf++);

        lora_write_reg(REG_PAYLOAD_LENGTH, size);

        /*
        * Start transmission and wait for conclusion.
        */
        lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
#else
        Radio.Standby();
        Radio.Send(buf, size);
#endif        
        c_begin_transmit_timestamp = systime_get_ms();
        
        systime_stop(&c_timer_transmit_timeout);
        systime_create(&c_timer_transmit_timeout);
        LORA_HAL_CHANGE_TX_STATE(LORA_HAL_PACKET_FSM_TRANSMITTING);

        retval = true;
    }

    return retval;
}

