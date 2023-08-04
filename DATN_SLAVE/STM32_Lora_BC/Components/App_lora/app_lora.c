#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "app_lora.h"
#include "lora_hal.h"
#include "app_debug.h"
#include "systime.h"





#define APP_LORA_TX_RX_IS_IDLE()                        (c_lora_hal_enter_standby_rx && c_lora_hal_enter_standby_tx) // No packet is transmiting or receiving
#define APP_LORA_TX_IDLE_TIMEOUT_RESET()                 {   \
                                                            c_lora_hal_enter_standby_tx = false; \
                                                        }                                                       






static int32_t c_tx_retry_counter;


static app_lora_cfg_t c_config;


static bool c_lora_hal_enter_standby_rx = false;
static bool c_lora_hal_enter_standby_tx = true;



static app_lora_retry_msg_ctx_t c_master_msg_ctx;

static systime_t c_systime_tx_timeout;

static void lora_hal_transmit_callback(bool success, uint32_t timeout);
static bool lora_rf_tx(uint8_t *bytes, uint32_t size, uint32_t timeout, lora_hal_transmit_cb_t callback);


void app_lora_initialize(app_lora_cfg_t *config)
{
    memcpy(&c_config, config, sizeof(c_config));
    
    // LoRa HAL initialize
    lora_hal_cfg_t hal_config;
    hal_config.hard_reset = config->lora_hw_reset;
    hal_config.delay_ms = config->delay_ms;
    hal_config.read_pin = config->read_pin;
    hal_config.spi_tx = config->spi_tx;
    hal_config.spi_rx = config->spi_rx;
    hal_config.tx_power = config->tx_power;
    hal_config.freq = config->freq;
    hal_config.sf = config->sf;
    hal_config.cr = config->cr;
    hal_config.bw = config->bw;
    hal_config.preamble_length = config->preamble_length;
//    hal_config.on_channel_free = on_channel_free_callback;
//    hal_config.on_frame_cb = on_lora_frame_callback;
    hal_config.sync_word = config->sync_word;
    lora_hal_init(&hal_config);
    
    

}

void app_lora_set_network_id(uint16_t network_id)
{
      c_master_msg_ctx.msg.header.network_id = network_id;
}
void app_lora_reset(void)
{
    APP_DBG_INF("LoRa middleware reset\r\n");
    c_lora_hal_enter_standby_rx = true;
    c_lora_hal_enter_standby_tx = true;
    lora_hal_clear_channel_busy_flag();
    lora_hal_reset();
}
void app_lora_set_temporary_config(uint32_t freq, uint8_t bw, uint8_t sf)
{
    lora_hal_set_temporary_config(freq, bw, sf);
}

void app_lora_stop_process_tx_timeout(void)
{
     c_lora_hal_enter_standby_tx = true;
     systime_stop(&c_systime_tx_timeout);
     c_tx_retry_counter = 0;
}
bool app_lora_is_ready_to_tx(void)
{
    bool val = false;
    // DEBUG_PRINTF("First %d %d %d %d\r\n", APP_LORA_TX_RX_IS_IDLE(), 
                                         // !lora_hal_is_packet_transmitting(), 
                                         // lora_hal_is_device_on_bus(), 
                                         // lora_hal_power_ready());

        if (APP_LORA_TX_RX_IS_IDLE() 
        && (!lora_hal_is_packet_transmitting()) 
        && (lora_hal_is_device_on_bus())
        && (lora_hal_power_ready()))
    {
        val = true;
    }
    else
    {
        val = false;
    }

    return val;
}

bool app_lora_master_tx(app_lora_data_t *data, 
                        app_lora_master_tx_cb_t master_cb, 
                        int retry, 
                        uint32_t timeout_ms,
                        void *arg)
{
    bool retval = false;

    if (!data)
    {
        goto end;
    }

    if (data->len && data->payload == NULL)
    {
        goto end;
    }

    if (!APP_LORA_TX_RX_IS_IDLE())
    {
        APP_DBG_INF("Lora TXRX is busy\r\n");
        goto end;
    }

    if (lora_hal_is_packet_transmitting())
    {
        lora_hal_packet_fsm_t *fsm = lora_hal_get_packet_fsm();
        APP_DBG_INF("LoRa packet is transmitting, state %d\r\n", c_config.get_ms(), *fsm);
        goto end;
    }

    if (!lora_hal_is_device_on_bus())
    {
        APP_DBG_ERR("Lora chip is not detected\r\n", c_config.get_ms());
        goto end;
    }

    if (data->len > APP_LORA_MAX_PAYLOAD_SIZE)
    {
        APP_DBG_ERR("Lora packet %d bytes, was too long\r\n", data->len);
        goto end;
    }

    // Copy data
    c_master_msg_ctx.msg.header.type.name.device_type = data->device_type;
    memcpy(c_master_msg_ctx.msg.header.mac, data->mac, 6);
    c_master_msg_ctx.msg.header.type.name.msg_type = data->msg_type;
    memcpy(c_master_msg_ctx.msg.payload, data->payload, data->len);

    c_master_msg_ctx.timeout_ms = timeout_ms;

    c_master_msg_ctx.msg.len = sizeof(app_lora_packet_t) 
                                - (sizeof(((app_lora_packet_t*)0)->payload))
                                - sizeof(((app_lora_packet_t*)0)->len); 

    APP_DBG_INF("Sending lora packet size %d, MAC %02X:%02X:%02X:%02X:%02X:%02X, timeout = %dms\r\n", 
                c_master_msg_ctx.msg.len, 
                c_master_msg_ctx.msg.header.mac[0],
                c_master_msg_ctx.msg.header.mac[1],
                c_master_msg_ctx.msg.header.mac[2],
                c_master_msg_ctx.msg.header.mac[3],
                c_master_msg_ctx.msg.header.mac[4],
                c_master_msg_ctx.msg.header.mac[5],
                c_master_msg_ctx.timeout_ms);


    lora_rf_tx((uint8_t*)&c_master_msg_ctx.msg, 
                c_master_msg_ctx.msg.len, 
                timeout_ms, 
                lora_hal_transmit_callback);


    APP_LORA_TX_IDLE_TIMEOUT_RESET();
    systime_create(&c_systime_tx_timeout);

    c_tx_retry_counter = retry;

    c_master_msg_ctx.arg = arg;
    c_master_msg_ctx.master_cb = master_cb;

    retval = true;

    end:
    return retval;
}

static void lora_hal_transmit_callback(bool success, uint32_t timeout)
{
    APP_DBG_INF("LoRa hal transmit %s in %dms\r\n", success ? "OK" : "FAIL", timeout);
}
static bool lora_rf_tx(uint8_t *bytes, uint32_t size, uint32_t timeout, lora_hal_transmit_cb_t callback)
{
#if 0
    DEBUG_PRINTF("---------Dump TX [%d] bytes-----------\r\n", size);
    for (uint32_t i = 0; i < size; i++)
    {
        DEBUG_RAW("%02x ", bytes[i]);
    }
    DEBUG_RAW("\r\n---------------------------\r\n");
#endif
    return lora_hal_send_packet(bytes, size, timeout, callback);
}

                            


    












    





