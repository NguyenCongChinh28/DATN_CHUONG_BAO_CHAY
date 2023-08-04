#include "app_lora.h"
#include "lora_hal.h"
#include <systime.h>
#include <stddef.h>
#include <string.h>
#include "app_debug.h"

#define APP_LORA_TX_RX_IS_IDLE()                        (m_lora_hal_enter_standby_rx && m_lora_hal_enter_standby_tx) // No packet is transmiting or receiving
#define APP_LORA_RX_IS_IDLE()                           (m_lora_hal_enter_standby_rx && !lora_hal_is_channel_busy())
#define APP_LORA_TX_IS_IDLE()                           (m_lora_hal_enter_standby_tx)
#define APP_LORA_RX_IDLE_TIMEOUT_RESET()                 {   \
                                                            m_lora_hal_enter_standby_rx = false; \
                                                        }
#define APP_LORA_TX_IDLE_TIMEOUT_RESET()                 {   \
                                                            m_lora_hal_enter_standby_tx = false; \
                                                        }                                                        
#define APP_LORA_MAX_TX_COUNTER                       (0x0FFFFFFF) // 2^24 is big enough for tx counter


static app_lora_cfg_t m_cfg;
static bool m_lora_hal_enter_standby_rx = false;
static bool m_lora_hal_enter_standby_tx = true;
static int32_t m_tx_retry_counter;

static void on_lora_frame_callback(uint8_t *data, uint8_t size);
static void on_channel_free_callback(void);
static bool lora_rf_tx(uint8_t *bytes, uint32_t size, uint32_t timeout, lora_hal_transmit_cb_t callback);
static void lora_hal_transmit_callback(bool success, uint32_t timeout);

static systime_t m_systime_auto_reset_lora_module;
static systime_t m_systime_auto_reset_rx_idle;
static systime_t m_systime_tx_timeout;

static bool m_reset_lora_module = false;

typedef struct 
{
    app_lora_header_t header;
    uint8_t payload[APP_LORA_MAX_PAYLOAD_SIZE];
    uint8_t len;
} __attribute__ ((packed))app_lora_packet_t;

typedef struct
{
    app_lora_master_tx_cb_t master_cb;
    app_lora_packet_t msg;
    uint32_t timeout_ms;
    void *arg;
} app_lora_retry_msg_ctx_t;

static bool app_lora_master_retry_tx(app_lora_retry_msg_ctx_t *msg);
static app_lora_retry_msg_ctx_t m_master_msg_ctx;
static bool get_rx_payload(uint8_t *input, 
                            uint32_t input_size, 
                            app_lora_header_t **header,
                            uint8_t **body,
                            uint32_t *size);


void app_lora_init(app_lora_cfg_t *conf)
{   
    // assert(conf);
    // assert(conf.get_ms);
    // assert(conf.period_reset_ms);
    // assert(conf.send_byte)
    memcpy(&m_cfg, conf, sizeof(m_cfg));
    
    // LoRa HAL initialize
    lora_hal_cfg_t hal_conf;
    hal_conf.spi_tx = conf->spi_tx;
    hal_conf.spi_rx = conf->spi_rx;
    hal_conf.on_frame_cb = on_lora_frame_callback;
    hal_conf.on_channel_free = on_channel_free_callback;
    hal_conf.delay_ms = conf->delay_ms;
    hal_conf.read_pin = conf->read_pin;
    hal_conf.hard_reset = conf->lora_hw_reset; 
    hal_conf.tx_power = conf->tx_power;
    hal_conf.sync_word = conf->sync_word;
    hal_conf.bw = conf->bw;
    hal_conf.freq = conf->freq;
    hal_conf.sf = conf->sf;
    hal_conf.cr = conf->cr;
    hal_conf.preamble_length = conf->preamble_length;
    hal_conf.symbol_timeout = conf->symbol_timeout;
    hal_conf.startup_ms = conf->startup_ms;
    
    hal_conf.enable_crc = conf->enable_crc;
    lora_hal_init(&hal_conf);

    m_master_msg_ctx.msg.header.network_id = conf->network_id;
    systime_create(&m_systime_auto_reset_lora_module);
    systime_create(&m_systime_auto_reset_rx_idle);
}

void app_lora_stop_process_tx_timeout(void)
{
    m_lora_hal_enter_standby_tx = true;
    systime_stop(&m_systime_tx_timeout);
    m_tx_retry_counter = 0;
}

void app_lora_hal_enter_shutdown_mode(void)
{
    m_lora_hal_enter_standby_tx = true;
    systime_stop(&m_systime_tx_timeout);
    m_tx_retry_counter = 0;
    lora_hal_enter_shutdown_mode();
}

bool app_lora_is_in_shutdown_mode(void)
{
    return lora_hal_is_in_shutdown_mode();
}

void app_lora_hal_wakeup(void)
{
    lora_hal_wakeup();
}

void app_lora_poll()
{
    if (APP_LORA_TX_RX_IS_IDLE())   
    {
        if (m_cfg.period_reset_ms_when_moderm_good && systime_is_timer_elapse(&m_systime_auto_reset_lora_module, m_cfg.period_reset_ms_when_moderm_good))
        {
            APP_DBG_ERR("Auto reboot module\r\n");
            m_reset_lora_module = true;
        }
    }

    if (m_reset_lora_module)
    {
        APP_DBG_ERR("Reseting lora module\r\n");
        lora_hal_reset();
		systime_stop(&m_systime_auto_reset_lora_module);
        systime_create(&m_systime_auto_reset_lora_module);
        m_reset_lora_module = false;
    }

    if (!APP_LORA_TX_IS_IDLE())
    {
        static uint32_t random_timeout_ms = 0;

        // TX timeout is over && lora power ready
        if (lora_hal_power_ready()
            && systime_is_timer_elapse(&m_systime_tx_timeout, (m_master_msg_ctx.timeout_ms + random_timeout_ms))) 
        {
            if (!lora_hal_is_device_on_bus())
            {
                // LoRa chip is not detected,we do not transmit anything
                APP_DBG_ERR("LoRA chip error\r\n");
                m_tx_retry_counter = 0;
            }

            if (m_tx_retry_counter-- > 0)
            {
                random_timeout_ms = m_cfg.get_ms() % 999;
                APP_DBG_ERR("LORA TX retry %d times, tx timeout %d\r\n",
                            m_tx_retry_counter, 
                            m_master_msg_ctx.timeout_ms + random_timeout_ms);
                app_lora_master_retry_tx(&m_master_msg_ctx);

                APP_LORA_TX_IDLE_TIMEOUT_RESET();
                systime_create(&m_systime_tx_timeout);
            }
            else
            {
                m_lora_hal_enter_standby_tx = true;
                systime_stop(&m_systime_tx_timeout);
                m_tx_retry_counter = 0;
                // LoRa timeout
                if (m_master_msg_ctx.master_cb)
                {
                    m_master_msg_ctx.master_cb(APP_LORA_ERR_TX_TIMEOUT, (void*)0);
                }
            }
        }
    }

    if (!m_lora_hal_enter_standby_rx)
    {
        if (systime_is_timer_elapse(&m_systime_auto_reset_rx_idle, m_cfg.rx_idle_timeout_ms) 
            && !lora_hal_is_channel_busy())
        {
            APP_DBG_INF("LoRa RX IDLE\r\n");
            m_lora_hal_enter_standby_rx = true;
            systime_stop(&m_systime_auto_reset_rx_idle);
        }
    }

    lora_hal_task();
}


void app_lora_mask_node_ack(void)
{
    m_lora_hal_enter_standby_tx = true;
    m_lora_hal_enter_standby_rx = true;
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


static void on_channel_free_callback(void)
{
    m_cfg.event(APP_LORA_EVENT_CHANNEL_FREE, NULL, NULL);
}

static void on_lora_frame_callback(uint8_t *data, uint8_t size)
{
	systime_stop(&m_systime_auto_reset_lora_module);
	systime_create(&m_systime_auto_reset_lora_module);
		
    APP_DBG_INF("---------Dump RX-----------\r\n");
    APP_DBG_RAW("\t\t");
    for (uint32_t i = 0; i < size; i++)
    {
        APP_DBG_RAW("%02X ", data[i]);
    }
    APP_DBG_RAW("\r\n-----------------------------------------------\r\n");
    
    app_lora_header_t *header;   
    uint8_t *body;
    uint32_t payload_len;

    if (get_rx_payload(data, size, &header, &body, &payload_len))
    {
        APP_DBG_INF("On lora frame callback, size %d bytes, header %d bytes, body %d bytes\r\n", 
                        size, 
                        sizeof(app_lora_header_t),
                        payload_len);
        
        APP_DBG_INF("Network id %d, msg type %d, device type %s,"
                    "node mac %02X:%02X:%02X:%02X:%02X:%02X:, from %s\r\n", 
                        header->network_id,
                        header->type.name.msg_type,
                        app_lora_get_device_type_str(header->type.name.device_type),
                        header->mac[0],
                        header->mac[1],
                        header->mac[2],
                        header->mac[3],
                        header->mac[4],
                        header->mac[5],
                        (header->type.name.device_type == APP_LORA_DEVICE_TYPE_GW) ? "GW" : "NODE");
            
        if (header->network_id == m_master_msg_ctx.msg.header.network_id
            && header->network_id)
        {
            if (m_cfg.event)
            {
                app_lora_data_t rx_msg;
                rx_msg.device_type = header->type.name.device_type;
                rx_msg.msg_type = header->type.name.msg_type;
                memcpy(rx_msg.mac, header->mac, 6);
                rx_msg.payload = body;
                rx_msg.len = payload_len;
                m_cfg.event(APP_LORA_EVENT_RX, &rx_msg, header);
            }
        }
        else        // Un-authen message
        {
            if (m_cfg.unauthen_data_event)
            {
                app_lora_data_t rx_msg;
                rx_msg.device_type = header->type.name.device_type;
                rx_msg.msg_type = header->type.name.msg_type;
                memcpy(rx_msg.mac, header->mac, 6);
                rx_msg.payload = body;
                rx_msg.len = payload_len;
                m_cfg.unauthen_data_event(APP_LORA_EVENT_RX, &rx_msg, header);
            }
        }
    }
    else
    {
        APP_DBG_ERR("Invalid LoRa frame\r\n");
    }
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
        APP_DBG_INF("LoRa packet is transmitting, state %d\r\n", m_cfg.get_ms(), *fsm);
        goto end;
    }

    if (!lora_hal_is_device_on_bus())
    {
        APP_DBG_ERR("Lora chip is not detected\r\n", m_cfg.get_ms());
        goto end;
    }

    if (data->len > APP_LORA_MAX_PAYLOAD_SIZE)
    {
        APP_DBG_ERR("Lora packet %d bytes, was too long\r\n", data->len);
        goto end;
    }

    // Copy data
    m_master_msg_ctx.msg.header.type.name.device_type = data->device_type;
    memcpy(m_master_msg_ctx.msg.header.mac, data->mac, 6);
    m_master_msg_ctx.msg.header.type.name.msg_type = data->msg_type;
    memcpy(m_master_msg_ctx.msg.payload, data->payload, data->len);

    m_master_msg_ctx.timeout_ms = timeout_ms;

    m_master_msg_ctx.msg.len = sizeof(app_lora_packet_t) 
                                - (sizeof(((app_lora_packet_t*)0)->payload) - data->len)
                                - sizeof(((app_lora_packet_t*)0)->len); 

    APP_DBG_INF("Sending lora packet size %d, MAC %02X:%02X:%02X:%02X:%02X:%02X\r\n", 
                m_master_msg_ctx.msg.len, 
                m_master_msg_ctx.msg.header.mac[0],
                m_master_msg_ctx.msg.header.mac[1],
                m_master_msg_ctx.msg.header.mac[2],
                m_master_msg_ctx.msg.header.mac[3],
                m_master_msg_ctx.msg.header.mac[4],
                m_master_msg_ctx.msg.header.mac[5]);


    lora_rf_tx((uint8_t*)&m_master_msg_ctx.msg, 
                m_master_msg_ctx.msg.len, 
                timeout_ms, 
                lora_hal_transmit_callback);


    APP_LORA_TX_IDLE_TIMEOUT_RESET();
    systime_create(&m_systime_tx_timeout);

    m_tx_retry_counter = retry;

    m_master_msg_ctx.arg = arg;
    m_master_msg_ctx.master_cb = master_cb;

    retval = true;

    end:
    return retval;
}

static bool app_lora_master_retry_tx(app_lora_retry_msg_ctx_t *msg)
{
    if (!APP_LORA_RX_IS_IDLE())
    {
        APP_DBG_ERR("Lora rx is busy\r\n");
        return false;
    }

    if (!lora_hal_is_device_on_bus())
    {
        APP_DBG_ERR("Lora chip is not detected\r\n");
    }

    APP_DBG_INF("Re-sending lora packet size %d\r\n", m_master_msg_ctx.msg.len);

    return lora_rf_tx((uint8_t*)&m_master_msg_ctx.msg, 
                m_master_msg_ctx.msg.len, 
                m_master_msg_ctx.timeout_ms, 
                lora_hal_transmit_callback);
}


static void lora_hal_transmit_callback(bool success, uint32_t timeout)
{
    APP_DBG_INF("LoRa hal transmit %s in %dms\r\n", success ? "OK" : "FAIL", timeout);
}


static bool get_rx_payload(uint8_t *input, 
                            uint32_t input_size, 
                            app_lora_header_t **header,
                            uint8_t **body,
                            uint32_t *size)
{
    if (!input)
    {
        return false;
    }
    
    uint32_t header_len = sizeof(app_lora_header_t); 

    if (input_size < header_len)
    {
        return false;
    }

    *header = (app_lora_header_t*)input;
    *body = input + header_len;
    *size = input_size - header_len;

    return true;
}

void app_lora_set_network_id(uint16_t network_id)
{
    m_master_msg_ctx.msg.header.network_id = network_id;
}



bool app_lora_master_force_tx(app_lora_data_t *data, 
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

    if (lora_hal_is_packet_transmitting())
    {
        lora_hal_packet_fsm_t *fsm = lora_hal_get_packet_fsm();
        APP_DBG_INF("LoRa packet is transmitting, state %d\r\n", *fsm);
        goto end;
    }

    if (!lora_hal_is_device_on_bus())
    {
        APP_DBG_ERR("Lora chip is not detected\r\n");
        goto end;
    }

    if (data->len > APP_LORA_MAX_PAYLOAD_SIZE)
    {
        APP_DBG_ERR("Lora packet %d bytes, was too long\r\n", data->len);
        goto end;
    }

    // Copy data
    m_master_msg_ctx.msg.header.type.name.device_type = data->device_type;
    memcpy(m_master_msg_ctx.msg.header.mac, data->mac, 6);
    m_master_msg_ctx.msg.header.type.name.msg_type = data->msg_type;
    memcpy(m_master_msg_ctx.msg.payload, data->payload, data->len);

    m_master_msg_ctx.timeout_ms = timeout_ms;

    m_master_msg_ctx.msg.len = sizeof(app_lora_packet_t) 
                                - (sizeof(((app_lora_packet_t*)0)->payload) - data->len)
                                - sizeof(((app_lora_packet_t*)0)->len); 

    APP_DBG_INF("Sending lora packet size %d, node %02X:%02X:%02X:%02X:%02X:%02X, from %s to %s\r\n", 
                    m_master_msg_ctx.msg.len, 
                    m_master_msg_ctx.msg.header.mac[0],
                    m_master_msg_ctx.msg.header.mac[1],
                    m_master_msg_ctx.msg.header.mac[2],
                    m_master_msg_ctx.msg.header.mac[3],
                    m_master_msg_ctx.msg.header.mac[4],
                    m_master_msg_ctx.msg.header.mac[5],
                    (m_master_msg_ctx.msg.header.type.name.device_type == APP_LORA_DEVICE_TYPE_GW) ? "GW" : "NODE",
                    (m_master_msg_ctx.msg.header.type.name.device_type == APP_LORA_DEVICE_TYPE_GW) ? "NODE" : "GW");


    lora_rf_tx((uint8_t*)&m_master_msg_ctx.msg, 
                m_master_msg_ctx.msg.len, 
                timeout_ms, 
                lora_hal_transmit_callback);


    APP_LORA_TX_IDLE_TIMEOUT_RESET();
    systime_create(&m_systime_tx_timeout);

    m_tx_retry_counter = retry;

    m_master_msg_ctx.arg = arg;
    m_master_msg_ctx.master_cb = master_cb;

    retval = true;

    end:
    return retval;
}

void app_lora_reset(void)
{
    APP_DBG_INF("LoRa middleware reset\r\n");
    m_lora_hal_enter_standby_rx = true;
    m_lora_hal_enter_standby_tx = true;
    lora_hal_clear_channel_busy_flag();
    lora_hal_reset();
}

void app_lora_set_temporary_config(uint32_t freq, uint8_t bw, uint8_t sf)
{
    lora_hal_set_temporary_config(freq, bw, sf);
}


const char *app_lora_get_device_type_str(uint8_t device_type)
{
    static const char *dev_type[] = {"GW", "FR", "S", "R", "B", "T", "D", "P", "TL", "TW", "TA", "ST", "PM", "NA"};
    if (device_type > 13)
    {
        device_type = 13;
    }
    return dev_type[device_type];
}
