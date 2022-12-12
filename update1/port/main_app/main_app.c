#include <stdio.h>
#include <string.h>
#include "main.h"
#include "systime.h"
#include "app_wdt.h"
#include "app_btn.h"
#include "app_led_status.h"
#include "app_lora.h"
#include "app_spi.h"
#include "app_debug.h"
#include "app_flash.h"
#include "app_btn.h"
#include "app_led_status.h"
#include "lwrb/lwrb.h"
#include "app_cli.h"
//#include "eeprom.h"
#include "lora_hal.h"
#include "SEGGER_RTT.h"
#include "board_hw.h"

#define LORA_RF_IDLE_MS (4000)
#define LORA_AUTO_RESET_MS (1000*3600)
#define LORA_MODERM_FAIL_RESET_MS (20000)
#define MAX_PAIR_TIMEOUT_MS (30000)

#define DEVICE_ID_INVALID 0xFF

#define LORA_RF_FREQUENCY_NORMAL (m_net_info.freq)  // Hz
#define LORA_RF_FREQUENCY_PAIR 433000000  // Hz
#define LORA_TX_OUTPUT_POWER 22		// dBm
#define LORA_BANDWIDTH_OPTIMIZE_RANGE 0		// [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR_OPTIMIZE_RANGE 12 // [SF7..SF12]
#define LORA_BANDWIDTH_OPTIMIZE_SPEED       2
#define LORA_SPREADING_FACTOR_OPTIMIZE_SPEED 7
//#define LORA_BANDWIDTH_OPTIMIZE_SPEED       LORA_BANDWIDTH_OPTIMIZE_RANGE
//#define LORA_SPREADING_FACTOR_OPTIMIZE_SPEED LORA_SPREADING_FACTOR_OPTIMIZE_RANGE

#define AUTO_RESET_SENSOR_ALARM_IN_SECOND   (1800)
#define LORA_CODINGRATE 1		// [1: 4/5, 2: 4/6,  3: 4/7,  4: 4/8]
#define LORA_PREAMBLE_LENGTH 8  // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0   // Symbols

#define LED_CONTACTOR_ERROR_BLINK()             app_led_blink(0, 4, 500, 500)
#define LED_PAIR_BLINK()                        app_led_blink(0, 4, 30, 30)

#define CONTACTOR_ERROR()                       (m_relay_value.name.set_value   \
                                                 && !board_hw_is_contactor_detect())


#define TIMEOUT_NOT_RECEIVED_ANY_MESSAGE_MS		300000
#define ALARM_BIT_BUTTON_SOS    0
#define ALARM_BIT_LORA_SOS      1
#define MAX_LIST_SENSOR_ALARM_SUPPORT   32

static lwrb_t m_ringbuff;
static void ringbuffer_uart_initialize(void);
void Delayms(uint32_t ms);
volatile uint8_t m_lora_tx_busy = 0;
static uint32_t m_local_alarm_status = 0;
static uint32_t m_remote_gateway_alarm_status = 0;
static uint32_t m_remote_gateway_io_priority = 0;
static uint32_t m_remote_sensor_alarm_status = 0;
static app_lora_list_alarm_device_t m_list_sensor_alarm[MAX_LIST_SENSOR_ALARM_SUPPORT];
typedef struct
{
    uint8_t node_id : 4;
    uint8_t level : 1;
    uint8_t do_ack : 1;
    uint8_t reserve : 2;
} __attribute__((packed)) relay_msg_t;

typedef union
{
    struct __attribute__((packed))
    {
        uint8_t get_back_value : 1;
        uint8_t set_value : 1;
        uint8_t phase_lost : 1;
        uint8_t dead : 1;
        uint8_t reserve : 4;
    } name;
    uint8_t value;
} __attribute__((packed)) relay_value_t;

typedef struct
{
    uint16_t network_id;
    uint8_t mac[6];
    uint8_t offer_id;
    uint8_t new_msg;
} __attribute__((packed)) provision_slave_reply_t;


static void task_wdt(uint32_t timeout_ms);
static void monitor_pair_timeout(void *arg);
static void task_lora_rx(void *arg);
static void task_cli(void);
static void task_led_btn(void *arg);
static void task_1000ms(void);

static uint32_t sys_now_ms(void);

static void process_join_offer_message(uint8_t *data, uint16_t len, void *header);
void send_pair_info(void);
static bool keep_alive(void);

static void on_new_lora_data(app_lora_event_t event, void *arg, void *header);
static void on_new_unknown_lora_data_received(app_lora_event_t event, void *arg, void *header);
static void on_lora_tx_event(app_lora_err_t err, void *arg);


static void button_init(void);
void on_button_event_cb(int button_pin, int event, void *data);
static void led_init(void);

static systime_t m_timer_wdt;
static systime_t m_timer_pair;
static systime_t m_timer_keep_alive;
static systime_t m_timer_led_btn;
static systime_t m_timer_1000ms;

static bool m_pending_keep_alive = false;
app_lora_provisioning_state_t m_join_network_state = APP_LORA_PROVISIONING_STATE_IDLE;
static uint8_t m_slave_mac[6]; // TODO generate device id
static provision_slave_reply_t m_slave_reply_composion_data;
static app_flash_network_info_t m_net_info __attribute__((aligned(4)));
static bool lora_read_pin(uint32_t pin);

bool ringbuffer_insert(uint8_t data);
static void led_toggle(uint32_t pin);
static void change_temporary_lora_conf(uint32_t freq, uint8_t bw, uint8_t sf);
static bool m_continues_pair_request = false;
static uint32_t m_keep_alive_interval = 0;
uint32_t reset_reason = 0;
volatile int32_t timeout_not_received_msg_reset_lora = TIMEOUT_NOT_RECEIVED_ANY_MESSAGE_MS;
volatile uint32_t timeout_not_received_msg_reset_mcu = 3*TIMEOUT_NOT_RECEIVED_ANY_MESSAGE_MS;
static uint8_t m_retries_after_fire_alarm_stop = 0;
void rtt_puts(uint8_t *buffer, uint32_t size);
uint32_t debug_puts(const void *buffer, uint32_t size);
int rtt_print(const char *msg);
static app_cli_cb_t m_cli = 
{
    .puts = rtt_puts,
    .printf = rtt_print,
};
void sos_fire_alarm_led_indicator_poll(void);

void main_app(void)
{
    reset_reason = board_hw_get_reset_reason();
    app_wdt_start();
    
    ringbuffer_uart_initialize();
    board_hw_uart_debug_initialize();
    
    app_dbg_init(board_hw_get_ms, NULL);
    app_dbg_register_callback_print(debug_puts);
    APP_DBG_INF("System reset\r\n");
        
    memset(&m_slave_reply_composion_data, 0, sizeof(m_slave_reply_composion_data));
    app_flash_get_mac_addr(m_slave_mac);
    
    APP_DBG_INF("Slave mac addr %02X:%02X:%02X:%02X:%02X:%02X\r\n",
             m_slave_mac[0], m_slave_mac[1], m_slave_mac[2],
             m_slave_mac[3], m_slave_mac[4], m_slave_mac[5]);
    systime_create(&m_timer_wdt);
    app_cli_start(&m_cli);
    app_spi_initialize();
    board_hw_init_gpio();    
    app_wdt_feed();
	board_hw_relay_status_clear_change();

    // APP_DBG_INF("Eeprom relay init value 0x%04X\r\n", m_eeprom_relay_val);
    APP_DBG_INF("Reset reason 0x%08X\r\n", reset_reason);

    button_init();
    led_init();

    if (app_flash_infor_is_exist_in_flash())
    {
        app_flash_read_info(&m_net_info);
        APP_DBG_INF("Network addr %d, device id %d, freq %u\r\n",
                     m_net_info.network_id,
                     m_net_info.device_id,
                     m_net_info.freq);
    }
    else
    {
        m_net_info.network_id = 0;
        m_net_info.device_id = DEVICE_ID_INVALID;
        m_net_info.freq = LORA_RF_FREQUENCY_PAIR;
        APP_DBG_INF("Device is not provisioned\r\n");
    }
    
    m_slave_reply_composion_data.new_msg = 0;

    // Config lora driver
    app_lora_cfg_t lora_cfg;
    lora_cfg.lora_hw_reset = board_hw_lora_write_lora_reset_pin;
    lora_cfg.delay_ms = board_hw_delay_ms;
    lora_cfg.read_pin = lora_read_pin;
    lora_cfg.spi_tx = app_spi_tx;
    lora_cfg.spi_rx = app_spi_rx;
    lora_cfg.tx_power = LORA_TX_OUTPUT_POWER;
    lora_cfg.freq = LORA_RF_FREQUENCY_NORMAL;
    lora_cfg.sf = LORA_SPREADING_FACTOR_OPTIMIZE_RANGE;
    lora_cfg.cr = LORA_CODINGRATE;
    lora_cfg.preamble_length = LORA_PREAMBLE_LENGTH;
    lora_cfg.symbol_timeout = 0;
    lora_cfg.bw = LORA_BANDWIDTH_OPTIMIZE_RANGE;
    lora_cfg.get_ms = sys_now_ms;
    lora_cfg.period_reset_ms_when_moderm_good = LORA_AUTO_RESET_MS;
    lora_cfg.period_reset_ms_when_moderm_fail = LORA_MODERM_FAIL_RESET_MS;
    lora_cfg.rx_idle_timeout_ms = 4000;
    lora_cfg.startup_ms = 50;
    lora_cfg.event = on_new_lora_data;
    lora_cfg.unauthen_data_event = on_new_unknown_lora_data_received;
    app_lora_init(&lora_cfg);

    app_lora_set_network_id(m_net_info.network_id);

    
    while (board_hw_get_ms() == 0)
    {
    }
    board_hw_internal_timer_start();
    
    m_keep_alive_interval = (m_net_info.device_id + 1) * 3000;
    systime_create(&m_timer_keep_alive);
    m_pending_keep_alive = true;
    while (1)
    {
        task_lora_rx(NULL);
        sos_fire_alarm_led_indicator_poll();
        task_led_btn((void*)20);
        task_wdt(100);
        task_1000ms();
    }
}

#if DEBUG_UART
void app_cli_gets(uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        if (!lwrb_read(&m_ringbuff, &buf[i], 1))
        {
            buf[i] = 0xFF;
        }
    }
}
#endif

static void task_cli(void)
{
    while (1)
    {
        int key = SEGGER_RTT_GetKey();
        if (key != -1)
        {
            SEGGER_RTT_GetKey();
            app_cli_poll((uint8_t)key);
        }
        else
        {
            break;
        }
    }
}

static void monitor_pair_timeout(void *arg)
{
    if (m_join_network_state == APP_LORA_PROVISIONING_STATE_IDLE)
    {
        return;
    }
    if (systime_is_timer_elapse(&m_timer_pair, MAX_PAIR_TIMEOUT_MS))
    {
        APP_DBG_WRN("Stop pair service, restore network id %u\r\n", m_net_info.network_id);
        app_lora_set_network_id(m_net_info.network_id);
        change_temporary_lora_conf(m_net_info.freq, LORA_BANDWIDTH_OPTIMIZE_RANGE, LORA_SPREADING_FACTOR_OPTIMIZE_RANGE);
        m_join_network_state = APP_LORA_PROVISIONING_STATE_IDLE;
        m_continues_pair_request = false;
    }
}

static void task_wdt(uint32_t timeout_ms)
{
    task_cli();
    if (systime_is_timer_elapse(&m_timer_wdt, timeout_ms))
    {
        systime_create(&m_timer_wdt);
        app_wdt_feed();
		if (timeout_not_received_msg_reset_lora <= 0)
		{
			timeout_not_received_msg_reset_lora = TIMEOUT_NOT_RECEIVED_ANY_MESSAGE_MS;
			app_lora_reset();
		}
		
		if (!lora_hal_is_device_on_bus())
		{
			if (app_led_get_blink_cnt(0) == 0)
			{
				app_led_blink(0, 8, 50, 50);
			}
		}
        
#if 1

        while (1)
        {
            uint8_t key;
            if (board_hw_uart_poll(&key))
            {
                app_cli_poll((uint8_t)key);
            }
            else
            {
                break;
            }
        }
#endif
    }
}

static void task_1000ms()
{
    if (systime_is_timer_elapse(&m_timer_1000ms, 1000))
    {
        systime_stop(&m_timer_1000ms);
        systime_create(&m_timer_1000ms);
        monitor_pair_timeout(NULL);
        
        if (m_lora_tx_busy > 30)
        {
            APP_DBG_ERR("Error\r\n");
            app_lora_reset();
//            NVIC_SystemReset();
        }
        
        if (systime_is_timer_started(&m_timer_keep_alive) 
            && m_net_info.network_id 
            && m_pending_keep_alive)
        {
            uint32_t remain = systime_get_timer_elapsed_time(&m_timer_keep_alive);
            if (m_keep_alive_interval > remain)
            {
                uint32_t elapse_s = (m_keep_alive_interval - remain)/1000;
                uint32_t hour = elapse_s/3600;
                uint32_t min = (elapse_s-hour*3600)/60;
                APP_DBG_INF("Keep alive in next %02d:%02d:%02d\r\n", hour, min, elapse_s % 60);
            }
        }
        
        uint32_t found_sensor_alarm = 0;
        for (uint32_t i = 0; i < MAX_LIST_SENSOR_ALARM_SUPPORT; i++)
        {
            if (m_list_sensor_alarm[i].auto_remove_alarm)
            {
                m_list_sensor_alarm[i].auto_remove_alarm--;
                if (m_list_sensor_alarm[i].auto_remove_alarm == 0)
                {
                    APP_DBG_WRN("Node id %d -> no more alarm\r\n", m_list_sensor_alarm[i].node_id);
                    m_list_sensor_alarm[i].node_id = 0;
                }
                else
                {
                    found_sensor_alarm++;
                }
            }
        }
        
        if (found_sensor_alarm == 0)
        {
            if (m_remote_sensor_alarm_status)
            {
                APP_DBG_WRN("No more remote sensor alarm\r\n");
            }
            m_remote_sensor_alarm_status = 0;
        }
    }
}


static uint32_t sys_now_ms(void)
{
    return board_hw_get_ms();
}


static void on_new_lora_data(app_lora_event_t event, void *arg, void *header)
{
    APP_DBG_INF("Lora event %d\r\n", (int)event);
	timeout_not_received_msg_reset_lora = TIMEOUT_NOT_RECEIVED_ANY_MESSAGE_MS; 
	timeout_not_received_msg_reset_mcu = 3*TIMEOUT_NOT_RECEIVED_ANY_MESSAGE_MS;
	app_lora_data_t *rx_msg = (app_lora_data_t *)arg;
    switch (event)
    {
        case APP_LORA_EVENT_RESET:
        {
            APP_DBG_ERR("Reset system\r\n");
            NVIC_SystemReset();
        }
        break;

        case APP_LORA_EVENT_TX:
        {
        }
        break;

        case APP_LORA_EVENT_RX:
        {
            led_toggle(0);

            if (m_join_network_state == APP_LORA_PROVISIONING_RECEIVING_NETWORK_INFO 
                && rx_msg->msg_type == APP_LORA_ID_PAIR_OFFER)
            {
                process_join_offer_message(rx_msg->payload, rx_msg->len, header);
            }
            else if (rx_msg->node_id == m_net_info.device_id
                    && rx_msg->device_type == APP_LORA_DEVICE_TYPE_GW)
            {
                if (APP_LORA_ID_ACK_ALARM_TRIGGER == rx_msg->msg_type)
                {
                    APP_DBG_WRN("Alarm -> ACK\r\n");
                    app_lora_stop_process_tx_timeout();
                    systime_stop(&m_timer_keep_alive);
                    m_pending_keep_alive = true;
                    m_keep_alive_interval = 300000;      //5p gui ban tin 1 lan neu co bao chay
                    systime_create(&m_timer_keep_alive);
                }
                else if (APP_LORA_ID_ACK_ALARM_REMOVED == rx_msg->msg_type)
                {
                    APP_DBG_INF("Heartbeat -> ACK\r\n");
                    app_lora_stop_process_tx_timeout();
                    systime_stop(&m_timer_keep_alive);
                    m_pending_keep_alive = true;
                    if (!m_local_alarm_status)
                    {
                        m_keep_alive_interval = 3*3600*1000;      //3 tieng gui ban tin 1 lan
                    }
                    else
                    {
                        m_keep_alive_interval = 5*60*1000;      //5p gui ban tin 1 lan
                    }
                    systime_create(&m_timer_keep_alive);
                    m_retries_after_fire_alarm_stop = 0;
                }
            }
            else if (rx_msg->device_type == APP_LORA_DEVICE_TYPE_GW
                    && rx_msg->msg_type == APP_LORA_ID_BROADCAST_CONTROL)
            {
                APP_DBG_INF("Boardcast mesasge\r\n");
                app_lora_broadcast_alarm_msg_t *msg = (app_lora_broadcast_alarm_msg_t*)rx_msg->payload;
                if (msg->name.sync_alarm != m_net_info.sync_alarm)
                {
                    APP_DBG_INF("Sync alarm changed from %d to %d\r\n", 
                                m_net_info.sync_alarm, msg->name.sync_alarm);
                    m_net_info.sync_alarm = msg->name.sync_alarm;
                    app_flash_store_info(&m_net_info);
                }
                
                if (msg->name.alarm != m_remote_gateway_alarm_status)
                {
                    APP_DBG_INF("Remote alarm changed from %d to %d\r\n", 
                                m_remote_gateway_alarm_status, msg->name.alarm);
                    m_remote_gateway_alarm_status = msg->name.alarm;
                }
                
                if (msg->name.priority_input != m_remote_gateway_io_priority)
                {
                    APP_DBG_INF("Remote priority IO changed from %d to %d\r\n", 
                                m_remote_gateway_io_priority, 
                                msg->name.priority_input);
                    m_remote_gateway_io_priority = msg->name.priority_input;
                    board_hw_control_remote_priority_io(m_remote_gateway_io_priority);
                }
            }
            else if (rx_msg->node_id != m_net_info.device_id        // Message from sensor
                    && rx_msg->node_id != 0
                    && (rx_msg->device_type == APP_LORA_DEVICE_TYPE_SENSOR_SMOKE_DETECTOR
                        || rx_msg->device_type == APP_LORA_DEVICE_TYPE_SENSOR_TEMP_DETECTOR
                        || rx_msg->device_type == APP_LORA_DEVICE_TYPE_SENSOR_TEMP_SMOKE
                        || rx_msg->device_type == APP_LORA_DEVICE_TYPE_SPEAKER))
            {
                // Check if device id exist
                int32_t found_id = -1;
                int32_t found_empty_slot = -1;
                app_lora_heartbeat_msg_t *heartbeat = (app_lora_heartbeat_msg_t*)rx_msg->payload;
                for (uint32_t i = 0; i < MAX_LIST_SENSOR_ALARM_SUPPORT; i++)
                {
                    if (m_list_sensor_alarm[i].node_id == rx_msg->node_id)
                    {
                        found_id = i;
                        if (heartbeat->alarm_value.value)
                        {
                            m_list_sensor_alarm[i].auto_remove_alarm = AUTO_RESET_SENSOR_ALARM_IN_SECOND;
                        }
                        else
                        {
                            m_list_sensor_alarm[i].auto_remove_alarm = 0;
                            m_list_sensor_alarm[i].node_id = 0;
                        }
                    }
                    
                    if (m_list_sensor_alarm[i].node_id == 0 && found_empty_slot == -1)
                    {
                        found_empty_slot = i;
                    }
                }
                
                // Neu bao dong tu sensor, ma sensor lai khong co trong queue -> Chen them vao queue
                if (heartbeat->alarm_value.value
                    && found_id == -1
                    && found_empty_slot != -1)
                {
                    m_list_sensor_alarm[found_empty_slot].auto_remove_alarm = AUTO_RESET_SENSOR_ALARM_IN_SECOND;
                    m_list_sensor_alarm[found_empty_slot].node_id = rx_msg->node_id;
                    APP_DBG_WRN("Insert node id %d to msg queue\r\n", rx_msg->node_id);
                }
            }
            else
            {
                APP_DBG_ERR("Unhandle node id %d\r\n", rx_msg->node_id);
            }
        }
        break;

        default:
            break;
    }
}


static void change_temporary_lora_conf(uint32_t freq, uint8_t bw, uint8_t sf)
{
    APP_DBG_INF("Change lora configuration to %uKHz, bw %u, sf %u\r\n", 
                freq/1000, bw, sf);
    app_lora_reset();
    app_lora_set_temporary_config(freq, bw, sf);
}

static void process_join_offer_message(uint8_t *data, uint16_t len, void *header)
{
    if (len == sizeof(app_lora_join_offer_msg_t))
    {
        uint8_t node_mac[6];
        app_lora_header_t *rx_header = (app_lora_header_t *)header;
        app_lora_join_offer_msg_t *offer = (app_lora_join_offer_msg_t *)data;

        memcpy(node_mac, offer->mac, 6);
        uint8_t offer_id = offer->offer_id & 0x7F; // Bit 7 MSB is boardcast message from gateway =>> ignore it
        

        APP_DBG_INF("Node %02X:%02X:%02X:%02X:%02X:%02X received offer id %d, "
                    "freq %uKhz, output 0x%02X, input 0x%02X\r\n",
                     node_mac[0], node_mac[1], node_mac[2],
                     node_mac[3], node_mac[4], node_mac[5],
                    offer_id,
                    offer->freq/1000,
                    offer->output,
                    offer->input);

        if (memcmp(node_mac, m_slave_mac, 6) == 0)
        {            
            m_net_info.flag = APP_FLASH_FLAG_WRITTEN;
            m_net_info.device_id = offer_id;
            m_net_info.network_id = rx_header->network_id;
            m_net_info.freq = offer->freq;
            m_net_info.sync_alarm = offer->sync_alarm;
            app_lora_set_network_id(rx_header->network_id);

            m_join_network_state = APP_LORA_PROVISIONING_STATE_IDLE;
            m_continues_pair_request = false;
            m_pending_keep_alive = true;

            app_flash_store_info(&m_net_info);

            app_led_blink(0, 6, 250, 250);

            systime_create(&m_timer_keep_alive);
            APP_DBG_WRN("Network joined, offer id %d\r\n", offer_id);
            m_keep_alive_interval = (m_net_info.device_id + 1) * 3000;
            change_temporary_lora_conf(LORA_RF_FREQUENCY_NORMAL, 
                                        LORA_BANDWIDTH_OPTIMIZE_RANGE, 
                                        LORA_SPREADING_FACTOR_OPTIMIZE_RANGE);
        }
    }
    else
    {
        APP_DBG_WRN("Invalid pair offer message\r\n");
    }
}

static void on_new_unknown_lora_data_received(app_lora_event_t event, void *arg, void *header)
{
    app_lora_data_t *rx_msg = (app_lora_data_t *)arg;
    APP_DBG_INF("Unknown authen lora data, event %d, len %d\r\n", (int)event, rx_msg->len);
	timeout_not_received_msg_reset_lora = TIMEOUT_NOT_RECEIVED_ANY_MESSAGE_MS;
	timeout_not_received_msg_reset_mcu = 3*TIMEOUT_NOT_RECEIVED_ANY_MESSAGE_MS;
	
    if (m_join_network_state == APP_LORA_PROVISIONING_RECEIVING_NETWORK_INFO 
        && rx_msg->msg_type == APP_LORA_ID_PAIR_OFFER)
    {
        process_join_offer_message(rx_msg->payload, rx_msg->len, header);
    }
}

extern void RadioOnDioIrq(void);
static uint32_t m_tick_poll_lora_spi = 0;
static void task_lora_rx(void *arg)
{
    static uint32_t m_last_tick = 0;
    if (sys_now_ms() != m_last_tick)
    {
        if (sys_now_ms() - m_tick_poll_lora_spi >= (uint32_t)20)
        {
            RadioOnDioIrq();
            m_tick_poll_lora_spi = sys_now_ms();
        }
        
        m_last_tick = sys_now_ms();
        
        if (app_lora_is_ready_to_tx())
        {
            m_lora_tx_busy = 0;
            if (m_continues_pair_request)
            {
                APP_DBG_WRN("Send pair request\r\n");
                app_lora_alarm_type_t heartbeat_msg;
                app_lora_data_t data;

                // Build heartbeat msg
                heartbeat_msg.value = 0;
                heartbeat_msg.name.io_alarm_input0 = (m_local_alarm_status & (1 << ALARM_BIT_BUTTON_SOS)) ? 1 : 0;
   
                
                uint8_t payload[sizeof(app_lora_alarm_type_t) + sizeof(m_slave_mac)];
                memcpy(payload, m_slave_mac, 6);
                memcpy(payload +6, (uint8_t*)&heartbeat_msg, sizeof(app_lora_alarm_type_t));
                
                data.device_type = APP_LORA_DEVICE_TYPE_SPEAKER;
                data.node_id = APP_LORA_NETWORK_ID_INVALID;
                data.msg_type = APP_LORA_ID_SLAVE_WANT_TO_JOIN;
                data.len = sizeof(payload);
                data.payload = payload;
                if (!app_lora_master_tx(&data, on_lora_tx_event, 0, 2000, NULL))
                {
                    APP_DBG_ERR("Send LoRa tx message failed\r\n");
                }
            }
            else if (m_slave_reply_composion_data.new_msg)
            {
                APP_DBG_INF("Send current network info\r\n");

                app_lora_data_t data;
                data.msg_type = APP_LORA_ID_PAIR_GET_COMPOSTION_DATA;
                data.len = sizeof(m_slave_reply_composion_data) - sizeof(((provision_slave_reply_t *)0)->new_msg);
                data.payload = (uint8_t *)&m_slave_reply_composion_data;

                if (app_lora_master_tx(&data, on_lora_tx_event, 0, 3000, NULL))
                {
                    m_slave_reply_composion_data.new_msg = 0;
                }
                else
                {
                    APP_DBG_ERR("Send LoRa tx message failed\r\n");
                }
            }
            else if (m_join_network_state == APP_LORA_PROVISIONING_PROCESSING_OFFER)
            {
                send_pair_info();
            }
            else if (m_pending_keep_alive)
            {       
                if (systime_is_timer_elapse(&m_timer_keep_alive, m_keep_alive_interval))
                {
                    if (keep_alive())
                    {
                        systime_stop(&m_timer_keep_alive);
                        if (m_retries_after_fire_alarm_stop == 0)
                        {
                            m_keep_alive_interval = 3*3600*1000;
                        }
                        else
                        {
                            m_retries_after_fire_alarm_stop--;
                            m_keep_alive_interval = 10*1000;
                        }
                        systime_create(&m_timer_keep_alive);
                    }
                }
            }
        }
        else if (m_continues_pair_request)
        {
            static uint32_t i = 0;
            if (i++ == 10000)
            {
                i = 0;
                APP_DBG_INF("In pair mode but tx busy\r\n");
            }
        }
        
        if (m_lora_tx_busy > 6)
        {
            m_lora_tx_busy = 0;
            APP_DBG_INF("TX busy for along time\r\n");
            app_lora_reset();
        }

        app_lora_poll();
    }
}

static void on_lora_tx_event(app_lora_err_t err, void *arg)
{
    switch (err)
    {
        case APP_LORA_ERR_TX_FINISH:
        {
            APP_DBG_INF("TX success\r\n");
            app_lora_reset();
        }
        break;

        case APP_LORA_ERR_TX_TIMEOUT:
        {
            APP_DBG_ERR("TX not received ack\r\n");
            app_lora_reset();
            systime_stop(&m_timer_keep_alive);
            m_pending_keep_alive = true;
            if (!m_local_alarm_status)
            {
                m_keep_alive_interval = 3*3600*1000;      //3 tieng gui ban tin 1 lan
            }
            else
            {
                m_keep_alive_interval = 15*1000;      //15s gui ban tin 1 lan
            }
            systime_create(&m_timer_keep_alive);
        }
        break;
        default:
            break;
    }
}

static bool keep_alive(void)
{
    app_lora_alarm_type_t heartbeat_msg;
    app_lora_data_t data;
    uint8_t retry = 0;
    // Build heartbeat msg
    heartbeat_msg.value = 0;
    heartbeat_msg.name.io_alarm_input0 = (m_local_alarm_status & (1 << ALARM_BIT_BUTTON_SOS)) ? 1 : 0;
    
    data.node_id = m_net_info.device_id;
    data.msg_type = APP_LORA_ID_KEEP_ALIVE;
    data.device_type = APP_LORA_DEVICE_TYPE_SPEAKER;
    data.len = sizeof(heartbeat_msg);
    data.payload = (uint8_t*)&heartbeat_msg;

    APP_DBG_INF("Send heartbeat\r\n");
    if (heartbeat_msg.value)
    {
        retry = 10;
    }
    if (m_net_info.network_id == 0 
        || app_lora_master_tx(&data, on_lora_tx_event, retry, 3500, NULL) == false)
    {
        APP_DBG_ERR("Send keep alive message failed\r\n");
        return false;
    }
    else
    {
        APP_DBG_VERBOSE("Send keep alive success\r\n");
    }

    return true;
}

void enter_state_get_network_information(void)
{
    APP_DBG_INF("Enter state get network information\r\n");
    m_continues_pair_request = true;
    m_join_network_state = APP_LORA_PROVISIONING_RECEIVING_NETWORK_INFO;
    systime_create(&m_timer_pair);
    app_lora_set_network_id(APP_LORA_NETWORK_ID_INVALID);
    change_temporary_lora_conf(LORA_RF_FREQUENCY_PAIR, 
                                LORA_BANDWIDTH_OPTIMIZE_SPEED, 
                                LORA_SPREADING_FACTOR_OPTIMIZE_SPEED);
}

bool enter_state_send_current_network_information_to_gateway(void)
{
    APP_DBG_INF("%s\r\n", __FUNCTION__);
    if (m_net_info.network_id)
    {
        m_join_network_state = APP_LORA_PROVISIONING_PROCESSING_OFFER;
        systime_create(&m_timer_pair);
        return true;
    }
    else
    {
        APP_DBG_ERR("Device not provisioned\r\n");
    }
    return false;
}

void send_pair_info(void)
{
    APP_DBG_INF("Send pair info to Gateway\r\n");

    if (m_net_info.network_id && m_net_info.device_id != DEVICE_ID_INVALID)
    {
        m_slave_reply_composion_data.network_id = m_net_info.network_id;
        m_slave_reply_composion_data.offer_id = m_net_info.device_id;
        memcpy(m_slave_reply_composion_data.mac, m_slave_mac, 6);
        m_slave_reply_composion_data.new_msg = 1;
    }
    else
    {
        APP_DBG_ERR("Device not provisioned\r\n");
    }
}

static void button_init(void)
{
    static app_btn_hw_config_t m_hw_button_initialize_value[APP_BTN_MAX_BTN_SUPPORT];

    for (uint32_t i = 0; i < APP_BTN_MAX_BTN_SUPPORT; i++)
    {
        m_hw_button_initialize_value[i].idle_level = 1;
        m_hw_button_initialize_value->last_state = board_hw_button_read(i);
        m_hw_button_initialize_value->pin = i;
    }

    app_btn_config_t conf;
    conf.config = &m_hw_button_initialize_value[0];
    conf.btn_count = APP_BTN_MAX_BTN_SUPPORT;
    conf.get_tick_cb = sys_now_ms;
    conf.btn_initialize = NULL;
    conf.btn_read = board_hw_button_read;

    app_btn_initialize(&conf);
    app_btn_register_callback(APP_BTN_EVT_PRESSED, on_button_event_cb, NULL);
    app_btn_register_callback(APP_BTN_EVT_RELEASED, on_button_event_cb, NULL);
    app_btn_register_callback(APP_BTN_EVT_TRIPLE_CLICK, on_button_event_cb, NULL);
}


void on_button_event_cb(int button_pin, int event, void *data)
{
    static const char *event_name[] = {"pressed", "release", "hold", 
                                        "hold so long", "double click", "tripple click", 
                                        "idle", "idle break", "invalid"};
    APP_DBG_WRN("Button %s %s\r\n", (button_pin == ALARM_BIT_BUTTON_SOS) ?  "SOS" : "PAIR", event_name[event]);
    switch (event)
    {
    case APP_BTN_EVT_PRESSED:
        if (button_pin == ALARM_BIT_BUTTON_SOS)        // buton control relay
        {
            m_pending_keep_alive = true;
            m_keep_alive_interval = 1000;     // delay for safety reason, maybe heavy noise in payload we decrease RF performance
            systime_stop(&m_timer_keep_alive);
            systime_create(&m_timer_keep_alive);
             
            m_local_alarm_status |= (1 << ALARM_BIT_BUTTON_SOS);
            systime_stop(&m_timer_1000ms);
            systime_create(&m_timer_1000ms);
        }
        else    // buton pair
        {
            enter_state_get_network_information();
            app_led_blink(0, 8, 50, 50);
        }
        break;

    case APP_BTN_EVT_RELEASED:
        if (button_pin == ALARM_BIT_BUTTON_SOS)
        {
            m_retries_after_fire_alarm_stop = 3;
            m_local_alarm_status &= ~(1 << ALARM_BIT_BUTTON_SOS);
            m_keep_alive_interval = 2000;     // delay for safety reason, maybe heavy noise in payload we decrease RF performance
            if (!systime_is_timer_started(&m_timer_keep_alive))
            {
                APP_DBG_INF("Create keep alive timer\r\n");
                m_pending_keep_alive = true;
                systime_create(&m_timer_keep_alive);
            }
            app_lora_stop_process_tx_timeout();
        }
        break;

    case APP_BTN_EVT_HOLD:
        break;

    case APP_BTN_EVT_DOUBLE_CLICK:
    case APP_BTN_EVT_TRIPLE_CLICK:
    {
    }
    break;

    case APP_BTN_EVT_HOLD_SO_LONG:
    {
//        if (enter_state_send_current_network_information_to_gateway())
//        {
//            app_led_blink(0, 8, 200, 200);
//        }
    }
    break;

    default:
        APP_DBG_ERR("[%s] Unhandle button event %d\r\n", __FUNCTION__, event);
        break;
    }
    
    // APP_DBG_INF("Relay feedback %s\r\n", board_hw_read_latch_feedback(BOARD_HW_RELAY_0) ? "on" : "off");
}

static void led_blink_done(uint8_t led_pin)
{
    if (led_pin == 0)
    {
        if (m_join_network_state != APP_LORA_PROVISIONING_STATE_IDLE)
        {
            LED_PAIR_BLINK();
        }
        else
        {
//            bool err = false;
//            if ((m_relay_value.name.get_back_value != m_relay_value.name.set_value) 
//                || m_relay_value.name.phase_lost)
//            {
//                err = true;
//            }

//            if (!err)
//            {
//                gpio_bit_write(GPIOC, GPIO_PIN_13, m_relay_value.name.set_value ? 0 : 1);
//            }
//            else
//            {
////                DEBUG_PRINTF("Blink led\r\n");
//                LED_CONTACTOR_ERROR_BLINK();
//            }
        }
    }
}

uint32_t led_get(uint32_t pin)
{
    return board_hw_led_get(pin);
}

void led_set(uint32_t pin, uint32_t value)
{
    board_hw_led_set(pin, value);
}

static void led_toggle(uint32_t pin)
{
    board_hw_led_toggle(pin);
}

static void led_init(void)
{
    static app_led_index_t led_hw[2];
    led_hw[0].idx = 0;
    led_hw[0].pin = 0;

    led_hw[1].idx = 1;
    led_hw[1].pin = 1;
    
    app_led_status_cfg_t led_cfg;
    led_cfg.tick_cb = sys_now_ms;
    led_cfg.post_blink_cb = led_blink_done;
    led_cfg.toggle = led_toggle;
    led_cfg.get = led_get;
    led_cfg.set = led_set;
    led_cfg.led_idx = &led_hw[0];
    led_cfg.led_cnt = 1;
    led_cfg.blink_enable = true;
    app_led_initialize(&led_cfg);
}

static void ringbuffer_uart_initialize(void)
{
    static uint8_t uart_buffer[64];
    lwrb_init(&m_ringbuff, uart_buffer, sizeof(uart_buffer));
}

bool ringbuffer_insert(uint8_t data)
{
    return lwrb_write(&m_ringbuff, &data, 1) ? true : false;
}

static bool lora_read_pin(uint32_t pin)
{
//    if (pin == 4)       // LORA_HAL_BUSY_PIN
//    {
//        return gpio_input_bit_get(GPIOB, GPIO_PIN_13);
//    }
//    else
//    {
//        APP_DBG_ERR("Invalid pin %d\r\n", pin);
//    }
    return false;
}

static void task_led_btn(void *arg)
{
    if (systime_is_timer_elapse(&m_timer_led_btn, (uint32_t)arg))
    {
        systime_create(&m_timer_led_btn);
//        m_relay_value.name.set_value = board_hw_read_latch_feedback(BOARD_HW_RELAY_0) ? 1 : 0;
        app_btn_scan(NULL);
        app_led_blink_scan();
    }
}

void app_debug_uart_print(uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        board_hw_debug_uart_send(data, len);
    }
}

void RadioControlTxEnAntennaSwitch(bool tx_en)
{
//    if (tx_en)
//    {
//        gpio_bit_set(GPIOB, GPIO_PIN_0);
//    }
//    else
//    {
//        gpio_bit_reset(GPIOB, GPIO_PIN_0);
//    }
}


bool SX126xWaitOnBusy(uint32_t ms)
{
    while (board_hw_get_lora_busy_pin() && ms)
    {
        board_hw_delay_ms(1);
        app_wdt_feed();
        ms--;
    }
    
    if (board_hw_get_lora_busy_pin())
    {
        APP_DBG_ERR("SX126X wait busy error\r\n");
        return false;
    }

    return true;
}

void SX126xReset(void)
{
//    DEBUG_PRINTF("SX126X reset\r\n");
//#ifdef GD32E230
//    gpio_bit_reset(GPIOB, GPIO_PIN_12);
//#else   // esp32
//    DEBUG_PRINTF("ESP32 reset sx126x MCU, implement later\r\n");
//#endif
//    Delayms(5);
//#ifdef GD32E230  
//    gpio_bit_set(GPIOB, GPIO_PIN_12);
//#else   // esp32
//#endif
//    Delayms(5);
}

void systime_update()
{
    systime_set_value(uwTick);
}

void factory_reset(void)
{
    memset(&m_net_info, 0, sizeof(m_net_info));
    app_flash_store_info(&m_net_info);
    board_hw_delay_ms(100);
    NVIC_SystemReset();
}

void system_irq_control(bool en)
{
	
}

void rtt_puts(uint8_t *buffer, uint32_t size)
{
    board_hw_debug_uart_send(buffer, size);
    SEGGER_RTT_Write(0, (const void*)buffer, size);
}

int rtt_print(const char *msg)
{
    int len = strlen(msg);
    board_hw_debug_uart_send(msg, len);
    SEGGER_RTT_Write(0, (const void*)msg, len);
    return len;
}

uint32_t debug_puts(const void *buffer, uint32_t size)
{
    SEGGER_RTT_Write(0, buffer, size);
    board_hw_debug_uart_send(buffer, size);
    return size;
}

void sos_fire_alarm_led_indicator_poll(void)
{
    if (m_local_alarm_status || m_remote_gateway_alarm_status || m_remote_sensor_alarm_status)
    {
        app_led_blink(1, 0xFFFFFFFF, 1000, 1000);
        board_hw_set_buzzer(1);
    }
    else
    {
        if (app_led_get_blink_cnt(1))
        {
            app_led_stop_blink(1);
        }
        board_hw_set_buzzer(0);
    }
}
