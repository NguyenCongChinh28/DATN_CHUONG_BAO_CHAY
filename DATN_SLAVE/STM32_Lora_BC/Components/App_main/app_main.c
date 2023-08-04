#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "main.h"
#include "app_spi.h"
#include "app_debug.h"
#include "board_hw.h"
#include "SEGGER_RTT.h"
#include "app_wdt.h"
#include "app_flash.h"
#include "systime.h"
#include "app_lora.h"
#include "lora_hal.h"





#define DEVICE_ID_INVALID                       0xFF

#define LORA_RF_FREQUENCY_NORMAL               (c_net_info.freq) //Hz
#define LORA_AUTO_RESET_MS                     (1000*3600)
#define LORA_MODERM_FAIL_RESET_MS              (20000)
#define TIMEOUT_NOT_RECEIVED_ANY_MESSAGE_S     (86400)
#define KEEP_ALIVE_INTERVAL_MS_IN_ALARM_MODE   (60*1000)
#define LORA_PERFORMANCE_TEST               1
#if LORA_PERFORMANCE_TEST
#define KEEP_ALIVE_INTERVAL_MS_IN_NORMAL_MODE (5*1000)
#else
#define KEEP_ALIVE_INTERVAL_MS_IN_NORMAL_MODE (3*3600*1000)
#endif

#define MAX_LIST_SENSOR_ALARM_SUPPORT   32
#define AUTO_RESET_SENSOR_ALARM_IN_SECOND   (1800)

#define ALARM_BIT_BUTTON_SOS    0
#define ALARM_BIT_LORA_SOS      1





uint32_t debug_puts(const void *buffer,uint32_t len);
static uint8_t c_mac_slave[6];
static const uint8_t invalid_mac[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};


static systime_t c_timer_wdt;
static systime_t c_timer_keep_alive;
static systime_t c_timer_1000ms;

static app_flash_network_info_t c_net_info __attribute__((aligned(4)));
static uint32_t sys_now_ms(void);
static void on_new_data_lora(app_lora_event_t event, void *arg, void *header);
static void on_new_unknown_data_lora(app_lora_event_t event, void *arg, void *header);
volatile uint32_t timeout_not_received_msg_reset_lora = TIMEOUT_NOT_RECEIVED_ANY_MESSAGE_S;
app_lora_provisioning_state_t c_join_network_state = APP_LORA_PROVISIONING_STATE_IDLE;
static void process_join_offer_message(uint8_t *data, uint16_t len, void *header);

static bool c_pending_keep_alive = false;
static uint32_t c_keep_alive_interval = 0;
static uint32_t c_local_alarm_status = 0;
static uint8_t c_retries_after_fire_alarm_stop = 0;
static uint32_t c_remote_gateway_alarm_status = 0;
static uint32_t c_remote_gateway_io_priority = 0;
static app_lora_list_alarm_device_t c_list_sensor_alarm[MAX_LIST_SENSOR_ALARM_SUPPORT];
static void change_temporary_lora_conf(uint32_t freq, uint8_t bw, uint8_t sf);
static bool c_continues_pair_request = false;


static void task_lora_rx(void *arg);

volatile uint8_t c_lora_tx_busy = 0;


static void on_lora_tx_event(app_lora_err_t err, void *arg);









uint32_t c_ping_counter = 0;
uint32_t c_rx_counter = 0;



void app_main(void)
{
    //init app debug
    app_dbg_init(Get_MS,NULL);
    app_dbg_register_callback_print(debug_puts);
    
    
    
    systime_create(&c_timer_wdt);
    app_wdt_feed();//app watchdog
    
    
    APP_DBG_INF(" SYSTEM RESET\r\n");
    
    app_get_mac_add_in_flash(c_mac_slave);
    APP_DBG_INF(" MAC slave %02X:%02X:%02X:%02X:%02X:%02X \r\n",
                c_mac_slave[0],c_mac_slave[1],c_mac_slave[2],
                c_mac_slave[3],c_mac_slave[4],c_mac_slave[5]);
    
    if (app_net_infor_is_exist_in_flash())
    {
        app_read_net_info_in_flash(&c_net_info);
        APP_DBG_INF("Network id %d, Device id %d, Freq %d \r\n",
                    c_net_info.network_id,
                    c_net_info.device_id,
                    c_net_info.freq);
    }
    else
    {
        c_net_info.network_id = 0;
        c_net_info.device_id = DEVICE_ID_INVALID;
        c_net_info.freq = APP_LORA_RF_FREQUENCY_PAIR;
        APP_DBG_INF(" Device is not provisioning\r\n");
        
    }
    
    // Config Lora driver
    app_lora_cfg_t lora_config;
    lora_config.lora_hw_reset = board_hw_lora_reset_pin;
    lora_config.delay_ms = board_hw_delay_ms;
    lora_config.spi_tx = app_spi_tx;
    lora_config.spi_rx = app_spi_rx;
    lora_config.tx_power = APP_LORA_TX_OUTPUT_POWER;
    lora_config.freq = LORA_RF_FREQUENCY_NORMAL;
    lora_config.sf = APP_LORA_SF_OPTIMIZE_RANGE;
    lora_config.cr = APP_LORA_CODINGRATE;
    lora_config.bw = APP_LORA_BW_OPTIMIZE_RANGE;
    lora_config.preamble_length = APP_LORA_PREAMBLE_LENGTH;
    lora_config.get_ms = sys_now_ms;
    lora_config.period_reset_ms_when_moderm_good = LORA_AUTO_RESET_MS;
    lora_config.period_reset_ms_when_moderm_fail = LORA_MODERM_FAIL_RESET_MS;
    lora_config.rx_idle_timeout_ms = 4000;
    lora_config.startup_ms =50;
    lora_config.event = on_new_data_lora;
    lora_config.unauthen_data_event = on_new_unknown_data_lora;
    app_lora_initialize(&lora_config);
    
    app_lora_set_network_id(c_net_info.network_id);  //ID of network
    APP_DBG_INF("Set network id = %d\r\n",c_net_info.network_id);
    while(Get_MS()==0)
    {
        
    }
    c_keep_alive_interval = (c_net_info.device_id + 1) *3000;
    systime_create(&c_timer_keep_alive);
    c_pending_keep_alive = true;
    
    systime_create(&c_timer_1000ms);
    //systime_create(c_timer_led_btn);
    
    while (1)
    {
        task_lora_rx(NULL);
        
    }
    
      
      
      
}
uint32_t debug_puts(const void *buffer,uint32_t len)
{
    SEGGER_RTT_Write(0,buffer,len);
    board_hw_debug_uart_send(buffer,len);
    return len; 
}
void systime_update()
{
    systime_set_value(uwTick);
}
static uint32_t sys_now_ms(void)
{
    return Get_MS();
}

static void on_new_data_lora(app_lora_event_t event, void *arg, void *header)
{
    
    APP_DBG_INF("Lora event %d\r\n", (int)event);
	timeout_not_received_msg_reset_lora = TIMEOUT_NOT_RECEIVED_ANY_MESSAGE_S; 
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
            //led_toggle(0);

            if (c_join_network_state == APP_LORA_PROVISIONING_RECEIVING_NETWORK_INFO 
                && rx_msg->msg_type == APP_LORA_ID_PAIR_OFFER)
            {
                process_join_offer_message(rx_msg->payload, rx_msg->len, header);
            }
            else if (memcmp(rx_msg->mac, c_mac_slave, 6) == 0
                    && rx_msg->device_type == APP_LORA_DEVICE_TYPE_GW)
            {
                if (APP_LORA_ID_ACK_ALARM_TRIGGER == rx_msg->msg_type)
                {
                    APP_DBG_WRN("Alarm -> ACK\r\n");
                    app_lora_stop_process_tx_timeout();
                    systime_stop(&c_timer_keep_alive);
                    c_pending_keep_alive = true;
                    c_keep_alive_interval = KEEP_ALIVE_INTERVAL_MS_IN_ALARM_MODE;      //5p gui ban tin 1 lan neu co bao chay
                    systime_create(&c_timer_keep_alive);
                }
                else if (APP_LORA_ID_ACK_ALARM_REMOVED == rx_msg->msg_type)
                {
                    ++c_rx_counter;
                    APP_DBG_INF("Heartbeat -> ACK, rx counter %u/%u\r\n", c_rx_counter, c_ping_counter);
                    app_lora_stop_process_tx_timeout();
                    systime_stop(&c_timer_keep_alive);
                    c_pending_keep_alive = true;
                    if (!c_local_alarm_status)
                    {
                        c_keep_alive_interval = KEEP_ALIVE_INTERVAL_MS_IN_NORMAL_MODE;      //3 tieng gui ban tin 1 lan
                    }
                    else
                    {
                        c_keep_alive_interval = KEEP_ALIVE_INTERVAL_MS_IN_ALARM_MODE;      //5p gui ban tin 1 lan
                    }
                    systime_create(&c_timer_keep_alive);
                    c_retries_after_fire_alarm_stop = 0;
                }
            }
            else if (rx_msg->device_type == APP_LORA_DEVICE_TYPE_GW
                    && rx_msg->msg_type == APP_LORA_ID_BROADCAST_CONTROL)
            {
                APP_DBG_INF("Boardcast mesasge\r\n");
                app_lora_broadcast_alarm_msg_t *msg = (app_lora_broadcast_alarm_msg_t*)rx_msg->payload;
                if (msg->name.sync_alarm != c_net_info.sync_alarm)
                {
                    APP_DBG_INF("Sync alarm changed from %d to %d\r\n", 
                                c_net_info.sync_alarm, msg->name.sync_alarm);
                    c_net_info.sync_alarm = msg->name.sync_alarm;
                    app_store_info_in_flash(&c_net_info);
                }
                
                if (msg->name.alarm != c_remote_gateway_alarm_status)
                {
                    APP_DBG_INF("Remote alarm changed from %d to %d\r\n", 
                                c_remote_gateway_alarm_status, msg->name.alarm);
                    c_remote_gateway_alarm_status = msg->name.alarm;
                }
                
                if (msg->name.priority_input != c_remote_gateway_io_priority)
                {
                    APP_DBG_INF("Remote priority IO changed from %d to %d\r\n", 
                                c_remote_gateway_io_priority, 
                                msg->name.priority_input);
                    c_remote_gateway_io_priority = msg->name.priority_input;
                    board_hw_control_remote_priority_io(c_remote_gateway_io_priority);
                }
            }
            else if (memcmp(rx_msg->mac, c_mac_slave, 6)        // Message from sensor
                    && (   rx_msg->device_type == APP_LORA_DEVICE_TYPE_SENSOR_SMOKE_DETECTOR
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
                    if (memcmp(c_list_sensor_alarm[i].mac, rx_msg->mac, 6) == 0)
                    {
                        found_id = i;
                        if (heartbeat->alarm.value)
                        {
                            c_list_sensor_alarm[i].auto_remove_alarm = AUTO_RESET_SENSOR_ALARM_IN_SECOND;
                        }
                        else
                        {
                            c_list_sensor_alarm[i].auto_remove_alarm = 0;
                            memset(c_list_sensor_alarm[i].mac, 0, 6);
                        }
                    }
                    
                    if (memcmp(c_list_sensor_alarm[i].mac, invalid_mac, 6) == 0 && found_empty_slot == -1)
                    {
                        found_empty_slot = i;
                    }
                }
                
                // Neu bao dong tu sensor, ma sensor lai khong co trong queue -> Chen them vao queue
                if (heartbeat->alarm.value
                    && found_id == -1
                    && found_empty_slot != -1)
                {
                    c_list_sensor_alarm[found_empty_slot].auto_remove_alarm = AUTO_RESET_SENSOR_ALARM_IN_SECOND;
                    memcpy(c_list_sensor_alarm[found_empty_slot].mac, rx_msg->mac, 6);
                    APP_DBG_WRN("Insert node %02X:%02X:%02X:%02X:%02X:%02X to queue\r\n",
                                c_list_sensor_alarm[found_empty_slot].mac[0],
                                c_list_sensor_alarm[found_empty_slot].mac[1],
                                c_list_sensor_alarm[found_empty_slot].mac[2],
                                c_list_sensor_alarm[found_empty_slot].mac[3],
                                c_list_sensor_alarm[found_empty_slot].mac[4],
                                c_list_sensor_alarm[found_empty_slot].mac[5]);
                }
            }
            else
            {
                APP_DBG_WRN("Unhandle node %02X:%02X:%02X:%02X:%02X:%02X to queue\r\n",
                                rx_msg->mac[0],
                                rx_msg->mac[1],
                                rx_msg->mac[2],
                                rx_msg->mac[3],
                                rx_msg->mac[4],
                                rx_msg->mac[5]);
            }
        }
        break;

        default:
            break;
    }
}


static void process_join_offer_message(uint8_t *data, uint16_t len, void *header)
{
    if (len == sizeof(app_lora_join_offer_msg_t))
    {
        uint8_t node_mac[6];
        app_lora_header_t *rx_header = (app_lora_header_t *)header;
        app_lora_join_offer_msg_t *offer = (app_lora_join_offer_msg_t *)data;

        memcpy(node_mac, offer->mac, 6);

       
        APP_DBG_INF("Node %02X:%02X:%02X:%02X:%02X:%02X received offer, "
                    "freq %uKhz, output 0x%02X, input 0x%02X\r\n",
                     node_mac[0], node_mac[1], node_mac[2],
                     node_mac[3], node_mac[4], node_mac[5],
                    offer->freq/1000,
                    offer->output,
                    offer->input);

        if (memcmp(node_mac, c_mac_slave, 6) == 0)
        {            
            c_net_info.flag = APP_FLASH_FLAG_WRITTEN;
            c_net_info.network_id = rx_header->network_id;
            c_net_info.freq = offer->freq;
            c_net_info.sync_alarm = offer->sync_alarm;
            app_lora_set_network_id(rx_header->network_id);

            c_join_network_state = APP_LORA_PROVISIONING_STATE_IDLE;
            c_continues_pair_request = false;
            c_pending_keep_alive = true;

            app_store_info_in_flash(&c_net_info);

            //app_led_blink(0, 6, 250, 250);

            systime_create(&c_timer_keep_alive);
            APP_DBG_WRN("Network joined\r\n");
            c_keep_alive_interval = (c_net_info.device_id + 1) * 3000;
            change_temporary_lora_conf(LORA_RF_FREQUENCY_NORMAL, 
                                       APP_LORA_BW_OPTIMIZE_RANGE, 
                                       APP_LORA_SF_OPTIMIZE_RANGE);
        }
    }
    else
    {
        APP_DBG_WRN("Invalid pair offer message\r\n");
    }
}


static void change_temporary_lora_conf(uint32_t freq, uint8_t bw, uint8_t sf)
{
    APP_DBG_INF("Change lora configuration to %uKHz, bw %u, sf %u\r\n", 
                freq/1000, bw, sf);
    app_lora_reset();
    app_lora_set_temporary_config(freq, bw, sf);
}
static void on_new_unknown_data_lora(app_lora_event_t event, void *arg, void *header)
{
    app_lora_data_t *rx_msg = (app_lora_data_t *)arg;
    APP_DBG_INF("Unknown authen lora data, event %d, len %d\r\n", (int)event, rx_msg->len);
	timeout_not_received_msg_reset_lora = TIMEOUT_NOT_RECEIVED_ANY_MESSAGE_S;
	
    if (c_join_network_state == APP_LORA_PROVISIONING_RECEIVING_NETWORK_INFO 
        && rx_msg->msg_type == APP_LORA_ID_PAIR_OFFER)
    {
        process_join_offer_message(rx_msg->payload, rx_msg->len, header);
    }
    
    
}

extern void RadioOnDioIrq(void);
static uint32_t c_tick_poll_lora_spi = 0;
static void task_lora_rx(void *arg)
{
    static uint32_t c_last_tick = 0;
    if (sys_now_ms() != c_last_tick)
    {
        if(sys_now_ms()-c_tick_poll_lora_spi >=20)
        {
            RadioOnDioIrq();
            c_tick_poll_lora_spi = sys_now_ms();
            APP_DBG_INF("Tick poll lora spi = %d\r\n",c_tick_poll_lora_spi);
        }
        c_last_tick = sys_now_ms();
        
        if (app_lora_is_ready_to_tx())
        {
            c_lora_tx_busy = 0;
            if (c_continues_pair_request)
            {
                APP_DBG_WRN("Send pair request \r\n");
                app_lora_alarm_type_t heartbeat_msg;
                
                
                //Build heartbeat msg
                heartbeat_msg.value = 0;
                heartbeat_msg.name.io_alarm_input0 =  (c_local_alarm_status & (1 << ALARM_BIT_BUTTON_SOS)) ?  1 : 0;
                
                //Build pair msg
                // battery + type + mac
                app_lora_pair_msg_t  pair_msg;
                pair_msg.alarm = heartbeat_msg;
                pair_msg.battery_percent = board_hw_get_battery_percent(); 
                memcpy(&pair_msg.mac, &c_mac_slave, sizeof(pair_msg));
                
                app_lora_data_t data;
                data.device_type = APP_LORA_DEVICE_TYPE_SPEAKER;
                memset(data.mac, 0, 6);
                data.msg_type = APP_LORA_ID_SLAVE_WANT_TO_JOIN;
                data.len = sizeof(pair_msg);
                data.payload = (uint8_t*)&pair_msg;
                
                if (app_lora_master_tx(&data, on_lora_tx_event, 0, 2000, NULL))
                { 
                    APP_DBG_INF("Lora master tx success\r\n");
                }   
                else
                {
                    APP_DBG_ERR("Send LoRa tx message failed\r\n");
                }
                
            }
        }
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
            systime_stop(&c_timer_keep_alive);
            c_pending_keep_alive = true;
            if (!c_local_alarm_status)
            {
                c_keep_alive_interval = KEEP_ALIVE_INTERVAL_MS_IN_NORMAL_MODE;      //3 tieng gui ban tin 1 lan
            }
            else
            {
                c_keep_alive_interval = KEEP_ALIVE_INTERVAL_MS_IN_ALARM_MODE;      //15s gui ban tin 1 lan
            }
            systime_create(&c_timer_keep_alive);
        }
        break;
        default:
            break;
    }
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