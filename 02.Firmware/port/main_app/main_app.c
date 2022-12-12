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
#include "spi.h"

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

static lwrb_t m_ringbuff;
static void ringbuffer_uart_initialize(void);
void Delayms(uint32_t ms);
volatile uint8_t m_lora_tx_busy = 0;
static uint32_t m_alarm_bit = 0;

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


typedef enum
{
    PAIR_STATE_IDLE,
    PAIR_STATE_PROCESSING_OFFER, // If device is slave, it will send current network information to Gateway
    PAIR_STATE_RECEIVING_NETWORK_INFO,
} provisioning_state_t;

static void task_wdt(uint32_t timeout_ms);
static void task_peripheral(void *arg);
static void task_lora_rx(void *arg);
static void task_cli(void);
static void task_led_btn(void *arg);
static void task_1000ms(void);

static uint32_t sys_now_ms(void);
void set_relay_value(uint8_t value);

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
provisioning_state_t m_join_network_state = PAIR_STATE_IDLE;

static relay_value_t m_relay_value;
static uint8_t m_slave_mac[6]; // TODO generate device id
static provision_slave_reply_t m_slave_reply_composion_data;
static app_flash_network_info_t m_net_info __attribute__((aligned(4)));
static bool lora_read_pin(uint32_t pin);

bool ringbuffer_insert(uint8_t data);
static void led_toggle(uint32_t pin);
static void change_temporary_lora_conf(uint32_t freq, uint8_t bw, uint8_t sf);
static bool m_continues_pair_request = false;
static uint32_t m_keep_alive_delay_ms = 0;
static uint32_t m_delay_monitor_relay_error = 0;
uint32_t reset_reason = 0;
volatile int32_t timeout_not_received_msg_reset_lora = TIMEOUT_NOT_RECEIVED_ANY_MESSAGE_MS;
volatile uint32_t timeout_not_received_msg_reset_mcu = 3*TIMEOUT_NOT_RECEIVED_ANY_MESSAGE_MS;

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
uint16_t m_eeprom_relay_val = 0;
uint16_t m_init_val = 0;
void rtt_puts(uint8_t *buffer, uint32_t size);
uint32_t debug_puts(const void *buffer, uint32_t size);
int rtt_print(const char *msg);
static app_cli_cb_t m_cli = 
{
    .puts = rtt_puts,
    .printf = rtt_print,
};

void main_app(void)
{
    reset_reason = board_hw_get_reset_reason();
    app_wdt_start();
    
    ringbuffer_uart_initialize();
    board_hw_uart_debug_initialize();
    
    app_dbg_init(board_hw_get_ms, NULL);
    app_dbg_register_callback_print(debug_puts);
    APP_DBG_INF("System reset\r\n");
        
    memset(&m_slave_reply_composion_data, 0, sizeof(m_slave_reply_composion_data)); // clear slave reply composition data
    
    APP_DBG_INF("Slave mac addr %02X:%02X:%02X:%02X:%02X:%02X\r\n",
             m_slave_mac[0], m_slave_mac[1], m_slave_mac[2],
             m_slave_mac[3], m_slave_mac[4], m_slave_mac[5]);
    systime_create(&m_timer_wdt);
#if 1
    app_cli_start(&m_cli);//test when no hardware
#endif
    app_spi_initialize();// init spi
    board_hw_init_gpio();    
    app_wdt_feed();//Refresh the IWDG
//    EEPROM_State eeprom_state = EEPROM_Init();
//    APP_DBG_INF("eeprom state %u\r\n", eeprom_state);
//	if (eeprom_state == OPERATION_OK)
//	{
//		eeprom_state = EEPROM_Read(0x00, &m_eeprom_relay_val);
//	}
//	if (m_eeprom_relay_val == 0xFF 
//		|| m_eeprom_relay_val == 0xFFFF)
//	{
//		m_eeprom_relay_val = 0;
//	}
//			
//    if (reset_reason & 0x00800000)
//    {
//        DEBUG_PRINTF("Power on reset\r\n");
//    }
//	if (eeprom_state == OPERATION_OK)
//	{
//		set_relay_value(m_eeprom_relay_val ? 1 : 0);
//	}
//	else
//	{
//		set_relay_value(0);
//	}
	board_hw_relay_status_clear_change();

    // APP_DBG_INF("Eeprom relay init value 0x%04X\r\n", m_eeprom_relay_val);
    APP_DBG_INF("Reset reason 0x%08X\r\n", reset_reason);
    m_init_val = m_eeprom_relay_val;

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
    lora_cfg.event = on_new_lora_data;
    lora_cfg.unauthen_data_event = on_new_unknown_lora_data_received;
    app_lora_init(&lora_cfg);

    app_lora_set_network_id(m_net_info.network_id);

    
    // Update current status, by read the latch input
    // m_relay_value.name.set_value = board_hw_read_latch_feedback(BOARD_HW_RELAY_0) ? 1 : 0;
    
   // gpio_bit_write(GPIOC, GPIO_PIN_13, m_relay_value.name.set_value ? 0 : 1);

//    APP_DBG_INF("Relay feedback %s\r\n", m_relay_value.name.set_value ? "on" : "off");
    
    while (board_hw_get_ms() == 0)
    {
    }
    board_hw_internal_timer_start();
    
    m_keep_alive_delay_ms = (m_net_info.device_id + 1) * 3000;
    systime_create(&m_timer_keep_alive);
    m_pending_keep_alive = true;
    while (1)
    {
        task_lora_rx(NULL);
        task_peripheral(NULL);
//        board_hw_internal_poll();
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

static void task_peripheral(void *arg)
{
    if (m_join_network_state != PAIR_STATE_IDLE)
    {
        if (systime_is_timer_elapse(&m_timer_pair, MAX_PAIR_TIMEOUT_MS))
        {
            APP_DBG_WRN("Stop pair service\r\n");
            change_temporary_lora_conf(LORA_RF_FREQUENCY_NORMAL, LORA_BANDWIDTH_OPTIMIZE_RANGE, LORA_SPREADING_FACTOR_OPTIMIZE_RANGE);
            m_join_network_state = PAIR_STATE_IDLE;
            m_continues_pair_request = false;
        }
    }

//    // m_relay_value.name.phase_lost = board_hw_is_phase_lost() ? 1 : 0;
//    m_relay_value.name.phase_lost = (m_relay_value.name.set_value != board_hw_is_contactor_detect()) ? 1 : 0;
//    m_relay_value.name.get_back_value = board_hw_is_contactor_detect() ? 1 : 0;

//    if (m_relay_value.name.phase_lost && app_led_get_blink_cnt(0) == 0)
//    {
//        LED_CONTACTOR_ERROR_BLINK();
//    }
}

static void task_wdt(uint32_t timeout_ms)
{
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
        task_cli();
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

uint32_t m_contactor_err = 0;
uint32_t test_relay = 0;
uint32_t cnt = 0;
static void task_1000ms()
{
    if (systime_is_timer_elapse(&m_timer_1000ms, 1000) && m_delay_monitor_relay_error == 0)
    {
        systime_stop(&m_timer_1000ms);
        bool status_change = false;
//        if (test_relay > 0)
//        {
//            test_relay--;
//            set_relay_value(cnt++%2);
//            DEBUG_PRINTF("Test relay %u times\r\n", 2000-test_relay);
//        }
        if (m_lora_tx_busy > 30)
        {
            APP_DBG_ERR("Error\r\n");
            app_lora_reset();
//            NVIC_SystemReset();
        }
        
        if (m_contactor_err != CONTACTOR_ERROR())
        {
            m_contactor_err = CONTACTOR_ERROR() ? 1 : 0;
            APP_DBG_ERR("Contactor error flag changed, contactor %d %s\r\n", m_contactor_err, m_contactor_err ? "error" : "ok");
            status_change = true;
        }
        
        if (board_hw_relay_status_change())
        {
            board_hw_relay_status_clear_change();
            status_change = true;
        }
        
//        if (!CONTACTOR_ERROR() && !app_led_get_blink_cnt(0))
//        {
//            gpio_bit_write(GPIOC, GPIO_PIN_13, m_relay_value.name.set_value ? 0 : 1);
//        }
        
        if (status_change)
        {
            APP_DBG_INF("Relay status change\r\n");
            if (m_net_info.network_id)
            {
                m_contactor_err = CONTACTOR_ERROR() ? 1 : 0;
                if (!systime_is_timer_started(&m_timer_keep_alive))
                {
                    systime_create(&m_timer_keep_alive);
                }
                m_pending_keep_alive = true;
                if (m_contactor_err)
                {
                    m_keep_alive_delay_ms = (board_hw_get_ms() % 5) * 1200;     // delay for safety reason, maybe heavy noise in payload we decrease RF performance
                }
                else
                {
                     m_keep_alive_delay_ms = (board_hw_get_ms() % 9) * 1200;     // delay for safety reason, maybe heavy noise in payload we decrease RF performance
                }
            }
        }
        
//        if (m_contactor_err)
//        {
//            DEBUG_PRINTF("Contactor error\r\n");
//        }
        
        if (systime_is_timer_started(&m_timer_keep_alive) 
            && m_net_info.network_id 
            && m_pending_keep_alive)
        {
            uint32_t elapse = m_keep_alive_delay_ms  - systime_get_timer_elapsed_time(&m_timer_keep_alive);
            APP_DBG_INF("Keep alive in %ums\r\n", elapse);
        }
        systime_create(&m_timer_1000ms);
//        if (m_relay_value.name.set_value != get_contactor_value())
//        {
//            APP_DBG_ERR("Phase lost %d - contactor %d, relay set value %d, get value %d\r\n", 
//                        board_hw_is_phase_lost(), 
//                        board_hw_is_contactor_detect(),
//                        m_relay_value.name.set_value,
//                        get_contactor_value());
//        }
        
//        DEBUG_PRINTF("Init val %d\r\n", m_init_val);
    }
}

static uint32_t sys_now_ms(void)
{
    return board_hw_get_ms();
}

//static app_lora_data_t ack_relay_ctrl;

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
    }
    break;

    case APP_LORA_EVENT_TX:
    {
    }
    break;

    case APP_LORA_EVENT_RX:
    {
        led_toggle(0);
        APP_DBG_INF("[%d] LoRa event RX\r\n", sys_now_ms());

        if (m_join_network_state == PAIR_STATE_RECEIVING_NETWORK_INFO 
            && rx_msg->msg_type == APP_LORA_ID_PAIR_OFFER)
        {
            process_join_offer_message(rx_msg->payload, rx_msg->len, header);
        }
        else if (rx_msg->node_id == m_net_info.device_id
                && rx_msg->device_type == APP_LORA_DEVICE_TYPE_GW)
        {
            APP_DBG_INF("Do ack\r\n");
            app_lora_data_t data;
            data.node_id = m_net_info.device_id;
            data.msg_type = APP_LORA_ID_ACK;
            data.len = 1;
			data.payload = &m_relay_value.value;
            if (!app_lora_master_force_tx(&data, NULL, 0, 3000, NULL))
            {
                APP_DBG_INF("Send ack failed\r\n");
            }
        }
//        else if (rx_msg->node_id & (APP_LORA_DEVICE_TYPE_GATE_WAY | APP_LORA_DEVICE_BROADCAST))
//        {
//            APP_DBG_INF("Boardcast message\r\n");
//            if (rx_msg->len >= sizeof(relay_msg_t) 
//                && rx_msg->msg_type == APP_LORA_ID_CONTROL)
//            {
//                uint32_t nb_off_relay_message = rx_msg->len / sizeof(relay_msg_t);
//                bool do_ack = false;

//                APP_DBG_INF("Relay message, nb of relay %d\r\n", nb_off_relay_message);
//                // Payload
//                for (uint32_t i = 0; i < nb_off_relay_message; i++)
//                {
//                    relay_msg_t *relay = (relay_msg_t *)&rx_msg->payload[sizeof(relay_msg_t) * i];
//                    if (relay->node_id == m_net_info.device_id)
//                    {
//                        APP_DBG_INF("Set relay value %d\r\n", relay->level);
//                        set_relay_value(relay->level);
//                        board_hw_delay_ms(60);        // 60ms enough for contactor detect
////                        APP_DBG_INF("Get contactor %d\r\n", get_contactor_value());
////                        if (CONTACTOR_ERROR())
////                        {
////                            APP_DBG_INF("Contactor error\r\n");
////                        }
//                        if (relay->do_ack)
//                        {
//                            do_ack = true;
//                            board_hw_relay_status_clear_change();
//                            m_delay_monitor_relay_error = 65;
//                        }
//                        app_led_blink(0, 2, 50, 50);
//                    }
//                    APP_DBG_INF("Node %d must ack, my id %d\r\n", relay->node_id, m_net_info.device_id);
//                }

//                if (do_ack)
//                {
//                    board_hw_delay_ms(500);
//                    APP_DBG_INF("Do ack broadcast relay message\r\n");
//                    ack_relay_ctrl.node_id = m_net_info.device_id;
//                    ack_relay_ctrl.msg_type = APP_LORA_ID_ACK;
//                    ack_relay_ctrl.len = 1;
//                    ack_relay_ctrl.payload = &m_relay_value.value;
//                    if (!app_lora_master_force_tx(&ack_relay_ctrl, NULL, 0, 3000, NULL))
//                    {
//                        APP_DBG_INF("Send ack failed\r\n");
//                    }
////                    else
////                    {
////                        DEBUG_PRINTF("Relay value %02X\r\n", m_relay_value.value);
////                    }
//                }
//            }
//            else if (rx_msg->msg_type == APP_LORA_ID_KEEP_ALIVE)
//            {
//                APP_DBG_INF("Gateway ping all...\r\n");
//                if (!systime_is_timer_started(&m_timer_keep_alive))
//                {
//                    m_pending_keep_alive = true;
//					m_keep_alive_delay_ms = (board_hw_get_ms() & 0x00B + 1) * 2136;     // random delay
//                    systime_create(&m_timer_keep_alive);
//                }
//            }
//			else if (rx_msg->msg_type == APP_LORA_ID_POLL_ONLY_ONE_NODE)
//			{
//				APP_DBG_INF("Gateway ping only node\r\n");
//				if (rx_msg->len == 0)
//				{
//					uint8_t id = rx_msg->node_id & 0x0F;		// only 4 bit for device id
//					APP_DBG_INF("Gateway ping id %u, my id %d\r\n", id, m_net_info.device_id);
//					if (id == m_net_info.device_id)
//					{
//						APP_DBG_INF("Ping me\r\n");
//						m_pending_keep_alive = true;
//						m_keep_alive_delay_ms = 100;     // random delay
//						systime_create(&m_timer_keep_alive);
//					}
//				}
//			}
//            else
//            {
//                APP_DBG_ERR("Unhandle payload len id %d, msg type %d\r\n",
//                             rx_msg->len,
//                             rx_msg->msg_type);
////                if (1)
////                {
////                    DEBUG_PRINTF("Do ack broadcast relay message\r\n");
////                    app_lora_data_t ack_relay_ctrl;
////                    ack_relay_ctrl.node_id = m_net_info.device_id;
////                    ack_relay_ctrl.msg_type = APP_LORA_ID_ACK;
////                    ack_relay_ctrl.len = 1;
////                    ack_relay_ctrl.payload = &m_relay_value.value;
////                    if (!app_lora_master_force_tx(&ack_relay_ctrl, NULL, 0, 3000, NULL))
////                    {
////                        DEBUG_PRINTF("Send ack failed\r\n");
////                    }
////                }
//            }
//        }
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

void set_relay_value(uint8_t value)
{
    m_relay_value.name.set_value = (value ? 1 : 0);
    
//    // only care about relay 1
//    uint8_t prev = gpio_output_bit_get(GPIOF, GPIO_PIN_6) ? 1 : 0;
//    if (prev != value || m_eeprom_relay_val != value)
//    {
//        m_eeprom_relay_val = value;
//        APP_DBG_INF("Write relay status to flash\r\n", m_eeprom_relay_val);
////        if (NO_ACTIVE_PAGE  == EEPROM_Write(0x00, m_eeprom_relay_val))
////		{
////			EEPROM_State err = EEPROM_Format();
////			APP_DBG_INF("EE prom format err code %u\r\n", err);
////		}
//    }
    board_hw_set_relay(BOARD_HW_RELAY_0, value);
}


static void change_temporary_lora_conf(uint32_t freq, uint8_t bw, uint8_t sf)
{
    APP_DBG_INF("Change lora configuration to %uKHz, bw %u, sf %u\r\n", 
                freq/1000, bw, sf);
    app_lora_set_network_id(0);
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
        

        APP_DBG_INF("Node %02X%02X%02X%02X%02X%02X received offer id %d, freq %uKhz, output 0x%02X, input 0x%02X\r\n",
                     node_mac[0],
                     node_mac[1],
                     node_mac[2],
                     node_mac[3],
                     node_mac[4],
                     node_mac[5],
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
            app_lora_set_network_id(rx_header->network_id);

            m_join_network_state = PAIR_STATE_IDLE;
            m_continues_pair_request = false;
            m_pending_keep_alive = true;

            app_flash_store_info(&m_net_info);

            app_led_blink(0, 6, 250, 250);

            systime_create(&m_timer_keep_alive);
            APP_DBG_INF("Joined, offer id %d\r\n", offer_id);
            m_keep_alive_delay_ms = (m_net_info.device_id + 1) * 3000;
            change_temporary_lora_conf(LORA_RF_FREQUENCY_NORMAL, LORA_BANDWIDTH_OPTIMIZE_RANGE, LORA_SPREADING_FACTOR_OPTIMIZE_RANGE);
        }
    }
    else
    {
        APP_DBG_INF("Invalid pair offer message\r\n");
    }
}

static void on_new_unknown_lora_data_received(app_lora_event_t event, void *arg, void *header)
{
    app_lora_data_t *rx_msg = (app_lora_data_t *)arg;
    APP_DBG_INF("Unknown authen lora data, event %d, len %d\r\n", (int)event, rx_msg->len);
	timeout_not_received_msg_reset_lora = TIMEOUT_NOT_RECEIVED_ANY_MESSAGE_MS;
	timeout_not_received_msg_reset_mcu = 3*TIMEOUT_NOT_RECEIVED_ANY_MESSAGE_MS;
	
    if (m_join_network_state == PAIR_STATE_RECEIVING_NETWORK_INFO 
        && rx_msg->msg_type == APP_LORA_ID_PAIR_OFFER)
    {
        process_join_offer_message(rx_msg->payload, rx_msg->len, header);
    }
}

extern void RadioOnDioIrq(void);
static void task_lora_rx(void *arg)
{
    static uint32_t m_last_tick = 0;
    if (sys_now_ms() != m_last_tick)
    {
        static uint32_t poll_irq = 0;
        if (poll_irq++ == 100)
        {
            RadioOnDioIrq();
            poll_irq = 0;
        }
//        if (m_delay_monitor_relay_error)
//        {
//            m_delay_monitor_relay_error--;
//            if (m_delay_monitor_relay_error == 0)
//            {
//                APP_DBG_INF("Reset monitor contactor timeout\r\n");
//            }
//        }
        
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
                heartbeat_msg.name.io_alarm_input0 = (m_alarm_bit & (1 << ALARM_BIT_BUTTON_SOS)) ? 1 : 0;
    
                
                uint8_t payload[sizeof(app_lora_alarm_type_t) + sizeof(m_slave_mac)];
                memcpy(payload, m_slave_mac, 6);
                memcpy(payload +6, (uint8_t*)&heartbeat_msg, sizeof(app_lora_alarm_type_t));
                
                data.device_type = APP_LORA_DEVICE_TYPE_SPEAKER;
                data.node_id = 0x00;
                data.msg_type = APP_LORA_ID_SLAVE_WANT_TO_JOIN;
                data.len = sizeof(payload);
                data.payload = payload;
                if (!app_lora_master_tx(&data, on_lora_tx_event, 0, 2000, NULL))
                {
                    APP_DBG_ERR("Send LoRa tx message failed\r\n");
                }
            }
//            else if (m_slave_reply_composion_data.new_msg)
//            {
//                APP_DBG_INF("Send current network info\r\n");

//                app_lora_data_t data;
//                data.msg_type = APP_LORA_ID_PAIR_GET_COMPOSTION_DATA;
//                data.len = sizeof(m_slave_reply_composion_data) - sizeof(((provision_slave_reply_t *)0)->new_msg);
//                data.payload = (uint8_t *)&m_slave_reply_composion_data;

//                if (app_lora_master_tx(&data, on_lora_tx_event, 0, 3000, NULL))
//                {
//                    m_slave_reply_composion_data.new_msg = 0;
//                }
//                else
//                {
//                    APP_DBG_ERR("Send LoRa tx message failed\r\n");
//                }
//            }
//            else if (m_join_network_state == PAIR_STATE_PROCESSING_OFFER)
//            {
//                send_pair_info();
//            }
            else if (m_pending_keep_alive)
            {       
                if (systime_is_timer_elapse(&m_timer_keep_alive, m_keep_alive_delay_ms)) // 4000ms for delay
                {
                    if (keep_alive())
                    {
//                        m_keep_alive_delay_ms = (m_net_info.device_id + 1) * 3000;
                        systime_stop(&m_timer_keep_alive);
                        
//                        static uint32_t m_delay_step = 0;
//                        if (m_relay_value.name.get_back_value ==  m_relay_value.name.set_value)
//                        {
//                            m_pending_keep_alive = false;
//                            m_delay_step = 0;
//                        }
//                        else
//                        {
//                            m_delay_step++;
//                            if (!CONTACTOR_ERROR())
//                            {
//                                m_keep_alive_delay_ms = (board_hw_get_ms() & 0x007 + 1) * 2136 + (m_delay_step % 7)*10000;     // random delay
//                            }
//                            else
//                            {
//                                 m_keep_alive_delay_ms = (board_hw_get_ms() & 0x007 + 1) * 2136 + (m_delay_step % 7)*2000;     // random delay
//                            }
//                            systime_create(&m_timer_keep_alive);
//                            APP_DBG_INF("Relay feedback wrong, send error to master after %ums\r\n", m_keep_alive_delay_ms);
//                        }
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

        lora_task_poll();
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
    
    // Build heartbeat msg
    heartbeat_msg.value = 0;
    heartbeat_msg.name.io_alarm_input0 = (m_alarm_bit & (1 << ALARM_BIT_BUTTON_SOS)) ? 1 : 0;
    
    data.node_id = m_net_info.device_id;
    data.msg_type = APP_LORA_ID_KEEP_ALIVE;
    data.device_type = APP_LORA_DEVICE_TYPE_SPEAKER;
    data.len = sizeof(heartbeat_msg);
    data.payload = (uint8_t*)&heartbeat_msg;

    APP_DBG_INF("Send heartbeat\r\n");
    if (m_net_info.network_id && !app_lora_master_tx(&data, on_lora_tx_event, 0, 4000, NULL))
    {
        APP_DBG_ERR("Send keep alive message failed\r\n");
        return false;
    }
    else
    {
        APP_DBG_ERR("Relay value 0x%02X\r\n", m_relay_value.value);
    }

    return true;
}

void enter_state_get_network_information(void)
{
    APP_DBG_INF("Enter state get network information\r\n");
    m_continues_pair_request = true;
    m_join_network_state = PAIR_STATE_RECEIVING_NETWORK_INFO;
    systime_create(&m_timer_pair);
    change_temporary_lora_conf(LORA_RF_FREQUENCY_PAIR, LORA_BANDWIDTH_OPTIMIZE_SPEED, LORA_SPREADING_FACTOR_OPTIMIZE_SPEED);
}

bool enter_state_send_current_network_information_to_gateway(void)
{
    APP_DBG_INF("%s\r\n", __FUNCTION__);
    if (m_net_info.network_id)
    {
        m_join_network_state = PAIR_STATE_PROCESSING_OFFER;
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
    static const char *event_name[] = {"pressed", "release", "hold", "hold so long", "double click", "tripple click", "idle", "idle break", "invalid"};
    APP_DBG_WRN("Button %s %s\r\n", button_pin ? "pair" : "ctrl", event_name[event]);
    switch (event)
    {
    case APP_BTN_EVT_PRESSED:
        if (button_pin == ALARM_BIT_BUTTON_SOS)        // buton control relay
        {
            m_pending_keep_alive = true;
            m_keep_alive_delay_ms = 3000;     // delay for safety reason, maybe heavy noise in payload we decrease RF performance
            if (!systime_is_timer_started(&m_timer_keep_alive))
            {
                systime_create(&m_timer_keep_alive);
            }
             
            m_alarm_bit |= (1 << ALARM_BIT_BUTTON_SOS);
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
            m_alarm_bit &= ~(1 << ALARM_BIT_BUTTON_SOS);
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
        if (m_join_network_state != PAIR_STATE_IDLE)
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
        RadioOnDioIrq();
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

void sos_fire_alarm_poll(void)
{
    if (m_alarm_bit)
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
