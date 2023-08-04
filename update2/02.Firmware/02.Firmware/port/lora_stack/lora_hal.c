#include "lora_hal.h"
#include <string.h>
#include <stdio.h>
#include "systime.h"
#include "app_debug.h"
#include "radio.h"
#include "sx126x.h"
#include "math.h"

#define RX_TIMEOUT_VALUE                            (0xFFFFFFFF)          //(max 24 bits)

#define TIMEOUT_RESET 100

#define LORA_HAL_CHANGE_TX_STATE(state) (m_packet_fsm = state)
#define LORA_HAL_TX_STATE_IS_IDLE() (m_packet_fsm == LORA_HAL_PACKET_FSM_IDLE)

#define OPTIMIZE_MEMORY     1
#define LORA_HAL_CHANNEL_BUSY_DELAY_MS      1000
static systime_t m_timer_channel_busy;

typedef enum
{
    LORA_PREPARE_OFF,
    LORA_POWER_OFF,
    LORA_POWER_ON,
    LORA_POWER_ALWAYS_OFF
} lora_power_state_t;

static int8_t m_rssi, m_snr;


uint16_t m_rx_bytes = 0;
#if OPTIMIZE_MEMORY == 0
#define MAX_LORA_BUFFER_SIZE                        128 // Define the payload size here
uint8_t m_rx_buffer[MAX_LORA_BUFFER_SIZE];
#else
#define MAX_LORA_BUFFER_SIZE                        255 // Max lora phy packet
uint8_t *m_rx_buffer;
#endif

static RadioEvents_t m_radio_phy_event;

static lora_power_state_t m_lora_power_state = LORA_PREPARE_OFF;
static lora_hal_cfg_t m_hal_conf;
static lora_hal_transmit_cb_t m_lora_tx_cb;
static uint32_t m_begin_transmit_timestamp = 0;
static bool m_lora_on_bus = false;

static lora_hal_packet_fsm_t m_packet_fsm = LORA_HAL_PACKET_FSM_INVALID;
static systime_t m_timer_transmit_timeout, m_timer_power_monitor;
static volatile uint8_t m_tx_active = 0;

static void on_lora_phy_new_tx_cplt(void);
static void on_lora_phy_new_rx_data(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
static void on_lora_phy_tx_timeout(void);

static void on_lora_phy_rx_timeout(void);
static void on_lora_phy_crc_error(void);
static void on_lora_preamble_detect(void);
static void on_lora_header_detect(void);
uint16_t lora_hal_read_sync_word(void);

extern const struct Radio_s Radio;

lora_hal_packet_fsm_t *lora_hal_get_packet_fsm(void)
{
    return &m_packet_fsm;
}


int8_t lora_hal_packet_rssi(void)
{
    return m_rssi;
}

/**
 * Return last packet's SNR (signal to noise ratio).
 */
int8_t lora_hal_packet_snr(void)
{
    return m_snr;
}

void lora_hal_power_poll(void)
{
    if (systime_is_timer_started(&m_timer_channel_busy))
    {
        if (systime_is_timer_elapse(&m_timer_channel_busy, LORA_HAL_CHANNEL_BUSY_DELAY_MS))
        {
            systime_stop(&m_timer_channel_busy);
            APP_DBG_INF("Channel free\r\n");
            Radio.Standby();
            Radio.RxBoosted(RX_TIMEOUT_VALUE);
            if (m_hal_conf.on_channel_free)
            {
            	m_hal_conf.on_channel_free();
            }
        }
    }
    
    switch (m_lora_power_state)
    {
    case LORA_PREPARE_OFF:
    {
        m_lora_on_bus = true;
        m_hal_conf.hard_reset(0);
        if (!systime_is_timer_started(&m_timer_power_monitor))
        {
            systime_create(&m_timer_power_monitor);
        }
        if (systime_is_timer_elapse(&m_timer_power_monitor, m_hal_conf.startup_ms)) // 10ms
        {
            m_lora_power_state = LORA_POWER_OFF;
        }
    }
    break;

    case LORA_POWER_OFF:
    {
        m_hal_conf.hard_reset(1);
        if (!systime_is_timer_started(&m_timer_power_monitor))
        {
            systime_create(&m_timer_power_monitor);
        }
        if (systime_is_timer_elapse(&m_timer_power_monitor, m_hal_conf.startup_ms)) // ms
        {
            m_lora_power_state = LORA_POWER_ON;
            m_radio_phy_event.TxDone = on_lora_phy_new_tx_cplt;
            m_radio_phy_event.RxDone = on_lora_phy_new_rx_data;
            m_radio_phy_event.TxTimeout = on_lora_phy_tx_timeout;
            m_radio_phy_event.RxTimeout = on_lora_phy_rx_timeout;
            m_radio_phy_event.RxError = on_lora_phy_crc_error;
            m_radio_phy_event.PreambleDetect = on_lora_preamble_detect;
            m_radio_phy_event.HeaderDetect = on_lora_header_detect;
            Radio.Init(&m_radio_phy_event);
            Radio.SetChannel(m_hal_conf.freq);
            Radio.SetTxConfig( MODEM_LORA, m_hal_conf.tx_power, 0, m_hal_conf.bw,
                             m_hal_conf.sf, m_hal_conf.cr,
                             m_hal_conf.preamble_length, false,
                             true, 0, 0, false, 3000);
                             
            Radio.SetRxConfig( MODEM_LORA,  m_hal_conf.bw, m_hal_conf.sf,
                             m_hal_conf.cr, 0, m_hal_conf.preamble_length,
                             0, false,
                             0, true, 0, 0, false, false );
            
            uint16_t read_sync_word = lora_hal_read_sync_word();
                             
            Radio.RxBoosted( RX_TIMEOUT_VALUE );
                     
            if (!((read_sync_word == 0x2414) || (read_sync_word == 0x4434)))
            {
                APP_DBG_INF("Invalid sync word 0x%04X\r\n", read_sync_word);
                m_lora_on_bus = false;
                return;
            }
            m_packet_fsm = LORA_HAL_PACKET_FSM_IDLE;
        }
    }
    break;

    case LORA_POWER_ON:
    {
    }
    break;
    
    case LORA_POWER_ALWAYS_OFF:
    {
        m_hal_conf.hard_reset(0);
    }
        break;
    
    default:
        break;
    }
}

void lora_hal_enter_shutdown_mode(void)
{
    m_lora_on_bus = true;
    m_hal_conf.hard_reset(0);
}

uint16_t lora_hal_read_sync_word()
{
    return Radio.Read(REG_LR_SYNCWORD) | (Radio.Read(REG_LR_SYNCWORD + 1) << 8);
}

uint8_t lora_hal_read_register(uint16_t addr)
{
    return Radio.Read(addr);
}

bool lora_hal_power_ready(void)
{
    return (m_lora_power_state == LORA_POWER_ON) ? true : false;
}

void lora_hal_reset(void)
{
	APP_DBG_INF("Hard reset\r\n");
    m_hal_conf.hard_reset(0);
    LORA_HAL_CHANGE_TX_STATE(LORA_HAL_PACKET_FSM_INVALID);
    m_lora_power_state = LORA_PREPARE_OFF;
    m_lora_on_bus = false;
    systime_stop(&m_timer_power_monitor);
    systime_stop(&m_timer_transmit_timeout);
	APP_DBG_VERBOSE("Clear busy flag\r\n");
    lora_hal_clear_channel_busy_flag();
}

bool lora_hal_is_packet_transmitting(void)
{
    return !LORA_HAL_TX_STATE_IS_IDLE();
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
        m_lora_tx_cb = cb;
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
        m_begin_transmit_timestamp = systime_get_ms();
        
        systime_stop(&m_timer_transmit_timeout);
        systime_create(&m_timer_transmit_timeout);
        LORA_HAL_CHANGE_TX_STATE(LORA_HAL_PACKET_FSM_TRANSMITTING);

        retval = true;
    }

    return retval;
}

void lora_hal_packet_fsm_poll(void)
{
    switch (m_packet_fsm)
    {
        case LORA_HAL_PACKET_FSM_INVALID:
            break;

        case LORA_HAL_PACKET_FSM_IDLE:
        {   
            Radio.IrqProcess( ); // Process Radio IRQ
        }
            break;

        case LORA_HAL_PACKET_FSM_TRANSMITTING:
        {
            Radio.IrqProcess( ); 
        }
            break;

        case LORA_HAL_PACKET_FSM_TRANSMIT_COMPLETE:
            m_tx_active = 0;
            m_packet_fsm = LORA_HAL_PACKET_FSM_IDLE;
            if (m_lora_tx_cb)
            {
                m_lora_tx_cb(true, systime_get_ms() - m_begin_transmit_timestamp);
            }
            break;

        case LORA_HAL_PACKET_FSM_TRANSMIT_TIMEOUT:
        {
            m_tx_active = 0;
            m_packet_fsm = LORA_HAL_PACKET_FSM_IDLE;
            lora_hal_reset();
            APP_DBG_INF("LoRa TX timeout, do hardware reset\r\n");
        }
            break;

        case LORA_HAL_PACKET_SHUTDOWN_MODE:
            m_tx_active = 0;
            break;
        
        default:
        {
            APP_DBG_INF("Unhandle lora fsm %d\r\n", m_packet_fsm);
        }
            break;
        
    }
}

lora_hal_cfg_t *lora_hal_get_current_configuration(void) 
{
    return &m_hal_conf;
} 

void lora_hal_init(lora_hal_cfg_t *hal_conf)
{
    memcpy(&m_hal_conf, hal_conf, sizeof(lora_hal_cfg_t));
    lora_hal_reset();
}

void lora_hal_task(void)
{
    lora_hal_power_poll();
    lora_hal_packet_fsm_poll();
}

bool lora_hal_is_device_on_bus(void)
{
    return m_lora_on_bus;
}

static void on_lora_phy_new_tx_cplt(void)
{
    APP_DBG_INF("[%s()] TxDone\r\n", __FUNCTION__);
    Radio.Standby();
    Radio.RxBoosted(RX_TIMEOUT_VALUE);
    m_packet_fsm = LORA_HAL_PACKET_FSM_TRANSMIT_COMPLETE;
}

static void on_lora_phy_new_rx_data(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    m_rx_bytes = (size > MAX_LORA_BUFFER_SIZE) ? MAX_LORA_BUFFER_SIZE : size;
    systime_stop(&m_timer_channel_busy);
    
#if OPTIMIZE_MEMORY == 0
    memcpy(m_rx_buffer , payload, m_rx_bytes);
#else
    m_rx_buffer = payload;
#endif
    
    m_rssi = rssi;
    m_snr = snr;

    Radio.Standby();
    APP_DBG_INF("[%s()] RxDone\r\nSize:%d\r\nRssi:%d\r\nSnr:%d\r\n", __FUNCTION__, size, rssi, snr);
    Radio.RxBoosted(RX_TIMEOUT_VALUE);
    m_hal_conf.on_frame_cb(m_rx_buffer, m_rx_bytes);
    
}

static void on_lora_phy_tx_timeout(void)
{
    Radio.Standby();
    APP_DBG_INF("Lora phy tx timeout\r\n");
    Radio.RxBoosted(RX_TIMEOUT_VALUE); 
    m_packet_fsm = LORA_HAL_PACKET_FSM_TRANSMIT_TIMEOUT;
    lora_hal_reset();
}

static void on_lora_phy_rx_timeout(void)
{
    Radio.Standby();
    APP_DBG_INF("[%s()] on_lora_phy_rx_timeout\r\n", __func__);
    Radio.RxBoosted(RX_TIMEOUT_VALUE); 
    APP_DBG_INF("LoRa RX timeout, reset\r\n");
    lora_hal_reset();
}

static void on_lora_phy_crc_error(void)
{
    Radio.Standby();
    APP_DBG_INF("[%s()] RxError retry receive\r\n", __FUNCTION__);
    Radio.RxBoosted(RX_TIMEOUT_VALUE); 
}

static void on_lora_preamble_detect(void)
{
//    APP_DBG_INF("[%s()] callback\r\n", __FUNCTION__);
//    systime_stop(&m_timer_channel_busy);
//    systime_create(&m_timer_channel_busy);
}

static void on_lora_header_detect(void)
{
    APP_DBG_INF("[%s()] callback\r\n", __FUNCTION__);
    systime_stop(&m_timer_channel_busy);
    systime_create(&m_timer_channel_busy);
}

bool lora_hal_is_channel_busy(void)
{
    return systime_is_timer_started(&m_timer_channel_busy) ? true : false; 
}

void lora_hal_clear_channel_busy_flag()
{
    systime_stop(&m_timer_channel_busy);
}

void lora_hal_set_temporary_config(uint32_t freq, uint32_t bw, uint32_t sf)
{
    m_hal_conf.freq = freq;
    m_hal_conf.bw = bw;
    m_hal_conf.sf = sf;
}

bool lora_hal_is_in_shutdown_mode(void)
{
    return (m_lora_power_state == LORA_POWER_ALWAYS_OFF) ? true : false;
}

void lora_hal_wakeup(void)
{
    if (m_lora_power_state == LORA_POWER_ALWAYS_OFF)
    {
        m_lora_power_state = LORA_PREPARE_OFF;
        APP_DBG_INF("[%s()] Wakeup LORA\r\n", __FUNCTION__);
    }
}


