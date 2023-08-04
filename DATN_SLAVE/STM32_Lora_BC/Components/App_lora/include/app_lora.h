#ifndef APP_LORA_H
#define APP_LORA_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>




#define APP_LORA_RF_FREQUENCY_PAIR                              433000000  // Hz
#define APP_LORA_TX_OUTPUT_POWER                                22		// dBm
#define APP_LORA_SF_OPTIMIZE_RANGE                              12 // [SF7..SF12]
#define APP_LORA_CODINGRATE                                     1		// [1: 4/5, 2: 4/6,  3: 4/7,  4: 4/8]
#define APP_LORA_BW_OPTIMIZE_RANGE                              0		// [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define APP_LORA_PREAMBLE_LENGTH                                8  // Same for Tx and Rx

#define APP_LORA_MAX_PAYLOAD_SIZE 32




#define APP_LORA_ID_PAIR_OFFER                                0x00
#define APP_LORA_ID_SLAVE_WANT_TO_JOIN                        0x01
#define APP_LORA_ID_PAIR_GET_COMPOSTION_DATA                  0x02
#define APP_LORA_ID_CONTROL                                   0x03
#define APP_LORA_ID_ACK_ALARM_TRIGGER                         0x04
#define APP_LORA_ID_ACK_ALARM_REMOVED                         0x05
#define APP_LORA_ID_UPDATE_RELAY_STATUS                       0x06
#define APP_LORA_ID_KEEP_ALIVE                                0x07
#define APP_LORA_ID_LORA_JOIN_OK                              0x08
#define APP_LORA_ID_POLL_ONLY_ONE_NODE	                      0x09
#define APP_LORA_ID_BROADCAST_CONTROL                         0x0A


#define APP_LORA_DEVICE_TYPE_GW                               (0x00)
#define APP_LORA_DEVICE_TYPE_SENSOR_FIRE_DETECTOR             (0x01)
#define APP_LORA_DEVICE_TYPE_SENSOR_SMOKE_DETECTOR            (0x02)
#define APP_LORA_DEVICE_TYPE_BUTTON_SOS                       (0x03)
#define APP_LORA_DEVICE_TYPE_SPEAKER                          (0x04)
#define APP_LORA_DEVICE_TYPE_SENSOR_TEMP_DETECTOR             (0x05)
#define APP_LORA_DEVICE_TYPE_SENSOR_DOOR_DETECTOR             (0x06)
#define APP_LORA_DEVICE_TYPE_SENSOR_PIR_DETECTOR              (0x07)
#define APP_LORA_DEVICE_TYPE_TOUCH_LIGHT                      (0x08)
#define APP_LORA_DEVICE_TYPE_TOUCH_WATER_WARMER               (0x09)
#define APP_LORA_DEVICE_TYPE_TOUCH_AIR_CON                    (0x0A)
#define APP_LORA_DEVICE_TYPE_SENSOR_TEMP_SMOKE                (0x0B)
#define APP_LORA_DEVICE_TYPE_POWER_METER                      (0x0C)
#define APP_LORA_DEVICE_TYPE_UNKNOWN                          (0x0D)
#define APP_LORA_DEVICE_TYPE_MAX                              (0x0F)
#define APP_LORA_NETWORK_ID_INVALID                            0














typedef enum
{
    APP_LORA_ERR_TX_FINISH,
    APP_LORA_ERR_TX_TIMEOUT
} app_lora_err_t;


typedef void (*app_lora_master_tx_cb_t)(app_lora_err_t err, void *arg);

typedef union 
{
    struct
    {
        uint8_t msg_type : 4;
        uint8_t device_type : 4;
    } __attribute__ ((packed)) name;
    uint8_t raw;
} __attribute__ ((packed))  app_lora_id_t;


typedef struct
{
    uint8_t mac[6];
    uint16_t network_id;
    app_lora_id_t type;
} __attribute__ ((packed)) app_lora_header_t;

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


typedef struct
{
    uint8_t mac[6];
    uint8_t msg_type;
    uint8_t device_type;
    uint8_t len;
    uint8_t *payload;
} app_lora_data_t;

typedef enum
{
    APP_LORA_PROVISIONING_STATE_IDLE,
    APP_LORA_PROVISIONING_PROCESSING_OFFER, // If device is slave, it will send current network information to Gateway
    APP_LORA_PROVISIONING_RECEIVING_NETWORK_INFO,
} app_lora_provisioning_state_t;
typedef struct
{
    uint8_t mac[6];     // MAC of node
    uint32_t freq;      // Working freq
    uint8_t output;     // Gia tri cai dat output   
    uint8_t input;      // Gia tri input hien tai cua gateway
    uint8_t sync_alarm; // Gia tri = 1 -> Khi chuong den nhen dc alarm tu bat ki cam bien nao -> bat canh bao luon ma ko can cho gateway 
} __attribute__((packed)) app_lora_join_offer_msg_t;
typedef union
{
    struct
    {
        uint8_t alarm               : 1;
        uint8_t priority_input      : 1;
        uint8_t input1              : 1;
        uint8_t sync_alarm          : 1;
        uint8_t reserve             : 4;
    } __attribute__((packed)) name;
    uint8_t value;
} __attribute__((packed)) app_lora_broadcast_alarm_msg_t;
typedef struct
{
    uint8_t mac[6];
    uint32_t auto_remove_alarm;     // second
} app_lora_list_alarm_device_t;
    

typedef union
{
    struct
    {
        uint8_t smoke : 1;
        uint8_t temp : 1;
        uint8_t io_alarm_input0 : 1;
        uint8_t io_alarm_input1 : 1;
        uint8_t io_active_level0 : 1;
        uint8_t io_active_level1 : 1;
        uint8_t reserve : 2;
    } __attribute__((packed)) name;
    uint8_t value;
} __attribute__((packed)) app_lora_alarm_type_t;

typedef struct
{
    app_lora_alarm_type_t alarm;
    uint8_t battery_percent;
} __attribute__((packed)) app_lora_heartbeat_msg_t;

typedef struct
{
    app_lora_alarm_type_t alarm;
    uint8_t battery_percent;
    uint8_t mac[6];
} __attribute__((packed)) app_lora_pair_msg_t;




typedef enum
{
    APP_LORA_EVENT_RESET,
    APP_LORA_EVENT_TX,
    APP_LORA_EVENT_RX,
    APP_LORA_EVENT_CHANNEL_FREE,
    APP_LORA_EVENT_SHUTDOWN
} app_lora_event_t;

typedef void (*app_lora_event_cb_t)(app_lora_event_t event, void *arg, void *header);
typedef uint32_t (*app_lora_get_ms_cb_t)(void);
typedef bool (*app_lora_read_pin_cb_t)(uint32_t pin);
typedef void (*app_lora_lora_delay_cb_t)(uint32_t ms);
typedef void (*app_lora_spi_tx_cb_t)(uint8_t *tx_data, uint8_t *rx_dummy_data, uint32_t len);
typedef void (*app_lora_spi_rx_cb_t) (uint8_t *tx_dummy_data, uint8_t *rx_data, uint32_t len);
typedef void (*app_lora_reset_pin_control_cb_t)(bool on);

typedef struct
{
    app_lora_event_cb_t event;
    app_lora_event_cb_t unauthen_data_event;
    app_lora_get_ms_cb_t get_ms;
    app_lora_read_pin_cb_t read_pin;
    app_lora_lora_delay_cb_t delay_ms;
    uint32_t period_reset_ms_when_moderm_good;
    uint32_t period_reset_ms_when_moderm_fail;
    uint32_t rx_idle_timeout_ms;
    app_lora_spi_tx_cb_t spi_tx;
    app_lora_spi_rx_cb_t spi_rx;

    app_lora_reset_pin_control_cb_t lora_hw_reset;
    uint8_t tx_power;
    uint32_t freq;
    uint8_t sf;
    uint32_t bw;
    uint8_t sync_word;
    uint8_t cr;
    uint8_t preamble_length;
    uint32_t symbol_timeout;
    bool enable_crc;
    uint32_t network_id;
    uint32_t startup_ms;
}app_lora_cfg_t;



/*
 **@brief  Initialize app lora
 */
void app_lora_initialize(app_lora_cfg_t *config);
 
/**
 * @brief           Set Network ID
 * @param[in]       Network ID
 */
void app_lora_set_network_id(uint16_t network_id);

/**
 * @brief           Stop process TX timeout
 */  
void app_lora_stop_process_tx_timeout(void);
/**
 * @brief           Reset lora
 */
void app_lora_reset(void);
/**
 * @brief           Set temporary configuration
 * @param[in]       freq : Lora frequency in Hz
 * @param[in]       bw : LoRa bandwitch
 * @param[in]       sf : LoRa SF
 */                        
void app_lora_set_temporary_config(uint32_t freq, uint8_t bw, uint8_t sf);
/**
 * @brief           Get lora busy status
 * @retval          TRUE : Lora is busy
 *                  FALSE : Lora is idle
 */
bool app_lora_is_ready_to_tx(void);     
/**
 * @brief           Send lora message
 * @param[in]       data : Lora data payload
 * @param[in]       Lora callback function
 * @param[in]       retry : Number of retry time
 * @param[in]       timeout_ms : Timeout in ms
 * @param[in]       arg : User data
 * @retval          TRUE : Operation success
 *                  FALSE : Operation failed
 */
bool app_lora_master_tx(app_lora_data_t *data, 
                        app_lora_master_tx_cb_t callback, 
                        int32_t retry, 
                        uint32_t timeout_ms, 
                        void *arg);











#endif /*APP_LORA_H*/
