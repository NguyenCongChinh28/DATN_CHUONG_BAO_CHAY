#ifndef LORA_HAL_H
#define LORA_HAL_H


#include <stdbool.h>
#include <stdint.h>


typedef void (*lora_hal_transmit_cb_t)(bool success, uint32_t timeout);


typedef void (*lora_reset_pin_control_cb_t)(bool on);
typedef bool (*lora_read_pin_cb_t)(uint32_t pin);
typedef void (*lora_delay_cb_t)(uint32_t ms);
typedef void (*lora_spi_tx_cb_t)(uint8_t *tx_data, uint8_t *rx_dummy_data, uint32_t len);
typedef void (*lora_spi_rx_cb_t)(uint8_t *tx_dummy_data, uint8_t *rx_data, uint32_t len);
typedef void (*lora_on_frame_callback_t)(uint8_t *data, uint8_t size);
typedef void (*lora_on_channel_free_callback_t)(void);
typedef struct
{
    lora_reset_pin_control_cb_t hard_reset;
    lora_read_pin_cb_t read_pin;
    lora_delay_cb_t delay_ms;
    lora_spi_tx_cb_t spi_tx;
    lora_spi_rx_cb_t spi_rx;
    lora_on_frame_callback_t on_frame_cb;
    lora_on_channel_free_callback_t on_channel_free;
    uint8_t tx_power;
    uint32_t freq;
    uint8_t sf;
    uint32_t bw;
    uint8_t sync_word;
    uint8_t cr;
    uint8_t preamble_length;
    uint8_t symbol_timeout;
    bool enable_crc;
}lora_hal_cfg_t;

typedef enum
{
    LORA_HAL_PACKET_FSM_INVALID,
    LORA_HAL_PACKET_FSM_IDLE,
    LORA_HAL_PACKET_FSM_TRANSMITTING,
    LORA_HAL_PACKET_FSM_TRANSMIT_COMPLETE,
    LORA_HAL_PACKET_FSM_TRANSMIT_TIMEOUT,
    LORA_HAL_PACKET_SHUTDOWN_MODE
} lora_hal_packet_fsm_t;


lora_hal_packet_fsm_t *lora_hal_get_packet_fsm(void);


void lora_hal_init(lora_hal_cfg_t *hal_config);

void lora_hal_reset(void);

/**
 * @brief               Clear channal busy flag
 */
void lora_hal_clear_channel_busy_flag(void);
/**
 * @brief               Change lora hal configuration
 */
void lora_hal_set_temporary_config(uint32_t freq, uint32_t bw, uint32_t sf);
/**
 * @brief               LoRa packet is transmitting on air
 * @retval              TRUE : Packet is transmitting
 *                      FALSE : No packet is transmitting on air
 */
bool lora_hal_is_packet_transmitting(void);

/**
 * @brief               Verify lora device is onbus
 * @retval              TRUE : Device is on bus
 *                      FALSE : Device is not detected 
 */
bool lora_hal_is_device_on_bus(void);

/**
 * @brief               Get lora power ready status
 * @retval              TRUE : Device is power on
 *                      FALSE : Device is power off
 */
bool lora_hal_power_ready(void);
/**
 * @brief               Transmit lora packet
 * @param[in]           buf Buffer for data
 * @param[in]           size Data size
 * @param[in]           timeout_ms : Transmit timeout in ms
 * @param[in]           cb : Transmit finish callback
 * @retval              TRUE : Packet is transmitting
 *                      FALSE : LoRa TX is busy, packet is not sent
 */
bool lora_hal_send_packet(uint8_t *buf, uint32_t size, uint32_t timeout_ms, lora_hal_transmit_cb_t cb);

    





#endif /*LORA_HAL_H*/
