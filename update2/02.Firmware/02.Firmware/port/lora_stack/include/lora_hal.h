
#ifndef LORA_HAL_H
#define LORA_HAL_H

#include <stdint.h>
#include <stdbool.h>

#define LORA_HAL_BUSY_PIN       4       // DIO4

typedef void (*lora_reset_pin_control_cb_t)(bool on);
typedef void (*lora_spi_tx_cb_t)(uint8_t *write_data, uint8_t *rx_dummy_data, uint32_t len);
typedef void (*lora_spi_rx_cb_t)(uint8_t *rx_data, uint8_t *tx_dummy_data, uint32_t len);
typedef void (*lora_hal_transmit_cb_t)(bool success, uint32_t timeout);
typedef bool (*lora_read_pin_cb_t)(uint32_t pin);
typedef void (*lora_delay_cb_t)(uint32_t ms);
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
    uint32_t startup_ms;
    bool enable_crc;
} lora_hal_cfg_t;

typedef enum
{
    LORA_HAL_PACKET_FSM_INVALID,
    LORA_HAL_PACKET_FSM_IDLE,
    LORA_HAL_PACKET_FSM_TRANSMITTING,
    LORA_HAL_PACKET_FSM_TRANSMIT_COMPLETE,
    LORA_HAL_PACKET_FSM_TRANSMIT_TIMEOUT,
    LORA_HAL_PACKET_SHUTDOWN_MODE
} lora_hal_packet_fsm_t;

/**
 *@brief                Configure explicit header mode.
 *@retval               Packet size will be included in the frame.
 */
void lora_hal_explicit_header_mode(void);

/**
 * @brief               Configure implicit header mode.
 *                      All packets will have a predefined size.
 * @param[in]               size Size of the packets.
 */
void lora_hal_implicit_header_mode(uint8_t size);

/**
 * @brief               Sets the radio transceiver in idle mode.
 *                      Must be used to change registers and access the FIFO.
 */
void lora_hal_enter_standby(void);

/**
 * @brief               Sets the radio transceiver in sleep mode.
 *                      Low power consumption and FIFO is lost.
 */
void lora_hal_sleep(void); 

/**
 * @brief               Sets the radio transceiver in receive mode.
 *                      Incoming packets will be received.
 */
void lora_hal_enter_receive_mode(void);

/**
 * @brief               Configure power level for transmission
 * @param[in]           level 2-17, from least to most power
 */
void lora_hal_set_tx_power(uint8_t level);

/**
 * @brief               Set carrier frequency.
 * @param[in]           frequency Frequency in Hz
 */
void lora_set_frequency(uint32_t frequency);

/**
 * @brief               Set spreading factor.
 * @param[in]               sf 6-12, Spreading factor to use.
 */
void lora_hal_set_spreading_factor(uint8_t sf);

/**
 * @brief               Set bandwidth (bit rate)
 * @param[in]               sbw Bandwidth in Hz (up to 500000)
 */
void lora_hal_set_bandwidth(uint32_t sbw);

/**
 * @brief               Set coding rate 
 * @param[in]               denominator 5-8, Denominator for the coding rate 4/x
 */
void lora_hal_set_coding_rate(uint8_t denominator);


/**
 * @brief               Set the size of preamble.
 * @param[in]           length Preamble length in symbols.
 */
void lora_hal_set_preamble_length(uint16_t length);

/**
 * @brief               Change radio sync word.
 * @param[in]               sw New sync word to use.
 */
void lora_hal_set_sync_word(int sw);

/**
 * @brief               Enable appending/verifying packet CRC.
 */
void lora_hal_enable_crc(void);

/**
 * @brief               Disable appending/verifying packet CRC.
 */
void lora_hal_disable_crc(void);


/**
 * @brief               Read a received packet in RX fifo buffer
 * @param[in]           buf Buffer for the data.
 * @param[out]          crc_valid Crc status    
 * @retval              Number of bytes received (zero if no packet available).
 */
int lora_get_packet_in_rx_fifo(uint8_t **buf, bool *crc_valid);

/**
 * @brief               Check lora new data status
 * @retval              TRUE There is data to read (packet received)
 *                      FALSE No packet availble
 */
bool lora_hal_is_new_data(void);

/**
 * @brief               Get last packet's SNR (signal to noise ratio).
 * @retval              Packet SNR
 */
int8_t lora_hal_packet_snr(void);


/**
 * @brief               GDump lora register
 */
void lora_hal_dump_registers(void);

/**
 * @brief               Get lora power ready status
 * @retval              TRUE : Device is power on
 *                      FALSE : Device is power off
 */
bool lora_hal_power_ready(void);

/**
 * @brief               Reset LoRa chip
 */
void lora_hal_reset(void);

/**
 * @brief               LoRa packet is transmitting on air
 * @retval              TRUE : Packet is transmitting
 *                      FALSE : No packet is transmitting on air
 */
bool lora_hal_is_packet_transmitting(void);

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


void lora_hal_init(lora_hal_cfg_t *hal_conf);

/**
 * @brief               Verify lora device is onbus
 * @retval              TRUE : Device is on bus
 *                      FALSE : Device is not detected 
 */
bool lora_hal_is_device_on_bus(void);

/**
 * @brief               Lora phy task
 */
void lora_hal_task(void);


/**
 * @brief               Get the last packet rssi
 * @retval              RSSI value
 */
int8_t lora_hal_packet_rssi(void);


/**
 * @brief               Read the current value of a register.
 * @param               reg Register index.
 * @return              Value of the register.
 */
uint8_t lora_hal_read_reg(uint8_t reg);


/**
 * @brief               Write a value to a register.
 * @param               reg Register index.
 * @param               val Value to write.
 */
void lora_write_reg(uint8_t reg, int val);

lora_hal_packet_fsm_t *lora_hal_get_packet_fsm(void);

/**
 * @brief               Read RX status via DIO gpio
 * @return              TRUE : New data
 *                      FALSE : No data availble
 */
bool lora_hal_get_dio_rx_done_notification(void);

/**
 * @brief               Reset DIO notification
 */
void lora_hal_reset_dio_rx_done_notification(void);

/**
 * @brief               Set DIO notification
 */
void lora_hal_set_dio_rx_done_notification(void);

/**
 * @brief               Read RX status via SPI bus
 * @return              TRUE : New data
 *                      FALSE : No data availble
 */
bool lora_hal_query_spi_for_new_data(void);

/**
 * @brief               Read channel busy status
 * @retval              TRUE : Channel is busy
 *                      FALSE : Channel is free
 */
bool lora_hal_is_channel_busy(void);

/**
 * @brief               Clear channal busy flag
 */
void lora_hal_clear_channel_busy_flag(void);

/**
 * @brief               Change lora hal configuration
 */
void lora_hal_set_temporary_config(uint32_t freq, uint32_t bw, uint32_t sf);

/**
 * @brief               Get current LoRa configuration
 * @retval              Pointer to current configuration
 */
lora_hal_cfg_t *lora_hal_get_current_configuration(void);

/*
 * @brief               Read lora register
 * @param[in]           addr Register address
 * @retval              Register value
 */
uint8_t lora_hal_read_register(uint16_t addr);

/*
 * @brief               Put device into low power mode
 */
void lora_hal_enter_shutdown_mode(void);

/*
 * @brief               Check if device in low power mode
 * @retval              TRUE : device is in force shutdown mode
 *                      FALSE : device is not in shutdown mode
 */
bool lora_hal_is_in_shutdown_mode(void);

/*
 * @brief               Wakeup device
 */
void lora_hal_wakeup(void);

#endif

