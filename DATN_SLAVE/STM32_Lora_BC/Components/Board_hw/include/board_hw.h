#ifndef BOARD_HW_H
#define BOARD_HW_H

#include"stdint.h"
#include<stdbool.h>



/**
 * @brief Get ms in milisecond
 */
 
uint32_t Get_MS(void);


/**
 * @brief         Send data to the uart port
 * @param[in]     data Data to send
 * @param[in]     size Size of data
 */
void board_hw_debug_uart_send(const void *data,uint32_t size);

/**
 * @brief Lora reset
 */
 
void board_hw_lora_reset_pin(bool on);




void board_hw_delay_ms(uint32_t ms);


/**
 *  @brief              Set priority IO
 *  @param[in]          uint8_t level Output level
 */
void board_hw_control_remote_priority_io(bool level);

/*!
 * \brief Delay in ms
 */
void SX126xDelayMs(uint32_t ms);


/*!
 * \brief Blocking loop to wait while the Busy pin in high
 */
bool SX126xWaitOnBusy(uint32_t timeout_ms);
/**
 *  @brief              Read lora busy pin
 *  @retval             LoRa busy pin
 */
uint32_t board_hw_get_lora_busy_pin(void);

/**
 *  @brief              Get battery in percent
 *  @retval             Battery percent (0-100)
 */
uint8_t board_hw_get_battery_percent(void);



#endif /*BOARD_HW_H*/
