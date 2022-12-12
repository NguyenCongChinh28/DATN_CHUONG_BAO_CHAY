#ifndef APP_SPI_H
#define APP_SPI_H

#include <stdint.h>

/**
 * @brief           Initialize SPI driver 
 */
void app_spi_initialize(void);

/**
 * @brief           Send spi data
 * @param[in]       write_data Buffer TX data
 * @param[out]      rx_dummy_data SPI rx dummy data, dont care it
 * @param[in]       len Tx data len
 */
void app_spi_tx(uint8_t *write_data, uint8_t *rx_dummy_data, uint32_t len);

/**
 * @brief           Receive spi data
 * @param[in]       rx_data Buffer RX data
 * @param[out]      tx_dummy_data SPI tx dummy data, dont care it
 * @param[in]       len Rx data len
 */
void app_spi_rx(uint8_t *rx_data, uint8_t *tx_dummy_data, uint32_t len);

#endif /* APP_SPI_H */
