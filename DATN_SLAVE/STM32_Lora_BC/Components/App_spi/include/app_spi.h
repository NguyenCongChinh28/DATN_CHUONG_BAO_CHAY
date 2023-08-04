#ifndef APP_SPI_H
#define APP_SPI_H
 
#include<stdint.h>


/**
 * @brief Initialize SPI driver
 */
 void app_spi_init(void);
 
/**
 * @brief Send spi data
 * @param[out]  tx_data SPI TX data 
 * @param[in]   rx_dummy_data  SPI rx dummy data 
 * @param[out]  len Length of tx data
 */
 void app_spi_tx(uint8_t *tx_data,uint8_t *rx_dummy_data,uint32_t len);
 
/**
 * @brief Receive spi data 
 * @param[in]   tx_dummy_data SPI tx dummy data
 * @param[out]  rx_data SPI RX data
 * @param[in]   len Length of rx data
 */
 void app_spi_rx(uint8_t *tx_dummy_data, uint8_t *rx_data,uint32_t len); 
 
 
#endif /*APP_SPI_H*/
