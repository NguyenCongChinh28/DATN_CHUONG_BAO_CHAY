#include "app_spi.h"
#include "spi.h"


void app_spi_initialize(void)
{
    
    
    
}

void app_spi_tx(uint8_t *tx_data,uint8_t *rx_dummy_data,uint32_t len)
{
    spi_tx(tx_data,rx_dummy_data,len);
}

void app_spi_rx(uint8_t *tx_dummy_data,uint8_t *rx_data,uint32_t len)
{
    spi_rx(tx_dummy_data,rx_data,len);
}

