#include "app_spi.h"
#include "sdkconfig.h"
#include <stdbool.h>

#ifndef GD32E230
#include "driver/spi_master.h"
#include "driver/gpio.h"

#if defined CONFIG_IDF_TARGET_ESP32S2
#define LORA_SPI SPI2_HOST
#else
#define LORA_SPI VSPI_HOST
#endif

static spi_device_handle_t m_spi_dev;
#else
#include "gd32e23x.h"
#include "app_debug.h"
#endif /* GD32E230 */


void app_spi_initialize(void)
{
#ifndef GD32E230
    /**
    * Configure CPU hardware to communicate with the radio chip
    */
    // gpio_pad_select_gpio(CONFIG_RST_GPIO);
    // gpio_set_direction(CONFIG_RST_GPIO, GPIO_MODE_OUTPUT);
    // gpio_set_level(CONFIG_RST_GPIO, 0);

    gpio_pad_select_gpio(CONFIG_CS_GPIO);
    gpio_set_direction(CONFIG_CS_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(CONFIG_CS_GPIO, 1);

    esp_err_t err;
    spi_bus_config_t bus =
    {
        .miso_io_num = CONFIG_MISO_GPIO,
        .mosi_io_num = CONFIG_MOSI_GPIO,
        .sclk_io_num = CONFIG_SCK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0
    };

    err = spi_bus_initialize(LORA_SPI, &bus, 0);
    printf("Esp32 spi error %d\r\n", err);
    assert(err == ESP_OK);

    spi_device_interface_config_t dev = 
    {
        .clock_speed_hz = 8000000,
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 1,
        .flags = 0,
        .pre_cb = NULL
    };

    err = spi_bus_add_device(LORA_SPI, &dev, &m_spi_dev);
    assert(err == ESP_OK);

    printf("SPI clock %d, miso %d, mosi %d, sck %d, cs %d\r\n",
            dev.clock_speed_hz,
            bus.miso_io_num,
            bus.mosi_io_num,
            bus.sclk_io_num,
            CONFIG_CS_GPIO);
#else    
    // SPI bus
    APP_DBG_INF("Initialize spi bus\r\n");
    rcu_periph_clock_enable(RCU_SPI0);
    rcu_periph_clock_enable(RCU_GPIOA);

    /* SPI0 GPIO config: SCK/PA5, MISO/PA6, MOSI/PA7 */
    gpio_af_set(GPIOA, GPIO_AF_0, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
    spi_parameter_struct spi_init_struct;

    /* deinitilize SPI and the parameters */
    spi_i2s_deinit(SPI0);
    spi_struct_para_init(&spi_init_struct);
    
    /* SPI0 parameter config */
    spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode          = SPI_MASTER;
    spi_init_struct.frame_size           = SPI_FRAMESIZE_8BIT;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
    spi_init_struct.nss                  = SPI_NSS_SOFT;
    spi_init_struct.prescale             = SPI_PSC_16;
    spi_init_struct.endian               = SPI_ENDIAN_MSB;
    spi_init(SPI0, &spi_init_struct);
    
    spi_enable(SPI0);
#endif
}

void app_spi_tx(uint8_t *write_data, uint8_t *rx_dummy_data, uint32_t len)
{
#ifndef GD32E230
    esp_err_t err;

    spi_transaction_t t = 
    {
        .flags = 0,
        .length = 8 * len,
        .tx_buffer = write_data,
        .rx_buffer = rx_dummy_data
    };

    gpio_set_level(CONFIG_CS_GPIO, 0);
    err = spi_device_transmit(m_spi_dev, &t);
    if (err != ESP_OK)
    {
        printf("SPI error %d\r\n", err);
        assert(0);
    }

    gpio_set_level(CONFIG_CS_GPIO, 1);
#else
    gpio_bit_reset(GPIOA, GPIO_PIN_4);      // cs low
    volatile uint32_t timeout = 0x000FF;
    while (timeout--)
    {
        
    }
    
    bool invalid = false;
    for (uint32_t i = 0; i < len; i++)
    {
        timeout = 0x000FFFFF;
        while (RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE) && timeout--);
        if (timeout == 0)
        {
            invalid = true;
        }
        spi_i2s_data_transmit(SPI0, write_data[i]);
        
        timeout = 0x000FFFFF;
        while (RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE) && timeout--);
        if (timeout == 0)
        {
            invalid = true;
        }
        rx_dummy_data[i] = spi_i2s_data_receive(SPI0);
    }
    gpio_bit_set(GPIOA, GPIO_PIN_4);
    if (invalid)
    {
        APP_DBG_ERR("SPI error\r\n");
    }
#endif
}

void app_spi_rx(uint8_t *rx_data, uint8_t *tx_dummy_data, uint32_t len)
{
#ifndef GD32E230
    esp_err_t err;
    
    if (len > 256)
    {
        printf("SPI rx len %u\r\n", len);
    }
    
    spi_transaction_t t = 
    {
        .flags = 0,
        .length = 8 * len,
        .tx_buffer = tx_dummy_data,
        .rx_buffer = rx_data
    };

    gpio_set_level(CONFIG_CS_GPIO, 0);
//    volatile uint32_t delay = 0xFF;
//    while (delay--);

    err = spi_device_transmit(m_spi_dev, &t);
    if (err != ESP_OK)
    {
        printf("SPI error %d\r\n", err);
        assert(0);
    }

    gpio_set_level(CONFIG_CS_GPIO, 1);

#else

    gpio_bit_reset(GPIOA, GPIO_PIN_4);
    volatile uint32_t timeout = 0x00000FF;
//    while (timeout--);
    
    for (uint32_t i = 0; i < len; i++)
    {
        timeout = 0x000FFFFF;
        while (RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE) && timeout--);
        spi_i2s_data_transmit(SPI0, tx_dummy_data[i]);
        
        timeout = 0x000FFFFF;
        while (RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE) && timeout--);
        rx_data[i] = spi_i2s_data_receive(SPI0);
    }
    
    gpio_bit_set(GPIOA,GPIO_PIN_4);
#endif
}