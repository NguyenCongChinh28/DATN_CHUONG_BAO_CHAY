/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: SX126x driver specific target board functions implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include "radio.h"
#include "sx126x.h"
#include "sx126x-board.h"
#include "board_hw.h"
#include "main.h"
#ifdef GD32E230
#include "gd32e23x.h"
#endif

#include "app_spi.h"
#include "app_debug.h"
#include <string.h>


#define SPI_MAX_BUFFER_SIZE 64

#define SPI_TX_BUFFER_INSERT(x, y)     (spi_buffer.tx_buffer[x++] = y)

struct
{
    uint8_t tx_buffer[SPI_MAX_BUFFER_SIZE];
    uint8_t rx_buffer[SPI_MAX_BUFFER_SIZE];
    uint32_t max_size;
} spi_buffer;



extern bool SX126xWaitOnBusy(uint32_t ms);
extern void SX126xReset(void);

void SX126xWakeup(void)
{
    uint32_t transfer_size = 0;
    
    SPI_TX_BUFFER_INSERT(transfer_size, RADIO_GET_STATUS);
    SPI_TX_BUFFER_INSERT(transfer_size, 0);
    
    app_spi_tx(spi_buffer.tx_buffer, spi_buffer.rx_buffer, transfer_size);

    // Wait for chip to be ready.
    SX126xWaitOnBusy(SX1276_WAIT_BUSY_TIME);
}

void SX126xWriteCommand(RadioCommands_t command, uint8_t *buffer, uint16_t size)
{
    // TODO check buffer size
    
    uint16_t i = 0;
    SX126xCheckDeviceReady();

    uint32_t transfer_size = 0;
    
    SPI_TX_BUFFER_INSERT(transfer_size, command);
    

    for (i = 0; i < size; i++)
    {
        SPI_TX_BUFFER_INSERT(transfer_size, buffer[i]);
    }

    app_spi_tx(spi_buffer.tx_buffer, spi_buffer.rx_buffer, transfer_size);

    if (command != RADIO_SET_SLEEP)
    {
        SX126xWaitOnBusy(SX1276_WAIT_BUSY_TIME);
    }
}

void SX126xReadCommand(RadioCommands_t command, uint8_t *buffer, uint16_t size)
{
    uint16_t i = 0;
    SX126xCheckDeviceReady();

    
    uint32_t transfer_size = 0;
    
    SPI_TX_BUFFER_INSERT(transfer_size, command);
    SPI_TX_BUFFER_INSERT(transfer_size, 0);
    

    for (i = 0; i < size; i++)
    {
        SPI_TX_BUFFER_INSERT(transfer_size, 0x00);
    }

    app_spi_rx(spi_buffer.rx_buffer, spi_buffer.tx_buffer, transfer_size);
    memcpy(buffer, &spi_buffer.rx_buffer[transfer_size-size], size);

    SX126xWaitOnBusy(SX1276_WAIT_BUSY_TIME);
}

void SX126xWriteRegisters(uint16_t address, uint8_t *buffer, uint16_t size)
{    
    uint16_t i = 0;
    SX126xCheckDeviceReady();

    uint32_t transfer_size = 0;
    
    SPI_TX_BUFFER_INSERT(transfer_size, RADIO_WRITE_REGISTER);
    SPI_TX_BUFFER_INSERT(transfer_size, (address & 0xFF00) >> 8);
    SPI_TX_BUFFER_INSERT(transfer_size, address & 0x00FF);
    

    for (i = 0; i < size; i++)
    {
        SPI_TX_BUFFER_INSERT(transfer_size, buffer[i]);
    }

    app_spi_tx(spi_buffer.tx_buffer, spi_buffer.rx_buffer, transfer_size);

    SX126xWaitOnBusy(SX1276_WAIT_BUSY_TIME);
    
}

void SX126xWriteRegister(uint16_t address, uint8_t value)
{
    SX126xWriteRegisters(address, &value, 1);
}

void SX126xReadRegisters(uint16_t address, uint8_t *buffer, uint16_t size)
{
//    DEBUG_PRINTF("Radio read register 0x%04X, size %dbytes\r\n", address, size);
    uint16_t i = 0;
    SX126xCheckDeviceReady();

    uint32_t transfer_size = 0;
    
    SPI_TX_BUFFER_INSERT(transfer_size, RADIO_READ_REGISTER);
    SPI_TX_BUFFER_INSERT(transfer_size, (address & 0xFF00) >> 8);
    SPI_TX_BUFFER_INSERT(transfer_size, address & 0x00FF);
    SPI_TX_BUFFER_INSERT(transfer_size, 0x00);

    for (i = 0; i < size; i++)
    {
        SPI_TX_BUFFER_INSERT(transfer_size, 0x00);
    }

    app_spi_rx(spi_buffer.rx_buffer, spi_buffer.tx_buffer, transfer_size);
    memcpy(buffer, &spi_buffer.rx_buffer[transfer_size-size], size);
    
    SX126xWaitOnBusy(SX1276_WAIT_BUSY_TIME);
}

uint8_t SX126xReadRegister(uint16_t address)
{
    uint8_t data;
    SX126xReadRegisters(address, &data, 1);
    return data;
}

void SX126xWriteBuffer(uint8_t offset, uint8_t *buffer, uint8_t size)
{
    uint16_t i = 0;
    SX126xCheckDeviceReady();

    uint32_t transfer_size = 0;
    
    SPI_TX_BUFFER_INSERT(transfer_size, RADIO_WRITE_BUFFER);
    SPI_TX_BUFFER_INSERT(transfer_size, offset);
    
    for (i = 0; i < size; i++)
    {
        SPI_TX_BUFFER_INSERT(transfer_size, buffer[i]);
    }

    app_spi_tx(spi_buffer.tx_buffer, spi_buffer.rx_buffer, transfer_size);

    SX126xWaitOnBusy(SX1276_WAIT_BUSY_TIME);
}

void SX126xReadBuffer(uint8_t offset, uint8_t *buffer, uint8_t size)
{    
    uint16_t i = 0;
    SX126xCheckDeviceReady();

    uint32_t transfer_size = 0;
    
    SPI_TX_BUFFER_INSERT(transfer_size, RADIO_READ_BUFFER);
    SPI_TX_BUFFER_INSERT(transfer_size, offset);
    SPI_TX_BUFFER_INSERT(transfer_size, 0x00);
    
    for (i = 0; i < size; i++)
    {
        SPI_TX_BUFFER_INSERT(transfer_size, buffer[i]);
    }

    app_spi_rx(spi_buffer.rx_buffer, spi_buffer.tx_buffer, transfer_size);
    memcpy(buffer, &spi_buffer.rx_buffer[transfer_size-size], size);
    
    SX126xWaitOnBusy(SX1276_WAIT_BUSY_TIME);
}

void SX126xSetRfTxPower(int8_t power)
{
    APP_DBG_VERBOSE("Set tx power %d\r\n", power);
    SX126xSetTxParams(power, RADIO_RAMP_40_US);
}

uint8_t SX126xGetPaSelect(uint32_t channel)
{
    //    if( GpioRead( &DeviceSel ) == 1 )
    //    {
    //        return SX1261;
    //    }
    //    else
    //    {
    //        return SX1262;
    //    }

    return SX1262;
}

void SX126xAntSwOn(void)
{
    //GpioInit( &AntPow, ANT_SWITCH_POWER, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
}

void SX126xAntSwOff(void)
{
    // GpioInit( &AntPow, ANT_SWITCH_POWER, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
}

bool SX126xCheckRfFrequency(uint32_t frequency)
{
    // Implement check. Currently all frequencies are supported
    return true;
}
