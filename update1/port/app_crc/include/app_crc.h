#ifndef APP_CRC_H
#define APP_CRC_H

#include "stdint.h"

/**
 * @brief               Compute CRC16
 * @param[in]           buffer : Data will be computed CRC
 * @param[in]           len : Buffer length (number of uint16_t elements)
 * @retval              CRC16 value
 */
uint16_t app_crc_compute(uint16_t *buffer, uint32_t len);

#endif /* APP_CRC */
