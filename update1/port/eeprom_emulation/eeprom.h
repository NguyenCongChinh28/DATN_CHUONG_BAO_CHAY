/*********************************************************************************************************//**
 * @file    EEPROM_Emulation/EEPROM_Emulation/eeprom.h
 * @version $Rev:: 1710         $
 * @date    $Date:: 2017-08-17 #$
 * @brief   The header file of EEPROM emulation APIs.
 *************************************************************************************************************
 * @attention
 *
 * Firmware Disclaimer Information
 *
 * 1. The customer hereby acknowledges and agrees that the program technical documentation, including the
 *    code, which is supplied by Holtek Semiconductor Inc., (hereinafter referred to as "HOLTEK") is the
 *    proprietary and confidential intellectual property of HOLTEK, and is protected by copyright law and
 *    other intellectual property laws.
 *
 * 2. The customer hereby acknowledges and agrees that the program technical documentation, including the
 *    code, is confidential information belonging to HOLTEK, and must not be disclosed to any third parties
 *    other than HOLTEK and the customer.
 *
 * 3. The program technical documentation, including the code, is provided "as is" and for customer reference
 *    only. After delivery by HOLTEK, the customer shall use the program technical documentation, including
 *    the code, at their own risk. HOLTEK disclaims any expressed, implied or statutory warranties, including
 *    the warranties of merchantability, satisfactory quality and fitness for a particular purpose.
 *
 * <h2><center>Copyright (C) Holtek Semiconductor Inc. All rights reserved</center></h2>
 ************************************************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------------------------------------*/
#ifndef __EEPROM_H
#define __EEPROM_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------------------------------------*/
#include "gd32e23x.h"

typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;
/* Exported types ------------------------------------------------------------------------------------------*/
typedef enum
{
  PAGE0_ACTIVE    = 0,
  PAGE1_ACTIVE    = 1,
  NO_ACTIVE_PAGE  = 2,
  FLASH_ERROR     = 3,
  PAGE_FULL       = 4,
  DATA_NOT_FOUND  = 5,
  OPERATION_FAIL  = 6,
  OPERATION_OK    = 7
}EEPROM_State;

/* Exported constants --------------------------------------------------------------------------------------*/
#define LIBCFG_FLASH_PAGESIZE       (1024)
#define LIBCFG_FLASH_SIZE           (0x10000 - 4*1024)

#define PAGE_SIZE             (LIBCFG_FLASH_PAGESIZE)
#define EEPROM_START_ADDR     (LIBCFG_FLASH_SIZE - LIBCFG_FLASH_PAGESIZE*2 + ((uint32_t)0x08000000U))    /*!< EEPROM emulation use Last two PAGE of the Flash memory */

#define PAGE0_BASE_ADDR       (EEPROM_START_ADDR + 0x000)
#define PAGE0_END_ADDR        (EEPROM_START_ADDR + (PAGE_SIZE - 1))

#define PAGE1_BASE_ADDR       (EEPROM_START_ADDR + PAGE_SIZE)
#define PAGE1_END_ADDR        (EEPROM_START_ADDR + (2 * PAGE_SIZE - 1))

#define ERASED                (0xFFFF)
#define TRANSFER              (0xCCCC)
#define ACTIVE                (0x0000)

#define READ_FROM_ACTIVE_PAGE (0)
#define WRITE_TO_ACTIVE_PAGE  (1)

#define NUM_OF_VAR            (4)             /*!< The number of variables is limited to less than 254 */

/* Exported macro ------------------------------------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------------------------------------*/
EEPROM_State EEPROM_Init(void);
EEPROM_State EEPROM_Read(u16 DataAddr, u16 *Data);
EEPROM_State EEPROM_Write(u16 DataAddr, u16 Data);
EEPROM_State EEPROM_Format(void);

#ifdef __cplusplus
}
#endif

#endif
