/*********************************************************************************************************/ /**
 * @file    EEPROM_Emulation/EEPROM_Emulation/eeprom.c
 * @version $Rev:: 1710         $
 * @date    $Date:: 2017-08-17 #$
 * @brief   The source file of EEPROM emulation APIs.
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

/* Includes ------------------------------------------------------------------------------------------------*/
#include "eeprom.h"
#include "app_debug.h"

#define wb(addr, value) (*((u8 volatile *)(addr)) = value)
#define rb(addr) (*((u8 volatile *)(addr)))
#define whw(addr, value) (*((u16 volatile *)(addr)) = value)
#define rhw(addr) (*((u16 volatile *)(addr)))
#define ww(addr, value) (*((u32 volatile *)(addr)) = value)
#define rw(addr) (*((u32 volatile *)(addr)))

typedef enum
{
    FLASH_COMPLETE = 0,
    FLASH_ERR_ADDR_OUT_OF_RANGE,
    FLASH_ERR_WRITE_PROTECTED,
    FLASH_TIME_OUT
} FLASH_State;

/** @addtogroup EEPROM_Emulation_Examples EEPROM_Emulation
  * @{
  */

/** @addtogroup EEPROM_Emulation
  * @{
  */

/* Private typedef -----------------------------------------------------------------------------------------*/
/* Private define ------------------------------------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------------------------------------*/
extern u16 EEPROM_VarAddrTab[NUM_OF_VAR];
u16 VarData;

/* Private function prototypes -----------------------------------------------------------------------------*/
EEPROM_State EEPROM_Format(void);
static EEPROM_State EEPROM_FindActivePage(u8 Operation);
static EEPROM_State EEPROM_WriteActivePage(u16 DataAddr, u16 Data);
static EEPROM_State EEPROM_TransferPage(void);

extern void system_irq_control(bool en);
/* Private functions ---------------------------------------------------------------------------------------*/

/*********************************************************************************************************/ /**
  * @brief  EEPROM init
  * @retval OPERATION_OK, OPERATION_FAIL, FLASH_ERROR
  ***********************************************************************************************************/

FLASH_State FLASH_ErasePage(uint32_t addr)
{
	uint32_t *p = (uint32_t*)addr;
	bool need_erase = false;
	for (uint32_t i = 0; i < PAGE_SIZE / 4; i++)
	{
		if (p[i] != 0xFFFFFFFF)
		{
			need_erase = true;
			break;
		}
	}
	
	if (!need_erase)
	{
		APP_DBG_ERR("We dont need to erase at addr 0x%08X\r\n", addr);
		return FLASH_COMPLETE;
	}
	
	system_irq_control(false);
    uint32_t retval = FLASH_COMPLETE;
    fmc_unlock();
    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR);
    // Erase the flash pages
    retval = fmc_page_erase(addr);
    fmc_lock();

    if (retval)
    {
        retval = FLASH_TIME_OUT;
    }
	APP_DBG_VERBOSE("Flash erase page 0x%08X ret %d\r\n", addr, retval);
	system_irq_control(true);
    return retval;
}

FLASH_State FLASH_ProgramWordData(uint32_t addr, uint32_t value)
{
	system_irq_control(false);
    uint32_t retval = FLASH_COMPLETE;

    if (*((volatile uint32_t *)addr) == value)
    {
        APP_DBG_VERBOSE("Same value, ignore write\r\n");
        system_irq_control(true);
        return retval;
    }

    fmc_unlock();
    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR);
    retval = fmc_word_program(addr, value);
    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR);
    fmc_lock();

    if (retval)
    {
		APP_DBG_ERR("[%s] Flash timeout\r\n", __FUNCTION__);
        retval = FLASH_TIME_OUT;
    }
	
    if (*((volatile uint32_t *)addr) != value)
    {
        APP_DBG_ERR("Write flash failed, readback error\r\n");
		retval = FLASH_TIME_OUT;
    }
	else
	{
		APP_DBG_VERBOSE("Write flash at addr success 0x%08X, value %d\r\n", addr, value);
	}
	
	system_irq_control(true);
    return retval;
}

EEPROM_State EEPROM_Init(void)
{
    FLASH_State FlashStatus;
    EEPROM_State EepromStatus;
    u16 Page0Status = rhw(PAGE0_BASE_ADDR);
    u16 Page1Status = rhw(PAGE1_BASE_ADDR);

    switch (Page0Status)
    {
    case ERASED:
        if (Page1Status == ACTIVE)
        {
            /* erase page0 */
            FlashStatus = FLASH_ErasePage(PAGE0_BASE_ADDR);

            if (FlashStatus != FLASH_COMPLETE)
            {
                APP_DBG_ERR("Erase page 0 error");
                return FLASH_ERROR;
            }
        }
        else if (Page1Status == TRANSFER)
        {
            /* erase page0 */
            FlashStatus = FLASH_ErasePage(PAGE0_BASE_ADDR);

            if (FlashStatus != FLASH_COMPLETE)
            {
                APP_DBG_ERR("Erase page 1 error");
                return FLASH_ERROR;
            }

            /* set page1 active */
            FlashStatus = FLASH_ProgramWordData(PAGE1_BASE_ADDR, ACTIVE);

            if (FlashStatus != FLASH_COMPLETE)
            {
                APP_DBG_ERR("[%s] FLASH_ProgramWordData error in line %u\r\n", __FUNCTION__, __LINE__);
                return FLASH_ERROR;
            }
        }
        else
        {
            /* format EEPROM */
            EepromStatus = EEPROM_Format();

            if (EepromStatus != OPERATION_OK)
            {
                APP_DBG_ERR("Format error on line %d\r\n", __LINE__);
                return FLASH_ERROR;
            }
        }
        break;

    case TRANSFER:
        if (Page1Status == ERASED)
        {
            /* erase page1 */
            FlashStatus = FLASH_ErasePage(PAGE1_BASE_ADDR);

            if (FlashStatus != FLASH_COMPLETE)
            {
                return FLASH_ERROR;
            }

            /* set page0 active */
            FlashStatus = FLASH_ProgramWordData(PAGE0_BASE_ADDR, ACTIVE);

            if (FlashStatus != FLASH_COMPLETE)
            {
                return FLASH_ERROR;
            }
        }
        else if (Page1Status == ACTIVE)
        {
            /* erase page0 */
            FlashStatus = FLASH_ErasePage(PAGE0_BASE_ADDR);

            if (FlashStatus != FLASH_COMPLETE)
            {
                APP_DBG_ERR("Format page0 addr error on line %d\r\n", __LINE__);
                return FLASH_ERROR;
            }
			else
			{
				APP_DBG_ERR("Format page0 addr  OK on line %d\r\n", __LINE__);
			}

            /* redo the transfer */
            EepromStatus = EEPROM_TransferPage();

            if (EepromStatus != OPERATION_OK)
            {
				APP_DBG_ERR("[%s] Transfer page error in line %u\r\n", __FUNCTION__, __LINE__);
                return EepromStatus;
            }
        }
        else
        {
            /* format EEPROM */
            EepromStatus = EEPROM_Format();

            if (EepromStatus != OPERATION_OK)
            {
                APP_DBG_ERR("Format error on line %d\r\n", __LINE__);
                return FLASH_ERROR;
            }
			else
			{
				APP_DBG_ERR("Format ok on line %d\r\n", __LINE__);
			}
        }
        break;

    case ACTIVE:
        if (Page1Status == ERASED)
        {
            /* erase page1 */
            FlashStatus = FLASH_ErasePage(PAGE1_BASE_ADDR);

            if (FlashStatus != FLASH_COMPLETE)
            {
                APP_DBG_ERR("Format error on line %d\r\n", __LINE__);
                return FLASH_ERROR;
            }
			else
			{
				APP_DBG_ERR("Page 1 format ok in line %d\r\n", __LINE__);
			}
        }
        else if (Page1Status == TRANSFER)
        {
            /* erase page1 */
            FlashStatus = FLASH_ErasePage(PAGE1_BASE_ADDR);

            if ((FlashStatus != FLASH_COMPLETE) || (0xFFFFFFFF != (*((uint32_t*)PAGE1_BASE_ADDR))))
            {
                APP_DBG_ERR("Format at addr 0x%08X error on line %d\r\n", PAGE1_BASE_ADDR, __LINE__);
                return FLASH_ERROR;
            }

            /* redo the transfer */
            EepromStatus = EEPROM_TransferPage();

            if (EepromStatus != OPERATION_OK)
            {
                APP_DBG_ERR("[%s] Transfer on line %d\r\n", __FUNCTION__, __LINE__);
                return EepromStatus;
            }
        }
        else
        {
            /* format EEPROM */
            EepromStatus = EEPROM_Format();

            if (EepromStatus != OPERATION_OK)
            {
                APP_DBG_ERR("Format error on line %d\r\n", __LINE__);
                return FLASH_ERROR;
            }
        }
        break;

    default:
        /* format EEPROM */
        EepromStatus = EEPROM_Format();

        if (EepromStatus != OPERATION_OK)
        {
            APP_DBG_ERR("Format error on line %d\r\n", __LINE__);
            return FLASH_ERROR;
        }
        break;
    }

    return OPERATION_OK;
}

/*********************************************************************************************************/ /**
  * @brief  Read Variable from EEPROM
  * @para   DataAddr: variable address
  * @para   Data: variable value
  * @retval OPERATION_OK, NO_ACTIVE_PAGE, DATA_NOT_FOUND
  ***********************************************************************************************************/
EEPROM_State EEPROM_Read(u16 DataAddr, u16 *Data)
{
    EEPROM_State ActivePage;
    u32 StopAddr, CompareAddr;

    /* get active page */
    ActivePage = EEPROM_FindActivePage(READ_FROM_ACTIVE_PAGE);

    if (ActivePage == NO_ACTIVE_PAGE)
    {
        return NO_ACTIVE_PAGE;
    }

    /* search for latest data */
    StopAddr = EEPROM_START_ADDR + ((u32)ActivePage * PAGE_SIZE + 4);          // skip page status field
    CompareAddr = EEPROM_START_ADDR + ((u32)(1 + ActivePage) * PAGE_SIZE - 2); // start from page end

    while (CompareAddr > StopAddr)
    {
        if (rhw(CompareAddr) == DataAddr)
        {
            *Data = rhw(CompareAddr - 2);

            return OPERATION_OK;
        }
        else
        {
            CompareAddr -= 4;
        }
    }

    return DATA_NOT_FOUND;
}

/*********************************************************************************************************/ /**
  * @brief  
  * @retval OPERATION_OK, NO_ACTIVE_PAGE, FLASH_ERROR 
  ***********************************************************************************************************/
EEPROM_State EEPROM_Write(u16 DataAddr, u16 Data)
{
    EEPROM_State EepromStatus;

    EepromStatus = EEPROM_WriteActivePage(DataAddr, Data);

    /* check page full */
    if (EepromStatus == PAGE_FULL)
    {
        EepromStatus = EEPROM_TransferPage();
    }
    else if (EepromStatus == OPERATION_FAIL)
    {
        EepromStatus = EEPROM_TransferPage();

        if (EepromStatus != OPERATION_OK)
        {
			APP_DBG_ERR("[%s] Transfer page error in line %u\r\n", __FUNCTION__, __LINE__);
            return EepromStatus;
        }

        EepromStatus = EEPROM_WriteActivePage(DataAddr, Data);
    }
	if (EepromStatus != OPERATION_OK)
	{
		APP_DBG_ERR("[%s] Transfer page error %d in line %u\r\n", __FUNCTION__, EepromStatus,__LINE__);
	}

    return EepromStatus;
}

/*********************************************************************************************************/ /**
  * @brief  Erase page0, page1 and set page0 active
  * @retval OPERATION_OK, FLASH_ERROR
  ***********************************************************************************************************/
EEPROM_State EEPROM_Format(void)
{
    FLASH_State FlashStatus;

    /* erase page0 */
    FlashStatus = FLASH_ErasePage(PAGE0_BASE_ADDR);

    if (FlashStatus != FLASH_COMPLETE)
    {
        return FLASH_ERROR;
    }

    /* erase page1 */
    FlashStatus = FLASH_ErasePage(PAGE1_BASE_ADDR);

    if (FlashStatus != FLASH_COMPLETE)
    {
        return FLASH_ERROR;
    }

    /* set page0 active */
    FlashStatus = FLASH_ProgramWordData(PAGE0_BASE_ADDR, ACTIVE);

    if (FlashStatus != FLASH_COMPLETE)
    {
        return FLASH_ERROR;
    }

    return OPERATION_OK;
}

/*********************************************************************************************************/ /**
  * @brief  
  * @retval PAGE0_ACTIVE, PAGE1_ACTIVE, NO_ACTIVE_PAGE
  ***********************************************************************************************************/
static EEPROM_State EEPROM_FindActivePage(u8 Operation)
{
    u16 Page0Status = rhw(PAGE0_BASE_ADDR);
    u16 Page1Status = rhw(PAGE1_BASE_ADDR);

    switch (Operation)
    {
    case WRITE_TO_ACTIVE_PAGE:
        if (Page1Status == ACTIVE)
        {
            if (Page0Status == TRANSFER)
            {
                return PAGE0_ACTIVE;
            }
            else
            {
                return PAGE1_ACTIVE;
            }
        }
        else if (Page0Status == ACTIVE)
        {
            if (Page1Status == TRANSFER)
            {
                return PAGE1_ACTIVE;
            }
            else
            {
                return PAGE0_ACTIVE;
            }
        }
        else
        {
            return NO_ACTIVE_PAGE;
        }

    case READ_FROM_ACTIVE_PAGE:
        if (Page0Status == ACTIVE)
        {
            return PAGE0_ACTIVE;
        }
        else if (Page1Status == ACTIVE)
        {
            return PAGE1_ACTIVE;
        }
        else
        {
            return NO_ACTIVE_PAGE;
        }

    default:
        return PAGE0_ACTIVE;
    }
}

/*********************************************************************************************************/ /**
  * @brief  
  * @retval OPERATION_OK, NO_ACTIVE_PAGE, FLASH_ERROR, PAGE_FULL, OPERATION_FAIL 
  ***********************************************************************************************************/
static EEPROM_State EEPROM_WriteActivePage(u16 DataAddr, u16 Data)
{
    FLASH_State FlashStatus;
    EEPROM_State ActivePage;
    u32 StopAddr, CompareAddr;

    /* get active page */
    ActivePage = EEPROM_FindActivePage(WRITE_TO_ACTIVE_PAGE);

    if (ActivePage == NO_ACTIVE_PAGE)
    {
        return NO_ACTIVE_PAGE;
    }

    /* search for lst empty location */
    StopAddr = EEPROM_START_ADDR + ((u32)(1 + ActivePage) * PAGE_SIZE);  // page end address
    CompareAddr = EEPROM_START_ADDR + ((u32)ActivePage * PAGE_SIZE + 4); // start from page head

    while (CompareAddr < StopAddr)
    {
        if (rw(CompareAddr) == 0xFFFFFFFF)
        {
            FlashStatus = FLASH_ProgramWordData(CompareAddr, ((u32)(DataAddr << 16) | Data));

            if (FlashStatus != FLASH_COMPLETE)
            {
                return FLASH_ERROR;
            }

            if (CompareAddr == (StopAddr - 4))
            {
                return PAGE_FULL;
            }

            return OPERATION_OK;
        }
        else
        {
            CompareAddr += 4;
        }
    }

    return OPERATION_FAIL;
}

/*********************************************************************************************************/ /**
  * @brief  
  * @retval OPERATION_OK, NO_ACTIVE_PAGE, FLASH_ERROR
  ***********************************************************************************************************/
static EEPROM_State EEPROM_TransferPage(void)
{
    FLASH_State FlashStatus;
    EEPROM_State EepromStatus;
    u32 NewPage, OldPage, i;

    /* get NewPage & OldPage */
    EepromStatus = EEPROM_FindActivePage(READ_FROM_ACTIVE_PAGE);

    if (EepromStatus == PAGE1_ACTIVE)
    {
        NewPage = PAGE0_BASE_ADDR;
        OldPage = PAGE1_BASE_ADDR;
    }
    else if (EepromStatus == PAGE0_ACTIVE)
    {
        NewPage = PAGE1_BASE_ADDR;
        OldPage = PAGE0_BASE_ADDR;
    }
    else
    {
        APP_DBG_ERR("[%s] No active page in line %u\r\n", __FUNCTION__, __LINE__);
        return NO_ACTIVE_PAGE;
    }

    /* set NewPage state as TRANSFER */
    FlashStatus = FLASH_ProgramWordData(NewPage, TRANSFER);

    if (FlashStatus != FLASH_COMPLETE)
    {
		APP_DBG_ERR("[%s] Program word error in line %u\r\n", __FUNCTION__, __LINE__);
        return FLASH_ERROR;
    }

    /* transfer data */
    for (i = 0; i < NUM_OF_VAR; i++)
    {
        /* write other variables */
        EepromStatus = EEPROM_Read(EEPROM_VarAddrTab[i], &VarData);

        if (EepromStatus != DATA_NOT_FOUND)
        {
            EepromStatus = EEPROM_WriteActivePage(EEPROM_VarAddrTab[i], VarData);

            if (EepromStatus != OPERATION_OK)
            {
                return EepromStatus;
            }
        }
    }

    /* erase OldPage */
    FlashStatus = FLASH_ErasePage(OldPage);

    if (FlashStatus != FLASH_COMPLETE)
    {
		APP_DBG_ERR("[%s] Erase old page error in line %u\r\n", __FUNCTION__, __LINE__);
        return FLASH_ERROR;
    }
	APP_DBG_VERBOSE("[%s] Erase old 0x%08X page ok in line %u\r\n", __FUNCTION__, OldPage, __LINE__);
    /* set NewPage state as ACTIVE */
    FlashStatus = FLASH_ProgramWordData(NewPage, ACTIVE);

    if (FlashStatus != FLASH_COMPLETE)
    {
		APP_DBG_VERBOSE("[%s] Program new page 0x%08X page ok in line %u\r\n", __FUNCTION__, NewPage, __LINE__);
        return FLASH_ERROR;
    }

    return OPERATION_OK;
}

/**
  * @}
  */

/**
  * @}
  */
