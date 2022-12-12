#include "app_flash.h"
#include <string.h>
#include "main.h"
#include "app_debug.h"
#define INFO_ADDR           (0x0807F800)

void app_flash_get_mac_addr(uint8_t *mac)
{
    uint32_t id;
    
    id = HAL_GetUIDw0();
    memcpy(mac + 2, &id, 4);
    
    id = HAL_GetUIDw1();
    mac[0] = (id & 0x0000FF00) >> 8;
    mac[1] = id & (0x000000FF);
}

bool app_flash_infor_is_exist_in_flash(void)
{
    app_flash_network_info_t *data = (app_flash_network_info_t*)INFO_ADDR;
    
    if (data->flag == APP_FLASH_FLAG_WRITTEN)
    {
        //APP_DBG_INF("return true");
        return true;
        

    }
    //APP_DBG_INF("return false");
    return false;
  
}

void app_flash_read_info(app_flash_network_info_t *info)
{
    if (info)
    {
        memcpy(info, (app_flash_network_info_t*)INFO_ADDR, sizeof(app_flash_network_info_t));
    }
}

void app_flash_store_info(app_flash_network_info_t *info)
{
    if (info)
    {
        /* Step 1 Erase flash */

        uint32_t page_err = 0;
        FLASH_EraseInitTypeDef erase_init;
        
        // Unlock the flash program/erase controller
        HAL_FLASH_Unlock();
        // Clear all pending flags
          /* Clear all FLASH flags */
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
     
        // Erase the flash pages
        erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
        erase_init.PageAddress = INFO_ADDR;
        erase_init.Banks = FLASH_BANK_1;
        erase_init.NbPages = 1;
        HAL_FLASHEx_Erase(&erase_init, &page_err);

        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
        
        uint32_t address = INFO_ADDR;
        uint32_t *wr_data = (uint32_t*)info;
        
        for (uint32_t i = 0; i < (sizeof(app_flash_network_info_t)/sizeof(uint32_t)); i++)
        {
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, wr_data[i]);
            address += 4U;
            __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
        }
    
        /* lock the main FMC after the erase operation */
        HAL_FLASH_Lock();
    }
}