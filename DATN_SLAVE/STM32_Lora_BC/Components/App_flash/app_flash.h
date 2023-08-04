#ifndef APP_FLASH_H
#define APP_FLASH_H

#include<stdint.h>
#include<stdbool.h>


#define APP_FLASH_FLAG_WRITTEN          0xAA

typedef struct
{
    uint16_t network_id;
    uint8_t device_id;
    uint8_t flag;
    uint32_t freq;
    uint32_t sync_alarm;
} __attribute__ ((packed)) app_flash_network_info_t;

/**
 *@brief   Get mac address in flash
 */
 
void app_get_mac_add_in_flash(uint8_t *mac);
/**
 *@brief   Check network information is in flash
 *@retval  True  Network information stored in flash
           False Network information is not stored in flash
 */
bool app_net_infor_is_exist_in_flash(void);

/**
 *@brief   Read network information in flash
 */
void app_read_net_info_in_flash(app_flash_network_info_t *net_info);

/**
 * @brief               Store network information into flash
 * @param[in]           info Network information
 */
void app_store_info_in_flash(app_flash_network_info_t *info);


        
 




#endif
