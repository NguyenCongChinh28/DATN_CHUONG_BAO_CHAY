#include "net_info.h"
#include <string.h>
#include "app_crc.h"
#include <stdio.h>
#include "nvs_flash.h"
#include "app_spiffs.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "net_info"
#define APP_SPIFFS_DEVICE_CONFIG		    APP_SPIFFS_PATH"/conf"

static volatile uint32_t m_number_of_nodes = 0;

bool net_info_load(net_info_t *net_info)
{
    printf("Flash init\r\n");

    static bool inited = false;
    if (inited == false)
    {
		esp_err_t err = nvs_flash_init();

		if (err == ESP_ERR_NVS_NO_FREE_PAGES)
		{
			ESP_ERROR_CHECK(nvs_flash_erase());
			err = nvs_flash_init();
		}

		ESP_LOGW(TAG, "Internal flash init\r\n");
		if (app_spiffs_init() == false)
		{
			printf("Flash error\r\n");
			esp_restart();
			return false;
		}
		inited = true;
    }

    uint32_t size = sizeof(net_info_t);
    if (!app_spiffs_check_exist_file(APP_SPIFFS_DEVICE_CONFIG))
    {
        return false;
    }

    if (!app_spiffs_read(APP_SPIFFS_DEVICE_CONFIG, (uint8_t*)net_info, &size) 
        || size != sizeof(net_info_t))
    {
        printf("Flash read device configuration error\r\n");
        vTaskDelay(100);
        esp_restart();
    }

    m_number_of_nodes = 0;
    
    printf("Network information\r\nNetwork id %u\r\n", 
            net_info->network_id);
    
    for (uint32_t i = 0; i < NET_INFO_MAX_NODE_ALLOWED; i++)
    {
        if (net_info->node_id[i].id != NET_INFO_ID_INVALID)
        {
            printf("Node mac addr %02X%02X%02X%02X%02X%02X\r\n",
                    net_info->node_id[i].mac[0],
                    net_info->node_id[i].mac[1],
                    net_info->node_id[i].mac[2],
                    net_info->node_id[i].mac[3],
                    net_info->node_id[i].mac[4],
                    net_info->node_id[i].mac[5]);
            m_number_of_nodes++;
        }
    }

    return true;
}


bool net_info_store_default(net_info_t *net_info)
{
    for (uint8_t i = 0; i < NET_INFO_MAX_NODE_ALLOWED; i++)
    {
        net_info->node_id[i].id = NET_INFO_ID_INVALID;      // mart as invalid node
    }


    if (!app_spiffs_write(APP_SPIFFS_DEVICE_CONFIG, (uint8_t*)net_info, sizeof(net_info_t)))
    {
        printf("Flash write device configuration error\r\n");
        return false;
    }

    m_number_of_nodes = 0;
    return true;
}


bool net_info_store_current_info(net_info_t *net_info)
{
    if (!app_spiffs_write(APP_SPIFFS_DEVICE_CONFIG, (uint8_t*)net_info, sizeof(net_info_t)))
    {
        printf("Flash write device configuration error\r\n");
        return false;
    }

    return true;
}


int8_t net_info_get_node_id(net_info_t *net_info, uint8_t *mac)
{
    for (uint32_t i = 0; i < NET_INFO_MAX_NODE_ALLOWED; i++)
    {
        if (memcmp(net_info->node_id[i].mac, mac, 6) == 0 
            && net_info->node_id[i].id != NET_INFO_ID_INVALID)
        {
            return net_info->node_id[i].id;
        }
    }
    return -1;
}

bool net_info_node_id_exist(net_info_t *net_info, int8_t node_id)
{
    if (node_id != -1 && node_id < NET_INFO_MAX_NODE_ALLOWED)
    {
        return (net_info->node_id[node_id].id != NET_INFO_ID_INVALID);
    }

    return false;
}

bool net_info_insert_node(net_info_t *net_info, uint8_t *mac, uint8_t *offer_id)
{
    int8_t first_free = -1;

    for (uint32_t i = 0; i < NET_INFO_MAX_NODE_ALLOWED; i++)
    {
        if (memcmp(net_info->node_id[i].mac, mac, 6) == 0)
        {
            *offer_id = i;
            return true;
        }
        else if (first_free == -1)
        {
            first_free = i;
        }
    }

    if (first_free != -1)
    {
        memcpy(net_info->node_id[first_free].mac, mac, 6);
        net_info->node_id[first_free].id = first_free;
        *offer_id = first_free;
        m_number_of_nodes++;
        if (!net_info_store_current_info(net_info))
        {
            printf("Cannot store information\r\n");
            vTaskDelay(100);
            esp_restart();
        }
        return true;
    }

    return false;
}

void net_info_remove_node(net_info_t *net_info, uint8_t *mac)
{
    for (uint32_t i = 0; i < NET_INFO_MAX_NODE_ALLOWED; i++)
    {
        if (memcmp(net_info->node_id[i].mac, mac, 6) == 0)
        {
            net_info->node_id[i].id = NET_INFO_ID_INVALID;
            memset(net_info->node_id[i].mac, 0x00, 6);
            if (m_number_of_nodes)
            {
                m_number_of_nodes--;
            }
        }
    }

    if (!net_info_store_current_info(net_info))
    {
        printf("Cannot store information\r\n");
        vTaskDelay(100);
        esp_restart();
    }
}

bool net_info_override_node(net_info_t *net_info, uint8_t *mac, uint8_t offer_id)
{
    if (offer_id > NET_INFO_MAX_NODE_ALLOWED)
    {
        return false;
    }

    for (uint32_t i = 0; i < NET_INFO_MAX_NODE_ALLOWED; i++)
    {
        if (memcmp(net_info->node_id[i].mac, mac, 6) == 0)
        {
            net_info->node_id[i].id = NET_INFO_ID_INVALID;
            memset(net_info->node_id[i].mac, 0x00, 6);
        }
    }

    memcpy(net_info->node_id[offer_id].mac, mac, 6);
    net_info->node_id[offer_id].id = offer_id;
    
    if (!net_info_store_current_info(net_info))
    {
        printf("Cannot store information\r\n");
        vTaskDelay(100);
        esp_restart();
    }

    return true;
}

uint32_t net_info_get_number_of_nodes_paired(void)
{
    return m_number_of_nodes;
}

void net_info_delele_all(void)
{
    app_spiffs_del_stored_file(APP_SPIFFS_DEVICE_CONFIG);
    vTaskDelay(1000);
    esp_restart();
}



