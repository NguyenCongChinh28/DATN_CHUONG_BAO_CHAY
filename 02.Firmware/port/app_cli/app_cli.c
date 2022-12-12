/******************************************************************************
 * @file:    app_cli.c
 * @brief:
 * @version: V0.0.0
 * @date:    2019/11/12
 * @author:
 * @email:
 *
 * THE SOURCE CODE AND ITS RELATED DOCUMENTATION IS PROVIDED "AS IS". VINSMART
 * JSC MAKES NO OTHER WARRANTY OF ANY KIND, WHETHER EXPRESS, IMPLIED OR,
 * STATUTORY AND DISCLAIMS ANY AND ALL IMPLIED WARRANTIES OF MERCHANTABILITY,
 * SATISFACTORY QUALITY, NON INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * THE SOURCE CODE AND DOCUMENTATION MAY INCLUDE ERRORS. VINSMART JSC
 * RESERVES THE RIGHT TO INCORPORATE MODIFICATIONS TO THE SOURCE CODE IN LATER
 * REVISIONS OF IT, AND TO MAKE IMPROVEMENTS OR CHANGES IN THE DOCUMENTATION OR
 * THE PRODUCTS OR TECHNOLOGIES DESCRIBED THEREIN AT ANY TIME.
 *
 * VINSMART JSC SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGE OR LIABILITY ARISING FROM YOUR USE OF THE SOURCE CODE OR
 * ANY DOCUMENTATION, INCLUDING BUT NOT LIMITED TO, LOST REVENUES, DATA OR
 * PROFITS, DAMAGES OF ANY SPECIAL, INCIDENTAL OR CONSEQUENTIAL NATURE, PUNITIVE
 * DAMAGES, LOSS OF PROPERTY OR LOSS OF PROFITS ARISING OUT OF OR IN CONNECTION
 * WITH THIS AGREEMENT, OR BEING UNUSABLE, EVEN IF ADVISED OF THE POSSIBILITY OR
 * PROBABILITY OF SUCH DAMAGES AND WHETHER A CLAIM FOR SUCH DAMAGE IS BASED UPON
 * WARRANTY, CONTRACT, TORT, NEGLIGENCE OR OTHERWISE.
 *
 * (C)Copyright VINSMART JSC 2019 All rights reserved
 ******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include "app_cli.h"
#include "app_shell.h"
#include "main.h"
// static const char *TAG = "cli";

static int32_t cli_enter_pair_mode(p_shell_context_t context, int32_t argc, char **argv);
static int32_t cli_reset(p_shell_context_t context, int32_t argc, char **argv);
//static int32_t cli_ota_update(p_shell_context_t context, int32_t argc, char **argv);
//static int32_t cli_get_server_state(p_shell_context_t context, int32_t argc, char **argv);
//static int32_t cli_change_mqtt_info(p_shell_context_t context, int32_t argc, char **argv);
//static int32_t cli_set_aes_key(p_shell_context_t context, int32_t argc, char **argv);
//static int32_t cli_factory_reset(p_shell_context_t context, int32_t argc, char **argv);
//static int32_t cli_disable_console(p_shell_context_t context, int32_t argc, char **argv);
//static int32_t cli_whitelist_server(p_shell_context_t context, int32_t argc, char **argv);
//static int32_t cli_get_firmware_version(p_shell_context_t context, int32_t argc, char **argv);

static const shell_command_context_t cli_command_table[] =
    {
        {"pair", "\tpair: Enter pair mode\r\n", cli_enter_pair_mode, 0},
        {"reset", "\treset: Reboot system\r\n", cli_reset, -1},
//        {"server", "\tserver: Set server info, [URL:PORT - username - pwd] mqtt://abc.vn:1883 admin 123456\r\n", cli_change_mqtt_info, -1},
//        {"ota", "\tota: Update firmware, format http://asdasd/bin\r\n", cli_ota_update, 1},
//        {"status", "\tstatus: Get server state\r\n", cli_get_server_state, 0},
//        {"disableconsole", "\tdisableconsole: Disable console\r\n", cli_disable_console, 0},
//        {"enc", "\tenc: Set encryption key\r\n", cli_set_aes_key, 1},
//        {"version", "\tversion: Get firmware version\r\n", cli_get_firmware_version, 0},
//        {"factory", "\tfactory: Factory reset\r\n", cli_factory_reset, 0},
//        {"whitelist", "\twhitelist: set-get whitelist servers addr\r\n", cli_whitelist_server, -1},
};

static shell_context_struct m_user_context;
static app_cli_cb_t *m_cb;

void app_cli_poll(uint8_t ch)
{
    app_shell_task(ch);
}

void app_cli_start(app_cli_cb_t *callback)
{
    m_cb = callback;
    app_shell_set_context(&m_user_context);
    app_shell_init(&m_user_context,
                   m_cb->puts,
                   m_cb->printf,
                   ">",
                   true);

    /* Register CLI commands */
    for (int i = 0; i < sizeof(cli_command_table) / sizeof(shell_command_context_t); i++)
    {
        app_shell_register_cmd(&cli_command_table[i]);
    }

    /* Run CLI task */
    app_shell_task(APP_SHELL_INVALID_CHAR);
}

/* Reset System */
    static int32_t cli_enter_pair_mode(p_shell_context_t context, int32_t argc, char **argv)
{
    extern void enter_state_get_network_information();
    enter_state_get_network_information();
    return 0;
}

static int32_t cli_reset(p_shell_context_t context, int32_t argc, char **argv)
{
    m_cb->printf("System will reset now\r\n");
    NVIC_SystemReset();
    return 0;
}

// extern void app_ota_start(char *url);
//static int32_t cli_ota_update(p_shell_context_t context, int32_t argc, char **argv)
//{
//    if (strstr(argv[1], "http://") || strstr(argv[1], "https://"))
//    {
//        ESP_LOGI(TAG, "OTA url %s\r\n", argv[1]);
//        // Create ota task
//        static char *url = NULL;
//        if (!url)
//        {
//            static app_ota_info_t info;
//            url = malloc(strlen(argv[1]) + 128);
//            info.url = url;
//            info.type = APP_OTA_DEVICE_ESP32;
//            sprintf(url, "%s", argv[1]);
//            m_cb->printf("Update firmware on url : ");
//            m_cb->printf(url);
//            m_cb->printf("\r\n");
//            vTaskDelay(50/portTICK_PERIOD_MS);
//            xTaskCreate(app_ota_download_task, "ota_task", 8192, (void*)&info, 5, NULL);
//            // Ko dc free contro URL
//        }
//    }

//    return 0;
//}

// static int32_t cli_get_current_time(p_shell_context_t context, int32_t argc, char **argv)
// {
//     // app_sntp_debug_timenow();
//     return 0;
// }

// static int32_t cli_get_config(p_shell_context_t context, int32_t argc, char **argv)
// {
//     if (strstr(argv[1], "dump"))
//     {
//         //		internal_flash_cfg_t *cfg = internal_flash_get_config();
//         //		(void)cfg;
//         //		ESP_LOGI(TAG, "\t\tConfig addr %s:%d\r\n\tPing nterval %ums", cfg->host_addr, cfg->port, cfg->ping_cycle);
//     }
//     return 0;
// }

//static int32_t cli_change_mqtt_info(p_shell_context_t context, int32_t argc, char **argv)
//{
//    if (argc == 4)      // url, username, pass
//    {
//        ESP_LOGI(TAG, "MQTT url %s, username %s, password %s", argv[1], argv[2], argv[3]);
//        char *reply = malloc(strlen(argv[1]) + strlen(argv[2]) + strlen(argv[3]) + 256);
//        if (reply)
//        {
//            app_flash_node_nvs_write_string(APP_FLASH_KEY_MQTT_SERVER_URL, argv[1]);
//            app_flash_node_nvs_write_string(APP_FLASH_KEY_MQTT_SERVER_USERNAME, argv[2]);
//            app_flash_node_nvs_write_string(APP_FLASH_KEY_MQTT_SERVER_PASSWORD, argv[3]);
//            sprintf(reply, "Set mqtt URL %s, username %s, password %s. Device will be reboot within 5s", argv[1], argv[2], argv[3]);
//            m_cb->printf(reply);
//            free(reply);
//            vTaskDelay(4000);
//            m_cb->printf("\r\nREBOOT now\r\n");
//            vTaskDelay(1000);
//            esp_restart();
//        }
//    }
//    else if (argc == 2)
//    {
//        char *reply = malloc(strlen(argv[1]) + 128);
//        if (reply)
//        {
//            app_flash_node_nvs_write_string(APP_FLASH_KEY_MQTT_SERVER_URL, argv[1]);
//            app_flash_node_nvs_write_string(APP_FLASH_KEY_MQTT_SERVER_USERNAME, "");
//            app_flash_node_nvs_write_string(APP_FLASH_KEY_MQTT_SERVER_PASSWORD, "");
//            sprintf(reply, "Set mqtt URL %s, username NULL, password NULL. Device will be within 5s", argv[1]);
//            m_cb->printf(reply);
//            free(reply);
//            vTaskDelay(4000);
//            m_cb->printf("\r\nREBOOT now\r\n");
//            vTaskDelay(1000);
//            esp_restart();
//        }
//    }
//    else
//    {
//        m_cb->printf("Invalid mqtt arguments");
//    }
//    return 0;
//}

//static int32_t cli_get_server_state(p_shell_context_t context, int32_t argc, char **argv)
//{
//    static const char *server_state[] = 
//    {
//        "Server state =>> Disconnected\r\n",
//        "Server state =>> Connecting\r\n",
//        "Server state =>> Connected\r\n"
//    };
//    m_cb->printf(server_state[app_mqtt_get_state()]);
//    return 0;
//}

// static int32_t cli_set_master(p_shell_context_t context, int32_t argc, char **argv)
// {
//     if (strlen(argv[1]) == 15)
//     {
//         memcpy(app_flash_get_master(APP_FLASH_MASTER_HUYEN1), argv[1], 15);
//         app_flash_node_nvs_write_string(APP_FLASH_MASTER_H1_KEY, app_flash_get_master(APP_FLASH_MASTER_HUYEN1));
//         m_cb->printf("Set master ");
//         m_cb->printf(argv[1]);
//         m_cb->printf("\r\nSystem will be reboot after 1s\r\n");
//         vTaskDelay(1000/portTICK_PERIOD_MS);
//         esp_restart();
//     }
//     return 0;
// }

//static int32_t cli_factory_reset(p_shell_context_t context, int32_t argc, char **argv)
//{
//    m_cb->printf("Factory reset\r\n");
//    m_cb->printf("\r\nSystem will be reboot after 1s\r\n");
//    app_flash_tcp_console_disable();
//    vTaskDelay(1000/portTICK_PERIOD_MS);
//    esp_restart();
//    return 0;
//}

//static int32_t cli_disable_console(p_shell_context_t context, int32_t argc, char **argv)
//{
//    m_cb->printf("Disable console reset\r\n");
//    m_cb->printf("\r\nSystem will be reboot after 1s\r\n");
//    return 0;
//}

//static void dump_whitelist_server(void)
//{
////    char *server0;
////    char *server1;
////    app_flash_get_whitelist_server(&server0, &server1);
////    ESP_LOGI(TAG, "Server %s, %s", server0, server1);
////    m_cb->printf("=>> Current whitelist servers : ");
////    m_cb->printf(server0);
////    m_cb->printf(", ");
////    m_cb->printf(server1);
////    m_cb->printf("\r\n");
//}

//static int32_t cli_whitelist_server(p_shell_context_t context, int32_t argc, char **argv)
//{
//    if (argc == 2 && (strcmp(argv[1], "get") == 0))
//    {
//        dump_whitelist_server();
//    }
//    else if (argc == 4 && (strcmp(argv[1], "set") == 0))
//    {
//        ESP_LOGI(TAG, "Set whitelist servers %s-%s\r\n", argv[2], argv[3]);
//        bool do_reset = app_flash_set_whitelist_server(argv[2], argv[3]);
//        dump_whitelist_server();

//        if (do_reset)
//        {
//            m_cb->printf("Server changed, device will be reset in 3s\r\n");
//            vTaskDelay(3000);
//            esp_restart();
//        }
//    }
//   return 0;
//}

//static int32_t cli_set_aes_key(p_shell_context_t context, int32_t argc, char **argv)
//{
//    if (strlen(argv[1]) == 16 && strcmp(argv[1], (char*)app_flash_get_aes_key()))
//    {
//        m_cb->printf("Set new encryption key ");
//        app_flash_set_aes_key((uint8_t*)argv[1]);
//        m_cb->printf((char*)app_flash_get_aes_key());
//        m_cb->printf("\r\nr\n");
//    }
//    return 0;
//}

//static int32_t cli_get_firmware_version(p_shell_context_t context, int32_t argc, char **argv)
//{
//    m_cb->printf(__FIRMWARE_VERSION__);
//    m_cb->printf("\r\n\r\n\r\n");
//    return 0;
//}
