#include "app_btn.h"

#include <stddef.h>
#include <string.h>

#define BUTTON_FILTER_COUNT 5

static uint8_t m_btn_data[APP_BTN_MAX_BTN_SUPPORT] = {0};
static uint32_t m_btn_press_evt = 0;
static uint32_t m_btn_release_evt = 0;
static uint32_t m_btn_hold_evt = 0;
static uint32_t m_btn_hold_evt_exec = 0;
static uint32_t m_btn_on_hold_evt = 0;
static uint32_t m_btn_hold_timestamp[APP_BTN_MAX_BTN_SUPPORT] = {0};
static uint32_t m_btn_on_hold_count[APP_BTN_MAX_BTN_SUPPORT] = {0};
static uint32_t m_btn_last_press_time[APP_BTN_MAX_BTN_SUPPORT] = {0};
static uint8_t m_btn_press_cnt[APP_BTN_MAX_BTN_SUPPORT] = {0};
static uint32_t m_btn_instances[APP_BTN_MAX_BTN_SUPPORT];
static app_btn_hw_config_t *m_hw_params;
static app_btn_evt_handler_t m_callback_table[APP_BTN_EVT_MAX] = {0};
static void * m_custom_data[APP_BTN_EVT_MAX] = {0};
static app_btn_config_t m_btn_config;

bool app_btn_initialize(app_btn_config_t *conf)
{
    uint8_t i = 0;

    if (!conf)
    {
        return false;
    }

    if (APP_BTN_MAX_BTN_SUPPORT < conf->btn_count)
    {
        return false;
    }
    
    memset(&m_btn_config, 0, sizeof(app_btn_config_t));
    memcpy(&m_btn_config, conf, sizeof(app_btn_config_t));
    m_hw_params = m_btn_config.config;
    
    for (i = 0; i < m_btn_config.btn_count; i++)
    {
        m_btn_instances[i] = m_hw_params[i].pin;
        if (m_btn_config.btn_initialize)
            m_btn_config.btn_initialize(m_hw_params[i].pin);    // Init GPIO pin as input for btn read
        
        m_hw_params[i].debounce_val = 0;
        m_hw_params[i].debounce_counter = 0;
        m_hw_params[i].last_state = m_hw_params[i].idle_level;
        m_btn_data[i] = m_hw_params[i].idle_level;
    }

    return true;
}



void app_btn_scan(void* params)
{
    uint8_t i = 0;
    uint8_t level = 0; 
    if (!m_hw_params)
    {
        return;
    }

    if (APP_BTN_MAX_BTN_SUPPORT < m_btn_config.btn_count)
    {
        return;
    }

    for (i = 0; i < m_btn_config.btn_count; i++)
    {
        level = m_btn_config.btn_read(m_btn_instances[i]) ? 1 : 0;
        m_hw_params[i].debounce_val += level;
        m_hw_params[i].debounce_counter++;

        // Read 8 times, if debounce_val = 0xFF =>> mean 1, debounce_val = 0x00 =>> mean 0
        // Else debounce_val != 0xFF && debounce_val != 0x00 ->> noise ->> poll again
        if (m_hw_params[i].debounce_counter < BUTTON_FILTER_COUNT)
        {
            continue;;
        }

        if (m_hw_params[i].debounce_val == BUTTON_FILTER_COUNT)
        {
            m_hw_params[i].debounce_val = BUTTON_FILTER_COUNT - 1;
            m_hw_params[i].debounce_counter = BUTTON_FILTER_COUNT;
            level = 1;
        }
        else if (m_hw_params[i].debounce_val == 0x00)
        {
            m_hw_params[i].debounce_val = BUTTON_FILTER_COUNT - 1;
            m_hw_params[i].debounce_counter = BUTTON_FILTER_COUNT;
            level = 0;
        }
        else        // Invalid value
        {
            m_hw_params[i].debounce_counter = 0;
            m_hw_params[i].debounce_val = 0;
            continue;;
        }

        m_btn_data[i] = level;
        if (m_btn_data[i] ^ m_hw_params[i].last_state)
        {
            if (m_btn_data[i] != m_hw_params[i].idle_level)
            {
                m_btn_press_evt |= 1 << i;
                m_btn_hold_timestamp[i] = m_btn_config.get_tick_cb();
            }
            else
            {
                m_btn_release_evt |= 1 << i;
                m_btn_hold_timestamp[i] = 0;
                m_btn_on_hold_count[i] = 0;
            }
        }
        m_hw_params[i].last_state = m_btn_data[i];

        uint32_t now = m_btn_config.get_tick_cb();
        if (m_btn_data[i] != m_hw_params[i].idle_level)
        {
            if (now - m_btn_hold_timestamp[i] >= APP_BTN_HOLD_SO_LONG_TIME_MS)
            {
                if (((m_btn_hold_evt >> i) & 0x1))
                {
                    if (now - m_btn_on_hold_count[i] >= APP_BTN_ON_HOLD_TIME_FIRE_EVENT_MS)
                    {
                        m_btn_on_hold_count[i] = m_btn_config.get_tick_cb();
                        m_btn_on_hold_evt |= (1 << i);
                    }
                }
                else
                {
                    m_btn_on_hold_count[i] = now;
                    m_btn_hold_evt |= (1 << i);
                    m_btn_hold_evt_exec &= ~(1 << i); // clear executed flag
                }
            }
        }

        if ((m_btn_press_evt >> i) & 0x01)
        {
            if (m_btn_last_press_time[i] == 0)
            {
                m_btn_last_press_time[i] = now;
            }

            if (now - m_btn_last_press_time[i] >= APP_BTN_DOUBLE_CLICK_TIME_MS)
            {
                m_btn_press_cnt[i] = 0;
            }
            else
            {
                m_btn_press_cnt[i] += 1;

                uint32_t evt = APP_BTN_EVT_MAX;

                if (m_btn_press_cnt[i] == 1) // double click
                {
                    evt = APP_BTN_EVT_DOUBLE_CLICK;
                }
                else if (m_btn_press_cnt[i] == 2)
                {
                    evt = APP_BTN_EVT_TRIPLE_CLICK;
                    m_btn_press_cnt[i] = 0;
                }

                if (evt != APP_BTN_EVT_MAX)
                {
                    if (NULL != m_callback_table[evt])
                    {
                        m_callback_table[evt](m_hw_params[i].pin, evt, m_custom_data[evt]);
                    }
                }
            }

            m_btn_last_press_time[i] = m_btn_config.get_tick_cb();

            if (NULL != m_callback_table[APP_BTN_EVT_PRESSED])
            {
                m_callback_table[APP_BTN_EVT_PRESSED](m_hw_params[i].pin, 
                                                        APP_BTN_EVT_PRESSED, 
                                                        m_custom_data[APP_BTN_EVT_PRESSED]);
            }
            m_btn_press_evt &= ~(1 << i);
        }
        if ((m_btn_release_evt >> i) & 0x01)
        {
            // APP_DEBUG("Hard btn release event, btn idx [%d]", i);
            if (NULL != m_callback_table[APP_BTN_EVT_RELEASED])
            {
                m_callback_table[APP_BTN_EVT_RELEASED](m_hw_params[i].pin, 
                                                        APP_BTN_EVT_RELEASED, 
                                                        m_custom_data[APP_BTN_EVT_RELEASED]);
            }
            m_btn_release_evt &= ~(1 << i);
            m_btn_hold_evt &= ~(1 << i); // clear
        }

        if (!((m_btn_hold_evt_exec >> i) & 0x1) &&  ((m_btn_hold_evt >> i) & 0x1))
        {
            m_btn_hold_evt_exec |= (1 << i);
            uint32_t hold_time = m_btn_config.get_tick_cb() - m_btn_hold_timestamp[i];
            if (m_callback_table[APP_BTN_EVT_HOLD_SO_LONG]) 
            {
                m_callback_table[APP_BTN_EVT_HOLD_SO_LONG](m_hw_params[i].pin, 
                                                            APP_BTN_EVT_HOLD_SO_LONG, 
                                                            (void*)&hold_time);
            }
        }

        if ((m_btn_on_hold_evt >> i) & 0x1)
        {
            uint32_t hold_time = m_btn_config.get_tick_cb() - m_btn_hold_timestamp[i];
            if (m_callback_table[APP_BTN_EVT_HOLD]) 
            {
                m_callback_table[APP_BTN_EVT_HOLD](m_hw_params[i].pin, 
                                                    APP_BTN_EVT_HOLD, 
                                                    (void*)&hold_time);
            }
            m_btn_on_hold_evt &= ~(1 << i);
        }
    }
}


 void app_btn_register_callback(app_btn_event_t event, app_btn_evt_handler_t cb, void* data)
{
    if (cb) 
    {
        m_callback_table[event] = cb;
        m_custom_data[event] = data;
    }
}

void app_btn_reset_state(void)
{
    m_btn_press_evt = 0;
    m_btn_release_evt = 0;
    m_btn_hold_evt = 0;
    m_btn_hold_evt_exec = 0;
    m_btn_on_hold_evt = 0;

    for (uint32_t i = 0; i < m_btn_config.btn_count; i++)
    {
        m_hw_params[i].last_state = m_hw_params[i].idle_level;
        m_btn_data[i] = m_hw_params[i].idle_level;
    }
}
