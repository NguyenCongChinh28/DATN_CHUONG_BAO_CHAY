#include "app_led_status.h"
#include "string.h"
#include "stdio.h"

#define APP_LED_PIN_INVALID           (0xFF)

typedef struct
{
    uint8_t status;
    uint8_t blink_state;
    uint8_t last_blink_state;
    uint16_t on_time_ms;
    uint16_t off_time_ms;
    uint32_t blink_count;
    uint32_t last_time;
} app_led_status_t;

static app_led_status_t m_led_ctx[APP_MAX_LED_SUPPORT];

static app_led_status_cfg_t m_cfg;

void app_led_blink_enable(void)
{
    m_cfg.blink_enable = 1;
}

void app_led_blink_disable(void)
{
    m_cfg.blink_enable = 0;
}

static inline uint32_t find_led_index_by_pin(uint8_t pin)
{
    if (m_cfg.led_idx == NULL)
        return APP_LED_PIN_INVALID;

    for (uint8_t led_index = 0; led_index < m_cfg.led_cnt; led_index++)
    {
        if (m_cfg.led_idx[led_index].pin == pin)
        {
            return m_cfg.led_idx[led_index].idx;
        }
    }

    return APP_LED_PIN_INVALID;
}

void app_led_blink(uint8_t led_pin, uint32_t count, uint16_t on_time_ms, uint16_t off_time_ms)
{
    uint8_t led_idx = find_led_index_by_pin(led_pin);

    if (led_idx == APP_LED_PIN_INVALID)
    {
        return;
    }

    m_led_ctx[led_idx].blink_count = count;
    m_led_ctx[led_idx].on_time_ms = on_time_ms;
    m_led_ctx[led_idx].off_time_ms = off_time_ms;
    m_led_ctx[led_idx].last_time = m_cfg.tick_cb ? m_cfg.tick_cb() : 0;
    m_led_ctx[led_idx].status = 1;
    m_led_ctx[led_idx].blink_state = 1;
    m_led_ctx[led_idx].last_blink_state = 0;
}

void app_led_stop_blink(uint8_t led_pin)
{
    uint8_t led_idx = find_led_index_by_pin(led_pin);

    if (led_idx == APP_LED_PIN_INVALID)
    {
        return;
    }

    m_led_ctx[led_idx].blink_count = 0;
    m_led_ctx[led_idx].on_time_ms = 0;
    m_led_ctx[led_idx].off_time_ms = 0;
    m_led_ctx[led_idx].last_time = 0;
    m_led_ctx[led_idx].status = 0;
    m_led_ctx[led_idx].blink_state = 0;
    m_led_ctx[led_idx].last_blink_state = 0;

    if (m_cfg.get(led_idx))
    {
    	m_cfg.set(led_idx, 0);
    }
    
}

void app_led_blink_scan(void)
{
    uint8_t i = 0;

    if (!m_cfg.blink_enable)
    {
        return;
    }

    for (i = 0; i < m_cfg.led_cnt; i++)
    {
        if (!m_led_ctx[i].status)
        {
            continue;
        }
        
        if (m_led_ctx[i].last_time)
        {
            if (m_led_ctx[i].blink_state ^ m_led_ctx[i].last_blink_state)
            {
                m_led_ctx[i].last_blink_state = m_led_ctx[i].blink_state;
                if (m_led_ctx[i].blink_state)
                {
                    if (m_cfg.set && m_cfg.get)
                    {
                        if (m_cfg.get && !m_cfg.get(m_cfg.led_idx[i].pin))
                        {
                            m_cfg.set(m_cfg.led_idx[i].pin, 1);
                        }
                    }
                }
                else
                {
                    if (m_cfg.set && m_cfg.get)
                    {
                        if (m_cfg.get && m_cfg.get(m_cfg.led_idx[i].pin))
                        {
                            m_cfg.set(m_cfg.led_idx[i].pin, 0);
                        }
                    }
                }
            }

            if (m_led_ctx[i].blink_state)
            {
                if (m_cfg.tick_cb() - m_led_ctx[i].last_time >= m_led_ctx[i].on_time_ms)
                {
                    m_led_ctx[i].last_time = m_cfg.tick_cb();
                    m_led_ctx[i].blink_state = 0;
                }
            }
            else
            {
                if (m_cfg.tick_cb() - m_led_ctx[i].last_time >= m_led_ctx[i].off_time_ms)
                {
                    m_led_ctx[i].last_time = m_cfg.tick_cb();
                    m_led_ctx[i].blink_state = 1;
                    m_led_ctx[i].blink_count--;
                }
            }
        }
        else
        {
            m_led_ctx[i].last_time = m_cfg.tick_cb();
        }

        if (m_led_ctx[i].blink_count == 0)
        {
            m_led_ctx[i].status = 0;
            if (m_cfg.post_blink_cb)
            {
                m_cfg.post_blink_cb(m_cfg.led_idx[i].pin);
            }
        }
    }
}

uint32_t app_led_get_max_led_support(void)
{
    return APP_MAX_LED_SUPPORT;
}

void app_led_initialize(app_led_status_cfg_t * conf)
{
    // assert(confg);
    memcpy(&m_cfg, conf, sizeof(app_led_status_cfg_t));
    if (m_cfg.led_cnt > APP_MAX_LED_SUPPORT)
        m_cfg.led_cnt = APP_MAX_LED_SUPPORT;
}

uint32_t app_led_get_blink_cnt(uint8_t led_pin)
{
    uint8_t led_idx = find_led_index_by_pin(led_pin);

    if (led_idx == APP_LED_PIN_INVALID)
    {
        return 0;
    }

    return m_led_ctx[led_idx].blink_count;
}

