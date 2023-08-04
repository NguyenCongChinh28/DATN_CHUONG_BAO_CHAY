#ifndef APP_LED_STATUS_H
#define APP_LED_STATUS_H

#include <stdint.h>
#include <stdbool.h>

#ifndef APP_MAX_LED_SUPPORT
#define APP_MAX_LED_SUPPORT             (24)
#endif

typedef struct
{
    uint8_t idx;
    uint8_t pin;
} app_led_index_t;

typedef uint32_t (*app_led_get_tick_cb_t)(void);

typedef void (*app_led_post_blink_cb_t)(uint8_t pin);

typedef void (*app_led_gpio_toggle_t)(uint32_t pin);
typedef void (*app_led_gpio_ctrl_t)(uint32_t pin, uint32_t value);
typedef uint32_t (*app_led_read_gpio_t)(uint32_t pin);

typedef struct
{
    app_led_get_tick_cb_t tick_cb;
    app_led_post_blink_cb_t post_blink_cb;
    app_led_gpio_toggle_t toggle;
    app_led_gpio_ctrl_t set;
    app_led_read_gpio_t get;
    app_led_index_t * led_idx;
    uint8_t led_cnt;
    uint8_t blink_enable;
} app_led_status_cfg_t;


/**
 * @brief Execute the blink pattern
 *
 * @param[in]  led_pin   Led pin
 * @param[in]  count     Number of blink
 * @param[in]  on_time   On time period in ms
 * @param[in]  off_time  Off time period in ms
 */
void app_led_blink(uint8_t led_pin, uint32_t count, uint16_t on_time_ms, uint16_t off_time_ms);

/**
 * @brief Stop blink pattern
 *
 * @param[in]  pin led pin
 */
void app_led_stop_blink(uint8_t pin);


/**
 * @brief Start led status service
 */
void app_led_blink_enable(void);

/**
 * @brief Stop led status service
 */
void app_led_blink_disable(void);

/**
 * @brief Led blink task
 */
void app_led_blink_scan(void);

/**
 * @brief Initialize led service
 * @param[in]  conf  Led configuration
 */
void app_led_initialize(app_led_status_cfg_t * conf);

/**
 * @brief Get led blink count
 * @param[in]  led_pin Led pin
 * @retval Number of led blink count remaining
 */
uint32_t app_led_get_blink_cnt(uint8_t led_pin);

#endif

