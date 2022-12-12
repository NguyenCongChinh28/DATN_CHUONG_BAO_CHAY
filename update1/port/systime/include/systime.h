#ifndef SYSTIME_H
#define SYSTIME_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    uint32_t tick;
    uint8_t started;
} systime_t;

/*
 * @brief           Set systime
 * @param[in] ms    Increase systime in miliseconds
 */
void systime_set_value(uint32_t ms);

/**
 * @brief           Increase systime
 * @param[in] ms    Increase systime in miliseconds
 */
void systime_increase(uint32_t ms);


void systime_create(systime_t * const timer_id);

/**
 * @brief Stop timer_id
 * @param[in] Timer id
 */
void systime_stop(systime_t * const timer_id);

/**
 * @brief Check timer status
 * @retval TRUE     Timer started
 *         FALSE    Timer not started
 */
bool systime_is_timer_started(const systime_t * timer_id);

/**
 * @brief Check if the timer is elapsed
 * @param[in] timer_id      Timer id
 * @param[in] timeout       Timeout in miliseconds
 */
bool systime_is_timer_elapse(systime_t * const timer_id, uint32_t timeout_ms);

/** 
 * @brief Get timer elapsed time
 * @param[in] timer_id Timer id
 * @retval Timer elease time
 */
uint32_t systime_get_timer_elapsed_time(const systime_t * const timer_id);

/** 
 * @brief Get current systime in ms
 * @retval Timer tick
 */
uint32_t systime_get_ms(void);

/** 
 * @brief Lock systime for critical session
 */
__attribute__((weak)) void systime_lock(void);

/** 
 * @brief UnLock systime critical session
 */
__attribute__((weak)) void systime_unlock(void);



#endif /* SYSTIME_H */

