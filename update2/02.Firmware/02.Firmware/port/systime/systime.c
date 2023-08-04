#include "systime.h"

static volatile uint32_t m_current_ms = 0;

__attribute__((weak)) void systime_lock(void)
{
}

__attribute__((weak)) void systime_unlock(void)
{
}

__attribute__((weak)) void systime_update(void)
{
    
}


//void systime_increase(uint32_t ms)
//{
//    systime_lock();
//    m_current_ms += ms;
//    systime_unlock();
//}

void systime_set_value(uint32_t ms)
{
    systime_lock();
    m_current_ms = ms;
    systime_unlock();
}

void systime_create(systime_t *const timer_id)
{
    systime_update();
    
    systime_lock();

    if (timer_id)
    {
        timer_id->tick = m_current_ms;
        timer_id->started = 1;
    }

    systime_unlock();
}

void systime_stop(systime_t *const timer_id)
{
    systime_lock();

    if (timer_id)
    {
        timer_id->started = 0;
    }

    systime_unlock();
}

bool systime_is_timer_started(const systime_t *timer_id)
{
    bool retval = false;
    systime_lock();

    if (timer_id && timer_id->started)
    {
        retval = true;
    }

    systime_unlock();
    return retval;
}

bool systime_is_timer_elapse(systime_t *const timer_id, const uint32_t timeout_ms)
{
    bool retval = false;
    systime_update();
    
    systime_lock();

    if (timer_id && timer_id->started)
    {
        uint32_t diff = 0;
        if (m_current_ms >= timer_id->tick)
        {
            diff = m_current_ms - timer_id->tick;
        }
        else
        {
            diff = 0xFFFFFFFFU - timer_id->tick + m_current_ms;
        }

        if (diff >= timeout_ms)
        {
            timer_id->started = 0;
            retval = true;
        }
    }
    else
    {
        if (!timer_id)
        {
            retval = true;
        }
        else if (timer_id->started)
        {
            retval = false;
        }
        
    }

    systime_unlock();
    return retval;
}

uint32_t systime_get_timer_elapsed_time(const systime_t *const timer_id)
{
    systime_update();
    
    systime_lock();

    uint32_t delay_count = 0;

    if (timer_id && timer_id->started)
    {
        // A timer is never equal to 0
        // The 0 value is reserved to the timer stoped
        if (timer_id->started)
        {
            if (m_current_ms >= timer_id->tick)
            {
                delay_count = m_current_ms - timer_id->tick;
            }
            else
            {
                delay_count = 0xFFFFFFFFU - timer_id->tick + m_current_ms;
            }
        }
    }

    systime_unlock();
    return delay_count;
}

uint32_t systime_get_ms(void)
{
    return m_current_ms;
}

