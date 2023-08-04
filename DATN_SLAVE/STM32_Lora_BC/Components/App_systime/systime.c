#include "systime.h"

static volatile uint32_t c_current_ms = 0;

__attribute__((weak)) void systime_lock(void)
{
}

__attribute__((weak)) void systime_unlock(void)
{
}

__attribute__((weak)) void systime_update(void)
{
    
}
void systime_create(systime_t * const timer_id)
{
    systime_update();
    systime_lock();
    if (timer_id)
    {
        timer_id->tick = c_current_ms;
        timer_id->started = 1;
    }
    systime_unlock();
    
}
void systime_set_value(uint32_t ms)
{
   systime_lock();
   c_current_ms = ms;
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


   
