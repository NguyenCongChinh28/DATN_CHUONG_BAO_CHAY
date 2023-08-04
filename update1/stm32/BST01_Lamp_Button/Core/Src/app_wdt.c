#include "app_wdt.h"
#include "main.h"
#include "iwdg.h"

void app_wdt_start(void)
{
    
}

void app_wdt_feed(void)
{
   iwdg_reload();
}
