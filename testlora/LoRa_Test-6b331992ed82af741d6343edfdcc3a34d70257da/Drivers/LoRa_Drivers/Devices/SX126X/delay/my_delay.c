
#include "my_delay.h"

void DelayMs(uint32_t delayMS){
	HAL_Delay(1000*delayMS);
}
