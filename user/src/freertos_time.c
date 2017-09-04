//freertos
#include "FreeRTOS.h"
#include "task.h"

#include "freertos_time.h"

void delay_ms(uint32_t delay_time)
{
    // As long as the configTICK_RATE_HZ = portTICK_PERIOD_MS = 100000
    // times 100 is required. With 1000 (default) instead of 100000, just remove
    // the 100. This is a quick fix that should get a better solution.
    vTaskDelay((SYSTICK_FREQUENCY / 1000) * (delay_time / portTICK_PERIOD_MS));
}

int get_ms_count(unsigned long *count)
{
    count[0] = xTaskGetTickCount() / portTICK_PERIOD_MS;
	return 0;
}

unsigned long get_tick_count() // Same as ms in my case?
{
    return xTaskGetTickCount();
}