#include "../stubs/FreeRTOS.h"
#include <stdint.h>
typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(x) (x)
uint32_t xTaskGetTickCount(void);
void vTaskDelay(uint32_t ticks);
uint32_t HAL_GetTick(void);
