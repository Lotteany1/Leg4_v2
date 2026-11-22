/** @file  cmsis_os2.h  @brief MPC test clock hooks  @version  v1.0  2026-10-26 */
#ifndef TEST_CMSIS_OS2_H
#define TEST_CMSIS_OS2_H
#include <stdint.h>
typedef void *osThreadId_t;
#define osFlagsWaitAny 0u
#define osFlagsError 0x80000000u
#define osWaitForever 0xffffffffu
uint32_t osKernelGetTickCount(void);
int osDelayUntil(uint32_t tick);
int osDelay(uint32_t ticks);
void osThreadExit(void);
uint32_t osThreadFlagsSet(osThreadId_t thread, uint32_t flags);
uint32_t osThreadFlagsWait(uint32_t flags, uint32_t options, uint32_t timeout);
#endif
