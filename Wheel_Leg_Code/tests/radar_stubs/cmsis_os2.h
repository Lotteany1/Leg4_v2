/** @file  cmsis_os2.h  @brief 雷达启动回归所需的任务时钟桩  @version  v1.0  2026-10-31 */
#ifndef RADAR_TEST_CMSIS_OS2_H
#define RADAR_TEST_CMSIS_OS2_H
#include <stdint.h>
typedef void *osThreadId_t;
uint32_t osKernelGetTickCount(void);
uint32_t osKernelGetTickFreq(void);
int32_t osDelay(uint32_t ticks);
int32_t osDelayUntil(uint32_t ticks);
#endif
