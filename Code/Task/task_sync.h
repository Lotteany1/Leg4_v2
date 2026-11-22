/** @file  task_sync.h  @brief 估计与控制任务交接  @version  v1.2  2026-11-01 */
#ifndef TASK_SYNC_H
#define TASK_SYNC_H

#include "cmsis_os2.h"
#include <stdbool.h>
#include <stdint.h>

#define TASK_ESTIMATE_READY 1u // 全局估计量就绪
#define TASK_CONTROL_DONE   2u // 本拍控制完成

extern volatile bool task_ready;
extern osThreadId_t ControlTaskHandle;
extern osThreadId_t EstimatorTaskHandle;

uint32_t Estimator_Frame_Tick(void);
uint32_t Estimator_Frame_Cycle(void); // 握手保护的 DWT 快照，与 Frame_Tick 成对消费

#endif
