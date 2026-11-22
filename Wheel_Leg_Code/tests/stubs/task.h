/** @file  task.h  @brief Model test critical section macros  @version  v1.0  2026-10-25 */
#ifndef TEST_TASK_H
#define TEST_TASK_H

#include "FreeRTOS.h"

#define taskENTER_CRITICAL() Test_Enter_Critical()
#define taskEXIT_CRITICAL()  Test_Exit_Critical()

#endif
