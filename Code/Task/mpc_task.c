/** @file  mpc_task.c  @brief TinyMPC14 后台求解与 RTOS 端口  @version  v1.4  2026-11-19 */
#include "balance.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "monitor.h"
#include "task.h"
#include "task_sync.h"
#include "tinympc14.h"

// TinyMPC14 预测节点与任务调度使用同一毫秒时标。
uint32_t Tinympc14_Port_Tick(void)
{
    return osKernelGetTickCount();
}

// 控制拍和求解任务原子交换模型、状态和预测序列。
void Tinympc14_Port_Enter_Critical(void)
{
    taskENTER_CRITICAL();
}

// 与进入临界区成对，恢复任务原有中断屏蔽级别。
void Tinympc14_Port_Exit_Critical(void)
{
    taskEXIT_CRITICAL();
}

// 正常按 4 ms 截止时刻调度。到期或超时后让出 1 tick，再以实际唤醒时刻起算。
void StartTinympc14Task(void *argument)
{
    (void)argument;
    Balance_Mpc_Init();
    while (!task_ready)
        osDelay(1u);
    uint32_t wake = osKernelGetTickCount();

    for (;;)
    {
        const uint32_t begin = Monitor_Begin();

        Balance_Mpc_Solve();
        Monitor_End(MONITOR_SLOT_MPC, begin);
        wake += BALANCE_MPC_PERIOD_MS;
        const uint32_t now = osKernelGetTickCount();

        if ((int32_t)(wake - now) <= 0)
        {
            monitor_total.mpc_deadline_misses++;
            osDelay(1u);
            wake = osKernelGetTickCount();
        }
        else
            osDelayUntil(wake);
    }
}
