/**
 * @file  control_task.c
 * @brief 1 ms 估计通知驱动控制任务：安全层 → 控制状态机，掉队只重置基准不追帧
 * @version  v1.9  2026-11-22
 */
#include "cmsis_os2.h"
#include "task_sync.h"
#include "fsm.h"
#include "app_safety.h"
#include "app_spring_bench.h"
#include "monitor.h"

// ! 一拍控制：只取一次遥控，先执行安全命令，再用同一份请求选动作。
void StartControlTask(void *argument)
{
    while (!task_ready)
        osDelay(1u);

    for (;;)
    {
#if SPRING_CAL_BENCH_ENABLE
        if (osThreadFlagsWait(TASK_ESTIMATE_READY, osFlagsWaitAny, 5u) != TASK_ESTIMATE_READY)
        {
            App_Spring_Bench_Tick(NULL);
            continue;
        }
#else
        osThreadFlagsWait(TASK_ESTIMATE_READY, osFlagsWaitAny, osWaitForever);
#endif
        uint32_t begin = Monitor_Begin();

        const dr16_t rc = Dr16_Snapshot();
#if SPRING_CAL_BENCH_ENABLE
        App_Spring_Bench_Tick(&rc);
#else
        const fsm_request_t req = Fsm_Read_Request(&rc);

        App_Safety_Online_Check(req.run);
        App_Safety_Apply_Flag();
        Fsm_Tick(&rc, &req);
#endif

        // 控制完成后交还全局估计量 下一拍由估计任务通知
        Monitor_End(MONITOR_SLOT_TICK, begin);
        monitor_can[0].heartbeat++;
        osThreadFlagsSet(EstimatorTaskHandle, TASK_CONTROL_DONE);
    }
}
