/** @file  estimator_task.c  @brief 1 ms 状态估计与控制快照时标交接  @version  v1.7  2026-11-01 */
#include "cmsis_os2.h"
#include "task_sync.h"
#include "code_config.h"
#include "system_key.h"
#include "dr16.h"

#include "leg.h"
#include "body.h"
#include "imu.h"
#include "dm_motor.h"
#include "dji_motor.h"
#include "imu_to_world.h"
#include "slip.h"
#include "air.h"
#include "monitor.h"
#include "stm32h7xx.h"

static volatile uint32_t estimator_tick;
static volatile uint32_t estimator_cycle; // 当前估计快照 DWT 时标，握手期间保持

uint32_t Estimator_Frame_Tick(void)
{
    return estimator_tick;
}

// 与估计 tick 同一握手快照；控制归还前保持，后台与 CAN 事件使用同一 DWT 时基。
uint32_t Estimator_Frame_Cycle(void)
{
    return estimator_cycle;
}

void StartEstimatorTask(void *argument)
{
    while (!task_ready)
        osDelay(1u);
    static uint8_t count = 0;
    uint32_t wake_tick   = osKernelGetTickCount();
    for (;;)
    {
        Dr16_Update();
        Imu_Parse();
        Imu_To_World();
        Dm_Motor_Parse();
        Dji_Motor_Parse();
        Leg_State_Update();
        Body_State_Update();
        Slip_Air_Update();
        Body_Position_Update();

        // 全局估计量就绪 控制本拍完成前不写下一拍
        estimator_tick = osKernelGetTickCount();
        estimator_cycle = DWT->CYCCNT;
        osThreadFlagsSet(ControlTaskHandle, TASK_ESTIMATE_READY);
        osThreadFlagsWait(TASK_CONTROL_DONE, osFlagsWaitAny, osWaitForever);

        if (count == 10)
        {
            count = 0;
        }

        count++;

        // 估计与控制都完成后再检查下一拍截止时刻 正常耗时计入 1 ms 周期
        uint32_t now = osKernelGetTickCount();

        // 截止时刻已到则记一次掉队 从当前时刻等满一周期 不追帧
        wake_tick += CTRL_PERIOD_MS;
        if ((int32_t)(wake_tick - now) <= 0)
        {
            monitor_win.overrun_cnt++;
            wake_tick = now + CTRL_PERIOD_MS;
        }
        osDelayUntil(wake_tick);
    }
}
