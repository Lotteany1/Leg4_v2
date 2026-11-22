#include "cmsis_os2.h"
#include "task_sync.h"
#include "system_key.h"
#include "monitor.h"

#include "music.h"
#include "songs.h"
#include "dr16.h"
#include "imu.h"
#include "can_bus.h"
#include "dm_motor.h"
#include "dji_motor.h"
#include "balance.h"
#include "body.h"
#include "air.h"
#include "app_gyro.h"
#include "app_step.h"
#include "slip.h"
volatile bool task_ready;

void StartLedTask(void *argument);
extern osThreadId_t VofaTaskHandle;

static const osThreadAttr_t led_task_attr =
    {
        .name       = "LedTask",
        .stack_size = 256u * 4u,
        .priority   = osPriorityNormal,
};

void StartInitTask(void *argument)
{
    Monitor_Init();

    // CAN 总线初始化
    Dm_Motor_Init();
    Dji_Motor_Init();
    Can_Bus_Init();

    // Imu 初始化
    Imu_Init();

    // 遥控初始化
    Dr16_Init();

    // 平衡控制初始化
    Body_State_Init();
    Balance_Control_Init();

    // 打滑速度融合初始化
    Slip_Init();

    // 离地检测初始化
    Air_Init();
    App_Step_Init();

    // 其他模块初始化
#if APP_USE_MUSIC
    Music_Init();
#endif
    App_Gyro_Init();

    osThreadNew(StartLedTask, NULL, &led_task_attr);
    // 控制与估计优先于后台求解 遥测留在低优先级
    osThreadSetPriority(ControlTaskHandle, osPriorityHigh);
    osThreadSetPriority(EstimatorTaskHandle, osPriorityHigh);
    osThreadSetPriority(VofaTaskHandle, osPriorityLow);
    // 在线模型初始化占用求解任务的 16 KiB 栈，发布后再唤醒控制与估计。
    while (!Balance_Mpc_Ready())
        osDelay(1u);
    task_ready = true;
    osThreadSuspend(osThreadGetId());

    for (;;)
    {
        osDelay(1000);
    }
}
