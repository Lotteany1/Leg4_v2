/** @file  app_safety.c  @brief 安全锁存与电机使能执行  @version  v3.6  2026-11-21 */
#include "app_safety.h"
#include "code_config.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "dr16.h"
#include "imu.h"
#include "monitor.h"
#include "distributor.h"

volatile uint8_t active_flag = APP_DISABLE;

static bool safety_latched;
static uint8_t safety_applied = APP_ENABLE;
static uint32_t safety_apply_tick;

// ! 运行请求只表达拨杆意图；遥控、IMU 或关节离线，就在本拍失能并锁存。
void App_Safety_Online_Check(bool control_requested)
{
    bool dr16_online = Dr16_Online();
    bool imu_online  = Imu_Online();
    bool dm_online   = Dm_Motor_Online();

    if (!control_requested)
        active_flag = APP_DISABLE;
    if (!dr16_online || !imu_online || !dm_online)
        App_Safety_Trip();
}

// 把当前请求发给电机；safety_applied 只记上次尝试的命令，四台确认由 FSM 读反馈。
void App_Safety_Apply_Flag(void)
{
    uint32_t now    = HAL_GetTick();
    uint32_t period = active_flag == APP_ENABLE ? APP_ENABLE_RETRY_MS : APP_DISABLE_RETRY_MS;
    if (active_flag == safety_applied && now - safety_apply_tick < period)
        return;
    safety_applied    = active_flag;
    safety_apply_tick = now;
    if (active_flag == APP_ENABLE)
    {
        Dm_Motor_All_Enable();
    }
    else
    {
        Execution_Invalidate();
        Dji_Motor_All_Stop();
        if (Dm_Motor_All_Disable() != HAL_OK)
            monitor_total.dm_disable_fail_cnt++;
    }
}

// 安全锁存只读 设备恢复不自动清零
bool App_Safety_Latched(void)
{
    return safety_latched;
}

// 与离线故障共用锁存与清锁存机制。
void App_Safety_Trip(void)
{
    active_flag = APP_DISABLE;
    safety_latched = true;
}

// 清离线锁存 唯一清零点 仅 FSM 失能态收到清锁存请求时调
void App_Safety_Reset(void)
{
    safety_latched = false;
}
