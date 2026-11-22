/** @file  distributor_runtime.c  @brief 真实驱动与分配链的主机 CAN/时钟夹具  @version  v1.0  2026-11-01 */
#include "stm32h7xx.h"
#include "can_bus.h"
#include "balance.h"
#include "air.h"
#include "tinympc14.h"
#include "app_self_rescue.h"
#include "body.h"
#include "leg.h"
#include "imu.h"
#include "monitor.h"
#include <assert.h>
#include <string.h>

test_dwt_t test_dwt;
uint32_t SystemCoreClock = 550000000u;
FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;
body_state_t body;
#ifndef TEST_LANDING
#include "fsm_position_stub.h"
#endif
leg_state_t leg_lft, leg_rgt;
imu_t imu;
volatile monitor_total_t monitor_total;
pid_t sr_pid_leg_len_lft, sr_pid_leg_len_rgt, sr_pid_leg_symmetry;
pid_t sr_pid_leg_ang_lft, sr_pid_leg_ang_rgt;
pid_t sr_pid_leg_omg_swing_lft, sr_pid_leg_omg_swing_rgt;
pid_t sr_pid_leg_omg_stand_lft, sr_pid_leg_omg_stand_rgt;
uint32_t test_tick, test_frame_cycle, test_fail_mask, test_bus_epoch, test_send_count;
uint32_t test_sample_delay; // ms，模拟状态就绪到控制处理之间的调度延迟
uint32_t test_id[5];
uint8_t test_payload[5][8];
static unsigned int test_critical_depth;

void Test_Enter_Critical(void)
{
    test_critical_depth++;
}
void Test_Exit_Critical(void)
{
    assert(test_critical_depth);
    test_critical_depth--;
}
uint32_t Estimator_Frame_Tick(void)
{
    return test_tick - test_sample_delay;
}
uint32_t Estimator_Frame_Cycle(void)
{
    return test_frame_cycle;
}
uint32_t osKernelGetTickCount(void)
{
    return test_tick;
}
uint32_t Tinympc14_Port_Tick(void)
{
    return test_tick;
}
void Tinympc14_Port_Enter_Critical(void)
{
    Test_Enter_Critical();
}
void Tinympc14_Port_Exit_Critical(void)
{
    Test_Exit_Critical();
}
uint32_t HAL_GetTick(void)
{
    return test_tick;
}
uint32_t xTaskGetTickCount(void)
{
    return test_tick;
}
void vTaskDelay(uint32_t ticks)
{
    test_tick += ticks;
}
uint32_t Can_Bus_Tx_Epoch(void)
{
    return test_bus_epoch;
}
bool Can_Bus_Tx_Ready(FDCAN_HandleTypeDef *handle)
{
    (void)handle;
    return true;
}
bool Can_Bus_Register(FDCAN_HandleTypeDef *handle, uint32_t id, can_parse_fn_t fn, void *ctx)
{
    (void)handle;
    (void)id;
    (void)fn;
    (void)ctx;
    return true;
}
HAL_StatusTypeDef Can_Bus_Send(FDCAN_HandleTypeDef *handle, uint32_t id, const uint8_t *data, uint8_t len)
{
    unsigned int slot = test_send_count++ % 5u;

    (void)handle;
    assert(len == 8u);
    test_id[slot] = id;
    memcpy(test_payload[slot], data, 8u);
    test_dwt.CYCCNT += 1000u;
    return (test_fail_mask & (1u << slot)) ? HAL_ERROR : HAL_OK;
}
#ifdef TEST_REAL_BACKENDS
#include "dr16.h"

dr16_t Dr16_Snapshot(void)
{
    return (dr16_t){0};
}
#endif
void App_Self_Rescue_Pid_Init(void)
{
}
void App_Self_Rescue_Pid_Update(balance_ctrl_mode_t mode)
{
    (void)mode;
}
#ifdef TEST_SPRING_AIR
#include "fsm.h"
#include "imu_to_world.h"

imu_world_t imu_world;
fsm_state_t Fsm_State_Get(void)
{
    return FSM_STATE_BALANCE;
}
#else
volatile air_status_t air_status;
uint8_t Air_Get(void)
{
    return 0u;
}
#endif
