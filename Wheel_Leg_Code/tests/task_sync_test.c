#include "task_sync.h"
#include "fsm.h"
#include "body.h"
#include "leg.h"
#include "imu.h"
#include "imu_to_world.h"
#include "dr16.h"
#include "dm_motor.h"
#include "dji_motor.h"
#include "air.h"
#include "slip.h"
#include "monitor.h"
#include "app_safety.h"
#include "code_config.h"
#include "stm32h7xx.h"
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>

void StartEstimatorTask(void *argument);
void StartControlTask(void *argument);
volatile bool task_ready         = true;
osThreadId_t ControlTaskHandle   = (void *)1;
osThreadId_t EstimatorTaskHandle = (void *)2;
body_state_t body;
leg_state_t leg_lft, leg_rgt;
imu_world_t imu_world;
volatile monitor_win_t monitor_win;
volatile monitor_can_t monitor_can[1];
test_dwt_t test_dwt;
static jmp_buf finished, control_return;
static uint32_t tick, ready, done, stage, cycles;
static const uint32_t work_ms[]   = {0u, 1u, 2u, 5u};
static const uint32_t next_tick[] = {1u, 3u, 6u, 12u};
uint32_t osKernelGetTickCount(void)
{
    return tick;
}
int osDelay(uint32_t ms)
{
    tick += ms;
    return 0;
}
int osDelayUntil(uint32_t next)
{
    assert(done && !ready && stage == 13u);
    assert(next == next_tick[cycles]);
    tick = next;
    if (++cycles == 4u) longjmp(finished, 1);
    stage = 0u;
    return 0;
}
uint32_t osThreadFlagsSet(osThreadId_t id, uint32_t flags)
{
    if (id == ControlTaskHandle)
    {
        assert(stage == 8u && flags == TASK_ESTIMATE_READY);
        assert(Estimator_Frame_Tick() == tick);
        assert(!ready);
        ready = 1u;
        done  = 0u;
    }
    else
    {
        assert(id == EstimatorTaskHandle && flags == TASK_CONTROL_DONE && stage == 12u);
        assert(monitor_can[0].heartbeat == cycles + 1u);
        done = 1u;
    }
    return flags;
}
uint32_t osThreadFlagsWait(uint32_t flags, uint32_t options, uint32_t timeout)
{
    assert(options == osFlagsWaitAny && timeout == osWaitForever);
    if (flags == TASK_ESTIMATE_READY)
    {
        if (done) longjmp(control_return, 1);
        assert(ready && stage == 8u);
        ready = 0u;
    }
    else
    {
        assert(flags == TASK_CONTROL_DONE);
        if (!setjmp(control_return)) StartControlTask(NULL);
        assert(done && stage == 12u);
    }
    return flags;
}
void Vofa_Capture_Joints(uint32_t frame_tick, uint32_t frame_cycle)
{
    assert(done && stage == 12u && frame_tick == tick - work_ms[cycles]);
    assert(frame_cycle == Estimator_Frame_Cycle());
    stage++;
}
void Dr16_Update(void)
{
    assert(stage == 0u);
    stage++;
}
bool Imu_Parse(void)
{
    assert(stage == 1u);
    stage++;
    return true;
}
void Imu_To_World(void)
{
    assert(stage == 2u);
    stage++;
}
void Dm_Motor_Parse(void)
{
    assert(stage == 3u);
    stage++;
}
void Dji_Motor_Parse(void)
{
    assert(stage == 4u);
    stage++;
}
void Leg_State_Update(void)
{
    assert(stage == 5u);
    leg_lft.len = 0.23f;
    stage++;
}
void Body_State_Update(void)
{
    assert(stage == 6u);
    body.vel = 0.5f;
    stage++;
}
void Slip_Air_Update(void)
{
    assert(stage == 7u);
    body.vel_kalman = body.vel;
}
void Body_Position_Update(void)
{
    assert(stage == 7u);
    stage++;
}
void Monitor_Init(void)
{
}
uint32_t Monitor_Begin(void)
{
    assert(stage == 8u);
    stage++;
    return tick;
}
dr16_t Dr16_Snapshot(void)
{
    const dr16_t rc = {.sw_left = DR16_SW_MID};

    return rc;
}

void App_Safety_Online_Check(bool control_requested)
{
    assert(control_requested);
    assert(stage == 9u);
    stage++;
}
void App_Safety_Apply_Flag(void)
{
    assert(stage == 10u);
    stage++;
}
fsm_request_t Fsm_Read_Request(const dr16_t *rc)
{
    return (fsm_request_t){.run = rc->sw_left == DR16_SW_MID};
}

void Fsm_Tick(const dr16_t *rc, const fsm_request_t *req)
{
    assert(rc->sw_left == DR16_SW_MID && req->run);
    assert(stage == 11u && body.vel == 0.5f && leg_lft.len == 0.23f);
    assert(body.vel_kalman == body.vel);
    stage++;
}
void Monitor_End(monitor_slot_t slot, uint32_t begin)
{
    assert(slot == MONITOR_SLOT_TICK && begin == tick && stage == 12u);
    tick += work_ms[cycles];
}
int main(void)
{
    if (!setjmp(finished)) StartEstimatorTask(NULL);
    assert(cycles == 4u && tick == 12u && monitor_win.overrun_cnt == 3u);
    puts("Real estimator/control task notification order, heartbeat and 1 ms cadence PASS");
    return 0;
}
