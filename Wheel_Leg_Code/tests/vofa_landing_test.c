/** @file  vofa_landing_test.c  @brief 验证逐拍关节缓存、双窗口回放及发送忙时重试  @version  v1.0  2026-11-24 */
#include "task_sync.h"
#include "air.h"
#include "balance.h"
#include "body.h"
#include "dm_motor.h"
#include "fsm_running.h"
#include "leg.h"
#include "monitor.h"
#include "stm32h7xx.h"
#include "vofa.h"
#include <assert.h>
#include <math.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>

void StartVofaTask(void *argument);
volatile bool task_ready = true;
volatile air_status_t air_status;
dm_motor_t dm_f_lft, dm_b_lft, dm_f_rgt, dm_b_rgt;
body_state_t body;
leg_state_t leg_lft, leg_rgt;
control_vector_t leg_control_lft, leg_control_rgt;
uint32_t SystemCoreClock = 1000000u;
test_dwt_t test_dwt;

static jmp_buf finished;
static uint32_t tick, calls, received[5];
static const uint32_t trigger[] = {0u, 800u, 900u, 1000u, 14000u};

void Test_Enter_Critical(void)
{
}

void Test_Exit_Critical(void)
{
}

void Monitor_Can_Sample(void)
{
}

uint32_t osKernelGetTickCount(void)
{
    return tick;
}

int osDelay(uint32_t ms)
{
    (void)ms;
    return 0;
}

fsm_landing_t Fsm_Running_Landing(void)
{
    return (fsm_landing_t){0};
}

void Balance_Force_Diagnostic(float request[2])
{
    request[0] = (float)tick;
    request[1] = -(float)tick;
}

float Dm_Motor_Torque_Decode(uint16_t code)
{
    return (float)code * 80.0f / 4095.0f - 40.0f;
}

// 慢任务在落地前后空跑 26 ms；估计器仍逐拍生产数据。第四次落地人为漏两拍以检查真实时标。
int osDelayUntil(uint32_t next)
{
    dm_motor_t *const joints[4] = {&dm_f_lft, &dm_b_lft, &dm_f_rgt, &dm_b_rgt};
    if (tick == 790u) next = tick + 26u;
    while (tick < next)
    {
        tick++;
        if (tick == 14008u || tick == 14009u) continue;
        for (unsigned int id = 1u; id <= 4u; id++)
            if (trigger[id] == tick) air_status.land_events = id;
        const uint32_t cycle = UINT32_MAX - 805000u + tick * 1000u;
        test_dwt.CYCCNT = cycle + 500u;
        for (unsigned int i = 0u; i < 4u; i++)
        {
            dm_motor_t *joint = joints[i];
            const uint32_t code = (tick + i) % 4096u;
            const uint32_t sent_cycle = i == 0u ? cycle - 100u : cycle + 100u;
            joint->frame_tx_stamp = ((uint64_t)(cycle - 300u) << 32) | (code + 1u);
            joint->mit_tx_stamp = ((uint64_t)sent_cycle << 32) | ((code + 2u) % 4096u + 1u);
            joint->frame_cycle = cycle - 100u;
            joint->frame_tick = tick;
            joint->online = 1u;
            joint->trq = (float)(tick + i);
        }
        leg_lft.pitch_1 = (float)tick;
        leg_lft.pitch_4 = (float)tick + 1.0f;
        leg_rgt.pitch_4 = (float)tick + 2.0f;
        leg_rgt.pitch_1 = (float)tick + 3.0f;
        body.ang_roll = (float)tick;
        body.roll_rate_obs = -(float)tick;
        body.omg_roll_kalman = 0.5f * (float)tick;
        Vofa_Capture_Joints(tick, cycle);
    }
    if (tick > 20000u) longjmp(finished, 1);
    return 0;
}

// 约三分之一的串口发送尝试返回忙；成功导出的同一窗口仍必须逐行连续。
bool Vofa_Send_Float(double first, ...)
{
    double ch[37] = {first};
    va_list args;
    va_start(args, first);
    for (unsigned int i = 1u; i < 37u; i++) ch[i] = va_arg(args, double);
    assert(isinf(va_arg(args, double)));
    va_end(args);
    if (++calls % 3u == 0u) return false;
    const unsigned int id = (unsigned int)ch[28];
    assert(ch[33] == ch[9] && ch[34] == -ch[9] && ch[35] == 0.5 * ch[9] && ch[36] == 2.0 * ch[9]);
    if (!id) return true;
    assert(id == 1u || id == 2u || id == 4u);
    const uint32_t index = received[id]++;
    const uint32_t expected = trigger[id] - 320u + index + (id == 4u && index >= 328u ? 2u : 0u);
    assert(ch[9] == expected && ch[29] == (int32_t)(expected - trigger[id]) && ch[32] == index + 1u);
    assert(ch[10] == expected && ch[11] == -(double)expected && ch[30] == 14u && ch[31] == 1u);
    for (unsigned int i = 0u; i < 4u; i++)
    {
        assert(ch[i] == (expected + i) % 4096u);
        assert(ch[4u + i] == expected + i && ch[12u + i] == expected + i);
        assert(ch[16u + i] == 100.0 && ch[20u + i] == 200.0);
        assert(ch[24u + i] == Dm_Motor_Torque_Decode((uint16_t)((expected + i + 2u) % 4096u)));
    }
    return true;
}

int main(void)
{
    if (!setjmp(finished)) StartVofaTask(NULL);
    assert(received[1] == 641u && received[2] == 641u && received[3] == 0u && received[4] == 639u);
    puts("PASS: 26 ms VOFA pause, 1 ms joint samples, overlapping landings, DMA retries, rearm and real clock gaps");
    return 0;
}
