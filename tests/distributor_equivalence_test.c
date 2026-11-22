/** @file  distributor_equivalence_test.c  @brief 固定交错输入的重构前后码流与历史对照  @version  v1.0  2026-11-02 */
#include "balance.h"
#include "distributor.h"
#include "dm_motor.h"
#include "dji_motor.h"
#include "body.h"
#include "leg.h"
#include "stm32h7xx.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

extern uint32_t test_tick, test_frame_cycle, test_fail_mask, test_bus_epoch, test_send_count;
extern uint32_t test_id[5];
extern uint8_t test_payload[5][8];
extern float test_model_scale;

// 浮点按位输出，不把结构体填充或 DWT 耗时当作行为差异。
static void Test_Floats(const float *value, unsigned int count)
{
    for (unsigned int i = 0u; i < count; i++)
    {
        uint32_t bits;

        memcpy(&bits, value + i, sizeof(bits));
        printf(" %08x", (unsigned int)bits);
    }
}

// 同一输入序列分别运行重构前后生产链；数值正确性另由独立优化/积分参照验证。
int main(void)
{
    body = (body_state_t){.h_body = 0.25f, .h_tgt = 0.25f, .F_lft = 40.0f, .F_rgt = 40.0f,
                          .leg_len_lft = 0.24f, .leg_len_rgt = 0.26f, .leg_len = 0.25f};
    Dm_Motor_Init();
    Balance_Control_Init();
    for (unsigned int beat = 0u; beat < 320u; beat++)
    {
        execution_sample_t sample;
        execution_interval_t interval;
        distributor_diag_t diag;
        balance_ctrl_mode_t mode = LEG_CTRL_MODE_STANDING;
        float request[2];

        test_tick = 0xfffffff0u + beat;
        test_frame_cycle = 0xffff8000u + beat * (SystemCoreClock / 1000u);
        test_dwt.CYCCNT = test_frame_cycle;
        test_send_count = 0u;
        test_fail_mask = beat % 13u == 0u ? 31u : (beat % 7u == 0u ? 16u : beat % 5u);
        test_model_scale = 0.5f + (float)(beat % 23u);
        leg_lft = (leg_state_t){.len = 0.24f + (float)(beat % 7u) * 0.001f,
                               .pitch_1 = 2.3f + (float)(beat % 9u) * 0.002f, .pitch_4 = 0.8f};
        leg_rgt = (leg_state_t){.len = 0.26f, .pitch_1 = 2.2f, .pitch_4 = 0.7f + (float)(beat % 11u) * 0.002f};
        body.leg_ang_lft = beat % 2u ? 0.12f : -0.08f;
        body.leg_ang_rgt = beat % 2u ? -0.09f : 0.14f;
        body.omg_yaw_filt = beat % 2u ? 0.4f : -0.3f;
        body.vel = body.vel_kalman = 0.2f;
        if (beat % 47u == 0u) mode = LEG_CTRL_MODE_AIR;
        if (beat % 71u == 0u) mode = LEG_CTRL_MODE_TEST;
        if (beat % 89u == 0u) mode = LEG_CTRL_MODE_SELF_RESCUE_TURNOVER;
        if (beat % 67u == 0u) test_bus_epoch++;
        if (beat % 101u == 0u) Dm_Motor_Enable(&hfdcan1, 1u);
        test_send_count = 0u;
        Balance_Control(mode);
        Execution_Latest(&sample);
        Distributor_Diagnostic(&diag);
        Balance_Force_Diagnostic(request);
        Execution_Read(Execution_Epoch(), test_frame_cycle - 2u * (SystemCoreClock / 1000u),
                       test_frame_cycle, test_tick, SystemCoreClock / 1000u, &interval);
        printf("%u %u %u %u %u %u %u %u", beat, (unsigned int)mode, (unsigned int)Execution_Epoch(),
               (unsigned int)sample.seq, (unsigned int)sample.valid_before, (unsigned int)sample.valid_mask,
               (unsigned int)sample.accepted_mask, (unsigned int)interval.complete);
        for (unsigned int ch = 0u; ch < 6u; ch++) printf(" %d %d", (int)sample.before[ch], (int)sample.code[ch]);
        for (unsigned int event = 0u; event < 5u; event++)
        {
            printf(" %u %u", (unsigned int)test_id[event], (unsigned int)sample.offset[event]);
            for (unsigned int byte = 0u; byte < 8u; byte++) printf(" %02x", test_payload[event][byte]);
        }
        if (mode == LEG_CTRL_MODE_STANDING && sample.seq != 0u)
        {
            for (unsigned int ch = 0u; ch < 4u; ch++)
                assert(sample.code[ch] == (int32_t)(((uint32_t)test_payload[ch][6] & 15u) * 256u + test_payload[ch][7]));
        }
        Test_Floats(sample.model_u, 4u);
        Test_Floats(sample.total_s, 2u);
        Test_Floats(interval.model_u, 4u);
        Test_Floats(interval.total_s, 2u);
        Test_Floats(interval.end_s, 2u);
        Test_Floats(interval.turn, 2u);
        Test_Floats(interval.min_u, 4u);
        Test_Floats(interval.max_u, 4u);
        Test_Floats(interval.min_s, 2u);
        Test_Floats(interval.max_s, 2u);
        Test_Floats(diag.result.tau_alloc, 4u);
        Test_Floats(leg_control_lft.out, 2u);
        Test_Floats(leg_control_rgt.out, 2u);
        Test_Floats(request, 2u);
        putchar('\n');
    }
    return 0;
}
