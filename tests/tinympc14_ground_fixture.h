#ifndef TINYMPC14_GROUND_FIXTURE_H
#define TINYMPC14_GROUND_FIXTURE_H

#include "balance.h"
#include "body.h"
#include "dm_motor.h"
#include "leg.h"
#include "Leg_SpringF.h"
#include "stm32h7xx.h"
#include "tinympc14.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

extern uint32_t test_tick, test_frame_cycle, test_fail_mask, test_bus_epoch, test_send_count;
extern uint32_t test_sample_delay;

// 使用实际控制入口，沿用 CAN 夹具模拟入队与总线代次。
static balance_mpc_diag_t Test_Beat(balance_ctrl_mode_t mode)
{
    balance_mpc_diag_t diag;

    test_tick += CTRL_PERIOD_MS;
    test_frame_cycle = test_tick * (SystemCoreClock / 1000u);
    test_dwt.CYCCNT  = test_frame_cycle;
    test_send_count  = 0u;
    Balance_Control(mode);
    Balance_Mpc_Diagnostic(&diag);
    return diag;
}

// 每个输入边界用例重新初始化控制与求解器，避免热启动历史掩盖差异。
static void Test_Reset(void)
{
    // 静立时电机仅承担被动弹簧之外的轴力，使采样时标测试不依赖特定车辆的预载。
    const float reference[TINYMPC14_NX] = {[8] = TINYMPC14_PITCH_REF};
    const float zero[TINYMPC14_NX] = {0.0f};
    float hold[TINYMPC14_NU], feedforward[TINYMPC14_NU];

    body = (body_state_t){.h_body = TINYMPC14_H_REF, .h_tgt = TINYMPC14_H_REF,
                         .leg_len_lft = TINYMPC14_H_REF, .leg_len_rgt = TINYMPC14_H_REF,
                         .ang_pitch = TINYMPC14_PITCH_REF};
    leg_lft = leg_rgt = (leg_state_t){.len = TINYMPC14_H_REF, .pitch_1 = 2.3f, .pitch_4 = 0.8f};
    Dm_Motor_Init();
    Balance_Control_Init();
    Balance_Mpc_Init();
    Tinympc14_Affine_Pub(reference, zero, zero, hold, feedforward);
    body.F_lft = hold[4] - Leg_SpringF(TINYMPC14_H_REF, 0.0f);
    body.F_rgt = hold[5] - Leg_SpringF(TINYMPC14_H_REF, 0.0f);
}

#endif
