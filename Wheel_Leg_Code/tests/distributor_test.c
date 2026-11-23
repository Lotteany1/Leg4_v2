/** @file  distributor_test.c  @brief 支撑约束、真实编码、发送失败与历史积分验证  @version  v1.3  2026-11-22 */
#include "balance.h"
#include "code_config.h"
#include "distributor.h"
#include "system_key.h"
#include "dm_motor.h"
#include "dji_motor.h"
#include "stm32h7xx.h"
#include "body.h"
#include "leg.h"
#include "Leg_Trq.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

extern uint32_t test_tick, test_frame_cycle, test_fail_mask, test_bus_epoch, test_send_count;
extern uint32_t test_id[5];
extern uint8_t test_payload[5][8];

// 支撑范围必须在髋饱和时保留；电机能力不足仅松弛到最近可达点，非饱和命令不被重分配。
static void Test_Support(void)
{
    distributor_input_t input = {.normal_mode = true, .strategy = DISTRIBUTOR_PROJECT, .dt = 0.001f};
    distributor_result_t result, clipped;

    for (unsigned int side = 0u; side < 2u; side++)
    {
        distributor_leg_t *leg = &input.leg[side];

        leg->map[0][0] = -0.15f;
        leg->map[0][1] = 0.5f;
        leg->map[1][0] = 0.15f;
        leg->map[1][1] = 0.5f;
        leg->s_req     = 80.0f;
        leg->p_req     = 10.0f;
        leg->spring_f  = 35.0f;
        for (unsigned int j = 0u; j < 2u; j++)
        {
            leg->tau_lo[j] = -54.0f;
            leg->tau_hi[j] = 54.0f;
        }
    }
    Distributor_Calculate(&input, &result);
    assert(result.result_valid && result.status == DISTRIBUTOR_STATUS_NOMINAL);
    for (unsigned int side = 0u; side < 2u; side++)
    {
        assert(fabsf(result.s_alloc[side] - 80.0f) < 1e-4f);
        assert(fabsf(result.p_alloc[side] - 10.0f) < 1e-4f);
        assert(result.tau_alloc[2u * side] == result.tau_req[2u * side]);
        assert(result.tau_alloc[2u * side + 1u] == result.tau_req[2u * side + 1u]);
    }

    // 同时覆盖低被动力下的压地边界与高被动力下的顶升边界。
    input.leg[0].spring_f = 0.0f;
    input.leg[1].spring_f = 200.0f;
    input.leg[0].p_req = input.leg[1].p_req = 220.0f;
    Distributor_Calculate(&input, &result);
    assert(result.result_valid && result.status == DISTRIBUTOR_STATUS_PROJECTED);
    assert(result.support_relaxed_mask == 0u);
    assert(fabsf(result.s_alloc[0] - DISTRIBUTOR_S_RESERVE) < 1e-4f);
    assert(fabsf(result.s_alloc[1] - DISTRIBUTOR_S_MAX) < 1e-4f);
    input.height_priority = true;
    Distributor_Calculate(&input, &clipped);
    assert(clipped.result_valid && clipped.support_relaxed_mask == 0u);
    assert(clipped.s_alloc[0] <= 80.0001f && clipped.s_alloc[1] <= 80.0001f);
    assert(fabsf(clipped.s_alloc[1] - 80.0f) < 1e-4f);
    input.height_priority = false;
    input.strategy = DISTRIBUTOR_CLIP;
    Distributor_Calculate(&input, &clipped);
    assert(clipped.status == DISTRIBUTOR_STATUS_CLIPPED);
    assert(clipped.s_alloc[0] < DISTRIBUTOR_S_MIN && clipped.s_alloc[1] > DISTRIBUTOR_S_MAX);
    input.strategy = DISTRIBUTOR_SCALE;
    Distributor_Calculate(&input, &clipped);
    assert(clipped.status == DISTRIBUTOR_STATUS_SCALED);

    // 上游主动卸载到 20 N 时，饱和分配不能擅自抬回单腿名义重力支撑。
    input.strategy     = DISTRIBUTOR_PROJECT;
    input.leg[0].s_req = 20.0f;
    Distributor_Calculate(&input, &result);
    assert(result.result_valid && result.support_relaxed_mask == 0u);
    assert(fabsf(result.s_alloc[0] - 20.0f) < 1e-4f);

    // 单向且不足的电机力矩箱无法满足保留支撑：最近可达为 10 N，不能停机或伪报满足。
    input.strategy = DISTRIBUTOR_PROJECT;
    for (unsigned int side = 0u; side < 2u; side++)
    {
        input.leg[side] = (distributor_leg_t){
            .map    = {{1.0f, 0.0f}, {0.0f, 1.0f}},
            .s_req  = 100.0f,
            .p_req  = 50.0f,
            .tau_lo = {2.0f, 2.0f},
            .tau_hi = {10.0f, 10.0f},
        };
    }
    Distributor_Calculate(&input, &result);
    assert(result.result_valid && result.status == DISTRIBUTOR_STATUS_PROJECTED);
    assert(result.support_relaxed_mask == 3u);
    for (unsigned int side = 0u; side < 2u; side++)
    {
        assert(fabsf(result.s_alloc[side] - 10.0f) < 1e-4f);
        assert(fabsf(result.p_alloc[side] - 10.0f) < 1e-4f);
    }

    // 正常力箱之外的明确请求仍可保持；不以压地边界改变上游交接或特殊支撑请求。
    for (unsigned int side = 0u; side < 2u; side++)
    {
        input.leg[side].s_req = 5.0f;
        input.leg[side].p_req = 5.0f;
    }
    Distributor_Calculate(&input, &result);
    assert(result.result_valid && result.status == DISTRIBUTOR_STATUS_NOMINAL);
    assert(result.s_alloc[0] == 5.0f && result.s_alloc[1] == 5.0f);

    // 固定电机力矩迫使支持力高于上界时同样取最近可达点，零面积可行域仍可求解。
    for (unsigned int side = 0u; side < 2u; side++)
    {
        input.leg[side].s_req     = 80.0f;
        input.leg[side].tau_lo[0] = input.leg[side].tau_hi[0] = 200.0f;
    }
    Distributor_Calculate(&input, &result);
    assert(result.result_valid && result.status == DISTRIBUTOR_STATUS_PROJECTED);
    assert(result.support_relaxed_mask == 3u);
    assert(result.s_alloc[0] == 200.0f && result.s_alloc[1] == 200.0f);
}

// 饱和统计不依赖遥测频率；比较分支不改变实际策略，失败拍不清空累计值。
static void Test_Diagnostics(void)
{
    distributor_frame_t frame = {
        .phi = {{2.3f, 0.8f}, {2.3f, 0.8f}},
        .force = {80.0f, 80.0f},
        .hip = {220.0f, 220.0f},
        .wheel = {1.0f, 2.0f},
    };
    distributor_diag_t diag;
    float nominal[2][2];

    Dm_Motor_Init();
    Distributor_Init();
    Distributor_Mode(LEG_CTRL_MODE_STANDING, true);
    test_send_count = 0u;
    assert(Distributor_Send(&frame, nominal));
    Distributor_Diagnostic(&diag);
    // 左右发不同的矩，检查 201/202 字段保留 V1 同侧命令方向及接纳记录。
    const int16_t left_code = (int16_t)(((uint16_t)test_payload[4][0] << 8) | test_payload[4][1]);
    const int16_t right_code = (int16_t)(((uint16_t)test_payload[4][2] << 8) | test_payload[4][3]);
    assert(dji_wheel_lft.id == 0x201u && dji_wheel_rgt.id == 0x202u && test_id[4] == 0x200u);
    assert(left_code == (int16_t)M3508_TO_TORQUE && right_code == (int16_t)(-2.0f * M3508_TO_TORQUE));
    assert(diag.execution.code[4] == left_code && diag.execution.code[5] == right_code);
    assert(fabsf(diag.execution.model_u[0] + frame.wheel[0]) <= 1.0f / M3508_TO_TORQUE);
    assert(fabsf(diag.execution.model_u[1] + frame.wheel[1]) <= 1.0f / M3508_TO_TORQUE);
    assert(diag.limited_count == 1u && diag.support_relaxed_count == 0u);
    assert(diag.normal_count == 1u && diag.changed_count == 1u && diag.fallback_count == 0u);
    assert(diag.send_failure_count == 0u && diag.wheel_limited_count == 0u && diag.tau_change_max > 0.0f);
    assert(diag.result.status == (BALANCE_USE_DISTRIBUTOR ? DISTRIBUTOR_STATUS_PROJECTED : DISTRIBUTOR_STATUS_CLIPPED));
    {
        distributor_result_t clipped;
        distributor_input_t input = diag.input;
        bool differs = false;

        input.strategy = DISTRIBUTOR_CLIP;
        Distributor_Calculate(&input, &clipped);
        for (unsigned int joint = 0u; joint < 4u; joint++)
        {
            uint16_t selected_code, clipped_code;

            assert(Dm_Motor_Torque_Encode(diag.result.tau_alloc[joint], -54.0f, 54.0f, &selected_code));
            assert(Dm_Motor_Torque_Encode(clipped.tau_alloc[joint], -54.0f, 54.0f, &clipped_code));
            assert(diag.execution.code[joint] == selected_code);
            differs |= selected_code != clipped_code;
        }
        assert(differs == (BALANCE_USE_DISTRIBUTOR != 0));
    }
#if BALANCE_DISTRIBUTOR_COMPARE
    assert(diag.compare[0].status == (BALANCE_USE_DISTRIBUTOR ? DISTRIBUTOR_STATUS_CLIPPED : DISTRIBUTOR_STATUS_PROJECTED));
    assert(diag.compare[1].status == DISTRIBUTOR_STATUS_SCALED);
#endif
    frame.hip[0] = frame.hip[1] = 10.0f;
    assert(Distributor_Send(&frame, nominal));
    Distributor_Diagnostic(&diag);
    assert(diag.result.status == DISTRIBUTOR_STATUS_NOMINAL && diag.limited_count == 1u);
    assert(diag.normal_count == 2u && diag.changed_count == 1u);
    for (unsigned int joint = 0u; joint < 4u; joint++)
    {
        uint16_t nominal_code;

        assert(diag.result.tau_alloc[joint] == diag.result.tau_req[joint]);
        assert(Dm_Motor_Torque_Encode(diag.result.tau_req[joint], -54.0f, 54.0f, &nominal_code));
        assert(diag.execution.code[joint] == nominal_code);
    }
    frame.phi[0][0] = NAN;
    assert(!Distributor_Send(&frame, nominal));
    Distributor_Diagnostic(&diag);
    assert(diag.failure_count == 1u && diag.limited_count == 1u);
    assert(diag.normal_count == 3u && diag.changed_count == 1u);

    // API 全部拒绝仍保留有效计算统计；轮饱和与关节重分配独立计数。
    frame.phi[0][0] = 2.3f;
    frame.hip[0] = frame.hip[1] = 220.0f;
    frame.wheel[0] = frame.wheel[1] = 100.0f;
    test_fail_mask  = 31u;
    assert(Distributor_Send(&frame, nominal));
    Distributor_Diagnostic(&diag);
    assert(diag.failure_count == 1u && diag.limited_count == 2u);
    assert(diag.normal_count == 4u && diag.changed_count == 2u);
    assert(diag.send_failure_count == 1u && diag.wheel_limited_count == 1u);
    assert(diag.execution.accepted_mask == 0u);
    test_fail_mask = 0u;
}

// 覆盖窄码区、端点和重复编码；比较线上真实字节，不以浮点近似当协议码。
static void Test_Encoding(void)
{
    uint16_t code;

    assert(!Dm_Motor_Torque_Encode(0.0f, -0.001f, 0.001f, &code));
    assert(!Dm_Motor_Torque_Encode(NAN, -1.0f, 1.0f, &code));
    for (unsigned int raw = 0u; raw <= 4095u; raw++)
    {
        float torque = Dm_Motor_Torque_Decode((uint16_t)raw);

        assert(Dm_Motor_Torque_Encode(torque, DM_T_MIN, DM_T_MAX, &code));
        assert(code == raw);
        assert(Dm_Motor_Torque_Encode(torque, torque, torque, &code));
        assert(code == raw);
        test_send_count = 0u;
        assert(Dm_Motor_Torque_Send(&hfdcan1, 1u, code) == HAL_OK);
        assert((((unsigned int)test_payload[0][6] & 15u) << 8 | test_payload[0][7]) == raw);
    }
    assert(!Dm_Motor_Torque_Encode(0.0f, 54.1f, 55.0f, &code));
    assert(Dm_Motor_Torque_Encode(2.0f, -0.1f, 0.1f, &code));
    assert(Dm_Motor_Torque_Decode(code) <= 0.1f);
}

// 真实 VMC 与实际发送路径结合，所有发送时间在一个控制拍内。
static void Test_Frame(uint32_t tick)
{
    test_tick        = tick;
    test_frame_cycle = tick * (SystemCoreClock / 1000u);
    test_dwt.CYCCNT  = test_frame_cycle;
    test_send_count  = 0u;
    Balance_Control(LEG_CTRL_MODE_STANDING);
    {
        execution_sample_t sample;

        Execution_Latest(&sample);
        for (unsigned int joint = 0u; joint < 4u; joint++)
        {
            unsigned int raw = ((unsigned int)(test_payload[joint][6] & 15u) << 8) | test_payload[joint][7];
            assert(sample.code[joint] == (int32_t)raw);
        }
        assert(sample.code[4] == (int16_t)(((uint16_t)test_payload[4][0] << 8) | test_payload[4][1]));
        assert(sample.code[5] == (int16_t)(((uint16_t)test_payload[4][2] << 8) | test_payload[4][3]));
    }
}

// 部分失败保持对应旧码；两轮必须同时被接纳；查询不消耗另一个消费者的区间。
static void Test_Closed_Path(void)
{
    execution_sample_t first, second, latest;
    execution_interval_t interval, another;
    distributor_diag_t diag;

    body    = (body_state_t){.h_body = 0.25f, .h_tgt = 0.25f, .F_lft = 40.0f, .F_rgt = 40.0f, .leg_len_lft = 0.24f, .leg_len_rgt = 0.26f, .leg_len = 0.25f, .ang_yaw_tgt = 0.02f};
    leg_lft = (leg_state_t){.len = 0.24f, .pitch_1 = 2.3f, .pitch_4 = 0.8f};
    leg_rgt = (leg_state_t){.len = 0.26f, .pitch_1 = 2.2f, .pitch_4 = 0.7f};
    Dm_Motor_Init();
    Balance_Control_Init();
    Balance_Mpc_Init();
    test_fail_mask = 17u;
    Test_Frame(0u);
    Execution_Latest(&first);
    assert(first.valid_mask == 14u && first.valid_before == 0u);
    test_fail_mask = 0u;
    Test_Frame(1u);
    Execution_Latest(&first);
    assert(first.valid_mask == 63u && first.accepted_mask == 63u);
    assert(test_id[0] == 1u && test_id[1] == 3u && test_id[2] == 2u && test_id[3] == 4u && test_id[4] == 0x200u);
    Distributor_Diagnostic(&diag);
    assert(diag.result.result_valid && diag.mapping_error < 0.001f);
    assert(diag.input.strategy == (BALANCE_USE_DISTRIBUTOR ? DISTRIBUTOR_PROJECT : DISTRIBUTOR_CLIP));
#if BALANCE_DISTRIBUTOR_COMPARE
    assert(diag.compare[0].result_valid && diag.compare[1].result_valid);
#endif
    assert(first.model_u[0] != first.model_u[1]);
    for (unsigned int fail = 0u; fail < 5u; fail++)
    {
        test_fail_mask = 1u << fail;
        body.ang_yaw_tgt += 0.02f;
        Test_Frame(2u + fail);
        Execution_Latest(&second);
        assert((second.accepted_mask & (fail == 4u ? 48u : (1u << fail))) == 0u);
        assert(second.valid_mask == 63u);
        if (fail == 4u)
        {
            assert(second.before[4] == first.code[4] && second.before[5] == first.code[5]);
        }
        first = second;
        // 为下一拍构造前一条记录的真实保持码，失败码不能冒充接纳码。
        for (unsigned int ch = 0u; ch < 6u; ch++)
            if (!(second.accepted_mask & (1u << ch))) first.code[ch] = second.before[ch];
    }
    test_fail_mask = 0u;
    Test_Frame(7u);
    Execution_Latest(&latest);
    Execution_Read(latest.epoch, 2u * 550000u, 7u * 550000u, 7u, 550000u, &interval);
    Execution_Read(latest.epoch, 2u * 550000u, 7u * 550000u, 7u, 550000u, &another);
    assert(interval.complete && another.complete);
    for (unsigned int ch = 0u; ch < 4u; ch++) assert(interval.model_u[ch] == another.model_u[ch]);
    test_bus_epoch++;
    Execution_Read(latest.epoch, 2u * 550000u, 7u * 550000u, 7u, 550000u, &interval);
    assert(!interval.complete);
    Test_Frame(8u);
    Execution_Latest(&latest);
    assert(latest.epoch != second.epoch && latest.valid_before == 0u);
    for (unsigned int tick = 9u; tick <= 82u; tick++) Test_Frame(tick);
    Execution_Latest(&latest);
    Execution_Read(latest.epoch, 75u * 550000u, 82u * 550000u, 82u, 550000u, &interval);
    assert(interval.complete);
    Execution_Read(latest.epoch, 8u * 550000u, 12u * 550000u, 82u, 550000u, &interval);
    assert(!interval.complete);
    for (unsigned int mode = 1u; mode < LEG_CTRL_MODE_NUM; mode++)
    {
        if (Balance_Mode_Grounded((balance_ctrl_mode_t)mode))
            continue;
        test_send_count = 0u;
        Balance_Control((balance_ctrl_mode_t)mode);
        Distributor_Diagnostic(&diag);
        assert(!diag.result.result_valid);
        assert(test_send_count == (mode == LEG_CTRL_MODE_AIR ? 6u : 5u));
        for (unsigned int joint = 0u; joint < 4u; joint++)
        {
            float torque      = joint < 2u ? leg_control_lft.out[joint] : leg_control_rgt.out[3u - joint];
            unsigned int slot = (test_send_count - 5u + joint) % 5u;
            unsigned int raw  = ((unsigned int)(test_payload[slot][6] & 15u) << 8) | test_payload[slot][7];

            if (!isfinite(torque)) torque = 0.0f;
            torque = fmaxf(DM_T_MIN, fminf(DM_T_MAX, torque));
            assert(raw == (unsigned int)((torque - DM_T_MIN) * 4095.0f / (DM_T_MAX - DM_T_MIN)));
        }
    }
    {
        uint32_t previous_epoch = Execution_Epoch();

        leg_lft.pitch_1 = NAN;
        Balance_Control(LEG_CTRL_MODE_STANDING);
        Execution_Latest(&latest);
        assert(Execution_Epoch() != previous_epoch && latest.seq == 0u);
        Distributor_Diagnostic(&diag);
        // 非有限编码器角在 Balance_State_Check 已被拒绝，不会进入分配器求解。
        assert(!diag.result.result_valid);
        leg_lft.pitch_1 = 2.3f;
        Test_Frame(90u);
        Execution_Latest(&latest);
        assert(latest.valid_before == 0u && latest.valid_mask == 63u);
    }
}

// 人工记录独立验证分段积分、末值与均值区别以及 DWT/tick 回绕。
static void Test_Integral(void)
{
    execution_sample_t sample;
    execution_interval_t result;
    uint32_t start = UINT32_MAX - 1000u;
    uint16_t low, high;

    Execution_Init();
    Execution_Mode(0u);
    assert(Dm_Motor_Torque_Encode(0.0f, -54.0f, 54.0f, &low));
    assert(Dm_Motor_Torque_Encode(10.0f, -54.0f, 54.0f, &high));
    for (unsigned int frame = 0u; frame < 4u; frame++)
    {
        Execution_Begin(&sample);
        sample.tick  = UINT32_MAX - 1u + frame;
        sample.cycle = start + 1000u * frame;
        for (unsigned int side = 0u; side < 2u; side++)
        {
            sample.map[side][0][0] = 1.0f;
            sample.map[side][1][1] = 1.0f;
        }
        for (unsigned int event = 0u; event < 5u; event++) sample.offset[event] = 100u + 100u * event;
        for (unsigned int ch = 0u; ch < 4u; ch++) sample.code[ch] = frame == 0u ? low : high;
        sample.accepted_mask = frame >= 2u ? 0u : 63u;
        if (frame >= 2u)
        {
            // 相同保持码配合新姿态，应恢复到新力，而非沿用旧姿态任务。
            sample.map[0][0][0] = 0.5f;
        }
        Execution_Publish(&sample);
    }
    Execution_Read(sample.epoch, start + 1000u, start + 2000u, 0u, 1000u, &result);
    assert(result.complete);
    {
        float before = Dm_Motor_Torque_Decode(low);
        float after  = Dm_Motor_Torque_Decode(high);

        assert(fabsf(result.total_s[0] - (0.1f * before + 0.9f * after)) < 1e-4f);
        assert(result.end_s[0] == after);
        assert(result.end_s[0] != result.total_s[0]);
    }
    Execution_Read(sample.epoch, start + 2000u, start + 3000u, 1u, 1000u, &result);
    assert(result.complete);
    assert(fabsf(result.total_s[0] - 2.0f * Dm_Motor_Torque_Decode(high)) < 1e-4f);
    Execution_Invalidate();
    Execution_Read(sample.epoch, start + 1000u, start + 2000u, 0u, 1000u, &result);
    assert(!result.complete);
}

int main(void)
{
    Test_Support();
    Test_Diagnostics();
    Test_Encoding();
    Test_Closed_Path();
    Test_Integral();
    puts("PASS: real DM/DJI encoding and payload, NORMAL allocation, five API failures, two readers, overwrite, bus invalidation, modes, weighted integral and wrap");
    return 0;
}
