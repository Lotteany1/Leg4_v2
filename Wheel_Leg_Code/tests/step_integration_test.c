/** @file  step_integration_test.c  @brief 收腿、接触、KF 与执行代次主机联测  @version  v1.0  2026-11-16 */
#include "air.h"
#include "app_safety.h"
#include "app_step.h"
#include "tinympc14.h"
#include "body.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "fsm_running.h"
#include "imu.h"
#include "imu_to_world.h"
#include "leg.h"
#include "Leg_SpringF.h"
#include "music.h"
#include "slip.h"
#include "code_config.h"
#include "stm32h7xx.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

extern uint32_t test_tick, test_frame_cycle, test_send_count;
volatile uint8_t active_flag = APP_ENABLE;
static float height_delta;
static float intent;

void App_Rc_Update_Targets(body_state_t *state, const dr16_t *rc)
{
    (void)rc;
    state->vel_tgt = intent;
    state->omg_yaw_tgt = 0.0f;
    state->h_tgt += height_delta;
}

void Music_Play(uint8_t index) { (void)index; }
void Music_Stop(void) {}
void App_Safety_Trip(void) { active_flag = APP_DISABLE; }
void App_Safety_Apply_Flag(void) { assert(active_flag == APP_DISABLE); }
float Imu_To_World_Leg_Ang(float angle) { return angle + imu_world.pitch; }

// 注入轮帧及力反馈，随后保持固件估计、目标仲裁、分配和后台顺序。
static balance_ctrl_mode_t Test_Frame(float left, float right, uint8_t wheel_frames)
{
    test_tick += CTRL_PERIOD_MS;
    test_frame_cycle = test_tick * (SystemCoreClock / 1000u);
    test_dwt.CYCCNT = test_frame_cycle;
    test_send_count = 0u;
    imu.last_rx_tick = test_tick;
    dm_f_lft.frame_tick = dm_b_lft.frame_tick = dm_f_rgt.frame_tick = dm_b_rgt.frame_tick = test_tick;
    if (wheel_frames & 1u)
    {
        dji_wheel_lft.frame_seq++;
        dji_wheel_lft.frame_tick = test_tick;
    }
    if (wheel_frames & 2u)
    {
        dji_wheel_rgt.frame_seq++;
        dji_wheel_rgt.frame_tick = test_tick;
    }
    body.F_lft = left - Leg_SpringF(leg_lft.len, leg_lft.ang);
    body.F_rgt = right - Leg_SpringF(leg_rgt.len, leg_rgt.ang);
    body.leg_len_lft = leg_lft.len;
    body.leg_len_rgt = leg_rgt.len;
    body.h_body = body.leg_len = 0.5f * (leg_lft.len + leg_rgt.len);
    Slip_Air_Update();
    const balance_ctrl_mode_t mode = Fsm_Running_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, &(dr16_t){0});

    Balance_Control(mode);
    if (test_tick % BALANCE_MPC_PERIOD_MS == 0u) Balance_Mpc_Solve();
    return mode;
}

// 单侧、双侧及无法分侧的残差都能请求打滑抑制；双侧卸载始终能进 AIR。
static void Test_Slip_Air_Guard(float lft, float rgt)
{
    const float force = 85.0f;

    intent = 0.0f;
    body.vel = body.vel_lft = body.vel_rgt = 0.0f;
    Air_Init();
    Slip_Init();
    Fsm_Running_Enter();
    for (uint32_t i = 0u; i < SLIP_CONTACT_MS + 5u; i++) Test_Frame(force, force, 3u);

    // 保持单侧承重，在第五拍注入当前打滑证据。
    for (uint32_t i = 0u; i < 4u; i++) Test_Frame(-15.0f, 40.0f, 3u);
    assert(!Air_Get() && !air_status.slip_inhibited);
    body.vel_lft = lft;
    body.vel_rgt = rgt;
    body.vel = 0.5f * (lft + rgt);
    Test_Frame(-15.0f, 40.0f, 3u);
    assert(air_status.slip_inhibited && !Air_Get() && !App_Step_Owns_Output());
    for (uint32_t i = 0u; i < SLIP_PREDICT_MAX_MS + SLIP_AIR_SLIP_HOLD_MS; i++)
    {
        const balance_ctrl_mode_t mode = Test_Frame(-15.0f, 40.0f, 3u);

        assert(air_status.slip_inhibited && !Air_Get());
        if (Slip_Prediction_Expired())
            assert(mode == LEG_CTRL_MODE_MOVING && body.vel_tgt == body.vel_kalman);
    }
    if (fabsf(lft) >= SLIP_ENTER && fabsf(rgt) >= SLIP_ENTER)
        assert(Slip_Prediction_Expired() && Slip_Observation() == 0u);

    // 即便当前仍是同一轮速异常，该拍 AIR 也立即约束 KF 的修正。
    for (uint32_t i = 0u; i <= AIR_DEBOUNCE_MS; i++) Test_Frame(0.0f, 0.0f, 3u);
    assert(Air_Get() && !air_status.slip_inhibited && Slip_Observation() == 0u);
    assert(!Fsm_Running_Landing().active);
}

// 反馈变旧时撤销抑制，过期速度不能继续挡住 AIR。
static void Test_Slip_Air_Stale(void)
{
    body.vel_lft = body.vel_rgt = body.vel = 0.0f;
    Air_Init();
    Slip_Init();
    Fsm_Running_Enter();
    for (uint32_t i = 0u; i < SLIP_CONTACT_MS + 5u; i++) Test_Frame(85.0f, 85.0f, 3u);
    body.vel_lft = 0.30f;
    Test_Frame(-15.0f, 40.0f, 3u);
    assert(air_status.slip_inhibited && !Air_Get());
    for (uint32_t i = 0u; i < SLIP_FEEDBACK_MS + 5u; i++) Test_Frame(-15.0f, 40.0f, 0u);
    assert(!air_status.slip_inhibited && !Air_Get());
    for (uint32_t i = 0u; i <= AIR_DEBOUNCE_MS; i++) Test_Frame(0.0f, 0.0f, 0u);
    assert(Air_Get());
}

// 模拟轮胎持续空转，同时经真实驱动接纳路径给出正向推进轮矩。
static void Test_Trigger(float force, uint8_t side)
{
    intent = 1.0f;
    for (uint32_t i = 0u; i < 20u; i++) Test_Frame(force, force, 3u);
    body.vel_lft = side == 1u ? 0.30f : 0.0f;
    body.vel_rgt = side == 2u ? 0.30f : 0.0f;
    balance_ctrl_mode_t mode = LEG_CTRL_MODE_MOVING;

    for (uint32_t i = 0u; i < SLIP_ENTER_MS + 5u && mode != LEG_CTRL_MODE_STEP; i++)
    {
        const distributor_frame_t drive = {
            .tick = test_tick,
            .cycle = test_frame_cycle,
            .phi = {{leg_lft.pitch_1, leg_lft.pitch_4}, {leg_rgt.pitch_1, leg_rgt.pitch_4}},
            .force = {leg_control_lft.F, leg_control_rgt.F},
            .hip = {leg_control_lft.Tp, leg_control_rgt.Tp},
            .spring_f = {-leg_control_lft.SpringF, -leg_control_rgt.SpringF},
            .wheel = {-0.25f, -0.25f},
        };
        float nominal[2][2];

        assert(Distributor_Send(&drive, nominal));
        mode = Test_Frame(force, force, 3u);
    }
    if (mode != LEG_CTRL_MODE_STEP)
    {
        step_status_t status;
        execution_sample_t sample;

        App_Step_Status(&status);
        Execution_Latest(&sample);
        fprintf(stderr, "side=%u mode=%d slip=%d reject=%d duration=%lu/%lu vel=%.3f residual=%.3f/%.3f torque=%.3f/%.3f\n",
                side, mode, status.slip.side, status.rejected, (unsigned long)status.slip.duration[0],
                (unsigned long)status.slip.duration[1], body.vel_kalman, status.slip.residual[0],
                status.slip.residual[1], sample.model_u[0], sample.model_u[1]);
    }
    assert(mode == LEG_CTRL_MODE_STEP);
    body.vel_lft = body.vel_rgt = 0.0f;
}

int main(void)
{
    step_status_t status;
    balance_mpc_diag_t mpc;
    execution_sample_t sample;
    const float force = 85.0f;

    leg_lft = leg_rgt = (leg_state_t){.len = 0.30f, .pitch_1 = 2.3f, .pitch_4 = 0.8f};
    body = (body_state_t){.leg_len = 0.30f, .h_body = 0.30f, .ang_pitch = BALANCE_PITCH_REF};
    imu.online = true;
    Dm_Motor_Init();
    Dji_Motor_Init();
    dm_f_lft.enabled = dm_b_lft.enabled = dm_f_rgt.enabled = dm_b_rgt.enabled = 1u;
    dm_f_lft.online = dm_b_lft.online = dm_f_rgt.online = dm_b_rgt.online = 1u;
    Air_Init();
    Slip_Init();
    App_Step_Init();
    Balance_Control_Init();
    Balance_Mpc_Init();
    Fsm_Running_Enter();
    for (uint32_t i = 0u; i < 102u; i++) Test_Frame(force, force, 3u);
    assert(air_status.side_contact_mask == 3u && Slip_Observation() == 3u);
    assert(Test_Frame(force, force, 0u) == LEG_CTRL_MODE_STANDING && Slip_Observation() == 0u);
    Balance_Mpc_Diagnostic(&mpc);
    const uint32_t old_plan = mpc.sample_tick;
    const uint32_t ground_epoch = Execution_Epoch();
    const float held_height = body.h_tgt;

    assert(old_plan > 0u);
    Test_Trigger(force, 1u);
    const uint32_t step_tick = test_tick;
    const uint32_t step_epoch = Execution_Epoch();
    Balance_Mpc_Diagnostic(&mpc);
    const uint32_t commits_before_step = mpc.commits;
    assert(step_epoch != ground_epoch && !mpc.paired && mpc.sample_tick < step_tick);
    Execution_Latest(&sample);
    assert(sample.epoch == step_epoch && sample.accepted_mask == EXECUTION_CHANNEL_MASK);

    height_delta = 0.00025f;
    App_Step_Status(&status);
    balance_ctrl_mode_t mode = LEG_CTRL_MODE_STEP;
    bool held = false, returned = false;
    for (uint32_t i = 0u; i < 4000u && mode == LEG_CTRL_MODE_STEP; i++)
    {
        leg_lft.len = status.target[0];
        leg_lft.d_len = status.target_rate;
        const float left = status.phase == STEP_RETRACT || status.phase == STEP_HOLD ? 0.0f : force;

        execution_sample_t previous;

        Execution_Latest(&previous);
        mode = Test_Frame(left, force, 3u);
        if (mode != LEG_CTRL_MODE_STEP)
        {
            Balance_Mpc_Diagnostic(&mpc);
            assert(mpc.paired && mpc.epoch != previous.epoch);
            for (unsigned int side = 0u; side < 2u; side++)
            {
                // 单腿支撑可能超过接地箱界；交接先投影到箱内，再按 1 ms 限速。
                const float anchor = Tinympc14_Clip(previous.total_s[side], TINYMPC14_F_MIN, TINYMPC14_F_MAX);

                assert(fabsf(mpc.plan[4u+side] - anchor) <= TINYMPC14_DF_RATE/2 + 0.001f);
            }
            Execution_Latest(&sample);
            for (unsigned int channel = 0u; channel < 4u; channel++)
                assert(fabsf(sample.model_u[channel] - previous.model_u[channel]) < 0.02f);
        }
        assert(!Air_Get() && !Fsm_Running_Landing().active);
        if (mode == LEG_CTRL_MODE_STEP)
        {
            assert(body.h_tgt == held_height);
            Balance_Mpc_Diagnostic(&mpc);
            assert(!mpc.paired && mpc.commits == commits_before_step);
        }
        App_Step_Status(&status);
        held |= status.phase == STEP_HOLD;
        returned |= status.phase == STEP_RETURN;
    }
    // 夹具的测量跟随上一控制拍的目标，允许一拍 0.5 mm 时差。
    assert(held && returned && fabsf(status.target[0] - 0.30f) < 1e-6f);
    assert(fabsf(leg_lft.len - status.target[0]) <= STEP_MOVE_RATE * CTRL_DT + 1e-6f);
    Balance_Mpc_Diagnostic(&mpc);
    assert(mpc.sample_tick >= step_tick && mpc.paired);
    height_delta = 0.0f;
    intent = 0.0f;
    App_Step_Status(&status);
    assert(status.phase == STEP_WAIT_CLEAR && status.terminated == STEP_REASON_COMPLETE);
    assert(mode == LEG_CTRL_MODE_MOVING && Execution_Epoch() != step_epoch);
    assert(fabsf(body.h_tgt - body.h_body) < 1e-5f && !Fsm_Running_Landing().active);
    assert(body.leg_len_tgt_lft == status.target[0] && body.leg_len_tgt_rgt == status.target[1]);
    for (uint32_t i = 0u; i < 200u; i++) Test_Frame(force, force, 3u);
    Balance_Mpc_Diagnostic(&mpc);
    assert(mpc.sample_tick > old_plan && Slip_Observation() == 3u);
    for (uint32_t i = 0u; i < SLIP_PREDICT_MAX_MS; i++) Test_Frame(force, force, 0u);
    assert(Slip_Observation() == 0u && Slip_Prediction_Expired());
    Test_Frame(force, force, 3u);
    assert(!Slip_Prediction_Expired());

    for (uint32_t i = 0u; i < SLIP_EXIT_MS + 5u; i++) Test_Frame(force, force, 3u);
    Test_Trigger(force, 2u);
    for (uint32_t i = 0u; i < SLIP_CONTACT_MS + 4u; i++)
        mode = Test_Frame(0.0f, 0.0f, 3u);
    App_Step_Status(&status);
    assert(Air_Get() && mode == LEG_CTRL_MODE_AIR && status.terminated == STEP_REASON_AIR);
    Test_Slip_Air_Guard(0.30f, 0.0f);
    Test_Slip_Air_Guard(0.0f, -0.30f);
    Test_Slip_Air_Guard(0.30f, 0.30f);
    Test_Slip_Air_Guard(0.15f, -0.15f);
    Test_Slip_Air_Stale();
    puts("PASS: side landing, KF, automatic step, same-frame slip guard and dual-unload AIR");
    return 0;
}
