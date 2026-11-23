/** @file  fsm_rescue_targets_test.c  @brief 平衡入场、目标更新顺序与倒地确认回归  @version  v1.5  2026-11-22 */
#include "fsm_running.h"
#include "air.h"
#include "body.h"
#include "leg.h"
#include "imu_to_world.h"
#include "code_config.h"
#include "system_key.h"
#include "wrap.h"
#include "music.h"
#include "songs.h"
#include "vel_limit.h"
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

body_state_t body;
leg_state_t leg_lft, leg_rgt;
imu_world_t imu_world;
static unsigned test_stage;         // 本拍外部调用进度：RC、陀螺切入、陀螺更新、限幅
static unsigned test_gyro_enters;   // 陀螺实际切入次数，Reset 不清此观测计数
static unsigned test_gyro_resets;   // 平衡入场复位次数
static unsigned test_gyro_inactive; // 切出后仍调用陀螺更新的拍数
static unsigned test_air_reads;     // 离地开关关闭时不应读取离地标志
static uint8_t test_air;            // 模拟离地电平
static uint8_t test_land_active;    // 回读运行态给离地检测的重武装选择
static uint32_t test_tick;          // ms 估计帧时间
volatile air_status_t air_status;
static bool test_expect_enter;      // 本拍是否应发生一次陀螺切入
static bool test_expect_active;     // 本拍陀螺更新入参
static float test_expect_pos;       // m 遥控映射前应看到的位置
static float test_expect_yaw;       // rad 遥控映射前应看到的偏航目标
static float test_rc_speed = 2.0f;
static float test_rc_yaw = 1.0f;
static float test_yaw_delta = 0.25f;
static float test_yaw_scale = 1.0f; // 转向限制对最终转速的缩放
static float test_rc_h_step;        // m 遥控映射本拍增加的高度
static float test_gyro_entry_h;     // m 陀螺切入实际锁存的高度

// 蜂鸣器是运行模块的外围副作用，不参与本夹具的目标更新顺序。
void Music_Play(uint8_t idx)
{
    (void)idx;
}

void Music_Stop(void)
{
}

// 测试浮点目标量；调用顺序通过边界处观测检查，不读取运行模块内部变量。
static void Test_Close(float actual, float expected)
{
    if (!(fabsf(actual - expected) < 0.0001f))
        fprintf(stderr, "target mismatch: actual=%g expected=%g stage=%u\n", (double)actual, (double)expected, test_stage);
    assert(fabsf(actual - expected) < 0.0001f);
}

// 离地电平只允许在目标更新前读取一次。
uint8_t Air_Get(void)
{
    assert(test_stage == 0u);
    test_air_reads++;
    return test_air;
}

void Air_Set_Landing(uint8_t active)
{
    test_land_active = active;
}

uint32_t Estimator_Frame_Tick(void)
{
    return test_tick;
}

// 保留累计入场次数，以区分重进平衡后的复位和真正的陀螺切入。
void App_Gyro_Reset(void)
{
    test_gyro_resets++;
}

// 切入必须位于遥控映射之后，锁定的是本拍遥控已更新的高度。
void App_Gyro_Enter(void)
{
    assert(test_expect_enter && test_stage == 1u);
    test_gyro_entry_h = body.h_tgt;
    test_gyro_enters++;
    test_stage = 2u;
}

// 切出仍逐拍接收 false；修改偏航目标供后续限幅检查调用顺序。
void App_Gyro_Update(bool active)
{
    assert(active == test_expect_active);
    assert(test_stage == (test_expect_enter ? 2u : 1u));
    Test_Close(body.omg_yaw_tgt, test_rc_yaw);
    body.omg_yaw_tgt += test_yaw_delta;
    if (!active)
        test_gyro_inactive++;
    test_stage = 3u;
}

// 落地沿应先锁偏航，再让遥控映射写本拍目标。
void App_Rc_Update_Targets(body_state_t *state, const dr16_t *rc)
{
    (void)rc;
    assert(test_stage == 0u);
    Test_Close(state->pos, test_expect_pos);
    Test_Close(state->ang_yaw_tgt, test_expect_yaw);
    state->vel_tgt     = test_rc_speed;
    state->omg_yaw_tgt = test_rc_yaw;
    state->h_tgt += test_rc_h_step;
    test_stage = 1u;
}

// 限幅只消费陀螺更新后的目标，不得提前执行。
void Turn_Limit_Update(body_state_t *state)
{
    assert(test_stage == 3u);
    Test_Close(state->omg_yaw_tgt, test_rc_yaw + test_yaw_delta);
    state->omg_yaw_tgt *= test_yaw_scale;
    test_stage = 4u;
}

// 与真实倒地判据使用相同的机体系到世界系角度转换边界。
float Imu_To_World_Leg_Ang(float ang)
{
    return Wrap_Pi(ang + imu_world.pitch);
}

#include "fsm_running_under_test.h"

// 推进一拍并检查边界调用完整性；位置和偏航期望反映本拍是否存在落地沿。
static balance_ctrl_mode_t Test_Tick(fsm_running_behavior_t behavior, bool enter, float pos, float yaw)
{
    unsigned air_reads = test_air_reads;
    unsigned enters    = test_gyro_enters;

    test_stage         = 0u;
    test_expect_enter  = enter;
    test_expect_active = behavior == FSM_RUNNING_BEHAVIOR_GYRO;
    test_expect_pos    = pos;
    test_expect_yaw    = yaw;
    const balance_ctrl_mode_t mode = Fsm_Running_Tick(behavior, &(dr16_t){0});
    assert(test_stage == 4u);
    assert(test_gyro_enters == enters + (enter ? 1u : 0u));
#if BALANCE_USE_AIR_CHECK
    assert(test_air_reads == air_reads + 1u);
#else
    assert(test_air_reads == air_reads);
#endif
    return mode;
}

// 入场保持实测长度与高度及速度观测；陀螺只在切入时锁本拍遥控目标。
static void Test_Entry_And_Behavior(void)
{
    leg_lft.len     = 0.22f;
    leg_rgt.len     = 0.24f;
    body.h_body     = 0.20f;
    body.vel        = 0.8f;
    body.vel_kalman = 0.9f;
    body.ang_yaw    = 0.7f;
    body.pos        = 0.4f;
    Fsm_Running_Enter();
    assert(test_gyro_resets == 1u);
    Test_Close(body.leg_len_tgt_lft, 0.23f);
    Test_Close(body.leg_len_tgt_rgt, 0.23f);
    Test_Close(body.h_tgt, 0.20f);
    Test_Close(body.vel, 0.8f);
    Test_Close(body.vel_kalman, 0.9f);
    assert(body.vel_tgt == 0.0f && body.omg_yaw_tgt == 0.0f);

    test_rc_h_step = 0.01f;
    Test_Tick(FSM_RUNNING_BEHAVIOR_GYRO, true, 0.4f, 0.7f);
    Test_Close(test_gyro_entry_h, 0.21f);
#if BALANCE_USE_VEL_LIMIT
    Test_Close(body.vel_tgt, 0.5f * VEL_LIMIT_S_CURVE_JERK_MAX * CTRL_DT * CTRL_DT);
#else
    Test_Close(body.vel_tgt, 2.0f);
#endif
    Test_Close(body.h_tgt, 0.21f);
    test_rc_h_step = 0.0f;
    Test_Tick(FSM_RUNNING_BEHAVIOR_GYRO, false, 0.4f, 0.7f + 1.25f * CTRL_DT);
    Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, 0.4f, 0.7f + 2.5f * CTRL_DT);
    Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, 0.4f, 0.7f + 3.75f * CTRL_DT);
    assert(test_gyro_inactive == 2u);
    Test_Tick(FSM_RUNNING_BEHAVIOR_GYRO, true, 0.4f, 0.7f + 5.0f * CTRL_DT);

    Fsm_Running_Enter();
    assert(test_gyro_resets == 2u);
    Test_Close(body.h_tgt, 0.20f);
    Test_Tick(FSM_RUNNING_BEHAVIOR_GYRO, true, 0.4f, 0.7f);
    Test_Close(test_gyro_entry_h, 0.20f);
    assert(test_gyro_enters == 3u);
}

// 离地沿与转向均可更新航向；重进平衡清除离地历史。
static void Test_Landing(void)
{
    test_air         = 1u;
    body.pos         = 2.0f;
    body.ang_yaw_tgt = 0.2f;
    body.ang_yaw     = 0.9f;
    Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, 2.0f, 0.2f);

    test_air = 0u;
    Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, 2.0f, BALANCE_USE_AIR_CHECK ? 0.9f : body.ang_yaw_tgt);

    body.pos         = 3.0f;
    body.ang_yaw_tgt = 0.4f;
    body.ang_yaw     = 1.0f;
    Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, 3.0f, 0.4f);
    test_air = 1u;
    Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, 3.0f, body.ang_yaw_tgt);
    Fsm_Running_Enter();
    body.pos         = 4.0f;
    body.ang_yaw_tgt = 0.5f;
    body.ang_yaw     = 1.2f;
    test_air         = 0u;
    Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, 4.0f, 0.5f);
}

// 独立激励俯仰、横滚或一条腿的世界系倾角；俯仰测试补偿两腿机构角以隔离腿阈值。
static void Test_Tilt_Set(unsigned axis, float angle)
{
    imu_world.pitch = 0.0f;
    body.ang_roll   = 0.0f;
    leg_lft.ang     = 0.0f;
    leg_rgt.ang     = 0.0f;

    switch (axis)
    {
        case 0u: // 俯仰单独越阈，两腿世界系角度保持为零。
            imu_world.pitch = angle;
            leg_lft.ang = leg_rgt.ang = -angle;
            break;

        case 1u: // 横滚单独越阈。
            body.ang_roll = angle;
            break;

        case 2u: // 左腿世界系倾角单独越阈。
            leg_lft.ang = angle;
            break;

        default: // 右腿世界系倾角单独越阈。
            leg_rgt.ang = angle;
            break;
    }
}

// 连续越阈未满 200 ms 不确认，达到边界才置位；不假定控制周期恒为 1 ms。
static void Test_Fall_Confirm(void)
{
    for (uint32_t ms = CTRL_PERIOD_MS; ms < 200u; ms += CTRL_PERIOD_MS)
        assert(!Fsm_Running_Fall_Update());
    assert(Fsm_Running_Fall_Update());
    assert(Fsm_Running_Fall_Update());
}

// 四项判据均检查正负阈值、连续计时、回窗清零及重新入场复位。
static void Test_Fall(void)
{
    for (unsigned axis = 0u; axis < 4u; axis++)
    {
        float threshold = axis < 2u ? 1.0f : 0.8f;

        for (int sign = -1; sign <= 1; sign += 2)
        {
            Fsm_Running_Enter();
            Test_Tilt_Set(axis, (float)sign * threshold);
            for (uint32_t ms = 0u; ms < 200u; ms += CTRL_PERIOD_MS)
                assert(!Fsm_Running_Fall_Update());
            Test_Tilt_Set(axis, (float)sign * (threshold + 0.1f));
            Test_Fall_Confirm();
            Test_Tilt_Set(axis, 0.0f);
            assert(!Fsm_Running_Fall_Update());

            Test_Tilt_Set(axis, (float)sign * (threshold + 0.1f));
            for (uint32_t ms = 0u; ms < 100u; ms += CTRL_PERIOD_MS)
                assert(!Fsm_Running_Fall_Update());
            Test_Tilt_Set(axis, 0.0f);
            assert(!Fsm_Running_Fall_Update());
            Test_Tilt_Set(axis, (float)sign * (threshold + 0.1f));
            Test_Fall_Confirm();

            Fsm_Running_Enter();
            Test_Fall_Confirm();
        }
    }
}

// 真实 FSM 固定输入逐拍验证捕获、驻车扰动、保护斜坡和转向退出。
static void Test_Standing(void)
{
    test_rc_speed = 0.0f;
    test_rc_yaw = 0.0f;
    test_yaw_delta = 0.0f;
    test_air = 0u;
    body = (body_state_t){.pos = 10.0f, .ang_yaw = 0.2f};
    Fsm_Running_Enter();
    const uint32_t captures = Fsm_Running_Position().capture;
    assert(Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, 10.0f, 0.2f) == LEG_CTRL_MODE_MOVING);
    body.vel = body.vel_kalman = FSM_RUNNING_STOP_VEL_TH + 0.01f;
    assert(Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, 10.0f, 0.2f) == LEG_CTRL_MODE_MOVING);
    body.vel = body.vel_kalman = -FSM_RUNNING_STOP_VEL_TH;
    assert(Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, 10.0f, 0.2f) == LEG_CTRL_MODE_STANDING);
    assert(Fsm_Running_Position().capture == captures + 1u && Fsm_Running_Position().error == 0.0f);
    Test_Close(Fsm_Running_Position().weight, FSM_RUNNING_POS_WEIGHT);
    Test_Close(body.vel_tgt, 0.0f);
    body.vel = body.vel_kalman = 0.5f;
    body.pos = 10.125f;
    assert(Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, body.pos, 0.2f) == LEG_CTRL_MODE_STANDING);
    Test_Close(Fsm_Running_Position().error, 0.125f);
    Test_Close(Fsm_Running_Position().weight, FSM_RUNNING_POS_WEIGHT);
    body.pos = 9.875f;
    body.vel = body.vel_kalman = -0.5f;
    assert(Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, body.pos, 0.2f) == LEG_CTRL_MODE_STANDING);
    Test_Close(Fsm_Running_Position().error, -0.125f);
    Test_Close(Fsm_Running_Position().weight, FSM_RUNNING_POS_WEIGHT);
    Test_Close(body.vel_tgt, 0.0f);
    assert(Fsm_Running_Position().capture == captures + 1u);
    body.pos = 10.125f;
    body.leg_ang_lft = FSM_RUNNING_POS_LEG_ANG_TH + 0.01f;
    assert(Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, body.pos, 0.2f) == LEG_CTRL_MODE_STANDING);
    assert(Fsm_Running_Position().weight == 0.0f && (Fsm_Running_Position().events & 1u));
    body.leg_ang_lft = 0.0f;
    for (uint32_t ms = CTRL_PERIOD_MS; ms <= FSM_RUNNING_POS_RAMP_MS; ms += CTRL_PERIOD_MS)
    {
        assert(Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, body.pos, 0.2f) == LEG_CTRL_MODE_STANDING);
        Test_Close(Fsm_Running_Position().weight, FSM_RUNNING_POS_WEIGHT * (float)ms / FSM_RUNNING_POS_RAMP_MS);
    }
    assert(Fsm_Running_Position().capture == captures + 1u && Fsm_Running_Position().error == 0.125f);
    assert(Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, body.pos, 0.2f) == LEG_CTRL_MODE_STANDING);
    Test_Close(Fsm_Running_Position().weight, FSM_RUNNING_POS_WEIGHT);
    body.pos = 10.0f + FSM_RUNNING_POS_RECAPTURE;
    body.vel = body.vel_kalman = 0.0f;
    assert(Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, body.pos, 0.2f) == LEG_CTRL_MODE_MOVING);
    assert(Fsm_Running_Position().weight == 0.0f);
    assert(Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, body.pos, 0.2f) == LEG_CTRL_MODE_STANDING);
    assert(Fsm_Running_Position().capture == captures + 2u && Fsm_Running_Position().error == 0.0f);
    test_rc_yaw = 0.4f;
    body.ang_yaw = 0.3f;
    assert(Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, body.pos, 0.2f) == LEG_CTRL_MODE_MOVING);
    Test_Close(body.ang_yaw_tgt, 0.2f + 0.4f * CTRL_DT);
    test_rc_yaw = 0.0f;
    body.ang_yaw = 0.4f;
    assert(Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, body.pos, 0.2f + 0.4f * CTRL_DT) == LEG_CTRL_MODE_STANDING);
    Test_Close(body.ang_yaw_tgt, 0.4f);
    test_rc_h_step = 0.001f;
    const uint32_t before_height = Fsm_Running_Position().capture;
    assert(Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, body.pos, 0.4f) == LEG_CTRL_MODE_STANDING);
    assert(Fsm_Running_Position().capture == before_height);
    test_rc_h_step = 0.0f;
#if BALANCE_USE_AIR_CHECK
    test_air = 1u;
    assert(Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, body.pos, 0.4f) == LEG_CTRL_MODE_AIR);
    assert(Fsm_Running_Position().weight == 0.0f);
    test_air = 0u;
    body.ang_yaw = 0.5f;
    air_status.contact_mask = 1u;
    leg_lft.d_len = -0.2f;
    leg_rgt.d_len = -0.3f;
    body.h_tgt = body.leg_len_tgt_lft = body.leg_len_tgt_rgt = Leg_Len_Max;
    assert(Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, body.pos, 0.5f) == LEG_CTRL_MODE_MOVING);
    assert(Fsm_Running_Landing().active);
    assert(test_land_active == 1u);
    const float land_target = LEG_LEN_CLAMP(FSM_LAND_LEN_TGT);

    Test_Close(body.h_tgt, land_target);
    Test_Close(body.leg_len_tgt_lft, land_target);
    Test_Close(body.leg_len_tgt_rgt, land_target);
    const uint32_t recover_ms = FSM_LAND_RECOVER_MS;
    const float start = recover_ms == 0u ? 1.0f : FSM_LAND_SPRING_START_PCT / 100.0f;
    float prev_scale = start;

    Test_Close(Fsm_Running_Landing().spring_scale, start);
    body.pos += 1.0f;
    for (test_tick = 1u; test_tick <= FSM_LAND_RECOVER_MS + 8u; test_tick++)
    {
        // 两侧伸缩方向不同，单侧回伸脉冲不改变共用斜坡。
        leg_lft.d_len = test_tick == 2u || (test_tick >= 20u && test_tick < 40u) ? 0.2f : -0.2f;
        leg_rgt.d_len = test_tick >= 20u ? 0.2f : -0.3f;
        const balance_ctrl_mode_t mode = Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, body.pos, 0.5f);
        const fsm_landing_t landing = Fsm_Running_Landing();
        const float expected = test_tick < recover_ms ? start + (1.0f - start) * ((float)test_tick / recover_ms) : 1.0f;

        assert(mode == LEG_CTRL_MODE_MOVING);
        assert(landing.active == (test_tick < recover_ms));
        Test_Close(landing.spring_scale, expected);
        assert(landing.spring_scale >= prev_scale && landing.spring_scale <= 1.0f);
        if (landing.active) assert(landing.elapsed_ms == test_tick);
        assert(Fsm_Running_Position().capture == before_height);
        assert(Fsm_Running_Position().weight == 0.0f && Fsm_Running_Position().error == 0.0f);
        prev_scale = landing.spring_scale;
    }

    air_status.contact_mask = 3u;
    // 即使均速为零，双腿反向伸缩期间也留在行驶控制，不捕获驻车。
    leg_lft.d_len = -0.2f;
    leg_rgt.d_len = 0.2f;
    for (unsigned int i = 0u; i < FSM_LAND_STABLE_MS; i++, test_tick++)
        assert(Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, body.pos, 0.5f) == LEG_CTRL_MODE_MOVING);
    assert(test_land_active == 1u);
    Test_Close(body.h_tgt, land_target);
    Test_Close(body.leg_len_tgt_lft, land_target);
    Test_Close(body.leg_len_tgt_rgt, land_target);
    leg_lft.d_len = leg_rgt.d_len = 0.0f;
    body.ang_roll = FSM_LAND_STABLE_ANGLE + 0.01f;
    for (unsigned int i = 0u; i < FSM_LAND_STABLE_MS; i++, test_tick++)
        assert(Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, body.pos, 0.5f) == LEG_CTRL_MODE_MOVING);
    body.ang_roll = 0.0f;
    // 若接触又丢失，停车必须重新等待稳定。
    air_status.contact_mask = 1u;
    assert(Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, body.pos, 0.5f) == LEG_CTRL_MODE_MOVING);
    air_status.contact_mask = 3u;
    for (unsigned int i = CTRL_PERIOD_MS; i < FSM_LAND_STABLE_MS; i += CTRL_PERIOD_MS, test_tick++)
        assert(Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, body.pos, 0.5f) == LEG_CTRL_MODE_MOVING);
    assert(Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, body.pos, 0.5f) == LEG_CTRL_MODE_STANDING);
    assert(test_land_active == 0u);
    assert(Fsm_Running_Position().capture == before_height + 1u);
    Test_Close(Fsm_Running_Position().error, 0.0f);
    Test_Close(Fsm_Running_Position().weight, 0.0f);
    assert(Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, body.pos, 0.5f) == LEG_CTRL_MODE_STANDING);
    Test_Close(Fsm_Running_Position().weight, FSM_RUNNING_POS_WEIGHT * CTRL_PERIOD_MS / FSM_RUNNING_POS_RAMP_MS);
    for (unsigned int ms = 2u * CTRL_PERIOD_MS; ms <= FSM_RUNNING_POS_RAMP_MS; ms += CTRL_PERIOD_MS)
    {
        test_tick += CTRL_PERIOD_MS;
        assert(Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, body.pos, 0.5f) == LEG_CTRL_MODE_STANDING);
        Test_Close(Fsm_Running_Position().weight, FSM_RUNNING_POS_WEIGHT * ms / FSM_RUNNING_POS_RAMP_MS);
    }
#endif
    Fsm_Running_Exit();
    assert(Fsm_Running_Position().weight == 0.0f);
}

// 正反转验证最终指令积分、跨 π 连续性、卡住限幅及松杆停稳后一次锁点。
static void Test_Yaw_Reference(void)
{
    test_air = 0u;
    test_rc_speed = 0.0f;
    test_rc_h_step = 0.0f;
    test_yaw_scale = 0.5f;

    for (int sign = -1; sign <= 1; sign += 2)
    {
        test_rc_yaw = 2.0f * sign;
        test_yaw_delta = 1.0f * sign;
        body = (body_state_t){.ang_yaw = sign * (PI - 0.01f)};
        Fsm_Running_Enter();
        const float origin = body.ang_yaw_tgt;

        for (unsigned int i = 0u; i < 20u; i++)
        {
            Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, body.pos, body.ang_yaw_tgt);
            Test_Close(body.omg_yaw_tgt, 1.5f * sign);
            Test_Close(body.ang_yaw_tgt, origin + (i + 1u) * 1.5f * sign * CTRL_DT);
            body.ang_yaw += sign * CTRL_DT;
        }
        assert(sign * body.ang_yaw_tgt > PI);
        for (unsigned int i = 0u; i < 1000u; i++)
            Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, body.pos, body.ang_yaw_tgt);
        Test_Close(body.ang_yaw_tgt - body.ang_yaw, sign * FSM_RUNNING_YAW_ERROR_MAX);

        // 反打从现有积分目标向回推进，不能跳回实测航向。
        test_rc_yaw = -test_rc_yaw;
        test_yaw_delta = -test_yaw_delta;
        const float before_reverse = body.ang_yaw_tgt;
        Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, body.pos, before_reverse);
        Test_Close(body.ang_yaw_tgt, before_reverse - sign * 1.5f * CTRL_DT);

        test_rc_yaw = test_yaw_delta = 0.0f;
        body.omg_yaw = sign * 2.0f;
        for (unsigned int i = 0u; i < 40u; i++)
        {
            body.ang_yaw += sign * 0.002f;
            Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, body.pos, body.ang_yaw_tgt);
            Test_Close(body.ang_yaw_tgt, body.ang_yaw);
        }

        // 体轴 r 为零，但 q 经横滚耦合后仍在偏航，不能误判为制动完成。
        body.ang_roll = 0.5f;
        body.omg_yaw = 0.0f;
        body.omg_pitch = sign * 0.3f;
        for (uint32_t ms = 0u; ms < 2u * FSM_RUNNING_YAW_STOP_MS; ms += CTRL_PERIOD_MS)
        {
            body.ang_yaw += sign * 0.001f;
            Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, body.pos, body.ang_yaw_tgt);
            Test_Close(body.ang_yaw_tgt, body.ang_yaw);
        }
        body.ang_roll = body.omg_pitch = 0.0f;
        for (uint32_t ms = CTRL_PERIOD_MS; ms < FSM_RUNNING_YAW_STOP_MS; ms += CTRL_PERIOD_MS)
            Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, body.pos, body.ang_yaw_tgt);
        body.omg_yaw = sign * (FSM_RUNNING_YAW_STOP_OMEGA + 0.01f);
        Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, body.pos, body.ang_yaw_tgt);
        body.omg_yaw = sign * 0.05f;
        for (uint32_t ms = 0u; ms < FSM_RUNNING_YAW_STOP_MS; ms += CTRL_PERIOD_MS)
        {
            body.ang_yaw += body.omg_yaw * CTRL_DT;
            Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, body.pos, body.ang_yaw_tgt);
            Test_Close(body.ang_yaw_tgt, body.ang_yaw);
        }
        const float captured = body.ang_yaw_tgt;

        body.ang_yaw += sign * 0.02f;
        Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, body.pos, captured);
        Test_Close(body.ang_yaw_tgt, captured);
        Fsm_Running_Exit();
        Fsm_Running_Enter();
        body.ang_yaw += sign * 0.02f;
        const float entry_yaw = body.ang_yaw_tgt;

        Test_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, false, body.pos, entry_yaw);
        Test_Close(body.ang_yaw_tgt, entry_yaw);
    }
    test_yaw_scale = 1.0f;
}

// 运行真实目标模块，外部算法使用边界替身；不对硬件运动效果作断言。
int main(void)
{
#ifdef _MSC_VER
    // 断言仍非零退出并输出证据，避免 WER 对每个失败场景弹窗/等待。
    _set_abort_behavior(0, _CALL_REPORTFAULT);
#endif
    Test_Entry_And_Behavior();
    Test_Landing();
    Test_Fall();
    Test_Standing();
    Test_Yaw_Reference();
    puts("running targets: entry, target ordering, gyro edges, landing history and fall timing PASS");
    return 0;
}
