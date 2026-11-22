/** @file  vel_limit_running_test.c  @brief 遥控映射、转向缩放与速度 S 曲线集成验证  @version  v1.3  2026-11-02 */
#include "fsm_running.h"
#include "air.h"
#include "body.h"
#include "leg.h"
#include "imu_to_world.h"
#include "dr16.h"
#include "music.h"
#include "code_config.h"
#include "system_key.h"
#include "turn_limit.h"
#include "vel_limit.h"
#include "wrap.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

body_state_t body;
leg_state_t leg_lft, leg_rgt;
imu_world_t imu_world;
dr16_t dr16;
volatile air_status_t air_status;

void Air_Set_Landing(uint8_t active)
{
    (void)active;
}

uint32_t Estimator_Frame_Tick(void)
{
    return 0u;
}

// 用固定遥控快照驱动真实映射 不依赖 DMA 或 ISR
dr16_t Dr16_Snapshot(void)
{
    return dr16;
}

// 仅复现映射所需的摇杆死区 外围通道解析不属于本测试
int16_t Dr16_Deadline(int16_t input, uint16_t deadline)
{
    return (input > -(int16_t)deadline && input < (int16_t)deadline) ? 0 : input;
}

// 场景保持落地 离地开关组合仍编译真实分支
uint8_t Air_Get(void)
{
    return 0u;
}

// 陀螺入场上下文与音乐为外围副作用 不替换真实目标更新链路
void App_Gyro_Reset(void)
{
}

void App_Gyro_Enter(void)
{
}

void App_Gyro_Update(bool active)
{
    assert(!active);
}

void Music_Play(uint8_t idx)
{
    (void)idx;
}

void Music_Stop(void)
{
}

// 提供倒地检测所需坐标变换 测试本身保持直立
float Imu_To_World_Leg_Ang(float ang)
{
    return Wrap_Pi(ang + imu_world.pitch);
}

// 模拟控制任务逐拍运行 旋转目标在速度 S 曲线之后应保持原映射值
static void Vel_Limit_Test_Ticks(unsigned int count)
{
    const float yaw_target = -(float)dr16.ch3 / DR16_Max * TURN_LIMIT_YAW_FULL;

    for (unsigned int i = 0u; i < count; i++)
    {
        Fsm_Running_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, &dr16);
        assert(isfinite(body.vel_tgt));
        assert(fabsf(body.omg_yaw_tgt - yaw_target) < 1.0e-6f);
    }
}

#if BALANCE_USE_VEL_LIMIT
// 第一拍与 0.25 s 时刻必须符合解析 S 曲线 防止漏接 每拍重置或一拍调用多次
static void Vel_Limit_Test_Entry(void)
{
    const float first = 0.5f * VEL_LIMIT_S_CURVE_JERK_MAX * CTRL_DT * CTRL_DT;

    body.vel        = 0.8f;
    body.vel_kalman = 0.9f;
    Fsm_Running_Enter();
    assert(body.vel_tgt == 0.0f);
    assert(body.vel == 0.8f && body.vel_kalman == 0.9f);
    Vel_Limit_Test_Ticks(1u);
    assert(fabsf(body.vel_tgt - first) < 1.0e-8f);
    Vel_Limit_Test_Ticks(249u);
    assert(fabsf(body.vel_tgt - 0.5f * VEL_LIMIT_S_CURVE_JERK_MAX * 0.25f * 0.25f) < 1.0e-5f);
    Vel_Limit_Test_Ticks(4000u);
    assert(body.vel_tgt == TURN_LIMIT_VEL_FULL);
}

// 缩放后的目标必须先经过斜坡 转向突变不得直接截断速度 恒定转向最终到达缩放目标
static void Vel_Limit_Test_Turn(void)
{
    float previous = body.vel_tgt;
    const float first_step = 0.5f * VEL_LIMIT_S_CURVE_JERK_MAX * CTRL_DT * CTRL_DT;

    dr16.ch3 = 330;
    Vel_Limit_Test_Ticks(1u);
    assert(fabsf(previous - body.vel_tgt - first_step) < 1.0e-6f);
    Vel_Limit_Test_Ticks(4000u);
    assert(body.vel_tgt == 0.5f * TURN_LIMIT_VEL_FULL);
    previous = body.vel_tgt;
    dr16.ch3 = 660;
    Vel_Limit_Test_Ticks(1u);
    assert(fabsf(previous - body.vel_tgt - first_step) < 1.0e-6f);
    Vel_Limit_Test_Ticks(4000u);
    assert(body.vel_tgt == 0.0f);
}

// 加速中松杆立即制动 随后准确回零 反打则连续经过零速而非阶跃
static void Vel_Limit_Test_Release_Reverse(void)
{
    float previous;
    const float first_step = 0.5f * VEL_LIMIT_S_CURVE_JERK_MAX * CTRL_DT * CTRL_DT;

    dr16.ch3 = 0;
    Vel_Limit_Test_Ticks(500u);
    previous = body.vel_tgt;
    dr16.ch2 = 0;
    Vel_Limit_Test_Ticks(1u);
    assert(body.vel_tgt < previous);
    Vel_Limit_Test_Ticks(4000u);
    assert(body.vel_tgt == 0.0f);
    dr16.ch2 = 660;
    Vel_Limit_Test_Ticks(4000u);
    assert(body.vel_tgt == TURN_LIMIT_VEL_FULL);
    dr16.ch2 = -660;
    previous = body.vel_tgt;
    Vel_Limit_Test_Ticks(1u);
    assert(fabsf(previous - body.vel_tgt - first_step) < 1.0e-6f);
    Vel_Limit_Test_Ticks(7000u);
    assert(body.vel_tgt == -TURN_LIMIT_VEL_FULL);
}
#endif

// 制动时停用位置反馈，接近零速捕获原点，然后用连续里程计算相对误差。
static void Vel_Limit_Test_Stop_Position(void)
{
    dr16.ch3 = 0;
    for (int sign = -1; sign <= 1; sign += 2)
    {
        body.vel = body.vel_kalman = (float)sign * 2.0f;
        dr16.ch2 = sign * 660;
        Fsm_Running_Enter();
        Vel_Limit_Test_Ticks(4000u);
        dr16.ch2 = 0;
        Vel_Limit_Test_Ticks(4000u);
        assert(body.vel_tgt == 0.0f);
        const uint32_t capture = Fsm_Running_Position().capture;
        body.pos = (float)sign * 0.47f;
        Vel_Limit_Test_Ticks(1u);
        assert(Fsm_Running_Position().weight == 0.0f && body.pos == (float)sign * 0.47f);
#if BALANCE_USE_SLIP
        body.vel = 0.0f;
#else
        body.vel_kalman = 0.0f;
#endif
        body.pos = (float)sign * 0.01f;
        Vel_Limit_Test_Ticks(1u);
        assert(Fsm_Running_Position().capture == capture && Fsm_Running_Position().weight == 0.0f);
        body.vel = body.vel_kalman = (float)sign * 0.05f;
        body.pos = (float)sign * 0.001f;
        Vel_Limit_Test_Ticks(1u);
        assert(Fsm_Running_Position().capture == capture + 1u && Fsm_Running_Position().error == 0.0f);
        body.pos = (float)sign * 0.02f;
        Vel_Limit_Test_Ticks(1u);
        assert(fabsf(Fsm_Running_Position().error - (float)sign * 0.019f) < 1.0e-6f);
        body.vel = body.vel_kalman = (float)sign * -0.3f;
        Vel_Limit_Test_Ticks(1u);
        assert(Fsm_Running_Position().capture == capture + 1u && body.pos == (float)sign * 0.02f);
        Fsm_Running_Enter();
        body.pos = (float)sign * 0.03f;
        Vel_Limit_Test_Ticks(1u);
        assert(Fsm_Running_Position().weight == 0.0f && body.pos == (float)sign * 0.03f);
    }
}

// 真实遥控映射与转向限幅都参与运行 结束前重进平衡验证斜坡历史不残留
int main(void)
{
    leg_lft.len = leg_rgt.len = 0.23f;
    body.h_body               = 0.20f;
    dr16.ch2                  = 660;
#if BALANCE_USE_VEL_LIMIT
    Vel_Limit_Test_Entry();
    Vel_Limit_Test_Turn();
    Vel_Limit_Test_Release_Reverse();
    dr16.ch2 = 660;
    Vel_Limit_Test_Ticks(500u);
    Vel_Limit_Test_Entry();
    printf("running S-curve PASS: AIR_CHECK=%d; entry, turn, release, reversal and re-entry\n",
           BALANCE_USE_AIR_CHECK);
#else
    // 关闭时无斜坡延迟 转向缩放继续生效 松杆与反向在当拍直接更新目标
    Fsm_Running_Enter();
    assert(body.vel_tgt == 0.0f);
    Vel_Limit_Test_Ticks(1u);
    assert(body.vel_tgt == TURN_LIMIT_VEL_FULL);
    dr16.ch3 = 330;
    Vel_Limit_Test_Ticks(1u);
    assert(body.vel_tgt == 0.5f * TURN_LIMIT_VEL_FULL);
    dr16.ch2 = 0;
    Vel_Limit_Test_Ticks(1u);
    assert(body.vel_tgt == 0.0f);
    dr16.ch3 = 0;
    dr16.ch2 = -660;
    Vel_Limit_Test_Ticks(1u);
    assert(body.vel_tgt == -TURN_LIMIT_VEL_FULL);
    Fsm_Running_Enter();
    assert(body.vel_tgt == 0.0f);
    Vel_Limit_Test_Ticks(1u);
    assert(body.vel_tgt == -TURN_LIMIT_VEL_FULL);
    printf("running bypass PASS: AIR_CHECK=%d; entry, turn, release, reversal and re-entry\n",
           BALANCE_USE_AIR_CHECK);
#endif
    Vel_Limit_Test_Stop_Position();
    printf("stop position PASS: SLIP=%d; brake, capture, drift hold, symmetry and re-entry\n", BALANCE_USE_SLIP);
    return 0;
}
