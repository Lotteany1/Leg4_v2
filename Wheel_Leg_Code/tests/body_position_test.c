/** @file  body_position_test.c  @brief 真实机体估计的位置积分与横滚角速度验证  @version  v1.2  2026-11-19 */
#include "body.h"
#include "code_config.h"
#include "system_key.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "imu.h"
#include "leg.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

imu_t imu;
leg_state_t leg_lft, leg_rgt;
dm_motor_t dm_f_lft, dm_b_lft, dm_f_rgt, dm_b_rgt;
dji_motor_t dji_wheel_lft, dji_wheel_rgt;

// 用独立正弦检测 9 Hz 相位，同时验证重复帧与一次长间隔后的恢复。
static void Test_Roll_Kalman(void)
{
    const float omega = 2.0f * PI * 9.0f * CTRL_DT;
    float in_phase = 0.0f;
    float quadrature = 0.0f;

    Body_State_Init();
    imu.online = true;
    imu.last_rx_tick = 100u;
    imu.gyr[IMU_GYR_ROLL] = 0.0f;
    Body_State_Update();
    assert(body.omg_roll_filt == 0.0f);

    for (unsigned int k = 1u; k <= 1200u; k++)
    {
        const float phase = omega * (float)k;

        imu.last_rx_tick++;
        imu.gyr[IMU_GYR_ROLL] = sinf(phase) * (180.0f / PI);
        Body_State_Update();
        if (k > 200u)
        {
            in_phase += body.omg_roll_filt * sinf(phase);
            quadrature += body.omg_roll_filt * cosf(phase);
        }
    }
    assert(in_phase > 480.0f && in_phase < 540.0f);
    assert(quadrature < 0.0f && -quadrature < 0.16f * in_phase);

    const float before = body.omg_roll_filt;

    imu.gyr[IMU_GYR_ROLL] = 30.0f;
    Body_State_Update();
    assert(body.omg_roll_filt == before);
    imu.last_rx_tick += IMU_OFFLINE_MS + 1u;
    Body_State_Update();
    assert(body.omg_roll_filt == body.omg_roll);

    imu.online = false;
    Body_State_Update();
    imu.online = true;
    imu.last_rx_tick++;
    imu.gyr[IMU_GYR_ROLL] = -20.0f;
    Body_State_Update();
    assert(body.omg_roll_filt == body.omg_roll);
    puts("PASS: 9 Hz roll Kalman tracking, duplicate IMU timestamp and reseed");
}

// 固定倾角的纯偏航以 6.5 Hz 往复，IMU 三轴均有信号，真实欧拉横滚导数始终为零。
static void Test_Yaw_Roll_Rejection(void)
{
    const float pitch = -0.12f;
    const float roll = 0.16f;

    Body_State_Init();
    imu.online = true;
    imu.eul[IMU_EUL_PITCH] = RAD_TO_DEG(pitch);
    imu.eul[IMU_EUL_ROLL] = RAD_TO_DEG(roll);
    for (unsigned int k = 0u; k < 200u; k++)
    {
        float yaw_rate = 2.0f * cosf(2.0f * PI * 6.5f * 0.005f * k);

        imu.last_rx_tick += 5u;
        imu.gyr[IMU_GYR_ROLL]  = RAD_TO_DEG(-yaw_rate * sinf(pitch));
        imu.gyr[IMU_GYR_PITCH] = RAD_TO_DEG(yaw_rate * sinf(roll) * cosf(pitch));
        imu.gyr[IMU_GYR_YAW]   = RAD_TO_DEG(yaw_rate * cosf(roll) * cosf(pitch));
        Body_State_Update();
        assert(fabsf(body.omg_roll_filt) < 1e-6f);
    }
    puts("PASS: tilted 6.5 Hz pure yaw gives zero filtered Euler roll rate");
}

// 实际轮腿速度与融合速度刻意取相反方向，验证积分选源而非只检查宏值。
// 驾驶指令与里程积分解耦；双腿倾角决定本拍是否积分。
int main(void)
{
    const float wheel_speed    = 0.25f;
    const float fused_speed    = -0.5f;
    const float expected_speed = BALANCE_USE_SLIP ? fused_speed : wheel_speed;

    leg_lft.len       = leg_rgt.len = 0.25f;
    leg_lft.pitch_1   = leg_rgt.pitch_1 = 0.5f;
    leg_lft.pitch_4   = leg_rgt.pitch_4 = 2.5f;
    dji_wheel_lft.vel = -wheel_speed * REDUCTION_RATIO / WHEEL_R;
    dji_wheel_rgt.vel = wheel_speed * REDUCTION_RATIO / WHEEL_R;
    body.vel_kalman   = fused_speed;
    Body_State_Init();

    for (unsigned int i = 0u; i < 100u; i++)
    {
        Body_State_Update();
        Body_Position_Update();
    }
    assert(fabsf(body.vel - wheel_speed) < 1e-6f);
    assert(fabsf(body.pos - 100.0f * CTRL_DT * expected_speed) < 1e-6f);

    body.pos = 3.6f;
    body.vel_tgt = 1.0f;
    body.omg_yaw_tgt = 1.0f;
    body.ang_yaw_tgt = 0.1f;
    leg_lft.ang = 0.2f;
    Body_State_Update();
    assert(body.pos == 3.6f && body.ang_yaw_tgt == 0.1f);
    Body_Position_Update();
    assert(body.pos == 3.6f);

    body.leg_ang_lft = DEG_TO_RAD(5.0f);
    Body_Position_Update();
    assert(body.pos == 3.6f);
    body.leg_ang_lft = DEG_TO_RAD(4.9f);
    body.leg_ang_rgt = DEG_TO_RAD(-5.0f);
    Body_Position_Update();
    assert(body.pos == 3.6f);

    body.leg_ang_rgt = DEG_TO_RAD(-4.9f);
    const float step = CTRL_DT * expected_speed;
    Body_Position_Update();
    assert(fabsf(body.pos - (3.6f + step)) < 1e-6f);

    const float previous = body.pos;
    body.vel = body.vel_kalman = NAN;
    Body_Position_Update();
    assert(body.pos == previous);
    Test_Roll_Kalman();
    Test_Yaw_Roll_Rejection();
    printf("PASS: body position SLIP=%u, velocity source, 5 deg leg gate and invalid sample\n", BALANCE_USE_SLIP);
    return 0;
}
