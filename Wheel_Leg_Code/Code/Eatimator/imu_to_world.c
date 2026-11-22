/**
 * @file  imu_to_world.c
 * @brief IMU 到世界坐标系转换（极性按上交建模约定）：pitch 折返还原
 * @version  v1.6  2026-10-26
 */
#include "imu_to_world.h"

#include "imu.h"
#include "code_config.h"
#include "system_key.h"
#include "lowpass.h"
#include <math.h>
#include <stdbool.h>

imu_world_t imu_world;


// 机体正立返回 +1 倒立返回 -1 首次确认前返回 0 确认后锁存
static float Imu_Gravity_Sign(void)
{
    static int16_t cnt = 0;
    static float sign  = 0.0f;

    if (imu.acc[IMU_ACC_Z] > IMU_G_DEAD)
    {
        if (cnt < 0)
        {
            cnt = 0; //! 换侧必须清零重数 否则从 -20 爬到 +20 确认延迟翻倍
        }
        if (cnt < IMU_G_CONFIRM)
        {
            cnt++;
        }
    }
    else if (imu.acc[IMU_ACC_Z] < -IMU_G_DEAD)
    {
        if (cnt > 0)
        {
            cnt = 0;
        }
        if (cnt > -IMU_G_CONFIRM)
        {
            cnt--;
        }
    }

    if (cnt >= IMU_G_CONFIRM)
    {
        sign = 1.0f;
    }
    else if (cnt <= -IMU_G_CONFIRM)
    {
        sign = -1.0f;
    }

    return sign;
}

// 折回 (-π, π]
static float Imu_Wrap_Pi(float ang)
{
    float w = fmodf(ang, TWO_PI);
    if (w > PI)
    {
        w -= TWO_PI;
    }
    else if (w <= -PI)
    {
        w += TWO_PI;
    }
    return w;
}

// 比力按四元数旋进世界系去 1g 三路低通后写入 单位 g
static void Imu_Accel_To_World(void)
{
    static lowpass1d_t lp_x = {.alpha = IMU_ACC_LPF_ALPHA};
    static lowpass1d_t lp_y = {.alpha = IMU_ACC_LPF_ALPHA};
    static lowpass1d_t lp_z = {.alpha = IMU_ACC_LPF_ALPHA};

    const float q0 = imu.quat[0], q1 = imu.quat[1], q2 = imu.quat[2], q3 = imu.quat[3];

    const float r00 = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    const float r01 = 2.0f * (q1 * q2 - q0 * q3);
    const float r02 = 2.0f * (q1 * q3 + q0 * q2);
    const float r10 = 2.0f * (q1 * q2 + q0 * q3);
    const float r11 = 1.0f - 2.0f * (q1 * q1 + q3 * q3);
    const float r12 = 2.0f * (q2 * q3 - q0 * q1);
    const float r20 = 2.0f * (q1 * q3 - q0 * q2);
    const float r21 = 2.0f * (q2 * q3 + q0 * q1);
    const float r22 = 1.0f - 2.0f * (q1 * q1 + q2 * q2);

    const float ax = imu.acc[IMU_ACC_X];
    const float ay = imu.acc[IMU_ACC_Y];
    const float az = imu.acc[IMU_ACC_Z];

    imu_world.acc_x = Lowpass_Update(&lp_x, r00 * ax + r01 * ay + r02 * az);
    imu_world.acc_y = Lowpass_Update(&lp_y, r10 * ax + r11 * ay + r12 * az);
    imu_world.acc_z = Lowpass_Update(&lp_z, r20 * ax + r21 * ay + r22 * az - 1.0f);
}

void Imu_To_World(void)
{
    static float pitch_last; // 上帧折返还原后的 pitch (rad)
    static bool inited;

    Imu_Accel_To_World();

    float pitch_raw = DEG_TO_RAD(imu.eul[IMU_EUL_PITCH]);
    float sign      = Imu_Gravity_Sign();

    //! eul 的 pitch 在 ±90° 处折返而非跳变 等价于丢了 cos 的符号
    //! 用 acc_z 的符号补回 把 ±90° 折返角还原成正常的 ±180° 表示
    //! 折返点处 cos≈0 atan2 结果与 sign 无关 故 sign 确认滞后不会在边界引入阶跃
    float pitch;
    if (sign != 0.0f)
    {
        pitch = atan2f(sinf(pitch_raw), sign * cosf(pitch_raw));
    }
    else
    {
        pitch = pitch_raw; // 首次确认前按正立处理
    }

    if (!inited)
    {
        pitch_last      = pitch;
        imu_world.pitch = pitch;
        inited          = true;
        return;
    }

    // 增量折回 (-π, π] 再累加：跨 ±180° 的跳变折成小增量 翻转方向由连续性决定
    //! imu_world.pitch 因此是多圈值 用前必须折回 (-π, π]
    float d    = pitch - pitch_last;
    pitch_last = pitch;
    while (d > PI) d -= 2.0f * PI;
    while (d < -PI) d += 2.0f * PI;

    imu_world.pitch += d;
}

// 腿世界系摆角 (rad)：腿电机系摆角 + 机体 pitch，出参折回 (-π, π]
//! 不可用 body.leg_ang_* 替代：它含折返的 body.ang_pitch 躺姿读成 60°，腿角跟着砍半
float Imu_To_World_Leg_Ang(float leg_ang)
{
    return Imu_Wrap_Pi(leg_ang + imu_world.pitch);
}
