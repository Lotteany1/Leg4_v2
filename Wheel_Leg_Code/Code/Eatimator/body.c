/** @file  body.c  @brief 机体姿态、轮腿速度与位置估计  @version  v1.16  2026-11-24 */
#include "body.h"
#include "leg.h"
#include "imu.h"
#include "code_config.h"
#include "system_key.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "Leg_InvTrq.h"
#include "arm_math.h"
#include "lowpass.h"
#include "kalman.h"

// 原始角在 ±180° 折返 用上周期值接续 免得 ang_yaw 过界时跳 2π
static float Ang_Unwrap(float cur, float last)
{
    float diff = cur - last;
    while (diff > PI)
        diff -= TWO_PI;
    while (diff < -PI)
        diff += TWO_PI;
    return last + diff;
}

body_state_t body;
lowpass1d_t body_omg_filt[2];
lowpass1d_t body_h_filt;

static kalman_accel_t body_roll_kf;
static uint32_t body_roll_imu_tick; // ms，末次进入横滚卡尔曼更新的 IMU 帧时标
static bool body_roll_kf_ready;

// 复用二维速度与偏置模型，速度状态为横滚角速度，偏置状态为负横滚角加速度。
// 只用新 IMU 帧更新；通信间隔过长时用恢复首帧重新播种，避免把旧加速度带入恢复拍。
static void Body_Roll_Rate_Update(float rate)
{
    const uint32_t imu_tick = imu.last_rx_tick;
    const uint32_t elapsed  = imu_tick - body_roll_imu_tick;

    if (!imu.online)
    {
        body_roll_kf_ready = false;
        body.omg_roll_filt = rate;
        return;
    }
    if (!body_roll_kf_ready || elapsed > IMU_OFFLINE_MS)
    {
        Kalman_Accel_Init(&body_roll_kf, rate, BODY_ROLL_KF_R, BODY_ROLL_KF_Q_W, BODY_ROLL_KF_Q_A, BODY_ROLL_KF_R);
        body_roll_kf.gate  = 0.0f; // 真实横滚加速需要迅速修正预测，IMU 入口已校验帧与有限性。
        body_roll_kf_ready = true;
    }
    else if (elapsed != 0u)
    {
        Kalman_Accel_Predict(&body_roll_kf, 0.0f, (float)elapsed * 0.001f);
        Kalman_Accel_Update(&body_roll_kf, rate);
    }
    body_roll_imu_tick = imu_tick;
    body.omg_roll_filt = body_roll_kf.vel;
}

void Body_State_Init(void)
{
    body.pos = 0.0f;
    Lowpass_Init(&body_omg_filt[0], BODY_PITCH_RATE_LP_ALPHA);
    Lowpass_Init(&body_omg_filt[1], BODY_YAW_RATE_LP_ALPHA);
    body_roll_kf_ready = false;
    Lowpass_Init(&body_h_filt, BODY_HEIGHT_LP_ALPHA);
}

// 每个 CTRL_DT 估计拍在 Leg_State_Update 后调用一次，更新姿态、轮腿速度、支持力与位置。
// 位置积分单独在本拍速度融合后执行。
void Body_State_Update(void)
{
    body.leg_len_lft = leg_lft.len;
    body.leg_len_rgt = leg_rgt.len;
    body.leg_len     = (body.leg_len_lft + body.leg_len_rgt) / 2.0f;

    body.ang_pitch = DEG_TO_RAD(imu.eul[IMU_EUL_PITCH]);
    body.omg_pitch = DEG_TO_RAD(imu.gyr[IMU_GYR_PITCH]);
    Lowpass_Update(&body_omg_filt[0], body.omg_pitch);
    body.omg_pitch_filt = body_omg_filt[0].out;

    body.ang_yaw = Ang_Unwrap(DEG_TO_RAD(imu.eul[IMU_EUL_YAW]), body.ang_yaw_last);
    body.omg_yaw = DEG_TO_RAD(imu.gyr[IMU_GYR_YAW]);
    Lowpass_Update(&body_omg_filt[1], body.omg_yaw);
    body.omg_yaw_filt = body_omg_filt[1].out;

    body.ang_roll = DEG_TO_RAD(imu.eul[IMU_EUL_ROLL]);
    body.omg_roll = DEG_TO_RAD(imu.gyr[IMU_GYR_ROLL]);
    // 先合成同一 IMU 帧的 ZYX 横滚导数，再滤波。转向时体轴 p 与 yaw 混项应同相抵消。
    // 倒地自救保留原始体轴角速度；接地 MPC 在欧拉坐标退化时退出。
    const float cos_pitch = cosf(body.ang_pitch);
    float roll_rate = body.omg_roll;

    if (cos_pitch > BODY_EULER_COS_MIN)
        roll_rate += sinf(body.ang_pitch) * (sinf(body.ang_roll) * body.omg_pitch +
                     cosf(body.ang_roll) * body.omg_yaw) / cos_pitch;
    Body_Roll_Rate_Update(roll_rate);

    body.body_accel_x = imu.acc[IMU_ACC_X];
    body.body_accel_y = imu.acc[IMU_ACC_Y];
    body.body_accel_z = imu.acc[IMU_ACC_Z];

    // 摆杆角加俯仰才是世界系 后面的水平速度分解都在这个系里算
    body.leg_ang_lft = leg_lft.ang + body.ang_pitch;
    body.leg_omg_lft = leg_lft.d_ang + body.omg_pitch;

    body.leg_ang_rgt = leg_rgt.ang + body.ang_pitch;
    body.leg_omg_rgt = leg_rgt.d_ang + body.omg_pitch;

    // 世界系腿角与横滚共同投影到竖直高度 与TinyMPC14 的高度速度使用同一几何定义
    body.h_body = 0.5f * arm_cos_f32(body.ang_roll) *
                  (body.leg_len_lft * arm_cos_f32(body.leg_ang_lft) + body.leg_len_rgt * arm_cos_f32(body.leg_ang_rgt));
    Lowpass_Update(&body_h_filt, body.h_body);
    body.h_body_filt = body_h_filt.out;

    Leg_InvTrq(dm_f_lft.trq, dm_b_lft.trq, leg_lft.pitch_1, leg_lft.pitch_4, &body.F_lft, &body.Tp_lft);
    Leg_InvTrq(dm_b_rgt.trq, dm_f_rgt.trq, leg_rgt.pitch_1, leg_rgt.pitch_4, &body.F_rgt, &body.Tp_rgt);
    body.Tp_rgt *= LEG_MIRROR_CHANGE;

    // 轮地相对角速度：电机反馈减掉摆杆摆动和机体俯仰 剩的才是相对地面的滚动
    body.whl_vel_lft = WHEEL_LFT_POLARITY * dji_wheel_lft.vel / REDUCTION_RATIO - leg_lft.d_ang - body.omg_pitch;
    body.whl_vel_rgt =  WHEEL_RGT_POLARITY * dji_wheel_rgt.vel / REDUCTION_RATIO - leg_rgt.d_ang - body.omg_pitch;

    // 机体髋部水平线速度 = 轮心线速度 + 摆杆角速度的水平分量 + 摆杆伸缩的水平分量
    body.vel_lft = body.whl_vel_lft * WHEEL_R + leg_lft.len * body.leg_omg_lft * arm_cos_f32(body.leg_ang_lft) +
                   leg_lft.d_len * arm_sin_f32(body.leg_ang_lft);
    body.vel_rgt = body.whl_vel_rgt * WHEEL_R + leg_rgt.len * body.leg_omg_rgt * arm_cos_f32(body.leg_ang_rgt) +
                   leg_rgt.d_len * arm_sin_f32(body.leg_ang_rgt);
    body.vel     = (body.vel_lft + body.vel_rgt) / 2.0f;

    body.leg_ang_lft_last = body.leg_ang_lft;
    body.leg_ang_rgt_last = body.leg_ang_rgt;
    body.leg_omg_lft_last = body.leg_omg_lft;
    body.leg_omg_rgt_last = body.leg_omg_rgt;
    body.ang_pitch_last   = body.ang_pitch;
    body.omg_pitch_last   = body.omg_pitch;
    body.ang_yaw_last     = body.ang_yaw;
}

void Body_Position_Update(void)
{
    const float leg_ang_limit = DEG_TO_RAD(4.5f);
#if BALANCE_USE_SLIP
    const float velocity = body.vel_kalman;
#else
    const float velocity = body.vel;
#endif

    if (isfinite(velocity) && fabsf(body.leg_ang_lft) < leg_ang_limit &&
        fabsf(body.leg_ang_rgt) < leg_ang_limit)
        body.pos += velocity * CTRL_DT;
}
