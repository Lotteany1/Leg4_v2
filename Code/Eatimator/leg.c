// 腿部状态估计
// 版本：v1.3  日期：2026-10-26
#include "leg.h"
#include "Leg_Pos.h"
#include "Leg_Spd.h"
#include "dm_motor.h"
#include "polarity_config.h"
#include "code_config.h"
#include "system_key.h"
#include <math.h>

leg_state_t leg_lft, leg_rgt;

static float Ang_Unwrap(float cur, float last)
{
    float diff = cur - last;
    while (diff > PI)
        diff -= TWO_PI;
    while (diff < -PI)
        diff += TWO_PI;
    return last + diff;
}

void Leg_State_Update(void)
{
    float vel[2], pos[2];
    float raw_ang_lft, raw_ang_rgt;
    float len_lft, ang_lft, d_len_lft, d_ang_lft;
    float len_rgt, ang_rgt, d_len_rgt, d_ang_rgt;

    // 1. 读取原始电机位置 —— 右腿前后电机与左腿镜像 _b_rgt→pitch_1、_f_rgt→pitch_4
    leg_lft.pitch_1 = dm_f_lft.pos + PI + DM_LEG_LFT_F_OFFSET;
    leg_lft.pitch_4 = dm_b_lft.pos + DM_LEG_LFT_B_OFFSET;
    leg_rgt.pitch_1 = dm_b_rgt.pos + PI + DM_LEG_RGT_F_OFFSET;
    leg_rgt.pitch_4 = dm_f_rgt.pos + DM_LEG_RGT_B_OFFSET;

    leg_lft.d_pitch_1 = dm_f_lft.vel;
    leg_lft.d_pitch_4 = dm_b_lft.vel;
    leg_rgt.d_pitch_1 = dm_b_rgt.vel;
    leg_rgt.d_pitch_4 = dm_f_rgt.vel;

    Leg_Pos(leg_lft.pitch_1, leg_lft.pitch_4, pos);
    len_lft     = pos[0];
    raw_ang_lft = pos[1] - PI_2;
    ang_lft     = Ang_Unwrap(raw_ang_lft, leg_lft.ang_last);

    Leg_Spd(leg_lft.pitch_1, leg_lft.pitch_4, leg_lft.d_pitch_1, leg_lft.d_pitch_4, vel);
    d_len_lft = vel[0];
    d_ang_lft = vel[1];

    // 2. 右腿运动学解算 —— 坐标系镜像：角度取反，速度取反
    Leg_Pos(leg_rgt.pitch_1, leg_rgt.pitch_4, pos);
    len_rgt     = pos[0];
    raw_ang_rgt = -pos[1] + PI_2;
    ang_rgt     = Ang_Unwrap(raw_ang_rgt, leg_rgt.ang_last);

    Leg_Spd(leg_rgt.pitch_1, leg_rgt.pitch_4, leg_rgt.d_pitch_1, leg_rgt.d_pitch_4, vel);
    d_len_rgt = vel[0];
    d_ang_rgt = -vel[1];

    //! 任一输出坏就整拍不提交 派生量停在上拍值 防坏值经 body 渗进控制链
    if (!isfinite(len_lft) || !isfinite(ang_lft) || !isfinite(d_len_lft) || !isfinite(d_ang_lft) ||
        !isfinite(len_rgt) || !isfinite(ang_rgt) || !isfinite(d_len_rgt) || !isfinite(d_ang_rgt))
    {
        return;
    }

    leg_lft.len      = len_lft;
    leg_lft.ang      = ang_lft;
    leg_lft.ang_last = ang_lft;
    leg_lft.d_len    = d_len_lft;
    leg_lft.d_ang    = d_ang_lft;

    leg_rgt.len      = len_rgt;
    leg_rgt.ang      = ang_rgt;
    leg_rgt.ang_last = ang_rgt;
    leg_rgt.d_len    = d_len_rgt;
    leg_rgt.d_ang    = d_ang_rgt;
}