/**
 * @file  body.h
 * @brief 机体状态估计头文件：命令目标量 + 姿态 / 腿 / 加速度观测量
 * @version  v1.14  2026-11-24
 */
#ifndef BODY_H
#define BODY_H

#include "system_key.h"
#include <stdbool.h>
#include <stdint.h>

#define BODY_EULER_COS_MIN            0.02f   // 接地欧拉速率转换的 cos(pitch) 下限
#define BODY_HEIGHT_LP_ALPHA          0.1f    // 1 ms 估计拍，机体高度低通系数

#define BODY_ROLL_KF_R                0.25f // (rad/s)²，静止段陀螺观测噪声方差
#define BODY_ROLL_KF_Q_W              15.0f  // (rad/s²)²，乘 IMU 帧间隔 dt² 注入角速度方差
#define BODY_ROLL_KF_Q_A              2.25f   // (rad/s²)²/s，角加速度随机游走强度

#define BODY_PITCH_KF_R               1.0e-3f  // (rad/s)²，俯仰角速度观测噪声方差
#define BODY_PITCH_KF_Q_W             15.0f  // (rad/s²)²，乘 IMU 帧间隔 dt² 注入角速度方差
#define BODY_PITCH_KF_Q_A             2.25f  // (rad/s²)²/s，俯仰角加速度随机游走强度

#define BODY_HEIGHT_KF_R              1.0e-4f // (m/s)²，升降速度观测噪声方差
#define BODY_HEIGHT_KF_Q_W            18.75f  // (m/s²)²，乘估计周期 dt² 注入速度方差
#define BODY_HEIGHT_KF_Q_A            1.25f   // (m/s²)²/s，升降加速度随机游走强度

typedef struct
{
    /* ---- 命令 ---- */
    float vel_tgt;                            /* 期望两髋平均水平速度 (m/s) */
    float ang_yaw_tgt, omg_yaw_tgt;           /* 期望偏航角/角速度 (rad, rad/s) */
    float omg_pitch_tgt;                      /* 期望俯仰角速度 (rad/s) */
    float leg_ang_tgt;                        /* 期望摆杆角 (rad) */
    float leg_len_tgt_lft, leg_len_tgt_rgt;   /* 期望腿长 (m) 自救与单侧动作使用 */
    float h_tgt;                              /* 期望机体高度 (m) TinyMPC14 高度参考 */
    float leg_ang_tgt_lft, leg_ang_tgt_rgt;   /* 期望摆杆角 电机系 (rad)，Recovery 用 */
    float leg_omg_tgt_lft, leg_omg_tgt_rgt;   /* 期望摆杆角速度 世界系 (rad/s)，自救摆腿段用 */

    /* ---- 姿态 ---- */
    float ang_yaw,   omg_yaw, omg_yaw_filt;  /* 偏航角 (rad)、体轴 r 及其直通值 (rad/s) */
    float ang_pitch, omg_pitch, omg_pitch_filt; /* 俯仰角 (rad)、体轴 q 及滤后 ZYX 俯仰导数 (rad/s) */
    float omg_pitch_kalman;                  /* 卡尔曼滤波后的 ZYX 俯仰导数 (rad/s) */
    float ang_roll,  omg_roll, omg_roll_filt; /* 横滚角 [rad]、原始体轴 p、滤后 ZYX 横滚导数 [rad/s] */
    float roll_rate_obs;                     /* ZYX 横滚导数观测 (rad/s) */
    float omg_roll_kalman;                   /* 卡尔曼滤波后的 ZYX 横滚导数 (rad/s) */
    float whl_vel_lft, whl_vel_rgt;           /* 轮地相对角速度 (rad/s) */
    float vel, vel_lft, vel_rgt;              /* 两髋平均、左髋、右髋水平速度 (m/s) */
    float vel_kalman;                         /* 融合速度 (m/s) slip.c 写入 */
    float pos;                                /* 两髋平均水平累计里程 (m) */

    /* ---- 腿：长度 / 摆杆 / 逆解力 ---- */
    float leg_len_lft, leg_len_rgt, leg_len;  /* m */
    float h_body, h_body_filt;                             /* m 两髋竖直投影均值 h_body 供腿长控制 h_body_filt 供遥测 目标见 h_tgt */
    float height_rate_obs;                  /* 几何升降速度观测 (m/s) */
    float omg_h_kalman;                     /* 卡尔曼滤波后的机体升降速度 (m/s) */
    float leg_ang_lft, leg_ang_rgt;           /* 世界系摆杆角 = 关节角 + 俯仰 (rad) */
    float leg_omg_lft, leg_omg_rgt;           /* rad/s */
    float F_lft,  F_rgt;                      /* 腿端支持力 (N) */
    float Tp_lft, Tp_rgt;                     /* 腿端髋扭矩 (N·m) */

    /* ---- 机体系 原始加速度含重力，直拷 imu.acc ---- */
    float body_accel_x, body_accel_y, body_accel_z; /* g */

    /* ---- 上周期快照 ---- */
    float ang_pitch_last, omg_pitch_last;
    float ang_yaw_last;
    float leg_ang_lft_last, leg_ang_rgt_last;
    float leg_omg_lft_last, leg_omg_rgt_last;
} body_state_t;

extern body_state_t body;

void Body_State_Init(void);
void Body_State_Update(void);
void Body_Position_Update(void); // 速度融合后每拍调用，双腿世界系倾角均在 ±5° 内才积分

#endif /* BODY_H */
