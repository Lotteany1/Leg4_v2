/**
 * @file  imu_to_world.h
 * @brief IMU 到世界坐标系转换头文件：世界系姿态 + 腿世界系摆角换算
 * @version  v1.4  2026-10-10
 */
#ifndef IMU_TO_WORLD_H
#define IMU_TO_WORLD_H
#include "code_config.h"

// 姿态估计
#define IMU_G_DEAD        0.1f                       // 重力方向死区 (g)
#define IMU_G_CONFIRM     (20 / (int)CTRL_PERIOD_MS) // 20 ms 重力方向确认
#define IMU_ACC_LPF_ALPHA 0.64f                      // 世界系加速度低通 适中偏弱 滤电机振动

//! pitch 为多圈累加值 用前必须折回 (-π,π] acc 三路为世界系去重力加速度
typedef struct
{
    /* ---- 世界系姿态 (rad) ---- */
    float pitch, roll, yaw;

    /* ---- 世界系加速度 (g) ---- */
    float acc_x, acc_y, acc_z;
} imu_world_t;

extern imu_world_t imu_world;

void Imu_To_World(void);

// 输入腿电机系摆角 返回腿世界系摆角 (rad) 已折回 (-π, π]
float Imu_To_World_Leg_Ang(float leg_ang);

#endif /* IMU_TO_WORLD_H */
