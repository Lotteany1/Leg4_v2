/**
 * @file  mahony.h
 * @brief 姿态解算头文件
 * @version  v1.0  2026-09-16
 */
#ifndef MAHONY_H
#define MAHONY_H

typedef struct
{
    float q[4];    /* 四元数 w x y z */
    float bias[3]; /* rad/s，陀螺零偏 x y z */
    float kp;      /* 比例增益 */
    float ki;      /* 积分增益 */
    float dt;      /* s，解算周期 */

    float roll;      /* rad，IMU 系横滚角 */
    float pitch;     /* rad，IMU 系俯仰角 */
    float yaw;       /* rad，IMU 系偏航角 */
    float omg_roll;  /* rad/s，IMU 系横滚角速度 */
    float omg_pitch; /* rad/s，IMU 系俯仰角速度 */
    float omg_yaw;   /* rad/s，IMU 系偏航角速度 */
} mahony_t;

void Mahony_Init(mahony_t *m, float kp, float ki, float dt); /* kp ki 增益 dt 解算周期 s */

void Mahony_Get_Body_Accel(mahony_t *m, float acc_x, float acc_y, float acc_z,
                           float *world_x, float *world_y, float *world_z);         /* 输入输出以 G 为单位 world_z 减 1G 去重力 */
void Mahony_Update_Quaternion(mahony_t *m, float q0, float q1, float q2, float q3); /* 外部直接写入四元数 */
#endif                                                                              /* MAHONY_H */
