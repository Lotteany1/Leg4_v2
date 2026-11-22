/**
 * @file  mahony.c
 * @brief 姿态解算
 * @version  v1.0  2026-09-16
 */
#include "mahony.h"

// 姿态四元数初始化 清积分反馈
void Mahony_Init(mahony_t *m, float kp, float ki, float dt)
{
    m->q[0] = 1.0f;
    m->q[1] = 0.0f;
    m->q[2] = 0.0f;
    m->q[3] = 0.0f;

    m->bias[0] = 0.0f;
    m->bias[1] = 0.0f;
    m->bias[2] = 0.0f;

    m->kp = kp;
    m->ki = ki;
    m->dt = dt;

    m->roll      = 0.0f;
    m->pitch     = 0.0f;
    m->yaw       = 0.0f;
    m->omg_roll  = 0.0f;
    m->omg_pitch = 0.0f;
    m->omg_yaw   = 0.0f;
}

// 机体系加速度转世界系 q=[w,x,y,z] 输出以 G 为单位 world_z 减 1G 去重力
// R body→world
//   R = [ 1-2(q2^2+q3^2)   2(q1q2-q0q3)    2(q1q3+q0q2)  ]
//       [ 2(q1q2+q0q3)     1-2(q1^2+q3^2)  2(q2q3-q0q1)  ]
//       [ 2(q1q3-q0q2)     2(q2q3+q0q1)    1-2(q1^2+q2^2)]
void Mahony_Get_Body_Accel(mahony_t *m, float acc_x, float acc_y, float acc_z,
                           float *world_x, float *world_y, float *world_z)
{
    float q0 = m->q[0], q1 = m->q[1], q2 = m->q[2], q3 = m->q[3];

    float r00 = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    float r01 = 2.0f * (q1 * q2 - q0 * q3);
    float r02 = 2.0f * (q1 * q3 + q0 * q2);
    float r10 = 2.0f * (q1 * q2 + q0 * q3);
    float r11 = 1.0f - 2.0f * (q1 * q1 + q3 * q3);
    float r12 = 2.0f * (q2 * q3 - q0 * q1);
    float r20 = 2.0f * (q1 * q3 - q0 * q2);
    float r21 = 2.0f * (q2 * q3 + q0 * q1);
    float r22 = 1.0f - 2.0f * (q1 * q1 + q2 * q2);

    *world_x = r00 * acc_x + r01 * acc_y + r02 * acc_z;
    *world_y = r10 * acc_x + r11 * acc_y + r12 * acc_z;
    *world_z = r20 * acc_x + r21 * acc_y + r22 * acc_z - 1.0f;
}

// 角速度积分与重力方向误差反馈 更新归一化四元数
void Mahony_Update_Quaternion(mahony_t *m, float q0, float q1, float q2, float q3)
{
    m->q[0] = q0;
    m->q[1] = q1;
    m->q[2] = q2;
    m->q[3] = q3;
}
