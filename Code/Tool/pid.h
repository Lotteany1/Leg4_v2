/**
 * @file  pid.h
 * @brief PID 控制器头文件
 * @version  v1.0  2026-09-16
 */
#ifndef PID_H
#define PID_H

#include <stdlib.h>
#include <stdint.h>

typedef struct
{
    float kp, ki, kd;             /* 比例 积分 微分增益 */
    float meas, tar;              /* 测量值 目标值 */
    float err, err_sum, err_last; /* 本拍误差 误差累加 上拍误差 */
    float p_out, i_out, d_out;    /* 比例 积分 微分分项出力 */
    float out;                    /* 限幅后总出力 */
    float out_max, out_min;       /* 出力上下限 */
} pid_t;

void Pid_Init(pid_t *pid, float kp, float ki, float kd, float out_max, float out_min); /* 增益与限幅一次写入 */
void Pid_Update(pid_t *pid, float tar, float meas);                                    /* 每拍推进 非有限值早退 */

#endif /* PID_H */
