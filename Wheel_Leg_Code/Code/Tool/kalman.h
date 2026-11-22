/**
 * @file  kalman.h
 * @brief 二维加速度偏置卡尔曼速度估计头文件
 * @version  v1.0  2026-09-16
 */
#ifndef KALMAN_H
#define KALMAN_H

#include <stdbool.h>

/* 状态 x = [v, b]
 * v_k = v_{k-1} + (a_body - a_bias)*dt + w_v   IMU 加速度积分
 * b_k = b_{k-1} + w_b                          偏置随机游走
 * q_a  (m/s²)²/Hz，加速度白噪声谱密度 调大信运动学测量 调小信加速度积分
 * q_b  偏置随机游走强度 决定偏置估计带宽 温漂大的 IMU 调大
 * r    (m/s)²，运动学速度测量噪声 打滑与倾角误差越大设越大
 * p0   初始速度不确定度 设 0 表示无条件信任初值 v0
 * v_max  m/s，速度钳位 0 = 关闭
 * gate   σ，新息门限 0 = 关闭 常用 3.0 超限按 (|y|/lim)² 放大 r 降权不丢弃 */
typedef struct
{
    float vel, a_bias;   /* m/s / m/s²，融合速度与加速度偏置 */
    float p00, p01, p11; /* 协方差 P 的速度 速度偏置 偏置 三项 */
    float p0;            /* 初始速度不确定度 Reset 复用 */
    float q_a, q_b;      /* 过程噪声 加速度谱密度与偏置游走强度 */
    float r;             /* (m/s)²，速度观测噪声方差 */
    float v_max;         /* m/s，速度钳位 0 = 关闭 */
    float gate;          /* σ，新息门限 0 = 关闭 常用 3.0 */
} kalman_accel_t;

// 偏置状态初值 0 其不确定度由 q_b 在运行中逐步张开
void Kalman_Accel_Init(kalman_accel_t *kf, float v0, float p0, float q_a, float q_b, float r);

void Kalman_Accel_Predict(kalman_accel_t *kf, float a_body, float dt);

// 返回 false 非有限值或协方差无效 以及新息降权融合
bool Kalman_Accel_Update(kalman_accel_t *kf, float z_vel);

void Kalman_Accel_Reset(kalman_accel_t *kf, float v0);

#endif /* KALMAN_H */
