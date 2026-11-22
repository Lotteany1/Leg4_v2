/**
 * @file  kalman.c
 * @brief 二维加速度偏置卡尔曼速度估计
 * @version  v1.0  2026-09-16
 */
#include "kalman.h"

#include <math.h>

/* 过程模型 状态 x = [v, b]
 * x⁻ = F*x + B*a_body      F = [1 -dt; 0 1]   B = [dt; 0]
 * P⁻ = F*P*Fᵀ + Q          Q = diag(q_a*dt², q_b*dt)
 * 更新 H = [1 0]  y = z - H*x⁻  S = H*P⁻*Hᵀ + r
 * |y| > gate*√S 按 (|y|/(gate*√S))² 放大 r 后更新 降权非丢弃
 * K = P⁻*Hᵀ/S  x = x⁻ + K*y  P = (I - K*H)*P⁻ */

// 速度与加速度偏置 KF 初始化 登记协方差和噪声
void Kalman_Accel_Init(kalman_accel_t *kf, float v0, float p0, float q_a, float q_b, float r)
{
    kf->vel    = v0;
    kf->a_bias = 0.0f;
    kf->p00    = p0;
    kf->p01    = 0.0f;
    kf->p11    = 0.0f;
    kf->p0     = p0;
    kf->q_a    = q_a;
    kf->q_b    = q_b;
    kf->r      = r;
    kf->v_max  = 0.0f;
    kf->gate   = 3.0f;
}

// 非有限加速度与非法 dt 直接早退 速度积分后钳位
void Kalman_Accel_Predict(kalman_accel_t *kf, float a_body, float dt)
{
    if (!isfinite(a_body) || !isfinite(dt) || dt <= 0.0f) return;

    // 1. 速度积分与钳位
    kf->vel += (a_body - kf->a_bias) * dt;
    if (kf->v_max > 0.0f)
    {
        if (kf->vel > kf->v_max) kf->vel = kf->v_max;
        if (kf->vel < -kf->v_max) kf->vel = -kf->v_max;
    }

    // 2. 协方差传播
    float p00 = kf->p00 - 2.0f * dt * kf->p01 + dt * dt * kf->p11 + kf->q_a * dt * dt;
    float p01 = kf->p01 - dt * kf->p11;
    float p11 = kf->p11 + kf->q_b * dt;

    kf->p00 = p00;
    kf->p01 = p01;
    kf->p11 = p11;
}

// 运动学速度观测融合 野值只降权不丢弃 门限与除零两处失效均返 false
bool Kalman_Accel_Update(kalman_accel_t *kf, float z_vel)
{
    if (!isfinite(z_vel)) return false;

    // 1. 新息方差 非正即协方差失效
    float s = kf->p00 + kf->r;
    if (!(s > 0.0f)) return false;

    // 2. 新息
    float y      = z_vel - kf->vel;
    bool nominal = true;

    // 3. 野值降权 放大 r 保留修正方向
    if (kf->gate > 0.0f)
    {
        float lim = kf->gate * sqrtf(s);
        float ay  = fabsf(y);
        if (ay > lim)
        {
            s       = kf->p00 + kf->r * ((ay * ay) / (lim * lim));
            nominal = false;
        }
        if (!(s > 0.0f)) return false;
    }

    // 4. 增益与协方差更新
    float k0 = kf->p00 / s;
    float k1 = kf->p01 / s;

    kf->vel += k0 * y;
    kf->a_bias += k1 * y;

    float p01 = kf->p01;
    kf->p00   = (1.0f - k0) * kf->p00;
    kf->p01   = (1.0f - k0) * p01;
    kf->p11   = kf->p11 - k1 * p01;

    return nominal;
}

// 速度重定基 偏置与协方差归零
void Kalman_Accel_Reset(kalman_accel_t *kf, float v0)
{
    kf->vel    = v0;
    kf->a_bias = 0.0f;
    kf->p00    = kf->p0;
    kf->p01    = 0.0f;
    kf->p11    = 0.0f;
}
