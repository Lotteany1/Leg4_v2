/**
 * @file  lowpass.c
 * @brief 一阶低通滤波
 * @version  v1.0  2026-09-16
 */
#include "lowpass.h"

// 输出清零 首拍从 0 起步
void Lowpass_Init(lowpass1d_t *lp, float alpha)
{
    lp->out   = 0.0f;
    lp->alpha = alpha;
}

// 一阶低通递推 输出跟输入的快慢由 alpha 定
//   out_k = alpha*in_k + (1.0f - alpha)*out_{k-1}
//   alpha = 1 - exp(-2*pi*fc*dt)  dt 为实际采样周期 小 alpha 近似 alpha ≈ 2*pi*fc*dt
//   alpha 0.1 强滤 0.3 适中 0.5 弱滤
float Lowpass_Update(lowpass1d_t *lp, float in)
{
    lp->out = lp->alpha * in + (1.0f - lp->alpha) * lp->out;
    return lp->out;
}

// 只清输出 保留滤波系数
void Lowpass_Reset(lowpass1d_t *lp)
{
    lp->out = 0.0f;
}
