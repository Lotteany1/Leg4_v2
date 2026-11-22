/**
 * @file  lowpass.h
 * @brief 一阶低通滤波头文件
 * @version  v1.0  2026-09-16
 */
#ifndef LOWPASS_H
#define LOWPASS_H

typedef struct
{
    float out;   /* 滤波输出 上一拍结果 */
    float alpha; /* 滤波系数 0.1 强滤 0.3 适中 0.5 弱滤 */
} lowpass1d_t;

void Lowpass_Init(lowpass1d_t *lp, float alpha);
float Lowpass_Update(lowpass1d_t *lp, float in);
void Lowpass_Reset(lowpass1d_t *lp);

#endif /* LOWPASS_H */
