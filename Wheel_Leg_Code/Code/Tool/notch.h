/** @file  notch.h  @brief 二阶对称陷波滤波器接口  @version  v1.0  2026-10-29 */
#ifndef NOTCH_H
#define NOTCH_H

#include <stdint.h>

// 对称二阶陷波系数 b2=b0、a1=b1，采样周期与设计时一致
typedef struct
{
    float b0; // 输入及两拍前输入系数
    float b1; // 上拍输入和上拍输出系数
    float a2; // 两拍前输出系数
} notch_coeff_t;

// 每路信号独立维护历史，首次更新以当前输入初始化
typedef struct
{
    float x1, x2; // 上一拍和上两拍输入
    float y1, y2; // 上一拍和上两拍输出
    uint8_t init; // 0 首拍需初始化历史
} notch_state_t;

// 每个设计采样周期调用一次，更新 state 并返回滤波结果
float Notch_Update(notch_state_t *state, const notch_coeff_t *coeff, float input);
// 清除初始化标志，下一拍以当时输入重新初始化历史
void Notch_Reset(notch_state_t *state);

#endif /* NOTCH_H */
