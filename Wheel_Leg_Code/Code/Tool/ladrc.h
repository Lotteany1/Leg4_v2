/**
 * @file  ladrc.h
 * @brief 线性自抗扰控制器头文件
 * @version  v1.1  2026-10-06
 */
#ifndef LADRC_H
#define LADRC_H

#include <stdint.h>

// 被控对象 y' = f + b0*u TD 可选 二阶 LESO P 控制 扰动补偿
typedef struct
{
    float tar;  /* 给定 */
    float meas; /* 实测 */

    float b0; /* 控制增益，b0 ≈ 1/J */
    float wc; /* rad/s，控制带宽 从小往大加 */
    float wo; /* rad/s，观测带宽 取 3~10×wc 常用 5×wc */
    float r;  /* TD 速度因子，0 即不用 TD 直通 */
    float h;  /* s，积分步长 */
    float h0; /* s，TD 滤波因子 ≥h */

    float v1; /* TD 跟踪值 */

    float z1; /* z1 输出观测 */
    float z2; /* z2 总扰动观测 */

    float beta1; /* 2×wo，LESO 增益 */
    float beta2; /* wo²，LESO 增益 */

    float u0;  /* wc×(v1-z1)，P 控制律 */
    float out; /* (u0-z2)/b0，扰动补偿后出力 */

    float out_max; /* 出力上限 */
    float out_min; /* 出力下限 */
} ladrc_t;

void Ladrc_Init(ladrc_t *ladrc, float b0, float wc, float wo,
                float r, float h, float h0, float out_max, float out_min);
void Ladrc_Update(ladrc_t *ladrc);
void Ladrc_Reset(ladrc_t *ladrc);

// 被控对象 y'' = f + b0*u 三阶 LESO PD 控制 扰动补偿 z3
typedef struct
{
    float tar;  /* 给定 */
    float meas; /* 实测 */

    float b0; /* 控制增益，b0 ≈ 1/m_eff */
    float wc; /* rad/s，控制带宽 从小往大加 */
    float wo; /* rad/s，观测带宽 wo = 3~5×wc */
    float kp; /* wc²，比例增益 */
    float kd; /* 2×wc，微分增益 */

    float z1; /* z1 位置观测 */
    float z2; /* z2 速度观测 */
    float z3; /* z3 总扰动观测 */

    float b1; /* 3×wo，LESO 增益 */
    float b2; /* 3×wo²，LESO 增益 */
    float b3; /* wo³，LESO 增益 */

    float u0;  /* kp×(tar-z1)-kd×z2，PD 控制律 */
    float out; /* (u0-z3)/b0，扰动补偿后出力 */

    float out_max; /* 出力上限 */
    float out_min; /* 出力下限 */
} ladrc2_t;

void Ladrc2_Init(ladrc2_t *ladrc, float b0, float wc, float wo, float out_max, float out_min);

// tar 与 meas 走行参 与 Pid_Update 相同调用形式
void Ladrc2_Update(ladrc2_t *ladrc, float tar, float meas); /* tar/meas 走行参 与 Pid_Update 同形 */
void Ladrc2_Reset(ladrc2_t *ladrc);

#endif /* LADRC_H */
