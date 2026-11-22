/** @file  ladrc.c  @brief 线性自抗扰控制器  @version  v3.0  2026-10-26 */
#include "ladrc.h"
#include "code_config.h"
#include <math.h>

// 符号函数 原点返回零
static inline float Ladrc_Fsign(float x)
{
    return (x > 0.0f) ? 1.0f : ((x < 0.0f) ? -1.0f : 0.0f);
}

// TD 最速综合函数 线性区外按 r 饱和
static float Ladrc_Fst(float x, float r, float h)
{
    float rh = r * h;
    if (fabsf(x) > rh)
    {
        return -r * Ladrc_Fsign(x);
    }
    else
    {
        return -x / h;
    }
}

// 一阶对象 LADRC 初始化 二阶 ESO 与目标跟踪状态清零
void Ladrc_Init(ladrc_t *ladrc, float b0, float wc, float wo,
                float r, float h, float h0, float out_max, float out_min)
{
    *ladrc = (ladrc_t){
        .tar  = 0.0f,
        .meas = 0.0f,

        .b0 = b0,
        .wc = wc,
        .wo = wo,
        .r  = r,
        .h  = h,
        .h0 = h0,

        .v1 = 0.0f,

        .z1 = 0.0f,
        .z2 = 0.0f,

        .beta1 = 2.0f * wo,
        .beta2 = wo * wo,

        .u0  = 0.0f,
        .out = 0.0f,

        .out_max = out_max,
        .out_min = out_min,
    };
}

// 目标跟踪后推进 ESO 扰动补偿控制量并限幅
void Ladrc_Update(ladrc_t *ladrc)
{
    float h  = ladrc->h;
    float b0 = ladrc->b0;

    if (ladrc->r > 0.0f)
    {
        float fh = Ladrc_Fst(ladrc->v1 - ladrc->tar, ladrc->r, ladrc->h0);
        ladrc->v1 += h * fh;
    }
    else
    {
        ladrc->v1 = ladrc->tar;
    }

    float e = ladrc->z1 - ladrc->meas;

    ladrc->z1 += h * (ladrc->z2 - ladrc->beta1 * e + b0 * ladrc->out);
    ladrc->z2 += h * (-ladrc->beta2 * e);

    ladrc->u0 = ladrc->wc * (ladrc->v1 - ladrc->z1);

    // out = (u0 - z2) / b0  z2 被减掉 → 扰动被主动补偿
    ladrc->out = (ladrc->u0 - ladrc->z2) / b0;

    if (ladrc->out > ladrc->out_max)
    {
        ladrc->out = ladrc->out_max;
    }
    else if (ladrc->out < ladrc->out_min)
    {
        ladrc->out = ladrc->out_min;
    }
}

// 清目标跟踪与观测状态 保留控制参数
void Ladrc_Reset(ladrc_t *ladrc)
{
    ladrc->tar  = 0.0f;
    ladrc->meas = 0.0f;
    ladrc->v1   = 0.0f;
    ladrc->z1   = 0.0f;
    ladrc->z2   = 0.0f;
    ladrc->u0   = 0.0f;
    ladrc->out  = 0.0f;
}

// 二阶对象 LADRC 初始化 三阶 ESO 与反馈参数登记
void Ladrc2_Init(ladrc2_t *ladrc, float b0, float wc, float wo, float out_max, float out_min)
{
    *ladrc = (ladrc2_t){
        .tar  = 0.0f,
        .meas = 0.0f,

        .b0 = b0,
        .wc = wc,
        .wo = wo,
        .kp = wc * wc,
        .kd = 2.0f * wc,

        .z1 = 0.0f,
        .z2 = 0.0f,
        .z3 = 0.0f,

        .b1 = 3.0f * wo,
        .b2 = 3.0f * wo * wo,
        .b3 = wo * wo * wo,

        .u0  = 0.0f,
        .out = 0.0f,

        .out_max = out_max,
        .out_min = out_min,
    };
}

// 三阶 ESO 估计总扰动 位置速度反馈合成控制量
void Ladrc2_Update(ladrc2_t *ladrc, float tar, float meas)
{

    if (!isfinite(tar) || !isfinite(meas))
    {
        return;
    }

    ladrc->tar  = tar;
    ladrc->meas = meas;

    float e = ladrc->z1 - ladrc->meas;

    ladrc->z1 += CTRL_DT * (ladrc->z2 - ladrc->b1 * e);
    ladrc->z2 += CTRL_DT * (ladrc->z3 - ladrc->b2 * e + ladrc->b0 * ladrc->out);
    ladrc->z3 += CTRL_DT * (-ladrc->b3 * e);

    // z2 是观测出来的速度 不微实测值 阶跃给定不产生微分冲击
    ladrc->u0 = ladrc->kp * (ladrc->tar - ladrc->z1) - ladrc->kd * ladrc->z2;

    // out = (u0 - z3) / b0  z3 被减掉 → 总扰动被主动补偿
    ladrc->out = (ladrc->u0 - ladrc->z3) / ladrc->b0;

    if (ladrc->out > ladrc->out_max)
    {
        ladrc->out = ladrc->out_max;
    }
    else if (ladrc->out < ladrc->out_min)
    {
        ladrc->out = ladrc->out_min;
    }
}

// 二阶对象控制状态清零 参数保持
void Ladrc2_Reset(ladrc2_t *ladrc)
{
    ladrc->tar  = 0.0f;
    ladrc->meas = 0.0f;
    ladrc->z1   = 0.0f;
    ladrc->z2   = 0.0f;
    ladrc->z3   = 0.0f;
    ladrc->u0   = 0.0f;
    ladrc->out  = 0.0f;
}
