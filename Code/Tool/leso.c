/**
 * @file  leso.c
 * @brief 预测注入型扩张状态观测器
 * @version  v0.2  2026-10-15
 */
#include "leso.h"
#include <math.h>
#include <stddef.h>

// 箱约束投影 坏值靠入口闸拦
static float Leso_Clip(float x, float lo, float hi)
{
    if (x > hi) return hi;
    if (x < lo) return lo;
    return x;
}

// 上电一次 极点与限幅登记 估计清零 首拍对齐挂起
void Leso_Init(leso_t *leso, float pole, float b_max)
{
    if (leso == NULL || !isfinite(pole) || !isfinite(b_max)) return;
    if (pole <= 0.0f || pole >= 1.0f || b_max <= 0.0f) return;

    leso->pole  = pole;
    leso->b_max = b_max;
    leso->b     = 0.0f;
    leso->l     = 0.0f;
    leso->pred  = 0.0f;
    leso->hold  = 1u;
}

// 模式交接与冻结结束 估计保持 首拍只对齐不修正
void Leso_Reset(leso_t *leso)
{
    if (leso == NULL) return;

    leso->hold = 1u;
}

// 一拍观测 增益随扰动系数现算 上拍预测与本拍实测的偏差修正估计 本拍预测存档供下拍
void Leso_Update(leso_t *leso, float ed, float pred, float y)
{
    if (leso == NULL || leso->pole <= 0.0f || leso->pole >= 1.0f) return;
    if (!isfinite(ed) || !isfinite(pred) || !isfinite(y) || ed == 0.0f) return;

    leso->l = (1.0f - leso->pole) / ed;
    if (leso->hold)
    {
        leso->hold = 0u;
    }
    else
    {
        leso->b = Leso_Clip(leso->b + leso->l * (y - leso->pred), -leso->b_max, leso->b_max);
    }
    leso->pred = pred + ed * leso->b;
}

// 只读遥测 扰动估计
float Leso_B(const leso_t *leso)
{
    if (leso == NULL) return 0.0f;
    return leso->b;
}
