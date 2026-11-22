/**
 * @file  ramp.c
 * @brief 斜坡限速
 * @version  v1.0  2026-09-16
 */
#include "ramp.h"
#include <math.h>

// 对称变化率斜坡初始化
void Ramp_Init(ramp_t *ramp, float rate)
{
    ramp->rate_up   = fabsf(rate);
    ramp->rate_down = fabsf(rate);
    ramp->current   = 0.0f;
}

// 上升下降分别登记变化率
void Ramp_Init_Asym(ramp_t *ramp, float rate_up, float rate_down)
{
    ramp->rate_up   = fabsf(rate_up);
    ramp->rate_down = fabsf(rate_down);
    ramp->current   = 0.0f;
}

// 斜坡输出直接重置为给定值
void Ramp_Reset(ramp_t *ramp, float current)
{
    ramp->current = current;
}

// 每拍按变化率逼近目标 步长够大就贴目标 dt 非正直接返回
float Ramp_Update(ramp_t *ramp, float target, float dt)
{
    if (dt <= 0.0f)
    {
        return ramp->current;
    }

    float err  = target - ramp->current;
    float step = ((err > 0.0f) ? ramp->rate_up : ramp->rate_down) * dt;

    if (fabsf(err) <= step)
    {
        ramp->current = target;
    }
    else
    {
        ramp->current += (err > 0.0f) ? step : -step;
    }

    return ramp->current;
}
