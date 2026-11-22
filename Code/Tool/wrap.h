/** @file  wrap.h  @brief 角度折回工具  @version  v1.1  2026-10-26 */
#ifndef TOOL_WRAP_H
#define TOOL_WRAP_H

#include "code_config.h"
#include <math.h>

// fmodf 负数得负余数，余数 <= -π 必须补一整圈，漏补则折回值落在 (-π, π] 之外
static inline float Wrap_Pi(float ang)
{
    float w = fmodf(ang, TWO_PI);
    if (w > PI)
    {
        w -= TWO_PI;
    }
    else if (w <= -PI)
    {
        w += TWO_PI;
    }
    return w;
}

#endif /* TOOL_WRAP_H */
