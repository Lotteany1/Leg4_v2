/*
 * arm_math.h —— 仅供 verify/ 在 PC 上对拍用的 CMSIS-DSP 影子实现。
 * 固件用的是 Drivers/CMSIS/DSP 里的真身；这里只按同样的行为接回 libm，
 * 好让同一份 cmsis/*.c 能在 PC 上编译并与 .m 对拍。
 * arm_sqrt_f32 对负数返回 0 的行为与 CMSIS-DSP 一致。
 */

#ifndef ARM_MATH_SHIM_H
#define ARM_MATH_SHIM_H

#include <math.h>

static float arm_cos_f32(float x)
{
    return cosf(x);
}

static float arm_sin_f32(float x)
{
    return sinf(x);
}

static int arm_sqrt_f32(float in, float *pOut)
{
    if (in >= 0.0f) {
        *pOut = sqrtf(in);
        return 0;
    }
    *pOut = 0.0f;
    return 1;
}

#endif /* ARM_MATH_SHIM_H */
