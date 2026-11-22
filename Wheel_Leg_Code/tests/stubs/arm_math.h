/** @file  arm_math.h  @brief Host math adapters  @version  v1.0  2026-10-26 */
#ifndef TEST_ARM_MATH_H
#define TEST_ARM_MATH_H
#include <math.h>
static inline float arm_cos_f32(float x)
{ return cosf(x); }
static inline float arm_sin_f32(float x)
{ return sinf(x); }
static inline int arm_sqrt_f32(float x, float *out)
{
    *out = sqrtf(x);
    return 0;
}
#endif
