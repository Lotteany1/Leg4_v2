/*
 * File: Leg_SpringF.c
 *
 * 气弹簧被动沿腿力 [N]（伸腿为正） —— 由 MATLAB Symbolic Math Toolbox 生成，
 * 已替换为 CMSIS-DSP 数学库（arm_cos_f32 / arm_sin_f32 / arm_sqrt_f32）。
 *
 * 原始生成日期: 2026-11-22 05:14:18
 */

/* Include Files */
#include "Leg_SpringF.h"
#include "arm_math.h"
#include <math.h>

/* Function Definitions */
/*
 * 腿长 L0 [m]，返回被动沿腿力 [N]，伸腿为正。
 *  两销随连杆转动，摆角 ang [rad] 保留用于兼容调用。
 *  Fs = F_spring(s) * ds/dL，控制器取反后加到轴向力 F。
 */
float Leg_SpringF(float L0, float ang)
{
  float L;
  float s;
  float theta;
  (void)ang;
  L = fminf(fmaxf(L0, 0.05F), 0.45F);
  theta = acosf((0.1066F - L * L) / 0.105F);
  arm_sqrt_f32(
      fmaxf(0.041849196F - 0.0196417961F * arm_cos_f32(theta - 0.110147551F), 0.0F), &s);
  return  1000.0F * fminf(fmaxf((0.22F - s) / 0.0699999928F, 0.0F), 1.0F) *
         (0.00982089806F * arm_sin_f32(theta - 0.110147551F) * L /
          fmaxf(0.0525F * arm_sin_f32(theta) * s, 1.0E-6F));
}

/*
 * File trailer for Leg_SpringF.c
 *
 * [EOF]
 */
