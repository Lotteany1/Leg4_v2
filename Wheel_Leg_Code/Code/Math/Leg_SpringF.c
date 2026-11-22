/*
 * File: Leg_SpringF.c
 *
 * 手感反馈试调轴向补偿表 [N]，左右共用，170～340 mm 采用18点线性插值 —— 由 MATLAB Coder 从 001 模型生成。
 *
 * 原始生成日期: 2026-11-22 10:57:33
 */

/* Include Files */
#include "Leg_SpringF.h"
#include <math.h>

/* MATLAB Coder nonfinite 兼容宏 */
#define rtIsNaNF(x)  isnan(x)
#define rtIsInfF(x)  isinf(x)
#define rtNaNF        NAN
#define RT_PIF        3.14159265F
#include <stdbool.h>
typedef unsigned char boolean_T;

/* Function Definitions */
/*
 * 手感反馈试调的轴向补偿表，左右共用，单位m、N，伸腿为正。
 *  短腿减小收腿补偿，中长腿增加补偿；170/340 mm采用已试过的75/185 N。
 *  ang 保留接口兼容，范围外保持端点值。
 */
float Leg_SpringF(float L0, float ang)
{
  static const float nodes[36] = {
      0.17F,  0.18F,  0.19F,  0.2F,   0.21F,  0.22F,  0.23F,  0.24F,  0.25F,
      0.26F,  0.27F,  0.28F,  0.29F,  0.3F,   0.31F,  0.32F,  0.33F,  0.34F,
      75.0F,  90.0F,  115.0F, 135.0F, 136.0F, 137.0F, 138.0F, 140.0F, 150.0F,
      152.0F, 154.0F, 156.0F, 158.0F, 164.0F, 168.0F, 175.0F, 180.0F, 185.0F};
  float Fs;
  (void)ang;
  Fs = 185.0F;
  if (L0 <= 0.17F) {
    Fs = 75.0F;
  } else {
    int k;
    boolean_T exitg1;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < 17)) {
      float f;
      f = nodes[k + 1];
      if (L0 <= f) {
        float Fs_tmp;
        Fs_tmp = nodes[k + 18];
        Fs = Fs_tmp +
             (L0 - nodes[k]) / (f - nodes[k]) * (nodes[k + 19] - Fs_tmp);
        exitg1 = true;
      } else {
        k++;
      }
    }
  }
  return Fs;
}

/*
 * File trailer for Leg_SpringF.c
 *
 * [EOF]
 */
