/*
 * File: Leg_SpringF.c
 *
 * 当前实车经验轴向补偿表 [N]，伸腿为正，左右共用 —— 由 MATLAB Symbolic Math Toolbox 生成。
 *
 * 原始生成日期: 2026-11-23 22:43:45
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
 * 实车试调：按原辨识的腿向补偿力缩放，左右共用，伸腿为正 [N]。
 *  本轮在 130–190 mm 增加 2 N，120–130 和 190–200 mm 线性过渡。
 */
float Leg_SpringF(float L0, float ang)
{
  static const float nodes[40] = {
      0.12F,   0.13F,   0.17F,   0.18F,   0.19F,   0.2F,    0.21F,   0.22F,
      0.23F,   0.24F,   0.25F,   0.26F,   0.27F,   0.28F,   0.29F,   0.3F,
      0.31F,   0.32F,   0.33F,   0.34F,   65.3F,   67.3F,   67.3F,   77.55F,
      95.83F,  110.89F, 112.49F, 112.65F, 112.76F, 113.61F, 120.34F, 122.21F,
      124.11F, 126.03F, 128.89F, 135.06F, 139.68F, 146.86F, 152.47F, 158.15F};
  float Fs;
  (void)ang;
  Fs = 158.15F;
  if (L0 <= 0.12F) {
    Fs = 65.3F;
  } else {
    int k;
    boolean_T exitg1;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < 19)) {
      float f;
      f = nodes[k + 1];
      if (L0 <= f) {
        float Fs_tmp;
        Fs_tmp = nodes[k + 20];
        Fs = Fs_tmp +
             (L0 - nodes[k]) / (f - nodes[k]) * (nodes[k + 21] - Fs_tmp);
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
