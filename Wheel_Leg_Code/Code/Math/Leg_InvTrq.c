/*
 * File: Leg_InvTrq.c
 *
 * VMC 逆映射：关节力矩 → 末端力/虚拟力矩 —— 由 MATLAB Coder 从 001 模型生成，
 * 已替换为 CMSIS-DSP 数学库（arm_cos_f32 / arm_sin_f32 / arm_sqrt_f32）。
 *
 * 原始生成日期: 2026-11-22 10:57:27
 */

/* Include Files */
#include "Leg_InvTrq.h"
#include "arm_math.h"
#include <math.h>

/* MATLAB Coder nonfinite 兼容宏 */
#define rtIsNaNF(x)  isnan(x)
#define rtIsInfF(x)  isinf(x)
#define rtNaNF        NAN
#define RT_PIF        3.14159265F

/* Function Declarations */
static float rt_atan2f_snf(float u0, float u1);

/* Function Definitions */
static float rt_atan2f_snf(float u0, float u1)
{
  float y;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = rtNaNF;
  } else if (rtIsInfF(u0) && rtIsInfF(u1)) {
    int i;
    int i1;
    if (u0 > 0.0F) {
      i = 1;
    } else {
      i = -1;
    }
    if (u1 > 0.0F) {
      i1 = 1;
    } else {
      i1 = -1;
    }
    y = atan2f((float)i, (float)i1);
  } else if (u1 == 0.0F) {
    if (u0 > 0.0F) {
      y = RT_PIF / 2.0F;
    } else if (u0 < 0.0F) {
      y = -(RT_PIF / 2.0F);
    } else {
      y = 0.0F;
    }
  } else {
    y = atan2f(u0, u1);
  }
  return y;
}

/*
 * 同一 001 闭环雅可比的正则逆，lambda=1e-6 保持既有近奇异抑制口径。
 */
void Leg_InvTrq(float tau1, float tau4, float phi1, float phi4, float *F,
                float *Tp)
{
  float JB[4];
  float JD[4];
  float y[2];
  float B_idx_0;
  float B_idx_1;
  float D_idx_0;
  float D_idx_1;
  float E_idx_0;
  float E_idx_1;
  float JP_idx_1;
  float P_idx_0;
  float P_idx_1;
  float a_idx_0;
  float a_idx_1;
  float b_a_idx_0;
  float b_y;
  float c;
  float d;
  float delta_idx_0;
  float delta_idx_1;
  float f;
  float pin_idx_0;
  float s;
  int k;
  /*  001 双闭环：输入保留安装标定的两个电机角坐标，输出 [腿长;极角]。 */
  /*  后电机仅在 CAD 零位与虚拟曲柄重合，运动及虚功经真实传动计算。 */
  d = arm_cos_f32(3.1376009F - phi1);
  delta_idx_1 = arm_sin_f32(3.1376009F - phi1);
  B_idx_0 = d * -0.209998325F - delta_idx_1 * -0.000838289096F;
  B_idx_1 = delta_idx_1 * -0.209998325F + d * -0.000838289096F;
  c = arm_cos_f32(0.00399186322F - phi4);
  s = arm_sin_f32(0.00399186322F - phi4);
  D_idx_0 = c * 0.0943607166F - s * 0.00512890378F;
  D_idx_1 = s * 0.0943607166F + c * 0.00512890378F;
  P_idx_0 = d * -0.0944992453F - delta_idx_1 * -0.000377230113F;
  P_idx_1 = delta_idx_1 * -0.0944992453F + d * -0.000377230113F;
  JB[0] = B_idx_1;
  JB[1] = -B_idx_0;
  JD[2] = D_idx_1;
  JD[3] = -D_idx_0;
  JP_idx_1 = -P_idx_0;
  /*  同一装配支路的圆交点；隐式微分两条等长约束得到解析雅可比。 */
  f = P_idx_0 - D_idx_0;
  delta_idx_0 = f;
  a_idx_0 = f * f;
  f = P_idx_1 - D_idx_1;
  arm_sqrt_f32(a_idx_0 + f * f, &d);
  c = d * d / (2.0F * d);
  s = 0.01265625F - c * c;
  if ((s <= 0.0F) || (d <= 0.0F)) {
    E_idx_0 = rtNaNF;
    E_idx_1 = rtNaNF;
    JD[0] = rtNaNF;
    JD[1] = rtNaNF;
    JD[2] = rtNaNF;
    JD[3] = rtNaNF;
  } else {
    delta_idx_0 /= d;
    delta_idx_1 = f / d;
    arm_sqrt_f32(s, &s);
    E_idx_0 = (D_idx_0 + c * delta_idx_0) + s * -delta_idx_1;
    E_idx_1 = (D_idx_1 + c * delta_idx_1) + s * delta_idx_0;
    D_idx_0 = E_idx_0 - D_idx_0;
    a_idx_0 = E_idx_0 - P_idx_0;
    D_idx_1 = E_idx_1 - D_idx_1;
    d = E_idx_1 - P_idx_1;
    b_a_idx_0 = a_idx_0 * P_idx_1 + d * -P_idx_0;
    s = D_idx_0 * 0.0F + D_idx_1 * 0.0F;
    delta_idx_1 = D_idx_0 * JD[2] + D_idx_1 * JD[3];
    a_idx_1 = a_idx_0 * 0.0F + d * 0.0F;
    b_y = D_idx_0 * d - a_idx_0 * D_idx_1;
    JD[0] = (d * s - D_idx_1 * b_a_idx_0) / b_y;
    JD[1] = (-a_idx_0 * s + D_idx_0 * b_a_idx_0) / b_y;
    JD[2] = (d * delta_idx_1 - D_idx_1 * a_idx_1) / b_y;
    JD[3] = (-a_idx_0 * delta_idx_1 + D_idx_0 * a_idx_1) / b_y;
  }
  f = E_idx_0 - P_idx_0;
  P_idx_0 = f;
  delta_idx_1 = 0.0962102F * f;
  y[0] = f * f;
  f = E_idx_1 - P_idx_1;
  delta_idx_1 += -0.0583082139F * f;
  s = rt_atan2f_snf(0.0962102F * f - -0.0583082139F * P_idx_0, delta_idx_1);
  c = arm_cos_f32(s);
  s = arm_sin_f32(s);
  D_idx_0 = E_idx_0 + (c * -0.141184792F - s * 0.105236739F);
  D_idx_1 = E_idx_1 + (s * -0.141184792F + c * 0.105236739F);
  b_y = y[0] + f * f;
  /*  同一装配支路的圆交点；隐式微分两条等长约束得到解析雅可比。 */
  E_idx_0 = D_idx_0 - E_idx_0;
  E_idx_1 = D_idx_1 - E_idx_1;
  delta_idx_1 = D_idx_0 - B_idx_0;
  delta_idx_0 = delta_idx_1;
  a_idx_0 = delta_idx_1 * delta_idx_1;
  delta_idx_1 = D_idx_1 - B_idx_1;
  arm_sqrt_f32(a_idx_0 + delta_idx_1 * delta_idx_1, &d);
  c = (d * d - 0.00885124784F) / (2.0F * d);
  s = 0.004489F - c * c;
  if ((s <= 0.0F) || (d <= 0.0F)) {
    pin_idx_0 = rtNaNF;
    delta_idx_1 = rtNaNF;
    JD[0] = rtNaNF;
    JD[1] = rtNaNF;
    JD[2] = rtNaNF;
    JD[3] = rtNaNF;
  } else {
    delta_idx_0 /= d;
    delta_idx_1 /= d;
    arm_sqrt_f32(s, &s);
    pin_idx_0 = (B_idx_0 + c * delta_idx_0) + s * -delta_idx_1;
    delta_idx_1 = (B_idx_1 + c * delta_idx_1) + s * delta_idx_0;
    a_idx_0 = pin_idx_0 - B_idx_0;
    D_idx_0 = pin_idx_0 - D_idx_0;
    d = delta_idx_1 - B_idx_1;
    D_idx_1 = delta_idx_1 - D_idx_1;
    JP_idx_1 = JD[1] - JP_idx_1;
    b_a_idx_0 = (-f * (JD[0] - P_idx_1) + P_idx_0 * JP_idx_1) / b_y;
    delta_idx_0 = JD[0] + -E_idx_1 * b_a_idx_0;
    P_idx_1 = JD[1] + E_idx_0 * b_a_idx_0;
    b_a_idx_0 = a_idx_0 * B_idx_1 + d * -B_idx_0;
    a_idx_1 = (-f * JD[2] + P_idx_0 * JD[3]) / b_y;
    s = JD[2] + -E_idx_1 * a_idx_1;
    c = JD[3] + E_idx_0 * a_idx_1;
    a_idx_1 = a_idx_0 * 0.0F + d * 0.0F;
    b_y = a_idx_0 * D_idx_1 - D_idx_0 * d;
    f = D_idx_0 * delta_idx_0 + D_idx_1 * P_idx_1;
    JD[0] = (D_idx_1 * b_a_idx_0 - d * f) / b_y;
    JD[1] = (-D_idx_0 * b_a_idx_0 + a_idx_0 * f) / b_y;
    f = D_idx_0 * s + D_idx_1 * c;
    JD[2] = (D_idx_1 * a_idx_1 - d * f) / b_y;
    JD[3] = (-D_idx_0 * a_idx_1 + a_idx_0 * f) / b_y;
  }
  d = pin_idx_0 - B_idx_0;
  pin_idx_0 = d;
  f = -0.0449527651F * d;
  y[0] = d * d;
  d = delta_idx_1 - B_idx_1;
  f += 0.0496814735F * d;
  s = rt_atan2f_snf(-0.0449527651F * d - 0.0496814735F * pin_idx_0, f);
  c = arm_cos_f32(s);
  s = arm_sin_f32(s);
  D_idx_0 = B_idx_0 + (c * 0.209998325F - s * -0.135649189F);
  D_idx_1 = B_idx_1 + (s * 0.209998325F + c * -0.135649189F);
  b_y = y[0] + d * d;
  B_idx_0 = D_idx_0 - B_idx_0;
  B_idx_1 = D_idx_1 - B_idx_1;
  JD[0] -= JB[0];
  JD[1] -= JB[1];
  b_a_idx_0 = (-d * JD[0] + pin_idx_0 * JD[1]) / b_y;
  delta_idx_1 = JB[0] + -B_idx_1 * b_a_idx_0;
  JP_idx_1 = JB[1] + B_idx_0 * b_a_idx_0;
  a_idx_1 = (-d * JD[2] + pin_idx_0 * JD[3]) / b_y;
  arm_sqrt_f32(D_idx_0 * D_idx_0 + -D_idx_1 * -D_idx_1, &s);
  b_y = s * s;
  P_idx_1 = D_idx_1 / b_y;
  c = D_idx_0 / b_y;
  delta_idx_0 = D_idx_0 / s;
  JB[0] = delta_idx_1;
  JB[1] = -JP_idx_1;
  s = -D_idx_1 / s;
  JB[2] = -B_idx_1 * a_idx_1;
  JB[3] = -(B_idx_0 * a_idx_1);
  for (k = 0; k < 2; k++) {
    int i;
    i = k << 1;
    f = JB[i];
    delta_idx_1 = JB[i + 1];
    d = delta_idx_0 * f + s * delta_idx_1;
    JD[k] = d;
    JD[k + 2] = P_idx_1 * f + c * delta_idx_1;
    y[k] = d * d;
  }
  b_y = y[0] + y[1];
  a_idx_0 = JD[0] * tau1 + JD[1] * tau4;
  delta_idx_1 = JD[0] * JD[2] + JD[1] * JD[3];
  f = JD[2] * tau1 + JD[3] * tau4;
  d = JD[2] * JD[2] + JD[3] * JD[3];
  s = (b_y + 1.0E-6F) * (d + 1.0E-6F) - delta_idx_1 * delta_idx_1;
  *F = ((d + 1.0E-6F) * a_idx_0 - delta_idx_1 * f) / s;
  *Tp = ((b_y + 1.0E-6F) * f - delta_idx_1 * a_idx_0) / s;
}

/*
 * File trailer for Leg_InvTrq.c
 *
 * [EOF]
 */
