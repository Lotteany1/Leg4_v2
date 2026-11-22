/*
 * File: Leg_Spd.c
 *
 * 001 实际闭环解析雅可比：输入电机角速度 [rad/s]，输出腿长速度 [m/s]、极角速度 [rad/s] —— 由 MATLAB Coder 从 001 模型生成，
 * 已替换为 CMSIS-DSP 数学库（arm_cos_f32 / arm_sin_f32 / arm_sqrt_f32）。
 *
 * 原始生成日期: 2026-11-22 10:57:21
 */

/* Include Files */
#include "Leg_Spd.h"
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
 * 与 001 闭环正解相同的解析雅可比，输出 [腿长速度;极角速度]。
 */
void Leg_Spd(float phi1, float phi4, float phi_dot_1, float phi_dot_4,
             float spd[2])
{
  float rhs[4];
  float B_idx_0;
  float B_idx_1;
  float D_idx_0;
  float D_idx_1;
  float E_idx_0;
  float E_idx_1;
  float JB_idx_0;
  float JB_idx_1;
  float JD_idx_0;
  float JD_idx_1;
  float JD_idx_2;
  float JD_idx_3;
  float JP_idx_1;
  float JP_idx_2;
  float P_idx_0;
  float P_idx_1;
  float a;
  float a_idx_0;
  float c;
  float d;
  float delta_idx_0;
  float f;
  float height2;
  float pin_idx_0;
  float pin_idx_1;
  float s;
  float y;
  int i;
  /*  001 双闭环：输入保留安装标定的两个电机角坐标，输出 [腿长;极角]。 */
  /*  后电机仅在 CAD 零位与虚拟曲柄重合，运动及虚功经真实传动计算。 */
  d = arm_cos_f32(3.1376009F - phi1);
  height2 = arm_sin_f32(3.1376009F - phi1);
  B_idx_0 = d * -0.209998325F - height2 * -0.000838289096F;
  B_idx_1 = height2 * -0.209998325F + d * -0.000838289096F;
  c = arm_cos_f32(0.00399186322F - phi4);
  s = arm_sin_f32(0.00399186322F - phi4);
  D_idx_0 = c * 0.0943607166F - s * 0.00512890378F;
  D_idx_1 = s * 0.0943607166F + c * 0.00512890378F;
  P_idx_0 = d * -0.0944992453F - height2 * -0.000377230113F;
  P_idx_1 = height2 * -0.0944992453F + d * -0.000377230113F;
  JB_idx_0 = B_idx_1;
  JB_idx_1 = -B_idx_0;
  JD_idx_2 = D_idx_1;
  JD_idx_3 = -D_idx_0;
  JP_idx_1 = -P_idx_0;
  /*  同一装配支路的圆交点；隐式微分两条等长约束得到解析雅可比。 */
  f = P_idx_0 - D_idx_0;
  delta_idx_0 = f;
  a_idx_0 = f * f;
  f = P_idx_1 - D_idx_1;
  arm_sqrt_f32(a_idx_0 + f * f, &d);
  s = d * d / (2.0F * d);
  height2 = 0.01265625F - s * s;
  if ((height2 <= 0.0F) || (d <= 0.0F)) {
    E_idx_0 = rtNaNF;
    E_idx_1 = rtNaNF;
    JD_idx_0 = rtNaNF;
    JD_idx_1 = rtNaNF;
    JD_idx_2 = rtNaNF;
    JD_idx_3 = rtNaNF;
  } else {
    delta_idx_0 /= d;
    d = f / d;
    arm_sqrt_f32(height2, &a);
    E_idx_0 = (D_idx_0 + s * delta_idx_0) + a * -d;
    E_idx_1 = (D_idx_1 + s * d) + a * delta_idx_0;
    D_idx_0 = E_idx_0 - D_idx_0;
    a_idx_0 = E_idx_0 - P_idx_0;
    D_idx_1 = E_idx_1 - D_idx_1;
    c = E_idx_1 - P_idx_1;
    a = a_idx_0 * P_idx_1 + c * -P_idx_0;
    d = D_idx_0 * 0.0F + D_idx_1 * 0.0F;
    s = D_idx_0 * JD_idx_2 + D_idx_1 * JD_idx_3;
    JP_idx_2 = a_idx_0 * 0.0F + c * 0.0F;
    y = D_idx_0 * c - a_idx_0 * D_idx_1;
    JD_idx_0 = (c * d - D_idx_1 * a) / y;
    JD_idx_1 = (-a_idx_0 * d + D_idx_0 * a) / y;
    JD_idx_2 = (c * s - D_idx_1 * JP_idx_2) / y;
    JD_idx_3 = (-a_idx_0 * s + D_idx_0 * JP_idx_2) / y;
  }
  f = E_idx_0 - P_idx_0;
  P_idx_0 = f;
  JP_idx_2 = 0.0962102F * f;
  a_idx_0 = f * f;
  f = E_idx_1 - P_idx_1;
  JP_idx_2 += -0.0583082139F * f;
  a = rt_atan2f_snf(0.0962102F * f - -0.0583082139F * P_idx_0, JP_idx_2);
  c = arm_cos_f32(a);
  s = arm_sin_f32(a);
  D_idx_0 = E_idx_0 + (c * -0.141184792F - s * 0.105236739F);
  D_idx_1 = E_idx_1 + (s * -0.141184792F + c * 0.105236739F);
  y = a_idx_0 + f * f;
  /*  同一装配支路的圆交点；隐式微分两条等长约束得到解析雅可比。 */
  E_idx_0 = D_idx_0 - E_idx_0;
  E_idx_1 = D_idx_1 - E_idx_1;
  JP_idx_2 = D_idx_0 - B_idx_0;
  delta_idx_0 = JP_idx_2;
  a_idx_0 = JP_idx_2 * JP_idx_2;
  JP_idx_2 = D_idx_1 - B_idx_1;
  arm_sqrt_f32(a_idx_0 + JP_idx_2 * JP_idx_2, &d);
  s = (d * d - 0.00885124784F) / (2.0F * d);
  height2 = 0.004489F - s * s;
  if ((height2 <= 0.0F) || (d <= 0.0F)) {
    pin_idx_0 = rtNaNF;
    pin_idx_1 = rtNaNF;
    delta_idx_0 = rtNaNF;
    JP_idx_1 = rtNaNF;
    JP_idx_2 = rtNaNF;
    height2 = rtNaNF;
  } else {
    delta_idx_0 /= d;
    d = JP_idx_2 / d;
    arm_sqrt_f32(height2, &a);
    pin_idx_0 = (B_idx_0 + s * delta_idx_0) + a * -d;
    pin_idx_1 = (B_idx_1 + s * d) + a * delta_idx_0;
    a_idx_0 = pin_idx_0 - B_idx_0;
    D_idx_0 = pin_idx_0 - D_idx_0;
    c = pin_idx_1 - B_idx_1;
    D_idx_1 = pin_idx_1 - D_idx_1;
    JP_idx_1 = JD_idx_1 - JP_idx_1;
    a = (-f * (JD_idx_0 - P_idx_1) + P_idx_0 * JP_idx_1) / y;
    JD_idx_0 += -E_idx_1 * a;
    JD_idx_1 += E_idx_0 * a;
    a = a_idx_0 * B_idx_1 + c * -B_idx_0;
    JP_idx_2 = (-f * JD_idx_2 + P_idx_0 * JD_idx_3) / y;
    height2 = a_idx_0 * 0.0F + c * 0.0F;
    d = D_idx_0 * JD_idx_0 + D_idx_1 * JD_idx_1;
    s = D_idx_0 * (JD_idx_2 + -E_idx_1 * JP_idx_2) +
        D_idx_1 * (JD_idx_3 + E_idx_0 * JP_idx_2);
    y = a_idx_0 * D_idx_1 - D_idx_0 * c;
    delta_idx_0 = (D_idx_1 * a - c * d) / y;
    JP_idx_1 = (-D_idx_0 * a + a_idx_0 * d) / y;
    JP_idx_2 = (D_idx_1 * height2 - c * s) / y;
    height2 = (-D_idx_0 * height2 + a_idx_0 * s) / y;
  }
  d = pin_idx_0 - B_idx_0;
  pin_idx_0 = d;
  f = -0.0449527651F * d;
  a_idx_0 = d * d;
  d = pin_idx_1 - B_idx_1;
  f += 0.0496814735F * d;
  a = rt_atan2f_snf(-0.0449527651F * d - 0.0496814735F * pin_idx_0, f);
  c = arm_cos_f32(a);
  s = arm_sin_f32(a);
  D_idx_0 = B_idx_0 + (c * 0.209998325F - s * -0.135649189F);
  D_idx_1 = B_idx_1 + (s * 0.209998325F + c * -0.135649189F);
  y = a_idx_0 + d * d;
  B_idx_0 = D_idx_0 - B_idx_0;
  B_idx_1 = D_idx_1 - B_idx_1;
  delta_idx_0 -= JB_idx_0;
  JP_idx_1 -= JB_idx_1;
  a = (-d * delta_idx_0 + pin_idx_0 * JP_idx_1) / y;
  JP_idx_2 = (-d * JP_idx_2 + pin_idx_0 * height2) / y;
  arm_sqrt_f32(D_idx_0 * D_idx_0 + -D_idx_1 * -D_idx_1, &d);
  y = d * d;
  rhs[1] = D_idx_1 / y;
  rhs[3] = D_idx_0 / y;
  rhs[0] = D_idx_0 / d;
  JD_idx_0 = JB_idx_0 + -B_idx_1 * a;
  JD_idx_1 = -(JB_idx_1 + B_idx_0 * a);
  rhs[2] = -D_idx_1 / d;
  JD_idx_2 = -B_idx_1 * JP_idx_2;
  JD_idx_3 = -(B_idx_0 * JP_idx_2);
  for (i = 0; i < 2; i++) {
    f = rhs[i + 2];
    JP_idx_2 = rhs[i];
    spd[i] = (JP_idx_2 * JD_idx_0 + f * JD_idx_1) * phi_dot_1 +
             (JP_idx_2 * JD_idx_2 + f * JD_idx_3) * phi_dot_4;
  }
}

/*
 * File trailer for Leg_Spd.c
 *
 * [EOF]
 */
