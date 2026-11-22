/*
 * File: Leg_Pos.c
 *
 * 001 实际闭环正运动学：输入电机标定角 [rad]；输出髋到轮轴距离 [m]、从 +x 到向下轴的极角 [rad] —— 由 MATLAB Coder 从 001 模型生成，
 * 已替换为 CMSIS-DSP 数学库（arm_cos_f32 / arm_sin_f32 / arm_sqrt_f32）。
 *
 * 原始生成日期: 2026-11-22 10:56:59
 */

/* Include Files */
#include "Leg_Pos.h"
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
 * 001 实际传动闭环正解。输入电机角 [rad]；输出腿长 [m] 和极角 [rad]。
 */
void Leg_Pos(float phi1, float phi4, float pos[2])
{
  float B_idx_0;
  float B_idx_1;
  float D_idx_0;
  float D_idx_1;
  float P_idx_0;
  float P_idx_1;
  float a_idx_0;
  float c;
  float d;
  float delta_idx_0;
  float height2;
  float s;
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
  /*  同一装配支路的圆交点；隐式微分两条等长约束得到解析雅可比。 */
  s = P_idx_0 - D_idx_0;
  delta_idx_0 = s;
  a_idx_0 = s * s;
  s = P_idx_1 - D_idx_1;
  arm_sqrt_f32(a_idx_0 + s * s, &d);
  c = d * d / (2.0F * d);
  height2 = 0.01265625F - c * c;
  if ((height2 <= 0.0F) || (d <= 0.0F)) {
    a_idx_0 = rtNaNF;
    height2 = rtNaNF;
  } else {
    delta_idx_0 /= d;
    s /= d;
    arm_sqrt_f32(height2, &d);
    a_idx_0 = (D_idx_0 + c * delta_idx_0) + d * -s;
    height2 = (D_idx_1 + c * s) + d * delta_idx_0;
  }
  s = a_idx_0 - P_idx_0;
  P_idx_0 = s;
  d = 0.0962102F * s;
  s = height2 - P_idx_1;
  d += -0.0583082139F * s;
  d = rt_atan2f_snf(0.0962102F * s - -0.0583082139F * P_idx_0, d);
  c = arm_cos_f32(d);
  s = arm_sin_f32(d);
  /*  同一装配支路的圆交点；隐式微分两条等长约束得到解析雅可比。 */
  delta_idx_0 = (a_idx_0 + (c * -0.141184792F - s * 0.105236739F)) - B_idx_0;
  s = (height2 + (s * -0.141184792F + c * 0.105236739F)) - B_idx_1;
  arm_sqrt_f32(delta_idx_0 * delta_idx_0 + s * s, &d);
  c = (d * d - 0.00885124784F) / (2.0F * d);
  height2 = 0.004489F - c * c;
  if ((height2 <= 0.0F) || (d <= 0.0F)) {
    a_idx_0 = rtNaNF;
    height2 = rtNaNF;
  } else {
    delta_idx_0 /= d;
    s /= d;
    arm_sqrt_f32(height2, &d);
    a_idx_0 = (B_idx_0 + c * delta_idx_0) + d * -s;
    height2 = (B_idx_1 + c * s) + d * delta_idx_0;
  }
  s = a_idx_0 - B_idx_0;
  a_idx_0 = s;
  d = -0.0449527651F * s;
  s = height2 - B_idx_1;
  d += 0.0496814735F * s;
  d = rt_atan2f_snf(-0.0449527651F * s - 0.0496814735F * a_idx_0, d);
  c = arm_cos_f32(d);
  s = arm_sin_f32(d);
  height2 = B_idx_0 + (c * 0.209998325F - s * -0.135649189F);
  d = -(B_idx_1 + (s * 0.209998325F + c * -0.135649189F));
  arm_sqrt_f32(height2 * height2 + d * d, &pos[0]);
  pos[1] = rt_atan2f_snf(d, height2);
}

/*
 * File trailer for Leg_Pos.c
 *
 * [EOF]
 */
