/*
 * File: Leg_RollComp.c
 *
 * 转弯向心力前馈：入参 L0 等效腿长 [m]、omega yaw 角速度 [rad/s]、v 前进速度 [m/s]; 出参 dF_lft/dF_rgt 为限幅后补偿 [N]，Fc_raw 为限幅前等效力 [N]，模型外部扰动为 [Fc_raw,-Fc_raw] —— 由 MATLAB Coder 从 001 模型生成。
 *
 * 原始生成日期: 2026-11-22 10:57:35
 */

/* Include Files */
#include "Leg_RollComp.h"
#include <math.h>

/* Function Definitions */
/*
 * 平地准静态载荷转移：CAD 全车质量乘离地质心高度，包含轮子与轮半径。
 *  H 是髋部相对轮心高度 [m]；v [m/s]、omega [rad/s]，前进左转右侧加载。
 */
void Leg_RollComp(float H, float omega, float v, float *dF_lft, float *dF_rgt,
                  float *Fc_raw)
{
  static const float T[40] = {
      0.15F,        0.159005359F, 0.168010712F, 0.177016065F, 0.186021432F,
      0.195026785F, 0.204032138F, 0.213037491F, 0.222042844F, 0.231048211F,
      0.240053564F, 0.249058917F, 0.25806427F,  0.267069638F, 0.276074976F,
      0.285080343F, 0.294085711F, 0.303091049F, 0.312096417F, 0.321101785F,
      2.47593427F,  2.58589959F,  2.6962862F,   2.80702901F,  2.91807604F,
      3.02938461F,  3.14092F,     3.25265336F,  3.36456013F,  3.4766202F,
      3.58881593F,  3.70113277F,  3.81355762F,  3.92607951F,  4.03868914F,
      4.15137768F,  4.26413822F,  4.37696362F,  4.48984861F,  4.60278797F};
  float x;
  int b_index;
  x = fminf(fmaxf(H, 0.17F), 0.321101785F);
  /*  固定大小物理参数表的分段线性插值，表外保持端点。 */
  b_index = 1;
  while ((b_index < 19) && (x >= T[b_index])) {
    b_index++;
  }
  float Fc_raw_tmp;
  float b_Fc_raw_tmp;
  Fc_raw_tmp = T[b_index - 1];
  b_Fc_raw_tmp = T[b_index + 19];
  *Fc_raw = (b_Fc_raw_tmp + (x - Fc_raw_tmp) / (T[b_index] - Fc_raw_tmp) *
                                (T[b_index + 20] - b_Fc_raw_tmp)) *
            v * omega / 0.414400041F;
  *dF_rgt = fminf(fmaxf(*Fc_raw, -150.0F), 150.0F);
  *dF_lft = -*dF_rgt;
}

/*
 * File trailer for Leg_RollComp.c
 *
 * [EOF]
 */
