/*
 * File: tinympc14_bal_block.c
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2026-11-22 05:18:28
 */

/* Include Files */
#include "tinympc14_bal_block.h"
#include "mldivide.h"
#include "rt_nonfinite.h"
#include "tinympc14_lpv_kernel_data.h"
#include "xgetrf.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * 方程 (19)-(23) 单点线性化：两腿各按自身腿长取表，10 态 A/B + 仿射 f
 *  grav_mode: 'B'  = B 族线性口径（对拍用）
 *             'A0' = A 族在 theta=0 线性化（常数 +C2*sin(theta0)）
 *             'V*' = A 族在真实平衡点 theta* 线性化（常数 tau_g(0)+余项）
 *
 * Arguments    : double lL
 *                double lR
 *                double A[100]
 *                double B[40]
 *                double f[10]
 * Return Type  : void
 */
void tinympc14_bal_block(double lL, double lR, double A[100], double B[40],
                         double f[10])
{
  static const signed char iv[5] = {0, 0, 1, 0, 0};
  static const signed char iv1[5] = {0, 0, 0, 1, 0};
  static const signed char iv2[5] = {0, 0, 0, 0, 1};
  double M[25];
  double T[25];
  double b_I[25];
  double Nu[20];
  double dv1[20];
  double Nx[15];
  double b_dv[15];
  double a;
  double a_tmp;
  double rL_idx_0;
  double rL_idx_1;
  double rL_idx_2_tmp;
  double rR_idx_1;
  int a_tmp_tmp;
  int b_a_tmp;
  int k;
  /*  分段线性插值 + 端点钳位（与 Leg_Comp.c 同口径） */
  rR_idx_1 = fmin(fmax(lL, 0.15), 0.34);
  k = 1;
  while ((k < 19) && (rR_idx_1 >= dv[k << 2])) {
    k++;
  }
  a_tmp_tmp = (k - 1) << 2;
  a_tmp = dv[a_tmp_tmp];
  b_a_tmp = k << 2;
  a = (rR_idx_1 - a_tmp) / (dv[b_a_tmp] - a_tmp);
  rR_idx_1 = dv[a_tmp_tmp + 1];
  rL_idx_0 = rR_idx_1 + a * (dv[b_a_tmp + 1] - rR_idx_1);
  rR_idx_1 = dv[a_tmp_tmp + 2];
  rL_idx_1 = rR_idx_1 + a * (dv[b_a_tmp + 2] - rR_idx_1);
  rL_idx_2_tmp = dv[a_tmp_tmp + 3];
  /*  分段线性插值 + 端点钳位（与 Leg_Comp.c 同口径） */
  rR_idx_1 = fmin(fmax(lR, 0.15), 0.34);
  k = 1;
  while ((k < 19) && (rR_idx_1 >= dv[k << 2])) {
    k++;
  }
  double b_a;
  double rR_idx_0;
  a_tmp_tmp = (k - 1) << 2;
  a_tmp = dv[a_tmp_tmp];
  k <<= 2;
  b_a = (rR_idx_1 - a_tmp) / (dv[k] - a_tmp);
  rR_idx_1 = dv[a_tmp_tmp + 1];
  rR_idx_0 = rR_idx_1 + b_a * (dv[k + 1] - rR_idx_1);
  rR_idx_1 = dv[a_tmp_tmp + 2];
  rR_idx_1 += b_a * (dv[k + 2] - rR_idx_1);
  a_tmp = dv[a_tmp_tmp + 3];
  /*  分段线性插值 + 端点钳位（与 Leg_Comp.c 同口径） */
  /*  左腿（打印口径沿用左腿） */
  /*  A 族整机项系数 = (m_b/2 + m_leg)*g*l */
  /*  A 族腿自重项系数 = m_leg*g*l_d */
  /*  B 族腿行重力刚度 */
  /*  tau_g(0) = M_LEG*g*x_off(H) */
  memset(&M[0], 0, 25U * sizeof(double));
  M[0] = (0.002878686969512 * lL / 0.06 + 0.07442784416748 * lL) +
         0.07837202662086 * rL_idx_1;
  M[2] = 1.306200443681 * rL_idx_0 * rL_idx_1 -
         (rL_idx_2_tmp + a * (dv[b_a_tmp + 3] - rL_idx_2_tmp));
  M[6] = (0.002878686969512 * lR / 0.06 + 0.07442784416748 * lR) +
         0.07837202662086 * rR_idx_1;
  M[8] = 1.306200443681 * rR_idx_0 * rR_idx_1 -
         (a_tmp + b_a * (dv[k + 3] - a_tmp));
  M[12] = -(0.07837202662086 * rL_idx_0 + 0.45513513740232003 * lL / 2.0);
  M[13] = -(0.07837202662086 * rR_idx_0 + 0.45513513740232003 * lR / 2.0);
  M[10] = -0.025700733338882002;
  M[15] = -0.0077069896755256849;
  M[11] = -0.025700733338882002;
  M[16] = -0.0077069896755256849;
  M[17] = 1.306200443681 * rL_idx_0 * -0.03838563077916;
  M[18] = 1.306200443681 * rR_idx_0 * -0.03838563077916;
  M[19] = -0.02655031277436;
  M[20] = 0.012263274467639805;
  M[21] = -0.012263274467639805;
  M[22] = 0.01603872210918 * lL / 0.4144;
  M[23] = -0.01603872210918 * lR / 0.4144;
  memset(&Nx[0], 0, 15U * sizeof(double));
  /*  N_s = -(方程里除 ddq 外的其余项)，故整体取负 */
  Nx[0] = -((1.306200443681 * rL_idx_0 + 7.5855856233720012 * lL / 2.0) * 9.81);
  Nx[4] = -((1.306200443681 * rR_idx_0 + 7.5855856233720012 * lR / 2.0) * 9.81);
  Nx[11] = 2.8564511669179509;
  /*  同上：Nu = -∂(其余项)/∂T */
  Nu[0] = lL / 0.06 + 1.0;
  Nu[1] = 0.0;
  Nu[2] = -1.0;
  Nu[3] = 0.0;
  Nu[4] = 0.0;
  Nu[5] = lR / 0.06 + 1.0;
  Nu[6] = 0.0;
  Nu[7] = -1.0;
  Nu[8] = -1.0;
  Nu[12] = -0.639760512986;
  Nu[16] = 3.4533333333333336;
  Nu[9] = -1.0;
  Nu[13] = -0.639760512986;
  Nu[17] = -3.4533333333333336;
  Nu[10] = 0.0;
  Nu[14] = 1.0;
  Nu[18] = 0.0;
  Nu[11] = 0.0;
  Nu[15] = 1.0;
  Nu[19] = 0.0;
  /*  s/ds use the mean hip position/velocity consumed from body.vel. */
  /*  At fixed measured lengths: s_hip=s_wheel+(lL*psiL+lR*psiR)/2. */
  T[0] = 0.03;
  T[1] = 0.03;
  T[2] = lL / 2.0;
  T[3] = lR / 2.0;
  T[4] = 0.0;
  T[5] = -0.14478764478764478;
  T[6] = 0.14478764478764478;
  T[7] = -lL / 0.4144;
  T[8] = lR / 0.4144;
  T[9] = 0.0;
  for (b_a_tmp = 0; b_a_tmp < 5; b_a_tmp++) {
    T[b_a_tmp + 10] = iv[b_a_tmp];
    T[b_a_tmp + 15] = iv1[b_a_tmp];
    T[b_a_tmp + 20] = iv2[b_a_tmp];
  }
  memset(&A[0], 0, 100U * sizeof(double));
  memset(&b_I[0], 0, 25U * sizeof(double));
  for (k = 0; k < 5; k++) {
    b_I[k + 5 * k] = 1.0;
  }
  for (b_a_tmp = 0; b_a_tmp < 5; b_a_tmp++) {
    for (k = 0; k < 5; k++) {
      A[((k << 1) + 10 * (b_a_tmp << 1)) + 1] = b_I[k + 5 * b_a_tmp];
    }
  }
  mldivide(M, Nx, b_dv);
  for (b_a_tmp = 0; b_a_tmp < 3; b_a_tmp++) {
    for (k = 0; k < 5; k++) {
      rR_idx_1 = 0.0;
      for (a_tmp_tmp = 0; a_tmp_tmp < 5; a_tmp_tmp++) {
        rR_idx_1 += b_dv[b_a_tmp + 3 * a_tmp_tmp] * T[a_tmp_tmp + 5 * k];
      }
      A[((b_a_tmp << 1) + 10 * ((k << 1) + 1)) + 4] = rR_idx_1;
    }
  }
  memset(&B[0], 0, 40U * sizeof(double));
  b_mldivide(M, Nu, dv1);
  for (b_a_tmp = 0; b_a_tmp < 4; b_a_tmp++) {
    for (k = 0; k < 5; k++) {
      rR_idx_1 = 0.0;
      for (a_tmp_tmp = 0; a_tmp_tmp < 5; a_tmp_tmp++) {
        rR_idx_1 += dv1[b_a_tmp + (a_tmp_tmp << 2)] * T[a_tmp_tmp + 5 * k];
      }
      B[b_a_tmp + (((k << 1) + 1) << 2)] = rR_idx_1;
    }
  }
  memset(&f[0], 0, 10U * sizeof(double));
  for (k = 0; k < 5; k++) {
    for (b_a_tmp = 0; b_a_tmp < 5; b_a_tmp++) {
      b_I[b_a_tmp + 5 * k] = M[k + 5 * b_a_tmp];
    }
  }
  int ipiv[5];
  xgetrf(b_I, ipiv);
  for (b_a_tmp = 0; b_a_tmp < 5; b_a_tmp++) {
    rR_idx_1 = 0.0;
    for (k = 0; k < 5; k++) {
      rR_idx_1 += 0.0 * T[k + 5 * b_a_tmp];
    }
    f[(b_a_tmp << 1) + 1] = rR_idx_1;
  }
}

/*
 * File trailer for tinympc14_bal_block.c
 *
 * [EOF]
 */
