/*
 * File: tinympc14_fused_op.c
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2026-11-21 22:02:36
 */

/* Include Files */
#include "tinympc14_fused_op.h"
#include "mldivide.h"
#include "rt_nonfinite.h"
#include "tinympc14_lpv_kernel_data.h"
#include "tinympc14_lpv_kernel_internal_types.h"
#include "xgetrf.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * 本调度点的平衡块与腿长块参数包：一次算好供非线性核与数值雅可比复用
 *
 * Arguments    : double sched_H
 *                double sched_lL
 *                double sched_lR
 *                double sched_psi
 *                double op_A_bal[100]
 *                double op_B_bal[40]
 *                double op_f_bal[10]
 *                struct_T *op_aux
 *                double *op_lR
 *                double *op_H
 *                double *op_M_v
 *                double *op_M_load
 *                double *op_K_eff
 *                double *op_psi
 * Return Type  : double
 */
double tinympc14_fused_op(double sched_H, double sched_lL, double sched_lR,
                          double sched_psi, double op_A_bal[100],
                          double op_B_bal[40], double op_f_bal[10],
                          struct_T *op_aux, double *op_lR, double *op_H,
                          double *op_M_v, double *op_M_load, double *op_K_eff,
                          double *op_psi)
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
  double C2g;
  double C2g_r;
  double cR_idx_1;
  double d;
  double kB;
  double kB_r;
  double op_lL;
  double rL_idx_0;
  double rL_idx_1;
  double rL_idx_2;
  int a_tmp_tmp;
  int i;
  int k;
  /*  方程 (19)-(23) 单点线性化：两腿各按自身腿长取表，10 态 A/B + 仿射 f */
  /*  grav_mode: 'B'  = B 族线性口径（对拍用） */
  /*             'A0' = A 族在 theta=0 线性化（常数 +C2*sin(theta0)） */
  /*             'V*' = A 族在真实平衡点 theta* 线性化（常数 tau_g(0)+余项） */
  /*  分段线性插值 + 端点钳位（与 Leg_Comp.c 同口径） */
  C2g = fmin(fmax(sched_lL, 0.15), 0.34);
  k = 1;
  while ((k < 19) && (C2g >= dv[k << 2])) {
    k++;
  }
  a_tmp_tmp = (k - 1) << 2;
  cR_idx_1 = dv[a_tmp_tmp];
  k <<= 2;
  kB = (C2g - cR_idx_1) / (dv[k] - cR_idx_1);
  cR_idx_1 = dv[a_tmp_tmp + 1];
  rL_idx_0 = cR_idx_1 + kB * (dv[k + 1] - cR_idx_1);
  cR_idx_1 = dv[a_tmp_tmp + 2];
  rL_idx_1 = cR_idx_1 + kB * (dv[k + 2] - cR_idx_1);
  cR_idx_1 = dv[a_tmp_tmp + 3];
  rL_idx_2 = cR_idx_1 + kB * (dv[k + 3] - cR_idx_1);
  /*  分段线性插值 + 端点钳位（与 Leg_Comp.c 同口径） */
  C2g_r = fmin(fmax(sched_lR, 0.15), 0.34);
  k = 1;
  while ((k < 19) && (C2g_r >= dv[k << 2])) {
    k++;
  }
  double rR_idx_0;
  double rR_idx_1;
  double rR_idx_2;
  a_tmp_tmp = (k - 1) << 2;
  cR_idx_1 = dv[a_tmp_tmp];
  k <<= 2;
  kB = (C2g_r - cR_idx_1) / (dv[k] - cR_idx_1);
  cR_idx_1 = dv[a_tmp_tmp + 1];
  rR_idx_0 = cR_idx_1 + kB * (dv[k + 1] - cR_idx_1);
  cR_idx_1 = dv[a_tmp_tmp + 2];
  rR_idx_1 = cR_idx_1 + kB * (dv[k + 2] - cR_idx_1);
  cR_idx_1 = dv[a_tmp_tmp + 3];
  rR_idx_2 = cR_idx_1 + kB * (dv[k + 3] - cR_idx_1);
  /*  分段线性插值 + 端点钳位（与 Leg_Comp.c 同口径） */
  /*  分段线性插值 + 端点钳位（与 Leg_Comp.c 同口径） */
  /*  左腿（打印口径沿用左腿） */
  /*  A 族整机项系数 = (m_b/2 + m_leg)*g*l */
  /*  A 族腿自重项系数 = m_leg*g*l_d */
  kB = (1.306200449528 * rL_idx_0 + 8.6486405243079982 * sched_lL / 2.0) * 9.81;
  /*  B 族腿行重力刚度 */
  kB_r =
      (1.306200449528 * rR_idx_0 + 8.6486405243079982 * sched_lR / 2.0) * 9.81;
  /*  tau_g(0) = M_LEG*g*x_off(H) */
  memset(&M[0], 0, 25U * sizeof(double));
  M[0] =
      (0.002878686969512 * sched_lL / 0.0525 + 0.065608159399545 * sched_lL) +
      0.06857552360022 * rL_idx_1;
  M[2] = 1.306200449528 * rL_idx_0 * rL_idx_1 - rL_idx_2;
  M[6] =
      (0.002878686969512 * sched_lR / 0.0525 + 0.065608159399545 * sched_lR) +
      0.06857552360022 * rR_idx_1;
  M[8] = 1.306200449528 * rR_idx_0 * rR_idx_1 - rR_idx_2;
  M[12] = -(0.06857552360022 * rL_idx_0 + 0.4540536275261699 * sched_lL / 2.0);
  M[13] = -(0.06857552360022 * rR_idx_0 + 0.4540536275261699 * sched_lR / 2.0);
  M[10] = -0.021842238049561619;
  M[15] = -0.0072125587924970645;
  M[11] = -0.021842238049561619;
  M[16] = -0.0072125587924970645;
  M[17] = 1.306200449528 * rL_idx_0 * -0.03815849358313;
  M[18] = 1.306200449528 * rR_idx_0 * -0.03815849358313;
  M[19] = -0.03626959445656;
  M[20] = 0.014142278876968648;
  M[21] = -0.014142278876968648;
  M[22] = 0.02195184126149 * sched_lL / 0.4144;
  M[23] = -0.02195184126149 * sched_lR / 0.4144;
  memset(&Nx[0], 0, 15U * sizeof(double));
  /*  N_s = -(方程里除 ddq 外的其余项)，故整体取负 */
  Nx[0] = -kB;
  Nx[4] = -kB_r;
  Nx[11] = 3.2374873116456233;
  /*  同上：Nu = -∂(其余项)/∂T */
  Nu[0] = sched_lL / 0.0525 + 1.0;
  Nu[1] = 0.0;
  Nu[2] = -1.0;
  Nu[3] = 0.0;
  Nu[4] = 0.0;
  Nu[5] = sched_lR / 0.0525 + 1.0;
  Nu[6] = 0.0;
  Nu[7] = -1.0;
  Nu[8] = -1.0;
  Nu[12] = -0.72682844920247613;
  Nu[16] = 3.9466666666666668;
  Nu[9] = -1.0;
  Nu[13] = -0.72682844920247613;
  Nu[17] = -3.9466666666666668;
  Nu[10] = 0.0;
  Nu[14] = 1.0;
  Nu[18] = 0.0;
  Nu[11] = 0.0;
  Nu[15] = 1.0;
  Nu[19] = 0.0;
  /*  s/ds use the mean hip position/velocity consumed from body.vel. */
  /*  At fixed measured lengths: s_hip=s_wheel+(lL*psiL+lR*psiR)/2. */
  T[0] = 0.02625;
  T[1] = 0.02625;
  T[2] = sched_lL / 2.0;
  T[3] = sched_lR / 2.0;
  T[4] = 0.0;
  T[5] = -0.1266891891891892;
  T[6] = 0.1266891891891892;
  T[7] = -sched_lL / 0.4144;
  T[8] = sched_lR / 0.4144;
  T[9] = 0.0;
  for (i = 0; i < 5; i++) {
    T[i + 10] = iv[i];
    T[i + 15] = iv1[i];
    T[i + 20] = iv2[i];
  }
  memset(&op_A_bal[0], 0, 100U * sizeof(double));
  memset(&b_I[0], 0, 25U * sizeof(double));
  for (k = 0; k < 5; k++) {
    b_I[k + 5 * k] = 1.0;
  }
  for (i = 0; i < 5; i++) {
    for (k = 0; k < 5; k++) {
      op_A_bal[((k << 1) + 10 * (i << 1)) + 1] = b_I[k + 5 * i];
    }
  }
  mldivide(M, Nx, b_dv);
  for (i = 0; i < 3; i++) {
    for (k = 0; k < 5; k++) {
      d = 0.0;
      for (a_tmp_tmp = 0; a_tmp_tmp < 5; a_tmp_tmp++) {
        d += b_dv[i + 3 * a_tmp_tmp] * T[a_tmp_tmp + 5 * k];
      }
      op_A_bal[((i << 1) + 10 * ((k << 1) + 1)) + 4] = d;
    }
  }
  memset(&op_B_bal[0], 0, 40U * sizeof(double));
  b_mldivide(M, Nu, dv1);
  for (i = 0; i < 4; i++) {
    for (k = 0; k < 5; k++) {
      d = 0.0;
      for (a_tmp_tmp = 0; a_tmp_tmp < 5; a_tmp_tmp++) {
        d += dv1[i + (a_tmp_tmp << 2)] * T[a_tmp_tmp + 5 * k];
      }
      op_B_bal[i + (((k << 1) + 1) << 2)] = d;
    }
  }
  memset(&op_f_bal[0], 0, 10U * sizeof(double));
  for (k = 0; k < 5; k++) {
    for (i = 0; i < 5; i++) {
      b_I[i + 5 * k] = M[k + 5 * i];
    }
  }
  int ipiv[5];
  xgetrf(b_I, ipiv);
  for (i = 0; i < 5; i++) {
    d = 0.0;
    for (k = 0; k < 5; k++) {
      d += 0.0 * T[k + 5 * i];
    }
    op_f_bal[(i << 1) + 1] = d;
  }
  op_aux->kB = kB;
  op_aux->kB_r = kB_r;
  op_lL = sched_lL;
  *op_lR = sched_lR;
  *op_H = sched_H;
  /*  sched.H is absolute; only x(13) uses H-H_REF. */
  /*  H 行等效竖直惯量的三档口径（见 cfg §3） */
  *op_M_v = 10.1610327214;
  /*  Old and training configurations use the wheel-excluded support mass. */
  *op_M_load = 10.37721374079;
  /*  (dV/dH)/g of the coupled leg mechanism at nominal height */
  /*  气弹簧等效刚度：'spring' 按 H 中心差分 dFs/dL0；'fixed' 取 cfg.K_EFF */
  *op_K_eff = 0.0;
  *op_psi = sched_psi;
  return op_lL;
}

/*
 * File trailer for tinympc14_fused_op.c
 *
 * [EOF]
 */
