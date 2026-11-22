/*
 * File: tinympc14_scheduled_ab.c
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2026-11-22 05:18:28
 */

/* Include Files */
#include "tinympc14_scheduled_ab.h"
#include "rt_nonfinite.h"
#include "tinympc14_bal_block.h"
#include "tinympc14_fused_dyn.h"
#include "tinympc14_fused_op.h"
#include "tinympc14_lpv_kernel_internal_types.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Continuous model at one LPV scheduler input, with no toolbox dependency.
 *
 * Arguments    : double left
 *                double right
 *                double psi
 *                double A[196]
 *                double B[84]
 *                double f[14]
 *                double *terminal_center
 * Return Type  : double
 */
double tinympc14_scheduled_ab(double left, double right, double psi,
                              double A[196], double B[84], double f[14],
                              double *terminal_center)
{
  struct_T op_aux;
  double op_A_bal[100];
  double op_B_bal[40];
  double length_gain[20];
  double x0[14];
  double op_f_bal[10];
  double ue[6];
  double b_roll_gain[4];
  double cos_psi[2];
  double gravity_stiffness[2];
  double height_gain[2];
  double roll_gain[2];
  double b_expl_temp;
  double c_expl_temp;
  double d;
  double d1;
  double expl_temp;
  double heave_scale;
  double load_tmp;
  double op_K_eff;
  double op_M_load;
  double op_M_v;
  double residual;
  double roll;
  double sched_H;
  double sched_H_tmp;
  double vertical_tmp;
  int i;
  int i1;
  int k;
  int side;
  /*  Keep the continuous model scratch separate from ZOH. */
  /*  Invert the firmware height observation on level ground. The deployment */
  /*  table uses one common sagittal angle for both measured leg lengths. */
  vertical_tmp = cos(psi);
  roll = atan((left - right) * vertical_tmp / 0.4144);
  sched_H_tmp = cos(roll);
  sched_H = 0.5 * (left + right) * vertical_tmp * sched_H_tmp;
  /*  非线性平衡：两腿同角、alpha=0、theta_b 取 pitch_ref，解腿角与六路输入使 14
   * 态加速度为零 */
  /*  牛顿迭代 + 数值雅可比；返回 x_eq/u_eq 供仿射项锚定 */
  tinympc14_fused_op(sched_H, left, right, psi, op_A_bal, op_B_bal, op_f_bal,
                     &op_aux, &expl_temp, &b_expl_temp, &op_M_v, &op_M_load,
                     &op_K_eff, &c_expl_temp);
  /*  Zero longitudinal and yaw acceleration imply zero wheel torque. */
  /*  The two leg gravity moments and the body pitch moment then fix the */
  /*  common leg lean and hip torques. Vertical and roll balance fix forces. */
  *terminal_center = -0.14282255834589755 / (op_aux.kB + op_aux.kB_r);
  expl_temp = -0.0 * roll;
  /*  两腿分担升降广义载荷。在该分担点的几何倾覆矩已归入外扰。 */
  heave_scale = sched_H_tmp * cos(*terminal_center);
  b_expl_temp = heave_scale * sched_H_tmp / 0.2072;
  memset(&x0[0], 0, 14U * sizeof(double));
  load_tmp = op_M_load * 9.81 + op_K_eff * (sched_H - 0.27);
  heave_scale *= load_tmp;
  ue[0] = 0.0;
  ue[1] = 0.0;
  x0[4] = *terminal_center;
  ue[2] = -op_aux.kB * *terminal_center;
  ue[4] = (heave_scale + expl_temp * b_expl_temp) / 2.0;
  x0[6] = *terminal_center;
  ue[3] = -op_aux.kB_r * *terminal_center;
  ue[5] = (heave_scale + -expl_temp * b_expl_temp) / 2.0;
  x0[8] = -0.05;
  x0[10] = roll;
  x0[12] = sched_H - 0.27;
  tinympc14_fused_dyn(op_A_bal, op_B_bal, op_f_bal, op_M_v, op_M_load, op_K_eff,
                      x0, ue, f);
  residual = 0.0;
  /*  融合 14 态线性化：力投影解析求导，腿长表用割线斜率，仿射项由动力学核反推。
   */
  /*  */
  /*    参考位姿 x_ref = [0 0 0 0 psi_l 0 psi_r 0 pitch_ref 0 alpha 0 H-H_REF
   * 0]， */
  /*    默认参考输入 u_ref = [0 0 0 0 Fbar Fbar]，Fbar = op.M_load*g/2。 */
  /*    f = dyn(x_ref,u_ref) - A*x_ref - B*u_ref，保证展开点处动力学一致。 */
  /*    展开点不一定是平衡点；不得通过改 f 强行制造一个跨调度点的定点。 */
  /*  */
  /*    腿轴力与腿长共轭。双轮平地约束下，H/alpha 两行使用同一腿长雅可比。 */
  /*    轮矩按轮心水平位移作功。psi 取本调度点的腿角。 */
  /*    Production F_pitch=0: direct horizontal force addition double-counts */
  /*    internal support after the balance equations have eliminated internal
   * forces. */
  /*  */
  /*    sw 沿用登记表开关：#1 sw.F_pitch、#2 sw.F_H_proj、#8 sw.Ts_H、#8alpha
   * sw.Ts_alpha */
  /*    sched 可带 psi_l/psi_r（缺省都取 sched.psi） */
  /*  */
  /*    版本：v0.6  日期：2026-10-22 */
  tinympc14_fused_op(sched_H, left, right, psi, op_A_bal, op_B_bal, op_f_bal,
                     &op_aux, &expl_temp, &b_expl_temp, &op_M_v, &op_M_load,
                     &op_K_eff, &c_expl_temp);
  for (k = 0; k < 14; k++) {
    heave_scale = fabs(f[k]);
    if (rtIsNaN(heave_scale) || (heave_scale > residual)) {
      residual = heave_scale;
    }
    x0[k] = 0.0;
  }
  x0[4] = psi;
  x0[6] = psi;
  x0[8] = -0.05;
  x0[10] = roll;
  x0[12] = sched_H - 0.27;
  /*  input expansion point only; never replace the affine residual */
  memset(&A[0], 0, 196U * sizeof(double));
  memset(&B[0], 0, 84U * sizeof(double));
  for (i = 0; i < 10; i++) {
    memcpy(&A[i * 14], &op_A_bal[i * 10], 10U * sizeof(double));
    k = i << 2;
    B[6 * i] = op_B_bal[k];
    B[6 * i + 1] = op_B_bal[k + 1];
    B[6 * i + 2] = op_B_bal[k + 2];
    B[6 * i + 3] = op_B_bal[k + 3];
  }
  double b_sched_H;
  double side_arm_idx_0;
  double side_arm_idx_1;
  double tan_phi_tmp;
  A[151] = 1.0;
  A[181] = 1.0;
  A[194] = -op_K_eff / op_M_v;
  A[195] = -8.0 / op_M_v;
  /*  双轮平地闭链 L_i=(H±b sin(phi))/(cos(phi)cos(psi_i))。 */
  /*  固定两腿矢状角的虚位移 dL_i=height_arm_i*dH+roll_arm_i*dphi。 */
  /*  轴力与 +L 共轭，乘上述两臂分别得到虚升降力和虚横滚力矩。 */
  heave_scale = 1.0 / sched_H_tmp;
  tan_phi_tmp = tan(roll);
  roll = sin(psi);
  side_arm_idx_0 = 0.2072 * heave_scale;
  side_arm_idx_1 = -0.2072 * heave_scale;
  height_gain[0] = heave_scale / vertical_tmp;
  height_gain[1] = heave_scale / vertical_tmp;
  b_sched_H = sched_H * tan_phi_tmp;
  d = 0.06 * op_M_v;
  sched_H_tmp = tan_phi_tmp * tan_phi_tmp;
  heave_scale = sched_H * (sched_H_tmp + heave_scale * heave_scale);
  d1 = height_gain[0] * (b_sched_H + side_arm_idx_0);
  roll_gain[0] = d1;
  B[82] = height_gain[0] / op_M_v;
  B[70] = d1 / 0.310399869075;
  expl_temp = ue[4] + 0.0 * roll / 0.06;
  B[78] = height_gain[0] * roll / d;
  B[66] = d1 * roll / 0.018623992144499998;
  b_expl_temp = height_gain[0] * expl_temp;
  gravity_stiffness[0] = b_expl_temp;
  c_expl_temp = tan(psi);
  A[186] = height_gain[0] * 0.0 * vertical_tmp / d +
           b_expl_temp * c_expl_temp / op_M_v;
  A[158] = d1 * 0.0 * vertical_tmp / 0.018623992144499998 +
           d1 * expl_temp * c_expl_temp / 0.310399869075;
  side_arm_idx_0 = height_gain[0] *
                   (heave_scale + 2.0 * side_arm_idx_0 * tan_phi_tmp) *
                   expl_temp;
  d1 = height_gain[1] * (b_sched_H + side_arm_idx_1);
  roll_gain[1] = d1;
  B[83] = height_gain[1] / op_M_v;
  B[71] = d1 / 0.310399869075;
  expl_temp = ue[5] + 0.0 * roll / 0.06;
  B[79] = height_gain[1] * roll / d;
  B[67] = d1 * roll / 0.018623992144499998;
  b_expl_temp = height_gain[1] * expl_temp;
  A[188] = height_gain[1] * 0.0 * vertical_tmp / d +
           b_expl_temp * c_expl_temp / op_M_v;
  A[160] = d1 * 0.0 * vertical_tmp / 0.018623992144499998 +
           d1 * expl_temp * c_expl_temp / 0.310399869075;
  side_arm_idx_1 = height_gain[1] *
                   (heave_scale + 2.0 * side_arm_idx_1 * tan_phi_tmp) *
                   expl_temp;
  heave_scale = gravity_stiffness[0] + b_expl_temp;
  A[192] = heave_scale * tan_phi_tmp / op_M_v;
  A[164] = ((side_arm_idx_0 + side_arm_idx_1) -
            load_tmp * sched_H * (sched_H_tmp + 1.0)) /
           0.310399869075;
  A[166] = ((heave_scale - load_tmp) - op_K_eff * sched_H) * tan_phi_tmp /
           0.310399869075;
  /*  Vary height and roll at fixed measured sagittal leg angles. Level-ground
   */
  /*  closure gives L_i = (H +/- half_track*sin(roll))/(cos(roll)*cos(psi_i)).
   */
  /*  m, half of the original hardware table spacing */
  cos_psi[0] = fmin(fmax(left, 0.15), 0.34);
  cos_psi[1] = fmin(fmax(right, 0.15), 0.34);
  d = ue[2];
  d1 = ue[3];
  for (side = 0; side < 2; side++) {
    double A0[100];
    double A1[100];
    double B0[40];
    double B1[40];
    double b_x0[10];
    double f0[10];
    double f1[10];
    double sin_psi[2];
    gravity_stiffness[0] = cos_psi[0];
    sin_psi[0] = cos_psi[0];
    gravity_stiffness[1] = cos_psi[1];
    sin_psi[1] = cos_psi[1];
    gravity_stiffness[side] = fmax(0.15, cos_psi[side] - 0.005);
    sin_psi[side] = fmin(0.34, cos_psi[side] + 0.005);
    tinympc14_bal_block(gravity_stiffness[0], gravity_stiffness[1], A0, B0, f0);
    tinympc14_bal_block(sin_psi[0], sin_psi[1], A1, B1, f1);
    for (i = 0; i < 100; i++) {
      A1[i] -= A0[i];
    }
    for (i = 0; i < 10; i++) {
      expl_temp = 0.0;
      for (i1 = 0; i1 < 10; i1++) {
        expl_temp += x0[i1] * A1[i1 + 10 * i];
      }
      b_x0[i] = expl_temp;
    }
    for (i = 0; i < 40; i++) {
      B1[i] -= B0[i];
    }
    heave_scale = sin_psi[side] - gravity_stiffness[side];
    for (i = 0; i < 10; i++) {
      i1 = i << 2;
      length_gain[side + (i << 1)] =
          (((b_x0[i] + (((0.0 * B1[i1] + 0.0 * B1[i1 + 1]) + d * B1[i1 + 2]) +
                        d1 * B1[i1 + 3])) +
            f1[i]) -
           f0[i]) /
          heave_scale;
    }
    k = side << 1;
    b_roll_gain[k] = roll_gain[side];
    b_roll_gain[k + 1] = height_gain[side];
  }
  for (i = 0; i < 2; i++) {
    d = b_roll_gain[i];
    d1 = b_roll_gain[i + 2];
    for (i1 = 0; i1 < 10; i1++) {
      k = i1 << 1;
      side = ((i << 1) + 14 * i1) + 10;
      A[side] += d * length_gain[k] + d1 * length_gain[k + 1];
    }
  }
  tinympc14_fused_dyn(op_A_bal, op_B_bal, op_f_bal, op_M_v, op_M_load, op_K_eff,
                      x0, ue, f);
  for (i = 0; i < 14; i++) {
    d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      d += x0[i1] * A[i1 + 14 * i];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 6; i1++) {
      d1 += ue[i1] * B[i1 + 6 * i];
    }
    f[i] = (f[i] - d) - d1;
  }
  /*  平衡块辅助量（tg0/xoff/theta_star 等）原样带出 */
  return residual;
}

/*
 * File trailer for tinympc14_scheduled_ab.c
 *
 * [EOF]
 */
