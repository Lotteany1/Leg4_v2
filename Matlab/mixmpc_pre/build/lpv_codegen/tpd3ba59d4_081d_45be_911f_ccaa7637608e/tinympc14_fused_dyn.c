/*
 * File: tinympc14_fused_dyn.c
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2026-11-21 21:03:23
 */

/* Include Files */
#include "tinympc14_fused_dyn.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * 14 态降阶动力学。前十态沿用冻结腿长的平衡块，高度和横滚使用同一闭链虚功。
 *  op 由 tinympc14_fused_op 提供（lL/lR 与本点平衡块）；psi 取 x(5)/x(7)
 *
 * Arguments    : const double op_A_bal[100]
 *                const double op_B_bal[40]
 *                const double op_f_bal[10]
 *                double op_M_v
 *                double op_M_load
 *                double op_K_eff
 *                const double x[14]
 *                const double u[6]
 *                double xd[14]
 * Return Type  : void
 */
void tinympc14_fused_dyn(const double op_A_bal[100], const double op_B_bal[40],
                         const double op_f_bal[10], double op_M_v,
                         double op_M_load, double op_K_eff, const double x[14],
                         const double u[6], double xd[14])
{
  double d;
  int i;
  int xd_tmp;
  memset(&xd[0], 0, 14U * sizeof(double));
  for (i = 0; i < 7; i++) {
    xd_tmp = i << 1;
    xd[xd_tmp] = x[xd_tmp + 1];
  }
  for (i = 0; i < 5; i++) {
    d = 0.0;
    for (xd_tmp = 0; xd_tmp < 10; xd_tmp++) {
      d += x[xd_tmp] * op_A_bal[xd_tmp + 10 * ((i << 1) + 1)];
    }
    int i1;
    xd_tmp = (i << 1) + 1;
    i1 = xd_tmp << 2;
    xd[xd_tmp] = (d + (((u[0] * op_B_bal[i1] + u[1] * op_B_bal[i1 + 1]) +
                        u[2] * op_B_bal[i1 + 2]) +
                       u[3] * op_B_bal[i1 + 3])) +
                 op_f_bal[xd_tmp];
  }
  double b_u;
  double b_x;
  double height_arm_idx_0;
  double height_arm_idx_1;
  double load;
  double sec_phi;
  double wheel_force;
  double wheel_force_idx_0;
  double wheel_force_idx_1;
  double x_tmp;
  load = op_M_load * 9.81 + op_K_eff * x[12];
  /*  双轮平地闭链 L_i=(H±b sin(phi))/(cos(phi)cos(psi_i))。 */
  /*  固定两腿矢状角的虚位移 dL_i=height_arm_i*dH+roll_arm_i*dphi。 */
  /*  轴力与 +L 共轭，乘上述两臂分别得到虚升降力和虚横滚力矩。 */
  sec_phi = 1.0 / cos(x[10]);
  height_arm_idx_0 = sec_phi / cos(x[4]);
  height_arm_idx_1 = sec_phi / cos(x[6]);
  x_tmp = tan(x[10]);
  b_x = (x[12] + 0.27) * x_tmp;
  /*  名义竖直承重 load、两腿等分时的几何倾覆矩。其合成效果已由 */
  /*  tinympc14_roll_force 给定，内部仅保留偏离该预载引起的力矩变化。 */
  /*  固定髋部前向位置和腿角，轮心水平位移为 -sin(psi_i)*dL_i。 */
  /*  模型正轮矩与电机轮矩反号，故等价轴力为 +Tw_i*sin(psi_i)/R_w。 */
  wheel_force_idx_0 = u[0] * sin(x[4]) / 0.0525;
  wheel_force_idx_1 = u[1] * sin(x[6]) / 0.0525;
  d = height_arm_idx_0 * (b_x + 0.1846999978917 * sec_phi);
  b_u = u[4] * d;
  wheel_force = wheel_force_idx_0 * d;
  d = height_arm_idx_1 * (b_x + -0.1846999978917 * sec_phi);
  b_u += u[5] * d;
  wheel_force += wheel_force_idx_1 * d;
  xd[13] = ((((u[4] * height_arm_idx_0 + u[5] * height_arm_idx_1) - load) -
             8.0 * x[13]) +
            (wheel_force_idx_0 * height_arm_idx_0 +
             wheel_force_idx_1 * height_arm_idx_1)) /
           op_M_v;
  xd[11] = 0.0 * x[10] + ((b_u - load * (x[12] + 0.27) * x_tmp) + wheel_force) /
                             0.3170176491397;
}

/*
 * File trailer for tinympc14_fused_dyn.c
 *
 * [EOF]
 */
