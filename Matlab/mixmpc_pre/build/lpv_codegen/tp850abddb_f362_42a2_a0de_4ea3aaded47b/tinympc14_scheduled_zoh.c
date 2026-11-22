/*
 * File: tinympc14_scheduled_zoh.c
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2026-11-22 05:14:55
 */

/* Include Files */
#include "tinympc14_scheduled_zoh.h"
#include "rt_nonfinite.h"
#include "tinympc14_scheduled_ab.h"
#include "tinympc14_zoh_embedded.h"

/* Function Definitions */
/*
 * Runtime model entry with constant calibrated parameters and three signals.
 *
 * Arguments    : double left
 *                double right
 *                double psi
 *                double Ad[196]
 *                double Bd[84]
 *                double fd[14]
 * Return Type  : double
 */
double tinympc14_scheduled_zoh(double left, double right, double psi,
                               double Ad[196], double Bd[84], double fd[14])
{
  double A[196];
  double B[84];
  double f[14];
  double terminal_center;
  tinympc14_scheduled_ab(left, right, psi, A, B, f, &terminal_center);
  tinympc14_zoh_embedded(A, B, f, Ad, Bd, fd);
  return terminal_center;
}

/*
 * File trailer for tinympc14_scheduled_zoh.c
 *
 * [EOF]
 */
