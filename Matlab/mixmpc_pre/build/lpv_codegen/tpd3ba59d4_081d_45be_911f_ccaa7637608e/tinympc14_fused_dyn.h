/*
 * File: tinympc14_fused_dyn.h
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2026-11-21 21:03:23
 */

#ifndef TINYMPC14_FUSED_DYN_H
#define TINYMPC14_FUSED_DYN_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void tinympc14_fused_dyn(const double op_A_bal[100], const double op_B_bal[40],
                         const double op_f_bal[10], double op_M_v,
                         double op_M_load, double op_K_eff, const double x[14],
                         const double u[6], double xd[14]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for tinympc14_fused_dyn.h
 *
 * [EOF]
 */
