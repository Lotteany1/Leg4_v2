/*
 * File: tinympc14_fused_op.h
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2026-11-21 22:02:36
 */

#ifndef TINYMPC14_FUSED_OP_H
#define TINYMPC14_FUSED_OP_H

/* Include Files */
#include "rtwtypes.h"
#include "tinympc14_lpv_kernel_internal_types.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
double tinympc14_fused_op(double sched_H, double sched_lL, double sched_lR,
                          double sched_psi, double op_A_bal[100],
                          double op_B_bal[40], double op_f_bal[10],
                          struct_T *op_aux, double *op_lR, double *op_H,
                          double *op_M_v, double *op_M_load, double *op_K_eff,
                          double *op_psi);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for tinympc14_fused_op.h
 *
 * [EOF]
 */
