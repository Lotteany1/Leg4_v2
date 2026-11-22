/*
 * File: tinympc14_lpv_kernel.h
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2026-11-22 05:18:28
 */

#ifndef TINYMPC14_LPV_KERNEL_H
#define TINYMPC14_LPV_KERNEL_H

/* Include Files */
#include "rtwtypes.h"
#include "tinympc14_lpv_kernel_types.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void tinympc14_lpv_kernel(const struct4_T *table, const double rdiag[6],
                                 double left, double right, double psi,
                                 double Ad[196], double Bd[84], double fd[14],
                                 double K[84], double P[196], double Qu[36],
                                 double At[196]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for tinympc14_lpv_kernel.h
 *
 * [EOF]
 */
