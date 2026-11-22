/*
 * File: _coder_tinympc14_lpv_kernel_api.h
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2026-11-22 05:16:27
 */

#ifndef _CODER_TINYMPC14_LPV_KERNEL_API_H
#define _CODER_TINYMPC14_LPV_KERNEL_API_H

/* Include Files */
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#include <string.h>

/* Type Definitions */
#ifndef typedef_struct4_T
#define typedef_struct4_T
typedef struct {
  real_T length_grid[19];
  real_T angle_axis[33];
  real_T angle_scale;
  real32_T basis[4536];
  real32_T offset[189];
} struct4_T;
#endif /* typedef_struct4_T */

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void tinympc14_lpv_kernel(struct4_T *table, real_T rdiag[6], real_T left,
                          real_T right, real_T psi, real_T Ad[196],
                          real_T Bd[84], real_T fd[14], real_T K[84],
                          real_T P[196], real_T Qu[36], real_T At[196]);

void tinympc14_lpv_kernel_api(const mxArray *const prhs[6], int32_T nlhs,
                              const mxArray *plhs[7]);

void tinympc14_lpv_kernel_atexit(void);

void tinympc14_lpv_kernel_initialize(void);

void tinympc14_lpv_kernel_terminate(void);

void tinympc14_lpv_kernel_xil_shutdown(void);

void tinympc14_lpv_kernel_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_tinympc14_lpv_kernel_api.h
 *
 * [EOF]
 */
