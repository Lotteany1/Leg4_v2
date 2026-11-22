/*
 * File: _coder_tinympc14_lpv_kernel_mex.h
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2026-11-21 21:03:23
 */

#ifndef _CODER_TINYMPC14_LPV_KERNEL_MEX_H
#define _CODER_TINYMPC14_LPV_KERNEL_MEX_H

/* Include Files */
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[],
                                     int32_T nrhs, const mxArray *prhs[]);

emlrtCTX mexFunctionCreateRootTLS(void);

void unsafe_tinympc14_lpv_kernel_mexFunction(int32_T nlhs, mxArray *plhs[7],
                                             int32_T nrhs,
                                             const mxArray *prhs[6]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_tinympc14_lpv_kernel_mex.h
 *
 * [EOF]
 */
