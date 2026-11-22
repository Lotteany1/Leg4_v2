/*
 * File: _coder_tinympc14_lpv_kernel_mex.c
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2026-11-20 17:28:57
 */

/* Include Files */
#include "_coder_tinympc14_lpv_kernel_mex.h"
#include "_coder_tinympc14_lpv_kernel_api.h"

/* Function Definitions */
/*
 * Arguments    : int32_T nlhs
 *                mxArray *plhs[]
 *                int32_T nrhs
 *                const mxArray *prhs[]
 * Return Type  : void
 */
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  mexAtExit(&tinympc14_lpv_kernel_atexit);
  /* Module initialization. */
  tinympc14_lpv_kernel_initialize();
  /* Dispatch the entry-point. */
  unsafe_tinympc14_lpv_kernel_mexFunction(nlhs, plhs, nrhs, prhs);
  /* Module termination. */
  tinympc14_lpv_kernel_terminate();
}

/*
 * Arguments    : void
 * Return Type  : emlrtCTX
 */
emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1,
                           NULL, "GBK", true);
  return emlrtRootTLSGlobal;
}

/*
 * Arguments    : int32_T nlhs
 *                mxArray *plhs[7]
 *                int32_T nrhs
 *                const mxArray *prhs[6]
 * Return Type  : void
 */
void unsafe_tinympc14_lpv_kernel_mexFunction(int32_T nlhs, mxArray *plhs[7],
                                             int32_T nrhs,
                                             const mxArray *prhs[6])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs[7];
  const mxArray *b_prhs[6];
  int32_T i;
  int32_T i1;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs < 6) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooFewInputsConstants", 9, 4, 20,
                        "tinympc14_lpv_kernel", 4, 20, "tinympc14_lpv_kernel",
                        4, 20, "tinympc14_lpv_kernel");
  }
  if (nrhs != 6) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 6, 4,
                        20, "tinympc14_lpv_kernel");
  }
  if (nlhs > 7) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 20,
                        "tinympc14_lpv_kernel");
  }
  /* Call the function. */
  for (i = 0; i < 6; i++) {
    b_prhs[i] = prhs[i];
  }
  tinympc14_lpv_kernel_api(b_prhs, nlhs, outputs);
  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    i1 = 1;
  } else {
    i1 = nlhs;
  }
  emlrtReturnArrays(i1, &plhs[0], &outputs[0]);
}

/*
 * File trailer for _coder_tinympc14_lpv_kernel_mex.c
 *
 * [EOF]
 */
