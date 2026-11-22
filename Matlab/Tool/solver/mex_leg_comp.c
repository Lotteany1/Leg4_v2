/*
 * mex_leg_comp.c -- comparison harness: exposes Leg_Comp.c under
 * cmsis/ or codegen/lib/ to MATLAB.
 * Input   N x 1 single : l [m]
 * Output  N x 1 single : theta0 [rad]  (leg CoM offset angle)
 * Output  N x 1 single : l_d    [m]    (leg CoM lever arm to the pitch axis)
 *
 * Separate from mex_leg_vmc.c on purpose: that one has the fixed 4-function
 * interface of the VMC chain, this one carries the single-input/two-output
 * Leg_Comp lookup table. ASCII only on purpose: MSVC assumes the ANSI code
 * page for BOM-less sources, so non-ASCII literals here would break the build.
 */

#include "mex.h"
#include "Leg_Comp.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    const mxArray *in;
    const float *p;
    float *outT, *outL;
    mwSize n, k;

    if (nrhs != 1) {
        mexErrMsgIdAndTxt("leg_comp:nrhs", "expects 1 input matrix");
    }
    if (nlhs != 2) {
        mexErrMsgIdAndTxt("leg_comp:nlhs", "expects 2 outputs");
    }
    in = prhs[0];
    if (!mxIsSingle(in) || mxIsComplex(in) || mxGetN(in) != 1) {
        mexErrMsgIdAndTxt("leg_comp:in", "input must be N x 1 single real");
    }

    n = mxGetM(in);
    p = (const float *)mxGetData(in);

    plhs[0] = mxCreateNumericMatrix(n, 1, mxSINGLE_CLASS, mxREAL);
    plhs[1] = mxCreateNumericMatrix(n, 1, mxSINGLE_CLASS, mxREAL);
    outT = (float *)mxGetData(plhs[0]);
    outL = (float *)mxGetData(plhs[1]);

    for (k = 0; k < n; ++k) {
        Leg_Comp(p[k], &outT[k], &outL[k]);
    }
}
