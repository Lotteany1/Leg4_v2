/* Compare Leg_SpringF for a length column or N-by-2 [length, angle]. */
#include "mex.h"
#include "Leg_SpringF.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    const float *input;
    float *out;
    mwSize n, k;
    int poses;

    if (nlhs != 1 || nrhs != 1 || !mxIsSingle(prhs[0]) || mxIsComplex(prhs[0]))
        mexErrMsgIdAndTxt("leg_springf:input", "Expected one real single matrix.");

    poses = mxGetN(prhs[0]) == 2;
    if (!poses && mxGetN(prhs[0]) != 1)
        mexErrMsgIdAndTxt("leg_springf:shape", "Expected a length column or a N-by-2 pose matrix.");

    n = mxGetM(prhs[0]);
    plhs[0] = mxCreateNumericMatrix(n, 1, mxSINGLE_CLASS, mxREAL);
    input = (const float *)mxGetData(prhs[0]);
    out = (float *)mxGetData(plhs[0]);
    for (k = 0; k < n; ++k) {
        out[k] = poses ? Leg_SpringF(input[k], input[n+k]) : Leg_SpringF(input[k], 0.0f);
    }
}
