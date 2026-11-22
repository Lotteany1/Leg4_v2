/* Compare generated turning compensation and its unclipped equivalent force. */
#include "mex.h"
#include "Leg_RollComp.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    const float *values;
    float *left, *right, *raw;
    mwSize n, row;

    if (nrhs != 1 || nlhs != 3 || !mxIsSingle(prhs[0]) || mxIsComplex(prhs[0]) || mxGetN(prhs[0]) != 3)
    {
        mexErrMsgIdAndTxt("rollcomp:shape", "Expected N x 3 real single [height, yaw, velocity] and three outputs.");
    }
    n = mxGetM(prhs[0]);
    values = (const float *)mxGetData(prhs[0]);
    plhs[0] = mxCreateNumericMatrix(n, 1, mxSINGLE_CLASS, mxREAL);
    plhs[1] = mxCreateNumericMatrix(n, 1, mxSINGLE_CLASS, mxREAL);
    plhs[2] = mxCreateNumericMatrix(n, 1, mxSINGLE_CLASS, mxREAL);
    left = (float *)mxGetData(plhs[0]);
    right = (float *)mxGetData(plhs[1]);
    raw = (float *)mxGetData(plhs[2]);
    for (row = 0; row < n; row++)
    {
        Leg_RollComp(values[row], values[row + n], values[row + 2 * n], &left[row], &right[row], &raw[row]);
    }
}
