/*
 * mex_leg_vmc.c -- comparison harness: exposes the C under cmsis/ or codegen/lib/ to MATLAB.
 * Input   N x 6 single : [phi1 phi4 phi_dot_1 phi_dot_4 tau1 tau4]
 * Output  N x 2 single : pos / spd / T / invFT
 *
 * ASCII only on purpose: MSVC assumes the ANSI code page for BOM-less sources,
 * so non-ASCII literals here would break the build.
 */

#include "mex.h"
#include "Leg_Pos.h"
#include "Leg_Spd.h"
#include "Leg_Trq.h"
#include "Leg_InvTrq.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    const mxArray *in;
    const float *p;
    float *outP, *outS, *outT, *outI;
    mwSize n, k;

    if (nrhs != 1) {
        mexErrMsgIdAndTxt("leg_vmc:nrhs", "expects 1 input matrix");
    }
    if (nlhs != 4) {
        mexErrMsgIdAndTxt("leg_vmc:nlhs", "expects 4 outputs");
    }
    in = prhs[0];
    if (!mxIsSingle(in) || mxIsComplex(in) || mxGetN(in) != 6) {
        mexErrMsgIdAndTxt("leg_vmc:in", "input must be N x 6 single real");
    }

    n = mxGetM(in);
    p = (const float *)mxGetData(in);

    plhs[0] = mxCreateNumericMatrix(n, 2, mxSINGLE_CLASS, mxREAL);
    plhs[1] = mxCreateNumericMatrix(n, 2, mxSINGLE_CLASS, mxREAL);
    plhs[2] = mxCreateNumericMatrix(n, 2, mxSINGLE_CLASS, mxREAL);
    plhs[3] = mxCreateNumericMatrix(n, 2, mxSINGLE_CLASS, mxREAL);
    outP = (float *)mxGetData(plhs[0]);
    outS = (float *)mxGetData(plhs[1]);
    outT = (float *)mxGetData(plhs[2]);
    outI = (float *)mxGetData(plhs[3]);

    for (k = 0; k < n; ++k) {
        float phi1 = p[k];
        float phi4 = p[k + n];
        float d1 = p[k + 2 * n];
        float d4 = p[k + 3 * n];
        float tau1 = p[k + 4 * n];
        float tau4 = p[k + 5 * n];
        float pos[2], spd[2], T[2], F, Tp;

        Leg_Pos(phi1, phi4, pos);
        Leg_Spd(phi1, phi4, d1, d4, spd);
        Leg_Trq(tau1, tau4, phi1, phi4, T);
        Leg_InvTrq(tau1, tau4, phi1, phi4, &F, &Tp);

        outP[k] = pos[0];
        outP[k + n] = pos[1];
        outS[k] = spd[0];
        outS[k + n] = spd[1];
        outT[k] = T[0];
        outT[k + n] = T[1];
        outI[k] = F;
        outI[k + n] = Tp;
    }
}
