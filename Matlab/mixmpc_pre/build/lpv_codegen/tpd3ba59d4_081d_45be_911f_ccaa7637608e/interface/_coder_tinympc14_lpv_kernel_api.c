/*
 * File: _coder_tinympc14_lpv_kernel_api.c
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2026-11-21 21:03:23
 */

/* Include Files */
#include "_coder_tinympc14_lpv_kernel_api.h"
#include "_coder_tinympc14_lpv_kernel_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;

emlrtContext emlrtContextGlobal = {
    true,                                                 /* bFirstTime */
    false,                                                /* bInitialized */
    131643U,                                              /* fVersionInfo */
    NULL,                                                 /* fErrorFunction */
    "tinympc14_lpv_kernel",                               /* fFunctionName */
    NULL,                                                 /* fRTCallStack */
    false,                                                /* bDebugMode */
    {2045744189U, 2170104910U, 2743257031U, 4284093946U}, /* fSigWrd */
    NULL                                                  /* fSigMem */
};

/* Function Declarations */
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               struct4_T *y);

static const mxArray *b_emlrt_marshallOut(const real_T u[84]);

static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y[19]);

static const mxArray *c_emlrt_marshallOut(const real_T u[14]);

static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y[33]);

static const mxArray *d_emlrt_marshallOut(const real_T u[84]);

static real_T e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId);

static const mxArray *e_emlrt_marshallOut(const real_T u[36]);

static void emlrtExitTimeCleanupDtorFcn(const void *r);

static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                             const char_T *identifier, struct4_T *y);

static const mxArray *emlrt_marshallOut(const real_T u[196]);

static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real32_T y[4536]);

static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real32_T y[189]);

static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                               const char_T *identifier, real_T y[6]);

static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[6]);

static real_T j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                 const char_T *identifier);

static void k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[19]);

static void l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[33]);

static real_T m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

static void n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               real32_T ret[4536]);

static void o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               real32_T ret[189]);

static void p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[6]);

/* Function Definitions */
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                struct4_T *y
 * Return Type  : void
 */
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, struct4_T *y)
{
  static const int32_T dims = 0;
  static const char_T *fieldNames[5] = {"length_grid", "angle_axis",
                                        "angle_scale", "basis", "offset"};
  emlrtMsgIdentifier thisId;
  int32_T i;
  int32_T i1;
  real32_T fv[4536];
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)sp, parentId, u, 5,
                         (const char_T **)&fieldNames[0], 0U,
                         (const void *)&dims);
  thisId.fIdentifier = "length_grid";
  c_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 0,
                                                    "length_grid")),
                     &thisId, y->length_grid);
  thisId.fIdentifier = "angle_axis";
  d_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 1, "angle_axis")),
      &thisId, y->angle_axis);
  thisId.fIdentifier = "angle_scale";
  y->angle_scale =
      e_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0,
                                                        2, "angle_scale")),
                         &thisId);
  thisId.fIdentifier = "basis";
  f_emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 3, "basis")),
      &thisId, fv);
  for (i = 0; i < 24; i++) {
    for (i1 = 0; i1 < 189; i1++) {
      y->basis[i1 + 189 * i] = fv[i + 24 * i1];
    }
  }
  thisId.fIdentifier = "offset";
  g_emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 4, "offset")),
      &thisId, y->offset);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const real_T u[84]
 * Return Type  : const mxArray *
 */
static const mxArray *b_emlrt_marshallOut(const real_T u[84])
{
  static const int32_T iv[2] = {14, 6};
  const mxArray *m;
  const mxArray *y;
  real_T dv[84];
  real_T *pData;
  int32_T b_i;
  int32_T c_i;
  int32_T i;
  y = NULL;
  for (i = 0; i < 6; i++) {
    for (b_i = 0; b_i < 14; b_i++) {
      dv[b_i + 14 * i] = u[i + 6 * b_i];
    }
  }
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  i = 0;
  for (b_i = 0; b_i < 6; b_i++) {
    for (c_i = 0; c_i < 14; c_i++) {
      pData[i + c_i] = dv[c_i + 14 * b_i];
    }
    i += 14;
  }
  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                real_T y[19]
 * Return Type  : void
 */
static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[19])
{
  k_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const real_T u[14]
 * Return Type  : const mxArray *
 */
static const mxArray *c_emlrt_marshallOut(const real_T u[14])
{
  static const int32_T i = 14;
  const mxArray *m;
  const mxArray *y;
  real_T *pData;
  int32_T b_i;
  y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  for (b_i = 0; b_i < 14; b_i++) {
    pData[b_i] = u[b_i];
  }
  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                real_T y[33]
 * Return Type  : void
 */
static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[33])
{
  l_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const real_T u[84]
 * Return Type  : const mxArray *
 */
static const mxArray *d_emlrt_marshallOut(const real_T u[84])
{
  static const int32_T iv[2] = {6, 14};
  const mxArray *m;
  const mxArray *y;
  real_T dv[84];
  real_T *pData;
  int32_T b_i;
  int32_T c_i;
  int32_T i;
  y = NULL;
  for (i = 0; i < 14; i++) {
    for (b_i = 0; b_i < 6; b_i++) {
      dv[b_i + 6 * i] = u[i + 14 * b_i];
    }
  }
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  i = 0;
  for (b_i = 0; b_i < 14; b_i++) {
    for (c_i = 0; c_i < 6; c_i++) {
      pData[i + c_i] = dv[c_i + 6 * b_i];
    }
    i += 6;
  }
  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T
 */
static real_T e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = m_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const real_T u[36]
 * Return Type  : const mxArray *
 */
static const mxArray *e_emlrt_marshallOut(const real_T u[36])
{
  static const int32_T iv[2] = {6, 6};
  const mxArray *m;
  const mxArray *y;
  real_T dv[36];
  real_T *pData;
  int32_T b_i;
  int32_T c_i;
  int32_T i;
  y = NULL;
  for (i = 0; i < 6; i++) {
    for (b_i = 0; b_i < 6; b_i++) {
      dv[b_i + 6 * i] = u[i + 6 * b_i];
    }
  }
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  i = 0;
  for (b_i = 0; b_i < 6; b_i++) {
    for (c_i = 0; c_i < 6; c_i++) {
      pData[i + c_i] = dv[c_i + 6 * b_i];
    }
    i += 6;
  }
  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : const void *r
 * Return Type  : void
 */
static void emlrtExitTimeCleanupDtorFcn(const void *r)
{
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *nullptr
 *                const char_T *identifier
 *                struct4_T *y
 * Return Type  : void
 */
static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                             const char_T *identifier, struct4_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  b_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId, y);
  emlrtDestroyArray(&nullptr);
}

/*
 * Arguments    : const real_T u[196]
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(const real_T u[196])
{
  static const int32_T iv[2] = {14, 14};
  const mxArray *m;
  const mxArray *y;
  real_T dv[196];
  real_T *pData;
  int32_T b_i;
  int32_T c_i;
  int32_T i;
  y = NULL;
  for (i = 0; i < 14; i++) {
    for (b_i = 0; b_i < 14; b_i++) {
      dv[b_i + 14 * i] = u[i + 14 * b_i];
    }
  }
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  i = 0;
  for (b_i = 0; b_i < 14; b_i++) {
    for (c_i = 0; c_i < 14; c_i++) {
      pData[i + c_i] = dv[c_i + 14 * b_i];
    }
    i += 14;
  }
  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                real32_T y[4536]
 * Return Type  : void
 */
static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real32_T y[4536])
{
  n_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                real32_T y[189]
 * Return Type  : void
 */
static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real32_T y[189])
{
  o_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *nullptr
 *                const char_T *identifier
 *                real_T y[6]
 * Return Type  : void
 */
static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                               const char_T *identifier, real_T y[6])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  i_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId, y);
  emlrtDestroyArray(&nullptr);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                real_T y[6]
 * Return Type  : void
 */
static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[6])
{
  p_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *nullptr
 *                const char_T *identifier
 * Return Type  : real_T
 */
static real_T j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                 const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real_T y;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = e_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                real_T ret[19]
 * Return Type  : void
 */
static void k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[19])
{
  static const int32_T dims[2] = {1, 19};
  real_T(*r)[19];
  int32_T i;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 2U,
                          (const void *)&dims[0]);
  r = (real_T(*)[19])emlrtMxGetData(src);
  for (i = 0; i < 19; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                real_T ret[33]
 * Return Type  : void
 */
static void l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[33])
{
  static const int32_T dims[2] = {1, 33};
  real_T(*r)[33];
  int32_T i;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 2U,
                          (const void *)&dims[0]);
  r = (real_T(*)[33])emlrtMxGetData(src);
  for (i = 0; i < 33; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T
 */
static real_T m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  real_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 0U,
                          (const void *)&dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                real32_T ret[4536]
 * Return Type  : void
 */
static void n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               real32_T ret[4536])
{
  static const int32_T dims[2] = {24, 189};
  int32_T i;
  real32_T(*r)[4536];
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "single", false, 2U,
                          (const void *)&dims[0]);
  r = (real32_T(*)[4536])emlrtMxGetData(src);
  for (i = 0; i < 4536; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                real32_T ret[189]
 * Return Type  : void
 */
static void o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               real32_T ret[189])
{
  static const int32_T dims[2] = {1, 189};
  int32_T i;
  real32_T(*r)[189];
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "single", false, 2U,
                          (const void *)&dims[0]);
  r = (real32_T(*)[189])emlrtMxGetData(src);
  for (i = 0; i < 189; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                real_T ret[6]
 * Return Type  : void
 */
static void p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[6])
{
  static const int32_T dims[2] = {1, 6};
  real_T(*r)[6];
  int32_T i;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 2U,
                          (const void *)&dims[0]);
  r = (real_T(*)[6])emlrtMxGetData(src);
  for (i = 0; i < 6; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const mxArray * const prhs[6]
 *                int32_T nlhs
 *                const mxArray *plhs[7]
 * Return Type  : void
 */
void tinympc14_lpv_kernel_api(const mxArray *const prhs[6], int32_T nlhs,
                              const mxArray *plhs[7])
{
  static const uint32_T uv[4] = {3826155107U, 284138046U, 1512279112U,
                                 4069831677U};
  static const char_T *s = "p";
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  struct4_T table;
  real_T Ad[196];
  real_T At[196];
  real_T P[196];
  real_T Bd[84];
  real_T K[84];
  real_T Qu[36];
  real_T fd[14];
  real_T rdiag[6];
  real_T left;
  real_T psi;
  real_T right;
  int32_T i;
  st.tls = emlrtRootTLSGlobal;
  /* Check constant function inputs */
  i = 4;
  emlrtCheckArrayChecksumR2018b(&st, prhs[0], false, &i, (const char_T **)&s,
                                &uv[0]);
  /* Marshall function inputs */
  emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "table", &table);
  h_emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "rdiag", rdiag);
  left = j_emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "left");
  right = j_emlrt_marshallIn(&st, emlrtAliasP(prhs[4]), "right");
  psi = j_emlrt_marshallIn(&st, emlrtAliasP(prhs[5]), "psi");
  /* Invoke the target function */
  tinympc14_lpv_kernel(&table, rdiag, left, right, psi, Ad, Bd, fd, K, P, Qu,
                       At);
  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(Ad);
  if (nlhs > 1) {
    plhs[1] = b_emlrt_marshallOut(Bd);
  }
  if (nlhs > 2) {
    plhs[2] = c_emlrt_marshallOut(fd);
  }
  if (nlhs > 3) {
    plhs[3] = d_emlrt_marshallOut(K);
  }
  if (nlhs > 4) {
    plhs[4] = emlrt_marshallOut(P);
  }
  if (nlhs > 5) {
    plhs[5] = e_emlrt_marshallOut(Qu);
  }
  if (nlhs > 6) {
    plhs[6] = emlrt_marshallOut(At);
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void tinympc14_lpv_kernel_atexit(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtPushHeapReferenceStackR2021a(
      &st, false, NULL, (void *)&emlrtExitTimeCleanupDtorFcn, NULL, NULL, NULL);
  emlrtEnterRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  tinympc14_lpv_kernel_xil_terminate();
  tinympc14_lpv_kernel_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void tinympc14_lpv_kernel_initialize(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, NULL);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void tinympc14_lpv_kernel_terminate(void)
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * File trailer for _coder_tinympc14_lpv_kernel_api.c
 *
 * [EOF]
 */
