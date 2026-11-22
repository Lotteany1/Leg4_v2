/*
 * File: mldivide.c
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2026-11-22 05:16:27
 */

/* Include Files */
#include "mldivide.h"
#include "rt_nonfinite.h"
#include "xgetrf.h"

/* Function Definitions */
/*
 * Arguments    : const double A[25]
 *                const double B[20]
 *                double Y[20]
 * Return Type  : void
 */
void b_mldivide(const double A[25], const double B[20], double Y[20])
{
  double b_A[25];
  double X[20];
  double temp;
  int ipiv[5];
  int X_tmp;
  int b_i;
  int i;
  int info;
  int j;
  int jBcol;
  int k;
  int kAcol;
  for (i = 0; i < 4; i++) {
    for (info = 0; info < 5; info++) {
      X[info + 5 * i] = B[i + (info << 2)];
    }
  }
  for (i = 0; i < 5; i++) {
    for (info = 0; info < 5; info++) {
      b_A[info + 5 * i] = A[i + 5 * info];
    }
  }
  xgetrf(b_A, ipiv);
  for (b_i = 0; b_i < 4; b_i++) {
    i = ipiv[b_i];
    if (i != b_i + 1) {
      temp = X[b_i];
      X[b_i] = X[i - 1];
      X[i - 1] = temp;
      temp = X[b_i + 5];
      X[b_i + 5] = X[i + 4];
      X[i + 4] = temp;
      temp = X[b_i + 10];
      X[b_i + 10] = X[i + 9];
      X[i + 9] = temp;
      temp = X[b_i + 15];
      X[b_i + 15] = X[i + 14];
      X[i + 14] = temp;
    }
  }
  for (j = 0; j < 4; j++) {
    jBcol = 5 * j;
    for (k = 0; k < 5; k++) {
      kAcol = 5 * k;
      i = k + jBcol;
      if (X[i] != 0.0) {
        info = k + 2;
        for (b_i = info; b_i < 6; b_i++) {
          X_tmp = (b_i + jBcol) - 1;
          X[X_tmp] -= X[i] * b_A[(b_i + kAcol) - 1];
        }
      }
    }
  }
  for (j = 0; j < 4; j++) {
    jBcol = 5 * j;
    for (k = 4; k >= 0; k--) {
      kAcol = 5 * k;
      i = k + jBcol;
      temp = X[i];
      if (temp != 0.0) {
        X[i] = temp / b_A[k + kAcol];
        for (b_i = 0; b_i < k; b_i++) {
          X_tmp = b_i + jBcol;
          X[X_tmp] -= X[i] * b_A[b_i + kAcol];
        }
      }
    }
  }
  for (i = 0; i < 5; i++) {
    info = i << 2;
    Y[info] = X[i];
    Y[info + 1] = X[i + 5];
    Y[info + 2] = X[i + 10];
    Y[info + 3] = X[i + 15];
  }
}

/*
 * Arguments    : const double A[25]
 *                const double B[15]
 *                double Y[15]
 * Return Type  : void
 */
void mldivide(const double A[25], const double B[15], double Y[15])
{
  double b_A[25];
  double X[15];
  double temp;
  int ipiv[5];
  int X_tmp;
  int b_i;
  int i;
  int info;
  int j;
  int jBcol;
  int k;
  int kAcol;
  for (info = 0; info < 3; info++) {
    for (i = 0; i < 5; i++) {
      X[i + 5 * info] = B[info + 3 * i];
    }
  }
  for (info = 0; info < 5; info++) {
    for (i = 0; i < 5; i++) {
      b_A[i + 5 * info] = A[info + 5 * i];
    }
  }
  xgetrf(b_A, ipiv);
  for (b_i = 0; b_i < 4; b_i++) {
    info = ipiv[b_i];
    if (info != b_i + 1) {
      temp = X[b_i];
      X[b_i] = X[info - 1];
      X[info - 1] = temp;
      temp = X[b_i + 5];
      X[b_i + 5] = X[info + 4];
      X[info + 4] = temp;
      temp = X[b_i + 10];
      X[b_i + 10] = X[info + 9];
      X[info + 9] = temp;
    }
  }
  for (j = 0; j < 3; j++) {
    jBcol = 5 * j;
    for (k = 0; k < 5; k++) {
      kAcol = 5 * k;
      info = k + jBcol;
      if (X[info] != 0.0) {
        i = k + 2;
        for (b_i = i; b_i < 6; b_i++) {
          X_tmp = (b_i + jBcol) - 1;
          X[X_tmp] -= X[info] * b_A[(b_i + kAcol) - 1];
        }
      }
    }
  }
  for (j = 0; j < 3; j++) {
    jBcol = 5 * j;
    for (k = 4; k >= 0; k--) {
      kAcol = 5 * k;
      info = k + jBcol;
      temp = X[info];
      if (temp != 0.0) {
        X[info] = temp / b_A[k + kAcol];
        for (b_i = 0; b_i < k; b_i++) {
          X_tmp = b_i + jBcol;
          X[X_tmp] -= X[info] * b_A[b_i + kAcol];
        }
      }
    }
  }
  for (info = 0; info < 5; info++) {
    Y[3 * info] = X[info];
    Y[3 * info + 1] = X[info + 5];
    Y[3 * info + 2] = X[info + 10];
  }
}

/*
 * File trailer for mldivide.c
 *
 * [EOF]
 */
