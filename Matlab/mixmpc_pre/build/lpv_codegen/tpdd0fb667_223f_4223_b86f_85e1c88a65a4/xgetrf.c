/*
 * File: xgetrf.c
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2026-11-22 05:16:27
 */

/* Include Files */
#include "xgetrf.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : double A[25]
 *                int ipiv[5]
 * Return Type  : int
 */
int xgetrf(double A[25], int ipiv[5])
{
  int i;
  int info;
  int j;
  int jA;
  int jp1j;
  int k;
  for (i = 0; i < 5; i++) {
    ipiv[i] = i + 1;
  }
  info = 0;
  for (j = 0; j < 4; j++) {
    double smax;
    int a;
    int b_tmp;
    int mmj_tmp;
    mmj_tmp = 3 - j;
    b_tmp = j * 6;
    jp1j = b_tmp + 2;
    jA = 5 - j;
    a = 0;
    smax = fabs(A[b_tmp]);
    for (k = 2; k <= jA; k++) {
      double s;
      s = fabs(A[(b_tmp + k) - 1]);
      if (s > smax) {
        a = k - 1;
        smax = s;
      }
    }
    if (A[b_tmp + a] != 0.0) {
      if (a != 0) {
        jA = j + a;
        ipiv[j] = jA + 1;
        for (k = 0; k < 5; k++) {
          a = j + k * 5;
          smax = A[a];
          i = jA + k * 5;
          A[a] = A[i];
          A[i] = smax;
        }
      }
      i = (b_tmp - j) + 5;
      for (jA = jp1j; jA <= i; jA++) {
        A[jA - 1] /= A[b_tmp];
      }
    } else {
      info = j + 1;
    }
    jA = b_tmp;
    for (jp1j = 0; jp1j <= mmj_tmp; jp1j++) {
      smax = A[(b_tmp + jp1j * 5) + 5];
      if (smax != 0.0) {
        i = jA + 7;
        a = (jA - j) + 10;
        for (k = i; k <= a; k++) {
          A[k - 1] += A[((b_tmp + k) - jA) - 6] * -smax;
        }
      }
      jA += 5;
    }
  }
  if ((info == 0) && (!(A[24] != 0.0))) {
    info = 5;
  }
  return info;
}

/*
 * File trailer for xgetrf.c
 *
 * [EOF]
 */
