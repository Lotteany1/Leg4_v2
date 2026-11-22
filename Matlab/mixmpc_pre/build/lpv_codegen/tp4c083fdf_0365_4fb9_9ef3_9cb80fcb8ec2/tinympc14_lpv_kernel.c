/*
 * File: tinympc14_lpv_kernel.c
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2026-11-22 05:18:28
 */

/* Include Files */
#include "tinympc14_lpv_kernel.h"
#include "rt_nonfinite.h"
#include "tinympc14_lpv_kernel_types.h"
#include "tinympc14_scheduled_zoh.h"
#include "tinympc14_terminal_cubic_eval.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Keep weights and terminal coefficients as read-only data, outside arithmetic.
 *
 * Arguments    : const struct4_T *table
 *                const double rdiag[6]
 *                double left
 *                double right
 *                double psi
 *                double Ad[196]
 *                double Bd[84]
 *                double fd[14]
 *                double K[84]
 *                double P[196]
 *                double Qu[36]
 *                double At[196]
 * Return Type  : void
 */
void tinympc14_lpv_kernel(const struct4_T *table, const double rdiag[6],
                          double left, double right, double psi, double Ad[196],
                          double Bd[84], double fd[14], double K[84],
                          double P[196], double Qu[36], double At[196])
{
  static const double y[36] = {
      0.001, 0.0, 0.0,   0.0, 0.0,   0.0, 0.0, 0.001, 0.0, 0.0,   0.0, 0.0,
      0.0,   0.0, 0.001, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.001, 0.0, 0.0,
      0.0,   0.0, 0.0,   0.0, 0.001, 0.0, 0.0, 0.0,   0.0, 0.0,   0.0, 0.001};
  static const signed char X[36] = {1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                                    0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                                    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1};
  double b_P[84];
  double A[36];
  double d[36];
  double smax;
  int A_tmp;
  int b_i;
  int i;
  int j;
  int jA;
  int jBcol;
  int jp1j;
  int k;
  signed char ipiv[6];
  /*  Shared MATLAB/C scheduler. Round the model before deriving its
   * auxiliaries, */
  /*  matching the float matrices consumed by the deployed Riccati recursion. */
  smax = tinympc14_scheduled_zoh(left, right, psi, Ad, Bd, fd);
  for (i = 0; i < 196; i++) {
    Ad[i] = (float)Ad[i];
  }
  for (i = 0; i < 84; i++) {
    Bd[i] = (float)Bd[i];
  }
  for (i = 0; i < 14; i++) {
    fd[i] = (float)fd[i];
  }
  tinympc14_terminal_cubic_eval(table->length_grid, table->angle_axis,
                                table->angle_scale, table->basis, table->offset,
                                left, right, psi - smax, P, K);
  for (i = 0; i < 196; i++) {
    P[i] = (float)P[i];
  }
  for (i = 0; i < 84; i++) {
    K[i] = (float)K[i];
  }
  memset(&d[0], 0, 36U * sizeof(double));
  for (j = 0; j < 6; j++) {
    d[j + 6 * j] = rdiag[j];
  }
  for (i = 0; i < 14; i++) {
    for (jp1j = 0; jp1j < 6; jp1j++) {
      smax = 0.0;
      for (jA = 0; jA < 14; jA++) {
        smax += P[i + 14 * jA] * Bd[jp1j + 6 * jA];
      }
      b_P[i + 14 * jp1j] = smax;
    }
  }
  for (i = 0; i < 6; i++) {
    for (jp1j = 0; jp1j < 6; jp1j++) {
      smax = 0.0;
      for (jA = 0; jA < 14; jA++) {
        smax += Bd[jp1j + 6 * jA] * b_P[jA + 14 * i];
      }
      A_tmp = jp1j + 6 * i;
      A[i + 6 * jp1j] = (d[A_tmp] + y[A_tmp]) + smax;
    }
    ipiv[i] = (signed char)(i + 1);
  }
  for (j = 0; j < 5; j++) {
    int b_tmp;
    int mmj_tmp;
    mmj_tmp = 4 - j;
    b_tmp = j * 7;
    jp1j = b_tmp + 2;
    jA = 6 - j;
    jBcol = 0;
    smax = fabs(A[b_tmp]);
    for (k = 2; k <= jA; k++) {
      double s;
      s = fabs(A[(b_tmp + k) - 1]);
      if (s > smax) {
        jBcol = k - 1;
        smax = s;
      }
    }
    if (A[b_tmp + jBcol] != 0.0) {
      if (jBcol != 0) {
        jA = j + jBcol;
        ipiv[j] = (signed char)(jA + 1);
        for (k = 0; k < 6; k++) {
          jBcol = j + k * 6;
          smax = A[jBcol];
          A_tmp = jA + k * 6;
          A[jBcol] = A[A_tmp];
          A[A_tmp] = smax;
        }
      }
      i = (b_tmp - j) + 6;
      for (b_i = jp1j; b_i <= i; b_i++) {
        A[b_i - 1] /= A[b_tmp];
      }
    }
    jA = b_tmp;
    for (jBcol = 0; jBcol <= mmj_tmp; jBcol++) {
      smax = A[(b_tmp + jBcol * 6) + 6];
      if (smax != 0.0) {
        i = jA + 8;
        jp1j = (jA - j) + 12;
        for (A_tmp = i; A_tmp <= jp1j; A_tmp++) {
          A[A_tmp - 1] += A[((b_tmp + A_tmp) - jA) - 7] * -smax;
        }
      }
      jA += 6;
    }
  }
  for (i = 0; i < 36; i++) {
    d[i] = X[i];
  }
  for (b_i = 0; b_i < 5; b_i++) {
    signed char i1;
    i1 = ipiv[b_i];
    if (i1 != b_i + 1) {
      for (j = 0; j < 6; j++) {
        jBcol = b_i + 6 * j;
        jA = (int)d[jBcol];
        A_tmp = (i1 + 6 * j) - 1;
        d[jBcol] = d[A_tmp];
        d[A_tmp] = jA;
      }
    }
  }
  for (j = 0; j < 6; j++) {
    jBcol = 6 * j;
    for (k = 0; k < 6; k++) {
      jA = 6 * k;
      i = k + jBcol;
      if (d[i] != 0.0) {
        jp1j = k + 2;
        for (b_i = jp1j; b_i < 7; b_i++) {
          A_tmp = (b_i + jBcol) - 1;
          d[A_tmp] -= d[i] * A[(b_i + jA) - 1];
        }
      }
    }
  }
  for (j = 0; j < 6; j++) {
    jBcol = 6 * j;
    for (k = 5; k >= 0; k--) {
      jA = 6 * k;
      i = k + jBcol;
      smax = d[i];
      if (smax != 0.0) {
        d[i] = smax / A[k + jA];
        for (b_i = 0; b_i < k; b_i++) {
          A_tmp = b_i + jBcol;
          d[A_tmp] -= d[i] * A[b_i + jA];
        }
      }
    }
  }
  for (i = 0; i < 6; i++) {
    for (jp1j = 0; jp1j < 6; jp1j++) {
      Qu[jp1j + 6 * i] = (float)d[i + 6 * jp1j];
    }
  }
  for (i = 0; i < 14; i++) {
    for (jp1j = 0; jp1j < 14; jp1j++) {
      smax = 0.0;
      for (jA = 0; jA < 6; jA++) {
        smax += K[jp1j + 14 * jA] * Bd[jA + 6 * i];
      }
      At[i + 14 * jp1j] = (float)(Ad[jp1j + 14 * i] - smax);
    }
  }
}

/*
 * File trailer for tinympc14_lpv_kernel.c
 *
 * [EOF]
 */
