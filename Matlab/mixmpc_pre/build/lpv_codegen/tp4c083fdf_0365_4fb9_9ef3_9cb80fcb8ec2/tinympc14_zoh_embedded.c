/*
 * File: tinympc14_zoh_embedded.c
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2026-11-22 05:18:28
 */

/* Include Files */
#include "tinympc14_zoh_embedded.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Fixed-order ZOH candidate for the 2 ms firmware scheduler.
 *  The top rows of the augmented series satisfy a left multiplication by A.
 *
 * Arguments    : const double A[196]
 *                const double B[84]
 *                const double f[14]
 *                double Ad[196]
 *                double Bd[84]
 *                double fd[14]
 * Return Type  : void
 */
void tinympc14_zoh_embedded(const double A[196], const double B[84],
                            const double f[14], double Ad[196], double Bd[84],
                            double fd[14])
{
  double delta[294];
  double term[294];
  double Astep[196];
  int i;
  int i1;
  int i2;
  int k;
  for (i = 0; i < 196; i++) {
    Astep[i] = 0.002 * A[i];
  }
  for (i = 0; i < 14; i++) {
    for (i1 = 0; i1 < 14; i1++) {
      term[i1 + 21 * i] = 0.002 * A[i1 + 14 * i];
    }
    for (i1 = 0; i1 < 6; i1++) {
      term[(i1 + 21 * i) + 14] = 0.002 * B[i1 + 6 * i];
    }
    term[21 * i + 20] = 0.002 * f[i];
  }
  memcpy(&delta[0], &term[0], 294U * sizeof(double));
  for (k = 0; k < 4; k++) {
    double b_term[294];
    double d;
    for (i = 0; i < 21; i++) {
      for (i1 = 0; i1 < 14; i1++) {
        d = 0.0;
        for (i2 = 0; i2 < 14; i2++) {
          d += term[i + 21 * i2] * Astep[i2 + 14 * i1];
        }
        b_term[i + 21 * i1] = d / ((double)k + 2.0);
      }
    }
    for (i = 0; i < 294; i++) {
      d = b_term[i];
      term[i] = d;
      delta[i] += d;
    }
  }
  memset(&Ad[0], 0, 196U * sizeof(double));
  for (k = 0; k < 14; k++) {
    Ad[k + 14 * k] = 1.0;
  }
  for (i = 0; i < 14; i++) {
    for (i1 = 0; i1 < 14; i1++) {
      k = i1 + 14 * i;
      Ad[k] += delta[i1 + 21 * i];
    }
    for (i1 = 0; i1 < 6; i1++) {
      Bd[i1 + 6 * i] = delta[(i1 + 21 * i) + 14];
    }
    fd[i] = delta[21 * i + 20];
  }
}

/*
 * File trailer for tinympc14_zoh_embedded.c
 *
 * [EOF]
 */
