/*
 * File: tinympc14_terminal_cubic_eval.c
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2026-11-21 21:03:23
 */

/* Include Files */
#include "tinympc14_terminal_cubic_eval.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Interpolate log-Cholesky coordinates, never the entries of the cost matrix.
 *  psi is relative to the local equilibrium lean. The asinh axis resolves the
 *  narrow gain transition there without changing the physical model or costs.
 *
 * Arguments    : const double table_length_grid[19]
 *                const double table_angle_axis[33]
 *                double table_angle_scale
 *                const float table_basis[4536]
 *                const float table_offset[189]
 *                double left
 *                double right
 *                double psi
 *                double P[196]
 *                double K[84]
 * Return Type  : void
 */
void tinympc14_terminal_cubic_eval(const double table_length_grid[19],
                                   const double table_angle_axis[33],
                                   double table_angle_scale,
                                   const float table_basis[4536],
                                   const float table_offset[189], double left,
                                   double right, double psi, double P[196],
                                   double K[84])
{
  static const double y[196] = {
      0.001, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0,
      0.0,   0.0, 0.0, 0.001, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0,
      0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.001, 0.0, 0.0, 0.0,   0.0, 0.0,
      0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.001, 0.0, 0.0,
      0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0,
      0.001, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0,
      0.0,   0.0, 0.0, 0.001, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0,
      0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.001, 0.0, 0.0, 0.0,   0.0, 0.0,
      0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.001, 0.0, 0.0,
      0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0,
      0.001, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0,
      0.0,   0.0, 0.0, 0.001, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0,
      0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.001, 0.0, 0.0, 0.0,   0.0, 0.0,
      0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.001, 0.0, 0.0,
      0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0,
      0.001, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0,
      0.0,   0.0, 0.0, 0.001};
  double L[196];
  double factor[189];
  double coords[24];
  double wl[4];
  double wp[4];
  double wr[4];
  double position;
  double position_tmp;
  int c;
  int element;
  int i;
  int il;
  int ip;
  int j;
  int k;
  short iv[24];
  boolean_T xneg;
  position_tmp = table_length_grid[1] - table_length_grid[0];
  position =
      fmin(fmax((left - table_length_grid[0]) / position_tmp, 0.0), 18.0);
  il = (int)fmin(fmax(floor(position), 1.0), 16.0);
  position -= (double)il - 1.0;
  wl[0] = -(position - 1.0) * (position - 2.0) * (position - 3.0) / 6.0;
  wl[1] = position * (position - 2.0) * (position - 3.0) / 2.0;
  wl[2] = -position * (position - 1.0) * (position - 3.0) / 2.0;
  wl[3] = position * (position - 1.0) * (position - 2.0) / 6.0;
  position =
      fmin(fmax((right - table_length_grid[0]) / position_tmp, 0.0), 18.0);
  element = (int)fmin(fmax(floor(position), 1.0), 16.0);
  position -= (double)element - 1.0;
  wr[0] = -(position - 1.0) * (position - 2.0) * (position - 3.0) / 6.0;
  wr[1] = position * (position - 2.0) * (position - 3.0) / 2.0;
  wr[2] = -position * (position - 1.0) * (position - 3.0) / 2.0;
  wr[3] = position * (position - 1.0) * (position - 2.0) / 6.0;
  position_tmp = psi / table_angle_scale;
  xneg = (position_tmp < 0.0);
  if (xneg) {
    position_tmp = -position_tmp;
  }
  if (position_tmp >= 2.68435456E+8) {
    position_tmp = log(position_tmp) + 0.69314718055994529;
  } else if (position_tmp > 2.0) {
    position_tmp =
        log(2.0 * position_tmp +
            1.0 / (sqrt(position_tmp * position_tmp + 1.0) + position_tmp));
  } else {
    position = position_tmp * position_tmp;
    position_tmp += position / (sqrt(position + 1.0) + 1.0);
    position = fabs(position_tmp);
    if ((position > 4.503599627370496E+15) ||
        (rtIsInf(position_tmp) || rtIsNaN(position_tmp))) {
      position_tmp++;
      position_tmp = log(position_tmp);
    } else if (!(position < 2.2204460492503131E-16)) {
      position_tmp = log(position_tmp + 1.0) *
                     (position_tmp / ((position_tmp + 1.0) - 1.0));
    }
  }
  if (xneg) {
    position_tmp = -position_tmp;
  }
  position = fmin(fmax((position_tmp - table_angle_axis[0]) /
                           (table_angle_axis[1] - table_angle_axis[0]),
                       0.0),
                  32.0);
  ip = (int)fmin(fmax(floor(position), 1.0), 30.0);
  position -= (double)ip - 1.0;
  wp[0] = -(position - 1.0) * (position - 2.0) * (position - 3.0) / 6.0;
  wp[1] = position * (position - 2.0) * (position - 3.0) / 2.0;
  wp[2] = -position * (position - 1.0) * (position - 3.0) / 2.0;
  wp[3] = position * (position - 1.0) * (position - 2.0) / 6.0;
  memset(&coords[0], 0, 24U * sizeof(double));
  for (k = 0; k < 4; k++) {
    for (j = 0; j < 4; j++) {
      for (i = 0; i < 4; i++) {
        position = wl[i] * wr[j] * wp[k];
        /*  Deployment reads just this node from the generated cost unit. */
        /*  Coder specializes this branch for a structure without coords. */
        tinympc14_terminal_decode_node(
            (unsigned int)((((il + i) + 19 * ((element + j) - 1)) +
                            361 * ((ip + k) - 1)) -
                           1),
            &iv[0]);
        for (c = 0; c < 24; c++) {
          coords[c] += position * (double)iv[c];
        }
      }
    }
  }
  for (ip = 0; ip < 189; ip++) {
    position = 0.0;
    for (element = 0; element < 24; element++) {
      position += table_basis[ip + 189 * element] * coords[element];
    }
    factor[ip] = position + table_offset[ip];
  }
  memset(&L[0], 0, 196U * sizeof(double));
  element = 0;
  for (j = 0; j < 14; j++) {
    ip = 13 - j;
    for (i = 0; i <= ip; i++) {
      L[j + 14 * (j + i)] = factor[element + i];
    }
    element = (element - j) + 14;
    il = j + 14 * j;
    L[il] = exp(L[il]);
  }
  for (ip = 0; ip < 14; ip++) {
    for (element = 0; element < 14; element++) {
      position = 0.0;
      for (il = 0; il < 14; il++) {
        position += L[il + 14 * ip] * L[il + 14 * element];
      }
      il = ip + 14 * element;
      P[il] = position + y[il];
    }
  }
  /*  The fallback gain is interpolated directly. Recovering it from a rounded,
   */
  /*  approximate P amplifies small cost errors in weakly weighted directions.
   */
  ip = 0;
  element = 0;
  for (il = 0; il < 84; il++) {
    K[element + 14 * ip] = factor[il + 105];
    ip++;
    if (ip > 5) {
      ip = 0;
      element++;
    }
  }
}

/*
 * File trailer for tinympc14_terminal_cubic_eval.c
 *
 * [EOF]
 */
