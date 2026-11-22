/*
 * File: tinympc14_terminal_cubic_eval.h
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2026-11-22 05:18:28
 */

#ifndef TINYMPC14_TERMINAL_CUBIC_EVAL_H
#define TINYMPC14_TERMINAL_CUBIC_EVAL_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void tinympc14_terminal_cubic_eval(const double table_length_grid[19],
                                   const double table_angle_axis[33],
                                   double table_angle_scale,
                                   const float table_basis[4536],
                                   const float table_offset[189], double left,
                                   double right, double psi, double P[196],
                                   double K[84]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for tinympc14_terminal_cubic_eval.h
 *
 * [EOF]
 */
