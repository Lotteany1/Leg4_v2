/*
 * File: main.c
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2026-11-22 05:14:55
 */

/*************************************************************************/
/* This automatically generated example C main file shows how to call    */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

/* Include Files */
#include "main.h"
#include "rt_nonfinite.h"
#include "tinympc14_lpv_kernel.h"
#include "tinympc14_lpv_kernel_terminate.h"
#include "tinympc14_lpv_kernel_types.h"

/* Function Declarations */
static void argInit_1x189_real32_T(float result[189]);

static void argInit_1x19_real_T(double result[19]);

static void argInit_1x33_real_T(double result[33]);

static void argInit_1x6_real_T(double result[6]);

static void argInit_24x189_real32_T(float result[4536]);

static float argInit_real32_T(void);

static double argInit_real_T(void);

static void argInit_struct4_T(struct4_T *result);

/* Function Definitions */
/*
 * Arguments    : float result[189]
 * Return Type  : void
 */
static void argInit_1x189_real32_T(float result[189])
{
  int idx1;
  /* Loop over the array to initialize each element. */
  for (idx1 = 0; idx1 < 189; idx1++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx1] = argInit_real32_T();
  }
}

/*
 * Arguments    : double result[19]
 * Return Type  : void
 */
static void argInit_1x19_real_T(double result[19])
{
  int idx1;
  /* Loop over the array to initialize each element. */
  for (idx1 = 0; idx1 < 19; idx1++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx1] = argInit_real_T();
  }
}

/*
 * Arguments    : double result[33]
 * Return Type  : void
 */
static void argInit_1x33_real_T(double result[33])
{
  int idx1;
  /* Loop over the array to initialize each element. */
  for (idx1 = 0; idx1 < 33; idx1++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx1] = argInit_real_T();
  }
}

/*
 * Arguments    : double result[6]
 * Return Type  : void
 */
static void argInit_1x6_real_T(double result[6])
{
  int idx1;
  /* Loop over the array to initialize each element. */
  for (idx1 = 0; idx1 < 6; idx1++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx1] = argInit_real_T();
  }
}

/*
 * Arguments    : float result[4536]
 * Return Type  : void
 */
static void argInit_24x189_real32_T(float result[4536])
{
  int i;
  /* Loop over the array to initialize each element. */
  for (i = 0; i < 4536; i++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[i] = argInit_real32_T();
  }
}

/*
 * Arguments    : void
 * Return Type  : float
 */
static float argInit_real32_T(void)
{
  return 0.0F;
}

/*
 * Arguments    : void
 * Return Type  : double
 */
static double argInit_real_T(void)
{
  return 0.0;
}

/*
 * Arguments    : struct4_T *result
 * Return Type  : void
 */
static void argInit_struct4_T(struct4_T *result)
{
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  argInit_1x19_real_T(result->length_grid);
  argInit_1x33_real_T(result->angle_axis);
  result->angle_scale = argInit_real_T();
  argInit_24x189_real32_T(result->basis);
  argInit_1x189_real32_T(result->offset);
}

/*
 * Arguments    : int argc
 *                char **argv
 * Return Type  : int
 */
int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;
  /* The initialize function is being called automatically from your entry-point
   * function. So, a call to initialize is not included here. */
  /* Invoke the entry-point functions.
You can call entry-point functions multiple times. */
  main_tinympc14_lpv_kernel();
  /* Terminate the application.
You do not need to do this more than one time. */
  tinympc14_lpv_kernel_terminate();
  return 0;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void main_tinympc14_lpv_kernel(void)
{
  struct4_T r;
  double Ad[196];
  double At[196];
  double P[196];
  double Bd[84];
  double K[84];
  double Qu[36];
  double fd[14];
  double b_dv[6];
  double left_tmp;
  /* Initialize function 'tinympc14_lpv_kernel' input arguments. */
  /* Initialize function input argument 'table'. */
  /* Initialize function input argument 'rdiag'. */
  left_tmp = argInit_real_T();
  /* Call the entry-point 'tinympc14_lpv_kernel'. */
  argInit_struct4_T(&r);
  argInit_1x6_real_T(b_dv);
  tinympc14_lpv_kernel(&r, b_dv, left_tmp, left_tmp, left_tmp, Ad, Bd, fd, K, P,
                       Qu, At);
}

/*
 * File trailer for main.c
 *
 * [EOF]
 */
