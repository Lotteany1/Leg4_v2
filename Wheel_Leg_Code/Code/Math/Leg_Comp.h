/*
 * File: Leg_Comp.h
 *
 * 腿部质心偏移模型：腿长 → 质心偏置角/等效重力力臂（重力补偿） —— 由 MATLAB Symbolic Math Toolbox 生成。
 */

#ifndef LEG_COMP_H
#define LEG_COMP_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void Leg_Comp(float l, float *theta0, float *l_d);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for Leg_Comp.h
 *
 * [EOF]
 */
