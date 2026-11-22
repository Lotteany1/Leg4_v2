/*
 * File: Leg_Trq.h
 *
 * 001 实际闭环虚功映射：轴力 [N]、极角力矩 [N·m] 到电机力矩 [N·m] —— 由 MATLAB Coder 从 001 模型生成。
 */

#ifndef LEG_TRQ_H
#define LEG_TRQ_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void Leg_Trq(float F, float Tp, float phi1, float phi4, float T[2]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for Leg_Trq.h
 *
 * [EOF]
 */
