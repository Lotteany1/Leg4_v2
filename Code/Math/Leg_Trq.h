/*
 * File: Leg_Trq.h
 *
 * 五连杆力矩映射（虚功原理）: 入参 F 沿 +L0 [N], Tp 沿 +phi0 [N·m]; 出参 tau=[tau1; tau4] 关节力矩 [N·m]; phi1≡phi4 (mod 2π) 奇异线上输出 NaN/Inf; 只能标量调用 —— 由 MATLAB Symbolic Math Toolbox 生成。
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
