/*
 * File: Leg_InvTrq.h
 *
 * VMC 逆映射：关节力矩 → 末端力/虚拟力矩 —— 由 MATLAB Symbolic Math Toolbox 生成。
 */

#ifndef LEG_INVTRQ_H
#define LEG_INVTRQ_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void Leg_InvTrq(float tau1, float tau4, float phi1, float phi4, float *F,
                float *Tp);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for Leg_InvTrq.h
 *
 * [EOF]
 */
