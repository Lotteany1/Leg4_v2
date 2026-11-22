/*
 * File: Leg_RollComp.h
 *
 * 转弯向心力前馈：入参 L0 等效腿长 [m]、omega yaw 角速度 [rad/s]、v 前进速度 [m/s]; 出参 dF_lft/dF_rgt 为限幅后补偿 [N]，Fc_raw 为限幅前等效力 [N]，模型外部扰动为 [Fc_raw,-Fc_raw] —— 由 MATLAB Coder 从 001 模型生成。
 */

#ifndef LEG_ROLLCOMP_H
#define LEG_ROLLCOMP_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void Leg_RollComp(float H, float omega, float v, float *dF_lft,
                  float *dF_rgt, float *Fc_raw);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for Leg_RollComp.h
 *
 * [EOF]
 */
