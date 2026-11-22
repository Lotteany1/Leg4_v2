/*
 * File: Leg_CoriolisComp.h
 *
 * 旋转耦合补偿：入参 L0 [m]、omega [rad/s]、pitch/roll [rad]、dpitch/droll [rad/s]; 出参 dT_w/dT_p pitch 通道轮/髋扭矩补偿 [N·m]、dF_lft/dF_rgt roll 通道腿推力补偿 [N] —— 由 MATLAB Coder 从 001 模型生成。
 */

#ifndef LEG_CORIOLISCOMP_H
#define LEG_CORIOLISCOMP_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void Leg_CoriolisComp(float L0, float omega, float pitch, float roll,
                      float dpitch, float droll, float *dT_w,
                             float *dT_p, float *dF_lft, float *dF_rgt);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for Leg_CoriolisComp.h
 *
 * [EOF]
 */
