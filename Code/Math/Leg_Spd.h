/*
 * File: Leg_Spd.h
 *
 * 五连杆雅可比速度映射: 入参 phi_dot_1/phi_dot_4 为关节角速度 [rad/s]; 出参 spd=[L0_dot; phi0_dot]; phi1≡phi4 (mod 2π) 奇异线上输出 NaN; 只能标量调用 —— 由 MATLAB Symbolic Math Toolbox 生成。
 */

#ifndef LEG_SPD_H
#define LEG_SPD_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void Leg_Spd(float phi1, float phi4, float phi_dot_1, float phi_dot_4,
             float spd[2]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for Leg_Spd.h
 *
 * [EOF]
 */
