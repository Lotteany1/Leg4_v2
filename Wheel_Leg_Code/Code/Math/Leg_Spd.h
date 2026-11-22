/*
 * File: Leg_Spd.h
 *
 * 001 实际闭环解析雅可比：输入电机角速度 [rad/s]，输出腿长速度 [m/s]、极角速度 [rad/s] —— 由 MATLAB Coder 从 001 模型生成。
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
