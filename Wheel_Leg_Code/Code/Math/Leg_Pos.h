/*
 * File: Leg_Pos.h
 *
 * 001 实际闭环正运动学：输入电机标定角 [rad]；输出髋到轮轴距离 [m]、从 +x 到向下轴的极角 [rad] —— 由 MATLAB Coder 从 001 模型生成。
 */

#ifndef LEG_POS_H
#define LEG_POS_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void Leg_Pos(float phi1, float phi4, float pos[2]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for Leg_Pos.h
 *
 * [EOF]
 */
