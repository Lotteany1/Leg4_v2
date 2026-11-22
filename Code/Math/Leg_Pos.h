/*
 * File: Leg_Pos.h
 *
 * 五连杆正运动学: 入参 (phi1,phi4) 曲柄角 [rad]; 出参 pos(1)=轮轴到髋轴(两髋同轴)的距离 [m] (几何可达 0.04~0.46), pos(2)=腿角, 从 +x 轴量起、+y(=竖直向下)为正 [rad]; 只能标量调用 —— 由 MATLAB Symbolic Math Toolbox 生成。
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
