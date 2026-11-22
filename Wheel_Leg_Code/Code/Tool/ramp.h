/**
 * @file  ramp.h
 * @brief 斜坡限速头文件
 * @version  v1.0  2026-09-16
 */
#ifndef RAMP_H
#define RAMP_H

typedef struct
{
    float current;                                 /* 当前斜坡值 单位随调用方 */
    float rate_up;                                 /* 单位/秒，上升最大变化率 */
    float rate_down; /* 单位/秒，下降最大变化率 */ // 最大变化率 (单位/秒)
} ramp_t;

void Ramp_Init(ramp_t *ramp, float rate);                          /* 对称斜率 当前值从 0 起 */
void Ramp_Init_Asym(ramp_t *ramp, float rate_up, float rate_down); /* 非对称斜率 当前值从 0 起 */
void Ramp_Reset(ramp_t *ramp, float current);                      /* 直接设定当前值 不限速 */
float Ramp_Update(ramp_t *ramp, float target, float dt);           /* 每拍逼近 target 返当前值 */

#endif /* RAMP_H */
