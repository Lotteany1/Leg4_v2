/**
 * @file  leso.h
 * @brief 预测注入型扩张状态观测器对外接口
 * @version  v0.2  2026-10-15
 */
#ifndef LESO_H
#define LESO_H

#include <stdint.h>

typedef struct
{
    float b;      /* 与 ed 同口径，扰动估计 */
    float l;      /* 修正增益 (1 - pole) / ed 每次观测现算 遥测用 */
    float pole;   /* 期望误差极点 取值 (0, 1) */
    float b_max;  /* 与 ed 同口径，扰动估计限幅 对称 */
    float pred;   /* 与 ed 同口径，含扰动项的本拍预测 修正用 */
    uint8_t hold; /* 1 首拍只存预测不修正 */
} leso_t;

void Leso_Init(leso_t *leso, float pole, float b_max); /* 上电一次 极点与限幅登记 估计清零 */                                      // 上电一次 极点与限幅登记 估计清零
void Leso_Reset(leso_t *leso); /* 模式交接与冻结结束 估计保持 首拍只对齐不修正 */                                                  // 模式交接与冻结结束 估计保持 首拍只对齐不修正
void Leso_Update(leso_t *leso, float ed, float pred, float y); /* 一拍观测 ed 随调度模型现算 pred 下拍预测不含扰动项 y 本拍实测 */ // 一拍观测 ed 随调度模型现算 pred 下拍预测不含扰动项 y 本拍实测
float Leso_B(const leso_t *leso); /* 只读遥测 扰动估计 */                                                                          // 只读遥测 扰动估计

#endif /* LESO_H */
