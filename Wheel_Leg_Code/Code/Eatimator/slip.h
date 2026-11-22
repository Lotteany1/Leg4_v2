// 打滑融合头文件
// 版本：v1.3  日期：2026-10-05
#ifndef SLIP_H
#define SLIP_H

#include <stdint.h>
#include <stdbool.h>
#include "slip_side.h"

// 速度融合与短时失去观测
#define SLIP_FUSION_RAMP_MS   80u   // ms，可信度恢复斜坡
#define SLIP_PREDICT_MAX_MS   120u  // ms，已武装接地融合连续缺观测预算
#define SLIP_AIR_SLIP_HOLD_MS  20u  // ms，短时保留已检出的滑移，覆盖撞击力估计波动

#define SLIP_KF_Q_A     1250.0f // 每采样加速度方差 q_a·dt²=0.005 保留旧离散标定
#define SLIP_KF_Q_B     1.0e-3f // 偏置随机游走
#define SLIP_KF_R       0.01f   // 观测噪声 (m/s)²
#define SLIP_KF_GATE    3.0f    // 野值门限 (σ 倍数)，超限降权不丢弃 (kalman.h)
#define SLIP_KF_V_MAX   3.0f    // 速度钳位 (m/s)
#define SLIP_BIAS_CLAMP 2.0f    // a_bias 夹紧 ±(m/s²)

typedef struct
{
    //0. 遥测量
    float   diff;        // 轮速 − 融合速度 (m/s) 纯遥测
    float   vel_fused;   // 融合速度 (m/s) 与 body.vel_kalman 同源
    float   a_lon;       // 去重力纵向加速度 (m/s²)
    uint8_t air;         // 外部离地开关电平镜像
} slip_status_t;

//! 打滑遥测实例
extern volatile slip_status_t slip_status;

void Slip_Init(void);

// 外部离地开关设置函数
//! 1=离地：停吃轮速修正
void Slip_Set_Air(uint8_t air);

void Slip_Update(void);
void Slip_Air_Update(void); // 同拍顺序：支持力、惯性预测、滑移请求、AIR 判定、速度修正

// 控制拍在握手窗口内交接驾驶请求、接纳轮矩和动作侧；估计拍仅读取。
void Slip_Set_Step(bool follow, float intent, const float propulsion[2], uint8_t action_mask, bool recovering);
void Slip_Side_Status(slip_side_status_t *out); // 控制任务在估计握手窗口读取
uint8_t Slip_Observation(void);                // 位 0/1 实际融合来源，0 为仅预测
bool Slip_Prediction_Expired(void);            // 已武装接地融合超过无观测预算

#endif /* SLIP_H */
