/** @file  vel_limit.h  @brief 速度参考 S 曲线生成接口  @version  v1.2  2026-11-02 */
#ifndef VEL_LIMIT_H
#define VEL_LIMIT_H

#include "code_config.h"

// 速度目标
// 速度 S 曲线由加速度爬升、恒加速度、加速度回零三段组成 小速度变化时省略恒加速度段
// 两项必须为正 前后与加减速共用 仅约束速度参考 不代表机体实测加速度
#define VEL_LIMIT_S_CURVE_ACC_MAX  24.0f // m/s² S 曲线最大速度斜率 越小加减速越慢
#define VEL_LIMIT_S_CURVE_JERK_MAX 20.0f // m/s³ S 曲线加速度变化率 越小起止弯曲段越柔和

// 从零加速度爬升至上限的时间为 ACC_MAX / JERK_MAX 小速度变化可能在达到上限前进入回零段
// 稳定 1 m/s 到零参考约需 1 s 大速度变化的中段仍为直线 非零目标变化时允许必要的越过目标
// 零目标优先制动 立即撤掉原方向加速分量 参考到零即停 这两处允许加速度不连续

// 速度参考状态 由本模块推进 调用方读取输出 不代表机体实测速度或加速度
typedef struct
{
    double vel; // m/s  轴向速度参考 双精度保存逐拍积分 避免终点量化摆动
    double acc; // m/s² 速度参考的当前加速度 正方向与调用方目标一致
} vel_limit_t;

// state 为输出 初始化速度与加速度为零 无硬件及 RTOS 副作用
void Vel_Limit_Init(vel_limit_t *state);
// state 为输出 velocity 为同轴向速度 m/s 状态切换时主动调用 加速度归零 本次重置不受斜坡约束
void Vel_Limit_Reset(vel_limit_t *state, float velocity);
// state 为输入输出 target 为同轴向目标 m/s dt 为本拍周期 s 正常每控制拍仅调用一次
// 非零目标保持速度及加速度连续 零目标按上述制动规则处理 非法输入保持状态
void Vel_Limit_Update(vel_limit_t *state, float target, float dt);

#endif
