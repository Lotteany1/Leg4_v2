/** @file  rls.h  @brief 带遗忘因子的递推最小二乘在线辨识接口  @version  v1.0  2026-10-30 */
#ifndef RLS_H
#define RLS_H

#include <stdint.h>

#define RLS_PARAM_MAX 4u // 最大参数数目 静态存储 不使用堆

// 线性参数模型 y = phi^T theta；输入可有物理单位，theta[i] 单位为 y/phi[i]。
// 例如摩擦模型由调用方构造 phi = [1, F, v, sign(v)]，近零速换向处理也由调用方负责。
// 参数与协方差使用双精度减小递推舍入误差；P 是 RLS 逆信息矩阵，不是物理误差的直接测量。
typedef struct
{
    uint8_t num;                              // 有效参数数目 1～RLS_PARAM_MAX 初始化后不直接修改
    float lambda;                             // 每次有效更新的遗忘因子 (0, 1]；1 表示不遗忘
    float p0;                                 // P 初始对角值 >0 复位复用 特征宜先归一化
    double theta[RLS_PARAM_MAX];               // 在线参数估计 仅前 num 项有效
    double p[RLS_PARAM_MAX][RLS_PARAM_MAX];     // 逆信息矩阵 行优先 对称存储
    double y_hat;                             // 本次样本更新前的预测 phi^T theta 与 y 同单位
    double error;                             // 本次样本更新前的残差 y - y_hat 与 y 同单位
} rls_t;

// 上电先初始化 参数与诊断量归零 P=p0*I 非法配置保持原状态。
// lambda 按更新次数衰减，不按墙钟时间衰减；0.999 在 100 Hz 下的有效记忆约为 10 s。
void Rls_Init(rls_t *rls, uint8_t num, float lambda, float p0);

// 已初始化实例复位 保留 num/lambda/p0 参数估计归零 P 恢复初值。
void Rls_Reset(rls_t *rls);

// 单一任务每个有效样本调用一次 phi 至少含 num 项 y 为同拍同坐标系观测。
// 非有限输入整拍不更新；缺少激励、饱和或数据不可信时由调用方跳过调用，状态原样保持。
// 不自动构造观测量、不判断参数已收敛、不输出控制补偿；跨任务访问由调用方同步。
void Rls_Update(rls_t *rls, const float *phi, float y);

#endif /* RLS_H */
