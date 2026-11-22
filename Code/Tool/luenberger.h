/** @file  luenberger.h  @brief 通用离散龙伯格状态观测器接口  @version  v1.1  2026-10-31 */
#ifndef LUENBERGER_H
#define LUENBERGER_H

#include <stdint.h>

#define LUENBERGER_STATE_MAX  8u // 最大状态维数 静态存储 不使用堆
#define LUENBERGER_INPUT_MAX  4u // 最大控制输入维数
#define LUENBERGER_OUTPUT_MAX 4u // 最大测量输出维数

// 离散模型 x[k+1] = A*x[k] + B*u[k]，y[k] = C*x[k]，不含直接馈通项。
// 矩阵行优先，仅使用有效维数内的元素；A/B/L 必须对应同一个固定采样周期。
// 调用方离散化模型并设计 L，使 A-L*C 的全部极点位于单位圆内；本工具不自动配置极点。
typedef struct
{
    uint8_t state_num;                                         // 状态维数 1～LUENBERGER_STATE_MAX
    uint8_t input_num;                                         // 输入维数 0～LUENBERGER_INPUT_MAX；0 表示无控制输入
    uint8_t output_num;                                        // 测量维数 1～LUENBERGER_OUTPUT_MAX
    float a[LUENBERGER_STATE_MAX][LUENBERGER_STATE_MAX];         // 离散状态矩阵 A[i][j] 单位 x[i]/x[j]
    float b[LUENBERGER_STATE_MAX][LUENBERGER_INPUT_MAX];         // 离散输入矩阵 B[i][j] 单位 x[i]/u[j]
    float c[LUENBERGER_OUTPUT_MAX][LUENBERGER_STATE_MAX];        // 测量矩阵 C[i][j] 单位 y[i]/x[j]
    float l[LUENBERGER_STATE_MAX][LUENBERGER_OUTPUT_MAX];        // 离散观测增益 L[i][j] 单位 x[i]/y[j]
} luenberger_cfg_t;

// 一实例对应一套模型；配置由 Init 复制，初始化后不直接修改维数或矩阵。
typedef struct
{
    luenberger_cfg_t cfg;                    // 模型与增益的实例副本
    float x[LUENBERGER_STATE_MAX];           // 状态估计 各分量单位与坐标系由模型定义
    float y_hat[LUENBERGER_OUTPUT_MAX];      // 本次更新前的测量预测 C*x[k] 与 y 同单位
    float error[LUENBERGER_OUTPUT_MAX];      // 本次更新前的残差 y[k]-C*x[k] 与 y 同单位
} luenberger_t;

// 各接口由调用方保证指针与数组有效，Reset/Update 前必须完成 Init。
// 上电复制 cfg 并清零状态与诊断量；非法维数早退，矩阵系数由调用方保证有限。
void Luenberger_Init(luenberger_t *luenberger, const luenberger_cfg_t *cfg);

// 已初始化实例复位 输入 x0 至少含 state_num 项，单位和坐标系与模型一致。
// 输出状态设为 x0，诊断量清零，保留模型与增益；非有限初值保持原实例。
void Luenberger_Reset(luenberger_t *luenberger, const float *x0);

// 单一任务每个固定采样周期调用一次；输入 u/y 至少含 input_num/output_num 项。
// 调用前 x 为 x_hat[k]，输入 y[k] 为同拍测量，u[k] 为 [k,k+1) 作用的已知输入。
// 输出 x 为 x_hat[k+1]；y_hat/error 保留本拍更新前的预测与残差，不是下一拍测量预测。
// input_num=0 时 u 可为 NULL；非有限输入整拍不更新，漏拍后由调用方重新对齐时序。
// 不做单位换算或坐标变换，跨任务访问由调用方同步；先 Init，需非零初值再 Reset。
void Luenberger_Update(luenberger_t *luenberger, const float *u, const float *y);

#endif /* LUENBERGER_H */
