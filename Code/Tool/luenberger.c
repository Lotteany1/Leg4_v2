/** @file  luenberger.c  @brief 通用离散龙伯格状态观测器  @version  v1.1  2026-10-31 */
#include "luenberger.h"
#include <math.h>

// 登记模型并清零状态与诊断量；维数仅在初始化时检查。
void Luenberger_Init(luenberger_t *luenberger, const luenberger_cfg_t *cfg)
{
    if (cfg->state_num == 0u || cfg->state_num > LUENBERGER_STATE_MAX) return;
    if (cfg->input_num > LUENBERGER_INPUT_MAX || cfg->output_num == 0u || cfg->output_num > LUENBERGER_OUTPUT_MAX) return;

    *luenberger = (luenberger_t){.cfg = *cfg};
}

// 模式交接时对齐状态初值，完整校验后再写入，避免部分状态已复位。
void Luenberger_Reset(luenberger_t *luenberger, const float *x0)
{
    for (uint32_t i = 0u; i < luenberger->cfg.state_num; i++)
    {
        if (!isfinite(x0[i])) return;
    }

    for (uint32_t i = 0u; i < luenberger->cfg.state_num; i++)
    {
        luenberger->x[i] = x0[i];
    }
    for (uint32_t i = 0u; i < luenberger->cfg.output_num; i++)
    {
        luenberger->y_hat[i] = 0.0f;
        luenberger->error[i] = 0.0f;
    }
}

// 先由旧状态统一计算残差，再计算全部新状态；最后整体交接，避免矩阵乘法被原地更新污染。
void Luenberger_Update(luenberger_t *luenberger, const float *u, const float *y)
{
    const luenberger_cfg_t *cfg = &luenberger->cfg;
    float next[LUENBERGER_STATE_MAX];

    for (uint32_t i = 0u; i < cfg->input_num; i++)
    {
        if (!isfinite(u[i])) return;
    }
    for (uint32_t i = 0u; i < cfg->output_num; i++)
    {
        if (!isfinite(y[i])) return;
    }

    // 测量和估计均属于 k 拍；不能先做 A*x，再拿 y[k] 修正下一拍预测。
    for (uint32_t i = 0u; i < cfg->output_num; i++)
    {
        float predicted = 0.0f;

        for (uint32_t j = 0u; j < cfg->state_num; j++)
        {
            predicted += cfg->c[i][j] * luenberger->x[j];
        }
        luenberger->y_hat[i] = predicted;
        luenberger->error[i] = y[i] - predicted;
    }

    // x_hat[k+1] = A*x_hat[k] + B*u[k] + L*(y[k]-C*x_hat[k])。
    for (uint32_t i = 0u; i < cfg->state_num; i++)
    {
        float value = 0.0f;

        for (uint32_t j = 0u; j < cfg->state_num; j++)
        {
            value += cfg->a[i][j] * luenberger->x[j];
        }
        for (uint32_t j = 0u; j < cfg->input_num; j++)
        {
            value += cfg->b[i][j] * u[j];
        }
        for (uint32_t j = 0u; j < cfg->output_num; j++)
        {
            value += cfg->l[i][j] * luenberger->error[j];
        }
        next[i] = value;
    }
    for (uint32_t i = 0u; i < cfg->state_num; i++)
    {
        luenberger->x[i] = next[i];
    }
}
