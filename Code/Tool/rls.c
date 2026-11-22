/** @file  rls.c  @brief 带遗忘因子的递推最小二乘在线辨识  @version  v1.0  2026-10-30 */
#include "rls.h"
#include <math.h>
#include <stddef.h>

// 登记维数与遗忘因子 清参数和诊断量 全部矩阵存储初始化以便复用实例。
void Rls_Init(rls_t *rls, uint8_t num, float lambda, float p0)
{
    if (rls == NULL || num == 0u || num > RLS_PARAM_MAX) return;
    if (!isfinite(lambda) || !isfinite(p0) || lambda <= 0.0f || lambda > 1.0f || p0 <= 0.0f) return;

    *rls = (rls_t){.num = num, .lambda = lambda, .p0 = p0};
    for (uint32_t i = 0u; i < num; i++)
    {
        rls->p[i][i] = p0;
    }
}

// 用原配置重新初始化 重新学习时不沿用旧参数或旧信息矩阵。
void Rls_Reset(rls_t *rls)
{
    if (rls == NULL) return;

    Rls_Init(rls, rls->num, rls->lambda, rls->p0);
}

// 先由旧 theta/P 计算预测和增益 再更新参数与矩阵；不充分激励时不保证参数可辨识。
void Rls_Update(rls_t *rls, const float *phi, float y)
{
    if (rls == NULL || phi == NULL || !isfinite(y)) return;
    if (rls->num == 0u || rls->num > RLS_PARAM_MAX) return;
    for (uint32_t i = 0u; i < rls->num; i++)
    {
        if (!isfinite(phi[i])) return;
    }

    double p_phi[RLS_PARAM_MAX] = {0.0};
    double y_hat                = 0.0;
    double denominator          = rls->lambda;

    // p_phi 必须全部由更新前的 P 计算，避免原地更新污染后续行。
    for (uint32_t i = 0u; i < rls->num; i++)
    {
        y_hat += rls->theta[i] * phi[i];
        for (uint32_t j = 0u; j < rls->num; j++)
        {
            p_phi[i] += rls->p[i][j] * phi[j];
        }
        denominator += phi[i] * p_phi[i];
    }
    rls->y_hat = y_hat;
    rls->error = (double)y - y_hat;

    // K=P*phi/(lambda+phi^T*P*phi)，theta+=K*error。
    for (uint32_t i = 0u; i < rls->num; i++)
    {
        rls->theta[i] += p_phi[i] / denominator * rls->error;
    }

    // P=(P-K*phi^T*P)/lambda；只计算上三角后镜像，保持矩阵对称。
    for (uint32_t i = 0u; i < rls->num; i++)
    {
        for (uint32_t j = i; j < rls->num; j++)
        {
            double value = (rls->p[i][j] - p_phi[i] / denominator * p_phi[j]) / rls->lambda;

            rls->p[i][j] = value;
            rls->p[j][i] = value;
        }
    }
}
