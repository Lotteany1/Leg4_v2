/** @file  distributor_reference.c  @brief 主机数值对照的标量数组适配  @version  v1.2  2026-11-02 */
#include "distributor.h"
#include "Leg_Trq.h"

// 只暴露实际编译参数；独立优化目标和 KKT 方程仍由 Python 构建。
void Test_Distributor_Parameters(float output[7])
{
    output[0] = DISTRIBUTOR_S_SCALE;
    output[1] = DISTRIBUTOR_P_SCALE;
    output[2] = DISTRIBUTOR_S_WEIGHT;
    output[3] = DISTRIBUTOR_P_WEIGHT;
    output[4] = DISTRIBUTOR_S_MIN;
    output[5] = DISTRIBUTOR_S_MAX;
    output[6] = DISTRIBUTOR_S_RESERVE;
}

// 每腿共用同一组输入，仅为外部 float64 参照暴露生产数学实现。
void Test_Distributor(const float data[12], int strategy, float output[8])
{
    distributor_input_t input = {.normal_mode = true, .strategy = (distributor_strategy_t)strategy, .dt = 0.001f};
    distributor_result_t result;

    for (unsigned int side = 0u; side < 2u; side++)
    {
        distributor_leg_t *leg = &input.leg[side];
        for (unsigned int i = 0u; i < 4u; i++) leg->map[i / 2u][i % 2u] = data[i];
        leg->s_req    = data[4];
        leg->p_req    = data[5];
        leg->spring_f = data[6];
        leg->hip_comp = data[7];
        for (unsigned int i = 0u; i < 2u; i++)
        {
            leg->tau_lo[i] = data[8u + i];
            leg->tau_hi[i] = data[10u + i];
        }
    }
    Distributor_Calculate(&input, &result);
    output[0] = (float)result.result_valid;
    output[1] = (float)result.status;
    output[2] = result.s_alloc[0];
    output[3] = result.p_alloc[0];
    output[4] = result.tau_alloc[0];
    output[5] = result.tau_alloc[1];
    output[6] = result.objective[0];
    output[7] = result.max_residual;
}
