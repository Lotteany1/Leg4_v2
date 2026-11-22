/** @file  notch.c  @brief 二阶对称陷波滤波器  @version  v1.0  2026-10-29 */
#include "notch.h"

// 首拍以输入填充历史，随后按二阶差分方程更新并返回输出
float Notch_Update(notch_state_t *state, const notch_coeff_t *coeff, float input)
{
    float output;

    if (!state->init)
    {
        state->x1 = state->x2 = input;
        state->y1 = state->y2 = input;
        state->init = 1u;
    }
    output = coeff->b0 * input + coeff->b1 * state->x1 + coeff->b0 * state->x2 -
             coeff->b1 * state->y1 - coeff->a2 * state->y2;
    state->x2 = state->x1;
    state->x1 = input;
    state->y2 = state->y1;
    state->y1 = output;
    return output;
}

// 保留历史值并清除标志，下一次更新重新以输入填充历史
void Notch_Reset(notch_state_t *state)
{
    state->init = 0u;
}
