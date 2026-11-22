/** @file  slip_side.c  @brief 运动学差速补偿和驱动空转检测  @version  v1.1  2026-11-16 */
#include "slip_side.h"
#include "code_config.h"
#include <math.h>

// 仅消费两轮均更新的配对样本；滞回计时用实际配对间隔。
void Slip_Side_Update(slip_side_detector_t *detector, const slip_side_input_t *input)
{
    slip_side_status_t *status = &detector->status;
    const float turn = SLIP_HALF_TRACK * input->yaw_rate;
    const bool paired = input->frame[0] != detector->frame[0] && input->frame[1] != detector->frame[1];
    const bool finite = isfinite(input->predicted) && isfinite(input->acceleration) &&
                        isfinite(turn) && isfinite(input->velocity[0]) && isfinite(input->velocity[1]);
    const int direction = (input->intent > 0.0f) - (input->intent < 0.0f);
    uint8_t abnormal = 0u;
    uint8_t candidate = 0u;

    // 逆时针左转时左轮在内侧，左速度加上偏航差速，右速度减去。
    status->center[0] = input->velocity[0] + turn;
    status->center[1] = input->velocity[1] - turn;
    status->valid_mask = finite && !input->air ? input->valid_mask : 0u;
    status->usable_mask = status->valid_mask & (uint8_t)~input->action_mask;
    for (unsigned int side = 0u; side < 2u; side++)
    {
        status->residual[side] = status->center[side] - input->predicted;
        if (fabsf(status->residual[side]) >= SLIP_ENTER) abnormal |= (uint8_t)(1u << side);
    }

    if (status->valid_mask != 3u || input->action_mask)
    {
        status->side = SLIP_SIDE_UNKNOWN;
        status->duration[0] = 0u;
        status->duration[1] = 0u;
        detector->suspect = 0u;
        detector->clear_ms = 0u;
        detector->frame[0] = input->frame[0];
        detector->frame[1] = input->frame[1];
        detector->pair_tick = input->now;
        return;
    }

    const bool driving = direction != 0 && input->predicted * direction >= -SLIP_EXIT &&
                         fabsf(input->acceleration) <= SLIP_ACCEL;
    const float split = (status->center[0] - status->center[1]) * direction;
    for (unsigned int side = 0u; side < 2u; side++)
    {
        const unsigned int other = 1u - side;
        const uint8_t bit = (uint8_t)(1u << side);
        const float threshold = detector->suspect & bit ? SLIP_EXIT : SLIP_ENTER;

        // 驱动轮必须沿运动方向超速，另一轮与惯性预测一致；反扭矩是制动样本。
        if (driving && input->propulsion[side] * direction > 0.0f &&
            status->residual[side] * direction >= threshold &&
            fabsf(status->residual[other]) <= SLIP_EXIT &&
            split * (side == 0u ? 1.0f : -1.0f) >= threshold)
            candidate |= bit;
    }

    if (paired)
    {
        const uint32_t age = input->now - detector->pair_tick;
        const uint32_t elapsed = age <= SLIP_FEEDBACK_MS && direction == detector->direction ? age : 0u;

        detector->frame[0] = input->frame[0];
        detector->frame[1] = input->frame[1];
        detector->pair_tick = input->now;
        detector->direction = direction;
        for (unsigned int side = 0u; side < 2u; side++)
        {
            const uint8_t bit = (uint8_t)(1u << side);

            if (!(candidate & bit) || elapsed == 0u)
                status->duration[side] = 0u;
            else if ((detector->suspect & bit) && status->duration[side] < SLIP_ENTER_MS)
                status->duration[side] += elapsed;
        }
        if (candidate)
        {
            detector->suspect = candidate;
            detector->clear_ms = 0u;
        }
        else if (fabsf(status->center[0] - status->center[1]) <= SLIP_EXIT && abnormal == 0u)
        {
            detector->clear_ms += elapsed;
            if (detector->clear_ms >= SLIP_EXIT_MS)
            {
                detector->suspect = 0u;
                detector->clear_ms = SLIP_EXIT_MS;
            }
        }
        else
            detector->clear_ms = 0u;
    }

    const float wheel_split = fabsf(status->center[0] - status->center[1]);
    // 同向加减速与停稳时允许一致的轮速修正共同的惯性预测误差。
    const bool common_dynamic = wheel_split <= SLIP_EXIT &&
        (fabsf(input->acceleration) > SLIP_ACCEL ||
         (direction == 0 && fabsf(status->center[0]) < SLIP_EXIT && fabsf(status->center[1]) < SLIP_EXIT));
    const bool ambiguous = (abnormal == 3u && !common_dynamic) ||
                           (candidate == 0u && wheel_split >= SLIP_ENTER);
    status->usable_mask &= (uint8_t)~(candidate | detector->suspect);
    if (ambiguous)
        status->usable_mask = 0u;

    // 消退滞回也用于动作重新武装；无异常但尚未确认时仍为 UNKNOWN。
    status->side = detector->clear_ms >= SLIP_EXIT_MS ? SLIP_SIDE_NONE : SLIP_SIDE_UNKNOWN;
    if (abnormal == 3u && !common_dynamic)
        status->side = SLIP_SIDE_BOTH;
    else if (direction == detector->direction && candidate == 1u && status->duration[0] >= SLIP_ENTER_MS)
        status->side = SLIP_SIDE_LEFT;
    else if (direction == detector->direction && candidate == 2u && status->duration[1] >= SLIP_ENTER_MS)
        status->side = SLIP_SIDE_RIGHT;
    else if (ambiguous || abnormal || detector->suspect || candidate)
        status->side = SLIP_SIDE_UNKNOWN;
}
