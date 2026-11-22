/** @file  turn_limit.c  @brief 转弯驾驶限制与腿长参考协调  @version  v3.5  2026-11-19 */
#include "turn_limit.h"
#include "system_key.h"
#include "code_config.h"
#include <math.h>
#include <stddef.h>

// 遥控与陀螺目标更新后每拍调用一次 按最终旋转指令缩放直行目标 保留前后方向；腿长协调启用时超侧倾同时限制行驶中的旋转目标
void Turn_Limit_Update(body_state_t *state)
{
#if BALANCE_USE_TURN_LIMIT
    float scale;

    if (state == NULL)
        return;
    if (!isfinite(state->vel_tgt) || !isfinite(state->omg_yaw_tgt))
        return;

    // 零旋转保留全部直行量 满量程及以上直行归零 中间按旋转百分比线性衰减
    scale = 1.0f - fabsf(state->omg_yaw_tgt) / TURN_LIMIT_YAW_FULL;
    state->vel_tgt *= fmaxf(scale, 0.0f);
#if LEGLEN_TURN_ENABLE
    // 超过允许侧倾后缩小驾驶需求；只对有转弯指令的场景生效，保留零速原地转向。
    if (isfinite(state->ang_roll) && fabsf(state->omg_yaw_tgt) > 0.01f)
    {
        float reserve = (LEGLEN_TURN_ROLL_STOP - fabsf(state->ang_roll)) /
                        (LEGLEN_TURN_ROLL_STOP - LEGLEN_TURN_ROLL_ALLOW);

        reserve = fminf(1.0f, fmaxf(0.0f, reserve));
        state->vel_tgt *= reserve;
        // 实际仍在行驶时同步限制 yaw 需求，避免仅减速时残留的大 v·omega。
#if BALANCE_USE_SLIP
        const float speed = state->vel_kalman;
#else
        const float speed = state->vel;
#endif
        if (isfinite(speed) && fabsf(speed) > 0.1f) state->omg_yaw_tgt *= reserve;
    }
#endif
#else
    (void)state;
#endif
}

// 调度层已校验测量和采样周期；按名义力余量连续降权，保持状态回滚由调用方负责。
void Turn_Limit_Leglen_Update(const turn_limit_leglen_input_t *input, turn_limit_leglen_t *state)
{
    state->ratio = 0.0f;
    state->roll_keep = 1.0f;
    state->roll_ref = 0.0f;
    state->h_target = input->h_target;
    state->height_priority = false;
#if LEGLEN_TURN_ENABLE
    // 利用率按静态支撑余量估计，不把超界合力当作更多 roll 能力。
#ifdef TINYMPC14_M_LOAD
    const float mean_support = 0.5f * TINYMPC14_M_LOAD * GRAVITY;
#else
    const float mean_support = 0.5f * LEG_MASS_SUPPORT * GRAVITY; // 旧数据包的静态轴力模型
#endif
    const float available = fmaxf(1.0f, fminf(mean_support - input->force_min, input->force_max - mean_support));
    float demand, relax_target, lower_room, drop;

    state->ratio = fabsf(input->turn_force) / available;
    demand = (state->ratio - LEGLEN_TURN_BEGIN) / (LEGLEN_TURN_FULL - LEGLEN_TURN_BEGIN);
    // 下降支路保留一小段利用率回差，稳态在带内保持，防阈值往复切换。
    if (demand < state->relax)
        demand = fminf(state->relax, (state->ratio + LEGLEN_TURN_HYST - LEGLEN_TURN_BEGIN) / (LEGLEN_TURN_FULL - LEGLEN_TURN_BEGIN));
    relax_target = fminf(1.0f, fmaxf(0.0f, demand));
    state->relax += fminf(LEGLEN_TURN_ATTACK * input->dt,
                          fmaxf(-LEGLEN_TURN_RELEASE * input->dt, relax_target - state->relax));
    lower_room = fmaxf(0.0f, fminf(input->len[0], input->len[1]) - Leg_Len_Min - LEGLEN_TURN_RESERVE);
    drop = fminf(LEGLEN_TURN_DROP * state->relax, fminf(lower_room, fmaxf(0.0f, input->h - LEGLEN_TURN_FLOOR)));
    // 以当前高度扣除剩余可降行程，已降到目标后不会把总降高量重复扣掉。
    state->h_target = fminf(input->h_target, fmaxf(input->h_target - LEGLEN_TURN_DROP * state->relax, input->h - drop));
    state->roll_ref = (1.0f - LEGLEN_TURN_ROLL_KEEP) * state->relax *
                      fminf(LEGLEN_TURN_ROLL_ALLOW, fmaxf(-LEGLEN_TURN_ROLL_ALLOW, input->roll));
    state->height_priority = fabsf(input->turn_force) > LEGLEN_TURN_ACTIVE_F || state->relax > 0.0f;
#endif
    state->roll_keep = 1.0f - (1.0f - LEGLEN_TURN_ROLL_KEEP) * state->relax;
}
