/** @file  vel_limit.c  @brief 加速度与加加速度受限的在线速度 S 曲线  @version  v1.2  2026-11-02 */
#include "vel_limit.h"
#include <math.h>
#include <stddef.h>

// 初始静止 不绑定机体状态 后续每拍由调用方提供目标与周期
void Vel_Limit_Init(vel_limit_t *state)
{
    if (state == NULL) return;

    state->vel = 0.0f;
    state->acc = 0.0f;
}

// 重置参考速度并清加速度 仅用于调用方明确要求的状态切换 不作为逐拍反馈校正
void Vel_Limit_Reset(vel_limit_t *state, float velocity)
{
    if (state == NULL || !isfinite(velocity)) return;

    state->vel = velocity;
    state->acc = 0.0f;
}

// 从当前参考状态规划最短时间三段轨迹 再推进 dt 到达终点后保持目标与零加速度
// 限值仅约束参考轨迹 不保证机体实测加速度与停车时间 不对控制器增加倾角或力矩前馈
void Vel_Limit_Update(vel_limit_t *state, float target, float dt)
{
    double error, stop_delta, direction, acc, peak, remaining, vel;
    double duration[3], jerk[3];
    const double acc_max  = VEL_LIMIT_S_CURVE_ACC_MAX;
    const double jerk_max = VEL_LIMIT_S_CURVE_JERK_MAX;

    if (state == NULL || !isfinite(target) || !isfinite(dt) || dt <= 0.0f ||
        !isfinite(state->vel) || !isfinite(state->acc) || fabs(state->acc) > VEL_LIMIT_S_CURVE_ACC_MAX)
        return;

    // 零目标优先制动 立即撤掉仍在提高速度绝对值的加速度 允许此处加速度不连续
    if (target == 0.0f && state->vel * state->acc > 0.0f)
        state->acc = 0.0f;

    // 回零加速度所需速度增量为 a|a|/(2J) 按切换曲面选方向 可覆盖反打与不可避免的越过目标
    // 中间量使用双精度 避免小误差与非零加速度相减时丢失短阶段的时间
    error      = (double)target - state->vel;
    stop_delta = (double)state->acc * fabs((double)state->acc) / (2.0f * jerk_max);
    direction  = (error >= stop_delta) ? 1.0f : -1.0f;
    acc        = direction * state->acc;
    peak       = sqrt(fmax(0.0f, jerk_max * direction * error + 0.5f * acc * acc));

    // 三角形加速度轨迹不足以到达限值时省略平台 否则按剩余速度增量求恒加速度时长
    duration[1] = 0.0f;
    if (peak > acc_max)
    {
        peak        = acc_max;
        duration[1] = (direction * error - (2.0f * peak * peak - acc * acc) / (2.0f * jerk_max)) / peak;
    }
    duration[0] = fmax(0.0f, (peak - acc) / jerk_max);
    duration[2] = peak / jerk_max;
    jerk[0]     = direction * jerk_max;
    jerk[1]     = 0.0f;
    jerk[2]     = -direction * jerk_max;

    remaining = dt;
    if (remaining >= duration[0] + duration[1] + duration[2])
    {
        state->vel = target;
        state->acc = 0.0f;
        return;
    }

    // 跨阶段边界时分段解析积分 不用目标钳位截断加速度 不在零速处额外停顿
    vel = state->vel;
    acc = state->acc;
    for (unsigned int i = 0u; i < 3u && remaining > 0.0f; i++)
    {
        const double step = fmin(remaining, duration[i]);

        vel += acc * step + 0.5f * jerk[i] * step * step;
        acc += jerk[i] * step;
        if (step == duration[i])
            acc = (i == 2u) ? 0.0f : direction * peak;
        remaining -= step;
    }
    // 松杆制动经过零速即结束 不为维持加速度连续而生成反向速度参考
    if (target == 0.0f && vel * state->vel <= 0.0f)
    {
        state->vel = 0.0f;
        state->acc = 0.0f;
    }
    else
    {
        state->vel = vel;
        state->acc = acc;
    }
}
