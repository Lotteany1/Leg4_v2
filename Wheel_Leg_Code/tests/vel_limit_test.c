/** @file  vel_limit_test.c  @brief 独立速度 S 曲线边界与轨迹验证  @version  v1.1  2026-11-02 */
#include "vel_limit.h"
#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

static unsigned int vel_limit_checks;

// 对实际逐拍输出检查有限性 加速度与 jerk 约束及速度积分范围
static void Vel_Limit_Test_Step(vel_limit_t *state, float target, float dt)
{
    const vel_limit_t prev = *state;
    const float tolerance  = 2.0e-6f;

    Vel_Limit_Update(state, target, dt);
    assert(isfinite(state->vel) && isfinite(state->acc));
    assert(fabs(state->acc) <= VEL_LIMIT_S_CURVE_ACC_MAX + tolerance);
    if (!(target == 0.0f && (prev.vel * prev.acc > 0.0f || (state->vel == 0.0f && state->acc == 0.0f))))
        assert(fabs(state->acc - prev.acc) <= VEL_LIMIT_S_CURVE_JERK_MAX * dt + tolerance);
    assert(fabs(state->vel - prev.vel) <= VEL_LIMIT_S_CURVE_ACC_MAX * dt + tolerance);
    vel_limit_checks++;
}

// 从零加速度出发的独立解析基准 覆盖三角形与带平台轨迹及非整周期终点
static void Vel_Limit_Test_Rest(float start, float target, float dt)
{
    vel_limit_t state;
    const double distance    = fabs((double)target - start);
    const double direction   = (target >= start) ? 1.0f : -1.0f;
    const double peak        = fmin(VEL_LIMIT_S_CURVE_ACC_MAX, sqrt(distance * VEL_LIMIT_S_CURVE_JERK_MAX));
    const double rise        = peak / VEL_LIMIT_S_CURVE_JERK_MAX;
    const double hold        = (peak > 0.0f) ? fmax(0.0f, distance / peak - rise) : 0.0f;
    const double total       = 2.0f * rise + hold;
    const unsigned int count = (unsigned int)ceil(total / dt) + 20u;

    Vel_Limit_Reset(&state, start);
    for (unsigned int i = 1u; i <= count; i++)
    {
        const double time = (double)i * dt;
        double delta, acc;

        if (time < rise)
        {
            delta = 0.5f * VEL_LIMIT_S_CURVE_JERK_MAX * time * time;
            acc   = VEL_LIMIT_S_CURVE_JERK_MAX * time;
        }
        else if (time < rise + hold)
        {
            delta = 0.5f * peak * rise + peak * (time - rise);
            acc   = peak;
        }
        else if (time < total)
        {
            const double remaining = total - time;

            delta = distance - 0.5f * VEL_LIMIT_S_CURVE_JERK_MAX * remaining * remaining;
            acc   = VEL_LIMIT_S_CURVE_JERK_MAX * remaining;
        }
        else
        {
            delta = distance;
            acc   = 0.0f;
        }
        Vel_Limit_Test_Step(&state, target, dt);
        assert(fabs(state.vel - (start + direction * delta)) < 2.0e-4f);
        assert(fabs(state.acc - direction * acc) < 3.0e-4f);
    }
    assert(state.vel == target && state.acc == 0.0f);
}

// 按配置解析计算稳定 1 m/s 松杆时长 验证回零段起点和终点 不固定参数值
static void Vel_Limit_Test_Stop_Time(void)
{
    vel_limit_t state;
    const float peak  = fminf(VEL_LIMIT_S_CURVE_ACC_MAX, sqrtf(VEL_LIMIT_S_CURVE_JERK_MAX));
    const float rise  = peak / VEL_LIMIT_S_CURVE_JERK_MAX;
    const float total = 1.0f / peak + rise;

    Vel_Limit_Reset(&state, 1.0f);
    Vel_Limit_Test_Step(&state, 0.0f, total - rise);
    assert(fabs(state.vel - 0.5f * peak * rise) < 1.0e-6f);
    assert(fabs(state.acc + peak) < 1.0e-6f);
    Vel_Limit_Test_Step(&state, 0.0f, rise + 1.0e-6f);
    assert(state.vel == 0.0f && state.acc == 0.0f);
}

// 非零加速度松杆必须立即制动 反打跨零时不强制加速度归零 变目标后最终收敛
static void Vel_Limit_Test_Retarget(void)
{
    vel_limit_t state    = {.vel = 0.3f, .acc = 0.8f};
    const float prev_vel = state.vel;
    int crossed          = 0;

    Vel_Limit_Test_Step(&state, 0.0f, 0.001f);
    assert(state.vel < prev_vel && state.acc < 0.0f);
    for (unsigned int i = 0u; i < 3000u; i++)
    {
        Vel_Limit_Test_Step(&state, 0.0f, 0.001f);
    }
    assert(state.vel == 0.0f && state.acc == 0.0f);

    Vel_Limit_Reset(&state, 1.0f);
    for (unsigned int i = 0u; i < 4000u; i++)
    {
        const float prev = state.vel;

        Vel_Limit_Test_Step(&state, -1.0f, 0.001f);
        if (prev > 0.0f && state.vel <= 0.0f)
        {
            assert(state.acc < -0.9f);
            crossed = 1;
        }
    }
    assert(crossed && state.vel == -1.0f && state.acc == 0.0f);
}

// 松杆后参考速度绝对值逐拍不增 不反向 最终保持准确零 覆盖加速中与制动中的正反方向
static void Vel_Limit_Test_Release(void)
{
    const float velocities[] = {0.0f, 0.0001f, 0.3f, 1.0f, 2.85f};

    for (unsigned int i = 0u; i < sizeof(velocities) / sizeof(velocities[0]); i++)
    {
        for (int direction = -1; direction <= 1; direction += 2)
        {
            for (int a = -4; a <= 4; a++)
            {
                vel_limit_t state = {
                    .vel = direction * velocities[i],
                    .acc = (double)a * 0.25f * VEL_LIMIT_S_CURVE_ACC_MAX,
                };

                for (unsigned int tick = 0u; tick < 4000u; tick++)
                {
                    const double previous = state.vel;

                    Vel_Limit_Test_Step(&state, 0.0f, 0.001f);
                    assert(fabs(state.vel) <= fabs(previous) + 1.0e-12f);
                    assert(direction * state.vel >= 0.0f);
                }
                assert(state.vel == 0.0f && state.acc == 0.0f);
            }
        }
    }
}

// 变化目标与不规则周期下逐拍检查限值 前后镜像必须对称 最后固定目标检验有限步收敛
static void Vel_Limit_Test_Changing(void)
{
    vel_limit_t fwd, reverse;

    Vel_Limit_Init(&fwd);
    Vel_Limit_Init(&reverse);
    for (unsigned int i = 0u; i < 30000u; i++)
    {
        const float dt     = (i % 3u == 0u) ? 0.0007f : ((i % 3u == 1u) ? 0.001f : 0.0037f);
        const float target = (i < 20000u) ? 2.85f * sinf((float)i * 0.013f) : 0.0f;

        Vel_Limit_Test_Step(&fwd, target, dt);
        Vel_Limit_Test_Step(&reverse, -target, dt);
        assert(fabs(fwd.vel + reverse.vel) < 1.0e-6f);
        assert(fabs(fwd.acc + reverse.acc) < 1.0e-6f);
    }
    assert(fwd.vel == 0.0f && fwd.acc == 0.0f);
}

// 覆盖切换曲面两侧和加速度边界 包括目标已等于当前速度但加速度尚未回零的情况
static void Vel_Limit_Test_Initial_Acceleration(void)
{
    const float errors[] = {-2.0f, -0.126f, -0.125f, -0.124f, -1.0e-6f, 0.0f, 1.0e-6f, 0.124f, 0.125f, 0.126f, 2.0f};

    for (unsigned int i = 0u; i < sizeof(errors) / sizeof(errors[0]); i++)
    {
        for (int a = -4; a <= 4; a++)
        {
            vel_limit_t state = {.vel = 0.0f, .acc = (float)a * 0.25f};

            for (unsigned int tick = 0u; tick < 5000u; tick++)
            {
                Vel_Limit_Test_Step(&state, errors[i], 0.001f);
            }
            assert(state.vel == errors[i] && state.acc == 0.0f);
        }
    }
}

// 非法输入不得部分更新状态 重置仅清加速度 空指针入口允许直接返回
static void Vel_Limit_Test_Invalid(void)
{
    vel_limit_t state        = {.vel = 0.5f, .acc = 0.25f};
    vel_limit_t saved        = state;
    const float invalid[]    = {NAN, INFINITY, -INFINITY};
    const float invalid_dt[] = {0.0f, -0.001f, NAN, INFINITY};

    Vel_Limit_Init(NULL);
    Vel_Limit_Reset(NULL, 0.0f);
    Vel_Limit_Update(NULL, 0.0f, 0.001f);
    for (unsigned int i = 0u; i < sizeof(invalid) / sizeof(invalid[0]); i++)
    {
        Vel_Limit_Update(&state, invalid[i], 0.001f);
        Vel_Limit_Reset(&state, invalid[i]);
        assert(memcmp(&state, &saved, sizeof(state)) == 0);
        state.vel = invalid[i];
        saved     = state;
        Vel_Limit_Update(&state, 0.0f, 0.001f);
        assert(memcmp(&state, &saved, sizeof(state)) == 0);
        state.vel = 0.5f;
        state.acc = invalid[i];
        saved     = state;
        Vel_Limit_Update(&state, 0.0f, 0.001f);
        assert(memcmp(&state, &saved, sizeof(state)) == 0);
        state.acc = 0.25f;
        saved     = state;
    }
    for (unsigned int i = 0u; i < sizeof(invalid_dt) / sizeof(invalid_dt[0]); i++)
    {
        Vel_Limit_Update(&state, 0.0f, invalid_dt[i]);
        assert(memcmp(&state, &saved, sizeof(state)) == 0);
    }
    state.acc = VEL_LIMIT_S_CURVE_ACC_MAX * 1.1f;
    saved     = state;
    Vel_Limit_Update(&state, 0.0f, 0.001f);
    assert(memcmp(&state, &saved, sizeof(state)) == 0);
    Vel_Limit_Reset(&state, -0.75f);
    assert(state.vel == -0.75f && state.acc == 0.0f);
    Vel_Limit_Init(&state);
    assert(state.vel == 0.0f && state.acc == 0.0f);
}

// 独立主机验证 不连接控制任务或硬件
int main(void)
{
    const float targets[] = {0.0f, 1.0e-6f, 0.03f, 0.25f, 1.0f, 2.85f};
    const float periods[] = {0.001f, 0.0037f, 0.2f, 0.31f, 5.0f};

    for (unsigned int i = 0u; i < sizeof(targets) / sizeof(targets[0]); i++)
    {
        for (unsigned int j = 0u; j < sizeof(periods) / sizeof(periods[0]); j++)
        {
            Vel_Limit_Test_Rest(0.0f, targets[i], periods[j]);
            Vel_Limit_Test_Rest(0.0f, -targets[i], periods[j]);
            Vel_Limit_Test_Rest(targets[i], 0.0f, periods[j]);
        }
    }
    Vel_Limit_Test_Stop_Time();
    Vel_Limit_Test_Retarget();
    Vel_Limit_Test_Release();
    Vel_Limit_Test_Changing();
    Vel_Limit_Test_Initial_Acceleration();
    Vel_Limit_Test_Invalid();
    printf("PASS: %u trajectory steps; configured stop profile, release braking, reversal, symmetry and invalid inputs\n", vel_limit_checks);
    return 0;
}
