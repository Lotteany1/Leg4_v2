/** @file  app_spring_cal.c  @brief 悬空单腿从满伸开始缓慢收腿并标记稳速数据  @version  v1.0  2026-11-22 */
#include "app_spring_cal.h"

#if SPRING_CAL_BENCH_ENABLE
#include <math.h>

#define SPRING_CAL_SPEED_BAND      0.0025f // m/s，达到最大加载或松载斜率所需的速度误差
#define SPRING_CAL_READY_SPEED     0.004f  // m/s，满伸就绪的最大绝对速度
#define SPRING_CAL_SAMPLE_MIN_MPS  0.00025f // m/s，窗口内可分辨的平均压缩速度下限
#define SPRING_CAL_SAMPLE_MAX_MPS  0.015f  // m/s，准静态连续压缩的最高速度
#define SPRING_CAL_STEADY_MS       250u    // ms，按编码器长度变化确认慢速压缩，排除原地静摩擦
#define SPRING_CAL_FRAME_GAP_MS    40u     // ms，控制停顿或成对反馈中断则撤力

// 只在台架显式开始一条新行程时调用。
void App_Spring_Cal_Reset(spring_cal_t *s)
{
    *s = (spring_cal_t){.phase = SPRING_CAL_IDLE};
}

// 锁存首次停止原因。同拍稍后的安全退出或接纳错误不会擦去此前的结果。
void App_Spring_Cal_Stop(spring_cal_t *s, spring_cal_reason_t reason)
{
    if (s->phase == SPRING_CAL_DONE || s->phase == SPRING_CAL_STOPPED) return;
    if (reason == SPRING_CAL_REASON_END)
        s->phase = SPRING_CAL_DONE;
    else
        s->phase = SPRING_CAL_STOPPED;
    s->reason        = reason;
    s->compression   = 0.0f;
    s->motor_command = 0.0f;
    s->sample_ok     = false;
}

// 每控制拍先检查实测边界，再更新直送 VMC 的电机轴力；成对反馈决定稳速采样资格。
void App_Spring_Cal_Update(spring_cal_t *s, const spring_cal_input_t *in)
{
    if (s->phase == SPRING_CAL_DONE || s->phase == SPRING_CAL_STOPPED) return;
    if (!in->valid || !(SPRING_CAL_FINE_RATE > 0.0f) || !isfinite(SPRING_CAL_FINE_RATE) || !(SPRING_CAL_FORCE_RATE > 0.0f) || !isfinite(SPRING_CAL_FORCE_RATE) ||
        !isfinite(in->length) || !isfinite(in->rate) || !isfinite(in->motor_force) ||
        !isfinite(in->min_length) || !isfinite(in->max_length) || in->min_length <= 0.0f ||
        in->max_length - in->min_length <= 2.0f * SPRING_CAL_SAMPLE_MARGIN_M)
    {
        App_Spring_Cal_Stop(s, SPRING_CAL_REASON_FEEDBACK);
        return;
    }
    if (in->limited)
    {
        App_Spring_Cal_Stop(s, SPRING_CAL_REASON_LIMIT);
        return;
    }
    if (fabsf(in->motor_force) >= SPRING_CAL_FORCE_MAX_N)
    {
        App_Spring_Cal_Stop(s, SPRING_CAL_REASON_FORCE);
        return;
    }
    if (s->phase == SPRING_CAL_IDLE)
    {
        if (in->length < in->max_length - SPRING_CAL_START_SHORT_TOL_M ||
            in->length > in->max_length + SPRING_CAL_START_TOL_M ||
            fabsf(in->rate) > SPRING_CAL_READY_SPEED)
        {
            App_Spring_Cal_Stop(s, SPRING_CAL_REASON_START);
            return;
        }
        s->phase           = SPRING_CAL_RUNNING;
        s->last_ms         = in->tick_ms;
        s->sample_tick     = in->sample_tick;
        s->sample_ms       = in->tick_ms;
        s->steady_ms       = in->tick_ms;
        s->window_length   = in->length;
        s->compression     = SPRING_CAL_START_N;
        s->motor_command   = -s->compression;
        return;
    }

    const uint32_t dt_ms = in->tick_ms - s->last_ms;
    if (dt_ms > SPRING_CAL_FRAME_GAP_MS || in->tick_ms - s->sample_ms > SPRING_CAL_FRAME_GAP_MS)
    {
        App_Spring_Cal_Stop(s, SPRING_CAL_REASON_FEEDBACK);
        return;
    }
    if (in->length < in->min_length || in->length > in->max_length + SPRING_CAL_START_TOL_M)
    {
        App_Spring_Cal_Stop(s, SPRING_CAL_REASON_RANGE);
        return;
    }
    if (in->length <= in->min_length + SPRING_CAL_STOP_MARGIN_M)
        s->hold_load = true;
    if (s->hold_load || fabsf(in->rate) >= SPRING_CAL_OVERSPEED_MPS)
    {
        s->last_ms = in->tick_ms;
        if (s->sample_tick != in->sample_tick)
        {
            s->sample_tick = in->sample_tick;
            s->sample_ms = in->tick_ms;
        }
        s->sample_ok = false;
        return;
    }

    const float speed_error = SPRING_CAL_SPEED_MPS + in->rate;
    const float ramp_ratio = fminf(1.0f, fmaxf(0.0f, speed_error / SPRING_CAL_SPEED_BAND));
    const bool coarse = s->compression < SPRING_CAL_TRANSITION_N;
    const double ceiling = coarse ? fminf(SPRING_CAL_TRANSITION_N, SPRING_CAL_FORCE_MAX_N)
                                  : SPRING_CAL_FORCE_MAX_N;
    const double rate = coarse ? SPRING_CAL_FORCE_RATE : SPRING_CAL_FINE_RATE;
    s->compression += rate * ramp_ratio * (0.001 * dt_ms);
    if (s->compression > ceiling) s->compression = ceiling;
    s->motor_command = -s->compression;
    s->last_ms       = in->tick_ms;

    // 同一对反馈的资格保持到新回读，便于较慢的 VOFA 采样，导出后用标记去重。
    if (in->sample_tick == s->sample_tick) return;
    s->sample_tick = in->sample_tick;
    s->sample_ms   = in->tick_ms;
    const bool window = in->length < in->max_length - SPRING_CAL_SAMPLE_MARGIN_M &&
                        in->length > in->min_length + SPRING_CAL_SAMPLE_MARGIN_M &&
                        in->rate <= SPRING_CAL_READY_SPEED && in->rate >= -SPRING_CAL_SAMPLE_MAX_MPS;
    const uint32_t window_ms = in->tick_ms - s->steady_ms;
    if (!window || window_ms >= SPRING_CAL_STEADY_MS)
    {
        const float distance = s->window_length - in->length;
        const float seconds = 0.001f * (float)window_ms;
        s->sample_ok = window && distance >= SPRING_CAL_SAMPLE_MIN_MPS * seconds && distance <= SPRING_CAL_SAMPLE_MAX_MPS * seconds;
        s->window_length = in->length;
        s->steady_ms = in->tick_ms;
    }
}
#endif
