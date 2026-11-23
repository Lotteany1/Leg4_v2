/** @file  app_gyro.c  @brief 陀螺行为目标更新  @version  v3.3  2026-11-22 */
#include "app_gyro.h"
#include "arm_math.h"
#include "body.h"
#include "code_config.h"
#include <math.h>

static gyro_cfg_t cfg = {
    .yaw_bias = GYRO_YAW_BIAS,
    .yaw_freq = GYRO_YAW_FREQ,
    .yaw_amp  = GYRO_YAW_AMP,
    .yaw_dir  = GYRO_YAW_DIR,
    .h_lo     = GYRO_H_LO,
    .h_hi     = GYRO_H_HI,
    .ramp_ms  = GYRO_RAMP_MS,
};

static float phase         = 0.0f;  // rad 正弦相位
static float h_ref         = 0.0f;  // m 出场机体高度终点
static float env           = 0.0f;  // 出入场包络 0~1
static float dir_locked    = +1.0f; // 旋转方向 ±1 进档时从参数锁存
static uint8_t running     = 0u;    // 1 仍持有偏航角速度与机体高度目标量
static uint8_t active_last = 0u;    // 上拍入参 切出沿锁出场终点

// 上下限限幅
static float Gyro_Clamp(float x, float min, float max)
{
    if (x < min)
        return min;
    if (x > max)
        return max;
    return x;
}

// 转速幅值在正弦取值范围里归一化后反向插值出机体高度 零转速取最高峰值转速取最低
static float Gyro_H_From_Yaw(float yaw)
{
    const float bias    = fabsf(cfg.yaw_bias);
    const float mag_min = (cfg.yaw_amp >= bias) ? 0.0f : (bias - cfg.yaw_amp);
    const float mag_max = bias + cfg.yaw_amp;
    const float span    = mag_max - mag_min;
    float t             = 0.0f;

    if (span > 0.0f)
    {
        t = (fabsf(yaw) - mag_min) / span;
        if (t < 0.0f)
            t = 0.0f;
        if (t > 1.0f)
            t = 1.0f;
    }

    return cfg.h_hi - t * (cfg.h_hi - cfg.h_lo);
}

// 参数合法性判据 任一非有限即整组作废
static bool Gyro_Cfg_Check(const gyro_cfg_t *c)
{
    return isfinite(c->yaw_bias) && isfinite(c->yaw_freq) && isfinite(c->yaw_amp) &&
           isfinite(c->yaw_dir) && isfinite(c->h_lo) && isfinite(c->h_hi);
}

// 陀螺参数与运行上下文初始化
void App_Gyro_Init(void)
{
    cfg.yaw_bias = GYRO_YAW_BIAS;
    cfg.yaw_freq = GYRO_YAW_FREQ;
    cfg.yaw_amp  = GYRO_YAW_AMP;
    cfg.yaw_dir  = GYRO_YAW_DIR;
    cfg.h_lo     = GYRO_H_LO;
    cfg.h_hi     = GYRO_H_HI;
    cfg.ramp_ms  = GYRO_RAMP_MS;

    App_Gyro_Reset();
}

// 平衡入场清运行上下文并释放目标量 保留运行时配置 不改机体高度与偏航目标
void App_Gyro_Reset(void)
{
    phase       = 0.0f;
    h_ref       = 0.0f;
    env         = 0.0f;
    dir_locked  = cfg.yaw_dir;
    running     = 0u;
    active_last = 0u;
}

// 参数落地 频率幅值取绝对 方向只取符号 高度区间钳到机构行程并保证下限小于上限 斜坡至少一拍
void App_Gyro_Cfg_Set(const gyro_cfg_t *in)
{
    float lo;
    float hi;

    if (in == NULL || !Gyro_Cfg_Check(in))
        return;

    cfg.yaw_freq = Gyro_Clamp(fabsf(in->yaw_freq), 0.0f, GYRO_FREQ_MAX);
    cfg.yaw_amp  = Gyro_Clamp(fabsf(in->yaw_amp), 0.0f, GYRO_YAW_MAX);
    cfg.yaw_bias = Gyro_Clamp(in->yaw_bias, -GYRO_YAW_MAX, GYRO_YAW_MAX);
    cfg.yaw_dir  = (in->yaw_dir < 0.0f) ? -1.0f : +1.0f;

    lo = LEG_LEN_CLAMP(in->h_lo);
    hi = LEG_LEN_CLAMP(in->h_hi);
    if (lo > hi)
    {
        const float tmp = lo;
        lo              = hi;
        hi              = tmp;
    }
    else if (hi - lo < 0.001f)
    {
        if (hi + 0.001f <= Leg_Len_Max)
            hi += 0.001f;
        else
            lo -= 0.001f;
    }

    cfg.h_lo    = lo;
    cfg.h_hi    = hi;
    cfg.ramp_ms = (in->ramp_ms > 0u) ? in->ramp_ms : 1u;

    h_ref = LEG_LEN_CLAMP(h_ref);
}

// 实际切入陀螺行为时锁旋转方向 高度从当前值起步 相位与包络归零做一次新的爬坡
void App_Gyro_Enter(void)
{
    phase       = 0.0f;
    h_ref       = LEG_LEN_CLAMP(body.h_tgt);
    env         = 0.0f;
    dir_locked  = cfg.yaw_dir;
    running     = 1u;
    active_last = 1u;
}

// 每拍推进包络与正弦 转速幅值联动高度 出场斜坡收尾那一拍把目标量还给遥控
void App_Gyro_Update(bool active)
{
    float rc_yaw;
    float yaw_wave;
    float h_wave;
    float env_step;

    if (!active && running == 0u)
        return;

    if (!active && active_last != 0u)
    {
        h_ref = GYRO_H_EXIT;
    }

    env_step = CTRL_DT * 1000.0f / (float)cfg.ramp_ms;

    if (active)
    {
        env += env_step;
        if (env > 1.0f)
            env = 1.0f;
    }
    else
    {
        env -= env_step;
        if (env < 0.0f)
            env = 0.0f;
    }

    phase += TWO_PI * cfg.yaw_freq * CTRL_DT;
    if (phase >= TWO_PI)
    {
        phase -= TWO_PI;
    }
    yaw_wave = dir_locked * (cfg.yaw_bias + cfg.yaw_amp * arm_sin_f32(phase));
    h_wave   = Gyro_H_From_Yaw(yaw_wave);

    rc_yaw = body.omg_yaw_tgt;

    body.omg_yaw_tgt = rc_yaw * (1.0f - env) + yaw_wave * env;
    body.h_tgt       = LEG_LEN_CLAMP(h_ref * (1.0f - env) + h_wave * env);

    if (!active && env <= 0.0f)
    {
        running = 0u;
    }

    active_last = active ? 1u : 0u;
}

// 行为仲裁读取包络是否仍有所有权。
bool App_Gyro_Running(void)
{
    return running != 0u;
}
