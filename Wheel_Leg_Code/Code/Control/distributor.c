/** @file  distributor.c  @brief 分配发送、接纳历史与观测输入恢复  @version  v1.11  2026-11-16 */
#include "distributor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "code_config.h"
#include "system_key.h"
#include "dm_motor.h"
#include "dji_motor.h"
#include "Leg_Trq.h"
#include "monitor.h"
#include "stm32h7xx.h"
#include <float.h>
#include <math.h>
#include <stddef.h>

static volatile distributor_diag_t distributor_diag;
static uint32_t distributor_seq;
static volatile execution_sample_t execution_history[EXECUTION_CAPACITY];
static volatile uint32_t execution_seq;
static volatile uint32_t execution_count;
static volatile uint32_t execution_epoch;
static volatile uint32_t execution_mode;
static volatile int32_t execution_code[6];
static volatile uint8_t execution_valid;
static volatile uint32_t execution_transport;
static volatile execution_observer_diag_t execution_observer_diag[2];

static const float execution_pitch_bound[4]  = EXECUTION_PITCH_BOUND;

// 对称限幅 非有限值由输入检查与发送前清洗拦截
static float Distributor_Clamp(float x, float max)
{
    if (x > max)
        return max;
    if (x < -max)
        return -max;
    return x;
}

// 检查电机与总支持力的六个归一化半平面并保留最小代价，输入已由公有入口验证。
static void Distributor_Consider(const float x[2], const float request[2], float g[6][2], const float h[6], float best[2], float *cost)
{
    float value;

    if (!isfinite(x[0]) || !isfinite(x[1])) return;
    for (unsigned int row = 0u; row < 6u; row++)
        if (g[row][0] * x[0] + g[row][1] * x[1] - h[row] > DISTRIBUTOR_TOL) return;
    value = 0.5f * (DISTRIBUTOR_S_WEIGHT * (x[0] - request[0]) * (x[0] - request[0]) +
                    DISTRIBUTOR_P_WEIGHT * (x[1] - request[1]) * (x[1] - request[1]));
    if (value < *cost)
    {
        *cost   = value;
        best[0] = x[0];
        best[1] = x[1];
    }
}

// 成功与失败共用发布点；先合并逐拍统计，再交接同一拍的状态、命令和接纳结果。
static void Distributor_Publish_Diagnostic(distributor_diag_t *diag)
{
    taskENTER_CRITICAL();
    diag->mode         = distributor_diag.mode;
    diag->normal_count = distributor_diag.normal_count + 1u;
    diag->failure_count += distributor_diag.failure_count;
    diag->protection_count += distributor_diag.protection_count;
    diag->changed_count += distributor_diag.changed_count;
    diag->fallback_count += distributor_diag.fallback_count;
    diag->send_failure_count += distributor_diag.send_failure_count;
    diag->wheel_limited_count += distributor_diag.wheel_limited_count;
    diag->support_relaxed_count += distributor_diag.support_relaxed_count;
    diag->limited_run     = diag->limited_count != 0u ? distributor_diag.limited_run + 1u : 0u;
    diag->limited_run_max = diag->limited_run > distributor_diag.limited_run_max ? diag->limited_run : distributor_diag.limited_run_max;
    diag->limited_count += distributor_diag.limited_count;
    for (unsigned int bit = 0u; bit < 8u; bit++)
        diag->joint_limited_count[bit] += distributor_diag.joint_limited_count[bit];
    diag->tau_change_max = fmaxf(diag->tau_change_max, distributor_diag.tau_change_max);
    diag->s_error_max    = fmaxf(diag->s_error_max, distributor_diag.s_error_max);
    diag->p_error_max    = fmaxf(diag->p_error_max, distributor_diag.p_error_max);
    distributor_diag     = *diag;
    taskEXIT_CRITICAL();
}

// 仅在已验证映射的记录上恢复任务；保持码必须随该拍姿态重新解释。
static void Execution_Values(const execution_sample_t *sample, const int32_t code[6], float u[4], float s[2])
{
    for (unsigned int side = 0u; side < 2u; side++)
    {
        const float(*a)[2] = sample->map[side];
        float det          = a[0][0] * a[1][1] - a[0][1] * a[1][0];
        float t0           = Dm_Motor_Torque_Decode((uint16_t)code[2u * side]);
        float t1           = Dm_Motor_Torque_Decode((uint16_t)code[2u * side + 1u]);

        s[side]      = (a[1][1] * t0 - a[0][1] * t1) / det + sample->spring_f[side];
        u[side + 2u] = (-a[1][0] * t0 + a[0][0] * t1) / det - sample->hip_comp[side];
    }
    u[0] = -WHEEL_LFT_CMD_POLARITY * (float)code[4] / M3508_TO_TORQUE;
    u[1] = -WHEEL_RGT_CMD_POLARITY * (float)code[5] / M3508_TO_TORQUE;
}

// 只累计查询交集，原始 DWT 用相对查询起点的有符号差处理回绕。
static bool Execution_Accumulate(const execution_sample_t *sample, const int32_t code[6], uint8_t valid, int32_t lo, int32_t hi, uint32_t duration, execution_interval_t *result, uint32_t *covered)
{
    float u[4], s[2];
    float weight;

    if (lo < 0) lo = 0;
    if (hi > (int32_t)duration) hi = (int32_t)duration;
    if (hi <= lo) return true;
    if (valid != EXECUTION_CHANNEL_MASK) return false;
    Execution_Values(sample, code, u, s);
    weight = (float)(hi - lo) / (float)duration;
    for (unsigned int ch = 0u; ch < 4u; ch++)
    {
        if (*covered == 0u) result->min_u[ch] = result->max_u[ch] = u[ch];
        result->min_u[ch] = fminf(result->min_u[ch], u[ch]);
        result->max_u[ch] = fmaxf(result->max_u[ch], u[ch]);
        result->model_u[ch] += weight * u[ch];
    }
    for (unsigned int side = 0u; side < 2u; side++)
    {
        float force = s[side] + sample->turn[side];

        if (*covered == 0u) result->min_s[side] = result->max_s[side] = force;
        result->min_s[side] = fminf(result->min_s[side], force);
        result->max_s[side] = fmaxf(result->max_s[side], force);
        result->total_s[side] += weight * s[side];
        result->turn[side] += weight * sample->turn[side];
        result->end_s[side] = s[side];
    }
    *covered += (uint32_t)(hi - lo);
    return true;
}

// 最终电机坐标反解；弹簧加回总支持力，髋补偿从模型髋输入扣除。
void Distributor_Recover(const distributor_leg_t leg[2], const float tau[4], float total_s[2], float hip[2])
{
    for (unsigned int side = 0u; side < 2u; side++)
    {
        const float(*a)[2] = leg[side].map;
        float det          = a[0][0] * a[1][1] - a[0][1] * a[1][0];
        float t0           = tau[side * 2u];
        float t1           = tau[side * 2u + 1u];

        total_s[side] = (a[1][1] * t0 - a[0][1] * t1) / det + leg[side].spring_f;
        hip[side]     = (-a[1][0] * t0 + a[0][0] * t1) / det - leg[side].hip_comp;
    }
}

// 单拍先验证数值和映射，再按同一幅值界计算所选策略，不代表量化和轴端跟随。
void Distributor_Calculate(const distributor_input_t *input, distributor_result_t *result)
{
    bool limited  = false;
    bool fallback = false;
    float row_norm[2][2];
    float determinant[2];

    if (result == NULL) return;
    *result = (distributor_result_t){0};
    if (input == NULL) return;
    result->state_tick = input->state_tick;
    result->seq        = input->seq;
    result->epoch      = input->epoch;
    if (!input->normal_mode)
    {
        result->status = DISTRIBUTOR_STATUS_BYPASS;
        return;
    }
    if (!isfinite(input->dt) || input->dt <= 0.0f || (unsigned int)input->strategy > DISTRIBUTOR_SCALE) return;
    if (DISTRIBUTOR_S_SCALE <= 0.0f || DISTRIBUTOR_P_SCALE <= 0.0f ||
        DISTRIBUTOR_S_WEIGHT <= 0.0f || DISTRIBUTOR_P_WEIGHT <= 0.0f ||
        !isfinite(DISTRIBUTOR_S_MIN) || !isfinite(DISTRIBUTOR_S_MAX) || !isfinite(DISTRIBUTOR_S_RESERVE) ||
        DISTRIBUTOR_S_MIN > DISTRIBUTOR_S_MAX) return;
    for (unsigned int side = 0u; side < 2u; side++)
    {
        const distributor_leg_t *leg = &input->leg[side];
        float det;
        float norm;

        if (!isfinite(leg->s_req) || !isfinite(leg->p_req) || !isfinite(leg->spring_f) || !isfinite(leg->hip_comp)) return;
        for (unsigned int j = 0u; j < 2u; j++)
        {
            if (!isfinite(leg->map[j][0]) || !isfinite(leg->map[j][1]) ||
                !isfinite(leg->tau_lo[j]) || !isfinite(leg->tau_hi[j]) || leg->tau_lo[j] > leg->tau_hi[j]) return;
            if (input->strategy == DISTRIBUTOR_SCALE && (leg->tau_lo[j] > 0.0f || leg->tau_hi[j] < 0.0f)) return;
        }
        det = leg->map[0][0] * leg->map[1][1] - leg->map[0][1] * leg->map[1][0];
        // 退化判定和半平面归一化使用同一行范数，每拍每行只算一次。
        for (unsigned int j = 0u; j < 2u; j++)
            row_norm[side][j] = hypotf(leg->map[j][0] * DISTRIBUTOR_S_SCALE, leg->map[j][1] * DISTRIBUTOR_P_SCALE);
        norm = row_norm[side][0] * row_norm[side][1];
        if (!isfinite(norm) || fabsf(det * DISTRIBUTOR_S_SCALE * DISTRIBUTOR_P_SCALE) <= 64.0f * FLT_EPSILON * norm) return;
        determinant[side] = det;
    }

    for (unsigned int side = 0u; side < 2u; side++)
    {
        const distributor_leg_t *leg = &input->leg[side];
        float request[2]             = {leg->s_req / DISTRIBUTOR_S_SCALE, leg->p_req / DISTRIBUTOR_P_SCALE};
        float g[6][2], h[6], best[2] = {0.0f, 0.0f};
        float cost   = FLT_MAX;
        float alpha  = 1.0f;
        bool outside = false;

        for (unsigned int j = 0u; j < 2u; j++)
        {
            unsigned int index = 2u * side + j;
            float bias         = -leg->map[j][0] * leg->spring_f + leg->map[j][1] * leg->hip_comp;
            float tau          = leg->map[j][0] * leg->s_req + leg->map[j][1] * leg->p_req + bias;
            float norm         = row_norm[side][j];

            if (!isfinite(tau)) return;
            result->tau_req[index]   = tau;
            result->tau_alloc[index] = fminf(leg->tau_hi[j], fmaxf(leg->tau_lo[j], tau));
            result->request_residual = fmaxf(result->request_residual, fmaxf(leg->tau_lo[j] - tau, tau - leg->tau_hi[j]));
            if (tau < leg->tau_lo[j]) result->request_mask |= (uint8_t)(1u << (2u * index));
            if (tau > leg->tau_hi[j]) result->request_mask |= (uint8_t)(2u << (2u * index));
            outside |= tau < leg->tau_lo[j] || tau > leg->tau_hi[j];
            if (tau > leg->tau_hi[j]) alpha = fminf(alpha, leg->tau_hi[j] / tau);
            if (tau < leg->tau_lo[j]) alpha = fminf(alpha, leg->tau_lo[j] / tau);
            g[2u * j][0]      = leg->map[j][0] * DISTRIBUTOR_S_SCALE / norm;
            g[2u * j][1]      = leg->map[j][1] * DISTRIBUTOR_P_SCALE / norm;
            h[2u * j]         = (leg->tau_hi[j] - bias) / norm;
            g[2u * j + 1u][0] = -g[2u * j][0];
            g[2u * j + 1u][1] = -g[2u * j][1];
            h[2u * j + 1u]    = (bias - leg->tau_lo[j]) / norm;
        }
        limited |= outside;
        if (!outside) continue;
        if (input->strategy == DISTRIBUTOR_SCALE)
        {
            for (unsigned int j = 0u; j < 2u; j++) result->tau_alloc[2u * side + j] = alpha * result->tau_req[2u * side + j];
        }
        else if (input->strategy == DISTRIBUTOR_PROJECT)
        {
            float inverse[2]   = {leg->map[1][1] / determinant[side], -leg->map[0][1] / determinant[side]};
            float reachable_lo = leg->spring_f;
            float reachable_hi = leg->spring_f;
            float reserve      = fminf(DISTRIBUTOR_S_MAX, fmaxf(DISTRIBUTOR_S_MIN, DISTRIBUTOR_S_RESERVE));
            float support_lo   = fminf(leg->s_req, reserve);
            float support_hi   = input->height_priority ? leg->s_req : fmaxf(leg->s_req, DISTRIBUTOR_S_MAX);

            // 反解力在电机矩形四角取极值；能力不足时仅将支撑范围移到最近可达点，不作废合法力矩。
            for (unsigned int j = 0u; j < 2u; j++)
            {
                float lo = inverse[j] * leg->tau_lo[j];
                float hi = inverse[j] * leg->tau_hi[j];

                reachable_lo += fminf(lo, hi);
                reachable_hi += fmaxf(lo, hi);
            }
            if (support_lo > reachable_hi || support_hi < reachable_lo)
                result->support_relaxed_mask |= (uint8_t)(1u << side);
            support_lo = fminf(reachable_hi, fmaxf(reachable_lo, support_lo));
            support_hi = fminf(reachable_hi, fmaxf(reachable_lo, support_hi));

            // 总支持力包含气弹簧；保留名义重力支撑但尊重主动卸载，余量用于加权髋跟踪。
            g[4][0] = 1.0f;
            g[4][1] = 0.0f;
            h[4]    = support_hi / DISTRIBUTOR_S_SCALE;
            g[5][0] = -1.0f;
            g[5][1] = 0.0f;
            h[5]    = -support_lo / DISTRIBUTOR_S_SCALE;

            // 请求、六条边投影及十五个交点，每腿最多 22 个候选；平行边不求交。
            Distributor_Consider(request, request, g, h, best, &cost);
            for (unsigned int row = 0u; row < 6u; row++)
            {
                float denom = g[row][0] * g[row][0] / DISTRIBUTOR_S_WEIGHT + g[row][1] * g[row][1] / DISTRIBUTOR_P_WEIGHT;
                float step  = (g[row][0] * request[0] + g[row][1] * request[1] - h[row]) / denom;
                float x[2]  = {request[0] - step * g[row][0] / DISTRIBUTOR_S_WEIGHT, request[1] - step * g[row][1] / DISTRIBUTOR_P_WEIGHT};

                Distributor_Consider(x, request, g, h, best, &cost);
                for (unsigned int other = row + 1u; other < 6u; other++)
                {
                    float det = g[row][0] * g[other][1] - g[row][1] * g[other][0];

                    if (fabsf(det) <= 16.0f * FLT_EPSILON) continue;
                    x[0] = (h[row] * g[other][1] - g[row][1] * h[other]) / det;
                    x[1] = (g[row][0] * h[other] - h[row] * g[other][0]) / det;
                    Distributor_Consider(x, request, g, h, best, &cost);
                }
            }
            if (cost == FLT_MAX)
            {
                fallback = true;
            }
            else
            {
                for (unsigned int j = 0u; j < 2u; j++)
                {
                    float tau = leg->map[j][0] * (best[0] * DISTRIBUTOR_S_SCALE - leg->spring_f) +
                                leg->map[j][1] * (best[1] * DISTRIBUTOR_P_SCALE + leg->hip_comp);

                    // 数值可行性容差不改变最终连续命令的精确幅值界。
                    result->tau_alloc[2u * side + j] = fminf(leg->tau_hi[j], fmaxf(leg->tau_lo[j], tau));
                }
            }
        }
    }
    Distributor_Recover(input->leg, result->tau_alloc, result->s_alloc, result->p_alloc);
    for (unsigned int side = 0u; side < 2u; side++)
    {
        if (!isfinite(result->s_alloc[side]) || !isfinite(result->p_alloc[side])) return;
        result->task_error[side][0] = result->s_alloc[side] - input->leg[side].s_req;
        result->task_error[side][1] = result->p_alloc[side] - input->leg[side].p_req;
        {
            float s_error = result->task_error[side][0] / DISTRIBUTOR_S_SCALE;
            float p_error = result->task_error[side][1] / DISTRIBUTOR_P_SCALE;

            result->objective[side] = 0.5f * (DISTRIBUTOR_S_WEIGHT * s_error * s_error + DISTRIBUTOR_P_WEIGHT * p_error * p_error);
            if (!isfinite(result->objective[side])) return;
        }
        for (unsigned int j = 0u; j < 2u; j++)
        {
            unsigned int index = 2u * side + j;
            float tau          = result->tau_alloc[index];

            result->max_residual = fmaxf(result->max_residual, fmaxf(input->leg[side].tau_lo[j] - tau, tau - input->leg[side].tau_hi[j]));
            if (fabsf(tau - input->leg[side].tau_lo[j]) <= 1e-4f) result->active_mask |= (uint8_t)(1u << (2u * index));
            if (fabsf(tau - input->leg[side].tau_hi[j]) <= 1e-4f) result->active_mask |= (uint8_t)(2u << (2u * index));
        }
    }
    result->status = DISTRIBUTOR_STATUS_NOMINAL;
    if (limited)
        result->status = input->strategy == DISTRIBUTOR_PROJECT ? DISTRIBUTOR_STATUS_PROJECTED : (input->strategy == DISTRIBUTOR_CLIP ? DISTRIBUTOR_STATUS_CLIPPED : DISTRIBUTOR_STATUS_SCALED);
    if (fallback) result->status = DISTRIBUTOR_STATUS_FALLBACK;
    result->result_valid = true;
}

// 上电清空历史，不在模式交接时重新使用旧序号。
void Execution_Init(void)
{
    for (unsigned int i = 0u; i < 2u; i++) execution_observer_diag[i] = (execution_observer_diag_t){0};
    execution_seq       = 0u;
    execution_count     = 0u;
    execution_epoch     = 1u;
    execution_mode      = UINT32_MAX;
    execution_transport = Can_Bus_Tx_Epoch();
    execution_valid     = 0u;
}

// 控制任务或总线恢复任务调用；在途发布与后台提交通过 epoch 作废。
void Execution_Invalidate(void)
{
    taskENTER_CRITICAL();
    execution_epoch++;
    execution_valid = 0u;
    execution_count = 0u;
    taskEXIT_CRITICAL();
}

// 模式变化才切代次，不把后台消费次数当作模式代次。
void Execution_Mode(uint32_t mode)
{
    taskENTER_CRITICAL();
    Execution_Epoch();
    if (mode != execution_mode)
    {
        execution_mode = mode;
        Execution_Invalidate();
    }
    taskEXIT_CRITICAL();
}

// 快环与后台提交都检查传输边界，不等下一控制拍才作废 bus-off/Abort 前的候选。
uint32_t Execution_Epoch(void)
{
    uint32_t epoch;
    uint32_t transport;

    taskENTER_CRITICAL();
    transport = Can_Bus_Tx_Epoch();
    if (transport != execution_transport)
    {
        execution_transport = transport;
        execution_epoch++;
        execution_valid = 0u;
        execution_count = 0u;
    }
    epoch = execution_epoch;
    taskEXIT_CRITICAL();
    return epoch;
}

// 控制任务独占构造快照，其余字段由调用者冻结后填入。
void Execution_Begin(execution_sample_t *sample)
{
    *sample = (execution_sample_t){0};
    taskENTER_CRITICAL();
    sample->epoch        = Execution_Epoch();
    sample->valid_before = execution_valid;
    for (unsigned int ch = 0u; ch < 6u; ch++) sample->before[ch] = execution_code[ch];
    taskEXIT_CRITICAL();
}

// 只提交当前代次；失败通道保留旧码，DJI 掩码由发送调用边界整组设置。
void Execution_Publish(execution_sample_t *sample)
{
    int32_t held[6];

    sample->valid_mask = sample->valid_before | sample->accepted_mask;
    for (unsigned int ch = 0u; ch < 6u; ch++) held[ch] = (sample->accepted_mask & (1u << ch)) ? sample->code[ch] : sample->before[ch];
    if (sample->valid_mask == EXECUTION_CHANNEL_MASK) Execution_Values(sample, held, sample->model_u, sample->total_s);
    taskENTER_CRITICAL();
    if (sample->epoch == execution_epoch && execution_transport == Can_Bus_Tx_Epoch())
    {
        for (unsigned int ch = 0u; ch < 6u; ch++)
            if (sample->accepted_mask & (1u << ch)) execution_code[ch] = sample->code[ch];
        execution_valid |= sample->accepted_mask;
        sample->seq                                                = ++execution_seq;
        execution_history[(sample->seq - 1u) % EXECUTION_CAPACITY] = *sample;
        if (execution_count < EXECUTION_CAPACITY) execution_count++;
    }
    else
        sample->valid_mask = 0u;
    taskEXIT_CRITICAL();
}

// 后台用自己的时间区间读取，不弹出记录，不会消耗另一个后台的输入。
void Execution_Read(uint32_t epoch, uint32_t begin, uint32_t end, uint32_t end_tick, uint32_t cycles_per_ms, execution_interval_t *result)
{
    uint32_t last, count, covered = 0u;
    uint32_t duration = end - begin;

    *result       = (execution_interval_t){0};
    result->epoch = epoch;
    if (cycles_per_ms == 0u || duration == 0u || duration > cycles_per_ms * EXECUTION_MAX_AGE_MS) return;
    taskENTER_CRITICAL();
    if (epoch != Execution_Epoch())
    {
        taskEXIT_CRITICAL();
        return;
    }
    last  = execution_seq;
    count = execution_count;
    taskEXIT_CRITICAL();
    for (uint32_t n = 0u; n < count; n++)
    {
        execution_sample_t sample;
        uint32_t next_seq, next_epoch, next_cycle, next_tick;
        uint32_t seq = last - count + 1u + n;
        int32_t stop;
        int32_t point;
        int32_t code[6];
        uint8_t valid;

        taskENTER_CRITICAL();
        sample = execution_history[(seq - 1u) % EXECUTION_CAPACITY];
        if (n + 1u < count)
        {
            // 相邻记录只用时标与身份；不在关中断区重复复制其映射、码和诊断结果。
            const volatile execution_sample_t *next = &execution_history[seq % EXECUTION_CAPACITY];

            next_seq   = next->seq;
            next_epoch = next->epoch;
            next_cycle = next->cycle;
            next_tick  = next->tick;
        }
        taskEXIT_CRITICAL();
        if (sample.seq != seq || sample.epoch != epoch) return;
        if (n + 1u < count)
        {
            if (next_seq != seq + 1u || next_epoch != epoch) return;
            stop = (int32_t)(next_cycle - begin);
        }
        else
        {
            stop = (int32_t)duration;
        }
        if (stop <= 0) continue;
        point = (int32_t)(sample.cycle - begin);
        if (point >= (int32_t)duration) break;
        if (stop > (int32_t)duration) stop = (int32_t)duration;
        if ((uint32_t)(end_tick - sample.tick) > EXECUTION_MAX_AGE_MS) return;
        // 查询终点以后的缺拍不否定本段；查询已进入缺拍时仍拒绝拼接。
        if (n + 1u < count && (uint32_t)(next_tick - sample.tick) > 1u &&
            ((int32_t)(next_cycle - begin) <= (int32_t)duration || (uint32_t)(stop - point) > cycles_per_ms)) return;
        if (n + 1u == count && (uint32_t)(end_tick - sample.tick) > 1u) return;
        valid = sample.valid_before;
        for (unsigned int ch = 0u; ch < 6u; ch++) code[ch] = sample.before[ch];
        for (unsigned int event = 0u; event < 5u; event++)
        {
            int32_t at   = (int32_t)(sample.cycle + sample.offset[event] - begin);
            uint8_t mask = event == 4u ? 48u : (uint8_t)(1u << event);

            if (at < point) return;
            // [begin,end) 之外的事件不进入均值和末值，也不要求未来发送已完成。
            if (at >= stop) break;
            if (!Execution_Accumulate(&sample, code, valid, point, at, duration, result, &covered)) return;
            for (unsigned int ch = 0u; ch < 6u; ch++)
                if (mask & sample.accepted_mask & (1u << ch)) code[ch] = sample.code[ch];
            valid |= mask & sample.accepted_mask;
            point = at;
        }
        if (!Execution_Accumulate(&sample, code, valid, point, stop, duration, result, &covered)) return;
        result->seq = seq;
    }
    result->complete = covered == duration && epoch == Execution_Epoch() && execution_transport == Can_Bus_Tx_Epoch();
}

// 遥测不重新编码和反解，不在读取时推进后台游标。
void Execution_Latest(execution_sample_t *sample)
{
    taskENTER_CRITICAL();
    Execution_Epoch();
    if (execution_count != 0u)
        *sample = execution_history[(execution_seq - 1u) % EXECUTION_CAPACITY];
    else
        *sample = (execution_sample_t){0};
    taskEXIT_CRITICAL();
}

// 每个后台独占自己的槽，在已经确认代次的提交点调用。
void Execution_Observation(uint32_t observer, uint32_t epoch, uint32_t tick, uint32_t quality)
{
    taskENTER_CRITICAL();
    if (epoch == Execution_Epoch())
    {
        execution_observer_diag[observer].epoch   = epoch;
        execution_observer_diag[observer].tick    = tick;
        execution_observer_diag[observer].quality = quality;
        if (quality)
            execution_observer_diag[observer].skipped++;
        else
            execution_observer_diag[observer].learned++;
    }
    taskEXIT_CRITICAL();
}

// 遥测读取已提交质量，不消费历史或更新观测。
void Execution_Observer_Diagnostic(execution_observer_diag_t out[2])
{
    taskENTER_CRITICAL();
    for (unsigned int i = 0u; i < 2u; i++) out[i] = execution_observer_diag[i];
    taskEXIT_CRITICAL();
}

// 实测时间偏离固定观测节点超过 5% 时不学习扰动，避免把掉拍当成正常模型步。
bool Execution_Quality_Time(uint32_t cycles, uint32_t frequency, uint32_t milliseconds)
{
    float expected = (float)frequency * (float)milliseconds * 0.001f;

    return (float)cycles >= expected * EXECUTION_TIME_MIN_RATIO && (float)cycles <= expected * EXECUTION_TIME_MAX_RATIO;
}

// 完整区间才判断均值预算，输入范围来自接纳事件，不是名义任务。
bool Execution_Quality_Balance(const execution_interval_t *interval)
{
    float error = 0.0f;

    for (unsigned int i = 0u; i < 4u; i++) error += execution_pitch_bound[i] * (interval->max_u[i] - interval->min_u[i]);
    return error <= EXECUTION_PITCH_BUDGET;
}

// 分配诊断与执行历史同属控制输出生命周期，不重置沿用的请求序号。
void Distributor_Init(void)
{
    Execution_Init();
    distributor_diag = (distributor_diag_t){0};
    distributor_diag.mode = UINT32_MAX;
}

// 模式先于模型交接；原输出路径不把上一 NORMAL 连续结果当成本拍结果。
void Distributor_Mode(uint32_t mode, bool normal_mode)
{
    // 接地各模式沿用代次，诊断单独保留实际模式。
    Execution_Mode(normal_mode ? 0u : mode);
    taskENTER_CRITICAL();
    distributor_diag.mode = mode;
    if (!normal_mode)
    {
        distributor_diag.input.normal_mode   = false;
        distributor_diag.land               = (distributor_landing_t){0};
        distributor_diag.result.result_valid = false;
        distributor_diag.result.status       = DISTRIBUTOR_STATUS_BYPASS;
        distributor_diag.limited_run         = 0u;
    }
    taskEXIT_CRITICAL();
}

// NORMAL 完成合成后冻结、分配并发送；只改变本模式命令，不修改名义请求向量。
bool Distributor_Send(const distributor_frame_t *frame, float nominal[2][2])
{
    distributor_diag_t diag = {0};
    execution_sample_t sample;
    const float lower[4] = {DISTRIBUTOR_LF_MIN, DISTRIBUTOR_LB_MIN, DISTRIBUTOR_RF_MIN, DISTRIBUTOR_RB_MIN};
    const float upper[4] = {DISTRIBUTOR_LF_MAX, DISTRIBUTOR_LB_MAX, DISTRIBUTOR_RF_MAX, DISTRIBUTOR_RB_MAX};
    const distributor_strategy_t strategy = frame->stepping || BALANCE_USE_DISTRIBUTOR ?
                                            DISTRIBUTOR_PROJECT : DISTRIBUTOR_CLIP;
    dm_motor_t *motor[4] = {&dm_f_lft, &dm_b_lft, &dm_f_rgt, &dm_b_rgt};
    uint16_t code[4];
    int16_t wheel_code[4] = {0};
    uint32_t begin        = DWT->CYCCNT;
    float us_per_cycle    = 1e6f / (float)SystemCoreClock;
    bool encoded          = true;

    Execution_Begin(&sample);
    sample.tick            = frame->tick;
    sample.cycle           = frame->cycle;
    diag.input.state_tick  = sample.tick;
    diag.input.epoch       = sample.epoch;
    diag.input.seq         = ++distributor_seq;
    diag.input.dt          = CTRL_DT;
    diag.input.normal_mode = true;
    diag.input.height_priority = frame->height_priority;
    diag.input.strategy    = strategy;
    diag.land              = frame->land;
    diag.state             = frame->state;
    diag.wheel_req[0]      = frame->wheel[0];
    diag.wheel_req[1]      = frame->wheel[1];
    for (unsigned int side = 0u; side < 2u; side++)
    {
        distributor_leg_t *leg = &diag.input.leg[side];
        float force_map[2], hip_map[2];
        float mirror = side == 0u ? 1.0f : LEG_MIRROR_CHANGE;
        float cp     = frame->hip_comp[side];

        Leg_Trq(1.0f, 0.0f, frame->phi[side][0], frame->phi[side][1], force_map);
        Leg_Trq(0.0f, mirror, frame->phi[side][0], frame->phi[side][1], hip_map);
        Leg_Trq(frame->force[side], mirror * frame->hip[side], frame->phi[side][0], frame->phi[side][1], nominal[side]);
        leg->spring_f         = frame->spring_f[side];
        leg->hip_comp         = cp;
        leg->s_req            = frame->force[side] + leg->spring_f;
        leg->p_req            = frame->hip[side] - cp;
        sample.spring_f[side] = leg->spring_f;
        sample.hip_comp[side] = cp;
        sample.turn[side]     = frame->turn[side];
        for (unsigned int joint = 0u; joint < 2u; joint++)
        {
            unsigned int raw   = side == 0u ? joint : 1u - joint;
            unsigned int index = 2u * side + joint;
            float mapped;

            leg->map[joint][0]         = force_map[raw];
            leg->map[joint][1]         = hip_map[raw];
            leg->tau_lo[joint]         = lower[index];
            leg->tau_hi[joint]         = upper[index];
            sample.map[side][joint][0] = force_map[raw];
            sample.map[side][joint][1] = hip_map[raw];
            mapped                     = force_map[raw] * frame->force[side] + hip_map[raw] * frame->hip[side];
            diag.mapping_error         = fmaxf(diag.mapping_error, fabsf(mapped - nominal[side][raw]));
        }
    }
    diag.mapping_us = (float)(DWT->CYCCNT - begin) * us_per_cycle;
    begin           = DWT->CYCCNT;
    Distributor_Calculate(&diag.input, &diag.result);
    if (frame->stepping && diag.result.result_valid &&
        (diag.result.s_alloc[0] > diag.input.leg[0].s_req + 0.01f ||
         diag.result.s_alloc[1] > diag.input.leg[1].s_req + 0.01f))
    {
        // 动作中的接触力上界已经含弹簧；最近可达分配超过上界时不发送。
        diag.failure_reason = 3u;
        diag.failure_count = 1u;
        Distributor_Publish_Diagnostic(&diag);
        return false;
    }
    if (diag.result.tau_req[0] < lower[0] || diag.result.tau_req[0] > upper[0] ||
        diag.result.tau_req[1] < lower[1] || diag.result.tau_req[1] > upper[1]) monitor_total.leg_clamp_lft_cnt++;
    if (diag.result.tau_req[2] < lower[2] || diag.result.tau_req[2] > upper[2] ||
        diag.result.tau_req[3] < lower[3] || diag.result.tau_req[3] > upper[3]) monitor_total.leg_clamp_rgt_cnt++;
    diag.allocation_us = (float)(DWT->CYCCNT - begin) * us_per_cycle;
    if (diag.result.result_valid)
    {
        for (unsigned int joint = 0u; joint < 4u; joint++)
        {
            float protected_tau = Distributor_Clamp(diag.result.tau_alloc[joint], DM_T_MAX);

            if (protected_tau != diag.result.tau_alloc[joint]) diag.protection_count++;
            encoded &= Dm_Motor_Torque_Encode(protected_tau, lower[joint], upper[joint], &code[joint]);
        }
    }
    else
        encoded = false;
    if (!encoded)
    {
        diag.failure_reason = diag.result.result_valid ? 2u : 1u;
        diag.failure_count  = 1u;
        Distributor_Publish_Diagnostic(&diag);
        return false;
    }
    diag.limited_count         = diag.result.request_mask != 0u ? 1u : 0u;
    diag.fallback_count        = diag.result.status == DISTRIBUTOR_STATUS_FALLBACK ? 1u : 0u;
    diag.support_relaxed_count = diag.result.support_relaxed_mask != 0u ? 1u : 0u;
    for (unsigned int bit = 0u; bit < 8u; bit++)
        diag.joint_limited_count[bit] = (diag.result.request_mask & (1u << bit)) != 0u ? 1u : 0u;
    for (unsigned int joint = 0u; joint < 4u; joint++)
        diag.tau_change_max = fmaxf(diag.tau_change_max, fabsf(diag.result.tau_alloc[joint] - diag.result.tau_req[joint]));
    diag.changed_count = diag.tau_change_max > 0.0f ? 1u : 0u;
    for (unsigned int side = 0u; side < 2u; side++)
    {
        diag.s_error_max = fmaxf(diag.s_error_max, fabsf(diag.result.task_error[side][0]));
        diag.p_error_max = fmaxf(diag.p_error_max, fabsf(diag.result.task_error[side][1]));
    }
    begin = DWT->CYCCNT;
    for (unsigned int joint = 0u; joint < 4u; joint++)
    {
        HAL_StatusTypeDef status = Dm_Motor_Torque_Send(motor[joint]->handle, motor[joint]->slave_id, code[joint]);

        sample.offset[joint] = DWT->CYCCNT - sample.cycle;
        sample.code[joint]   = code[joint];
        if (status == HAL_OK) sample.accepted_mask |= (uint8_t)(1u << joint);
    }
    {
        float wl                 = WHEEL_LFT_CMD_POLARITY * Distributor_Clamp(frame->wheel[0], M3508_OUT_MAX / M3508_TO_TORQUE) * M3508_TO_TORQUE;
        float wr                 = WHEEL_RGT_CMD_POLARITY * Distributor_Clamp(frame->wheel[1], M3508_OUT_MAX / M3508_TO_TORQUE) * M3508_TO_TORQUE;
        HAL_StatusTypeDef status = Dji_Send_Group_Encoded(DJI_GROUP_A, (float[4]){wr, wl, 0.0f, 0.0f}, wheel_code);

        if (fabsf(frame->wheel[0]) > M3508_OUT_MAX / M3508_TO_TORQUE ||
            fabsf(frame->wheel[1]) > M3508_OUT_MAX / M3508_TO_TORQUE) diag.wheel_limited_count = 1u;
        sample.offset[4] = DWT->CYCCNT - sample.cycle;
        sample.code[4]   = wheel_code[1];
        sample.code[5]   = wheel_code[0];
        if (status == HAL_OK) sample.accepted_mask |= 48u;
    }
    Execution_Publish(&sample);
    diag.execution          = sample;
    if (frame->land.active)
    {
        for (unsigned int side = 0u; side < 2u; side++)
            if (diag.input.leg[side].s_req > frame->land.nominal[side] &&
                (diag.result.s_alloc[side] + 0.01f < diag.input.leg[side].s_req ||
                 sample.valid_mask != EXECUTION_CHANNEL_MASK || sample.total_s[side] + 0.01f < diag.input.leg[side].s_req ||
                 (sample.accepted_mask & (3u << (2u * side))) != (3u << (2u * side))))
                diag.land_limited_mask |= (uint8_t)(1u << side);
    }
    diag.send_failure_count = sample.accepted_mask != EXECUTION_CHANNEL_MASK ? 1u : 0u;
    diag.send_us = (float)(DWT->CYCCNT - begin) * us_per_cycle;
#if BALANCE_DISTRIBUTOR_COMPARE
    diag.input.strategy = strategy == DISTRIBUTOR_PROJECT ? DISTRIBUTOR_CLIP : DISTRIBUTOR_PROJECT;
    Distributor_Calculate(&diag.input, &diag.compare[0]);
    diag.input.strategy = DISTRIBUTOR_SCALE;
    Distributor_Calculate(&diag.input, &diag.compare[1]);
    diag.input.strategy = strategy;
#endif
    Distributor_Publish_Diagnostic(&diag);
    return true;
}

// 遥测只复制控制拍已完成的分配，不在慢任务重新计算补偿。
void Distributor_Diagnostic(distributor_diag_t *out)
{
    taskENTER_CRITICAL();
    *out = distributor_diag;
    taskEXIT_CRITICAL();
}

// 控制任务读取同拍分配结果，避免复制完整遥测结构。
uint8_t Distributor_Output_Limits(void)
{
    return distributor_diag.result.request_mask ||
           fabsf(distributor_diag.wheel_req[0]) > M3508_OUT_MAX / M3508_TO_TORQUE ||
           fabsf(distributor_diag.wheel_req[1]) > M3508_OUT_MAX / M3508_TO_TORQUE ||
           distributor_diag.result.support_relaxed_mask ? 4u : 0u;
}
