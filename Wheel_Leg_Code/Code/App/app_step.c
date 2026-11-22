/** @file  app_step.c  @brief 单侧固定行程收腿、保持与原长恢复  @version  v1.5  2026-11-16 */
#include "app_step.h"
#include "code_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

#if STEP_ENABLE
#include "air.h"
#include "app_safety.h"
#include "body.h"
#include "distributor.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "imu.h"
#include "leg.h"
#include "slip.h"
#include "task_sync.h"
#endif

static step_status_t step_published; // 临界区发布，供遥测与调试任务读取

// 启动任务在估计与控制握手启动前初始化。
void App_Step_Init(void)
{
#if STEP_ENABLE
    App_Step_Exit(STEP_REASON_DISABLED);
#endif
    step_published = (step_status_t){.slip = {.side = SLIP_SIDE_UNKNOWN}};
}

// 复制最近一次发布的完整遥测。
void App_Step_Status(step_status_t *status)
{
    taskENTER_CRITICAL();
    *status = step_published;
    taskEXIT_CRITICAL();
}

// 五次 Hermite 曲线，初始加速度、末速度和末加速度均为零。
void App_Step_Profile(float start, float target, float initial_rate, float duration,
                      float time, float *length, float *rate)
{
    if (time <= 0.0f)
    {
        *length = start;
        *rate = initial_rate;
        return;
    }
    if (time >= duration)
    {
        *length = target;
        *rate = 0.0f;
        return;
    }
    const float u = time / duration;
    const float s = u * u * u * (10.0f + u * (-15.0f + 6.0f * u));
    const float ds = 30.0f * u * u * (1.0f - u) * (1.0f - u);
    const float v = u * (1.0f + u * u * (-6.0f + u * (8.0f - 3.0f * u)));
    const float dv = 1.0f + u * u * (-18.0f + u * (32.0f - 15.0f * u));

    *length = start + (target - start) * s + initial_rate * duration * v;
    *rate = (target - start) * ds / duration + initial_rate * dv;
}

// 动作内外共用同一出力交接斜坡，time 为进入交接后的秒数。
float App_Step_Blend(float time)
{
    float blend, rate;

    App_Step_Profile(0.0f, 1.0f, 0.0f, 0.001f * STEP_PREPARE_MS, time, &blend, &rate);
    return blend;
}

#if STEP_ENABLE
static step_status_t step_state;
static execution_sample_t step_last; // 进入阶段前的最近接纳快照
static execution_sample_t step_seed; // 当前出力斜坡的始端
static uint32_t step_enter_tick;     // ms，阶段起点
static uint32_t step_bad_send;       // ms，连续发送失败
static float step_initial_len;       // m，入场动作腿长度
static float step_leg_angle[2];      // rad，初始世界系腿角
static float step_start, step_goal;  // m，轨迹两端
static float step_duration;          // s，轨迹时长
static float step_motion;            // m/s，保持段允许的原方向速度
static bool step_fault;              // 未找到可恢复的支撑
static bool step_auto_armed = true;         // 动作外连续确认 NONE 后重新武装

// 上游几何和策略使用同一个对称限幅。
static float App_Step_Clamp(float value, float limit)
{
    return fmaxf(-limit, fminf(limit, value));
}

// 动作所需的控制和观测必须编入；固定行程与速度必须为正。
static bool App_Step_Config_Ready(void)
{
    return BALANCE_USE_SLIP && BALANCE_USE_AIR_CHECK &&
           isfinite(STEP_STROKE) && STEP_STROKE > 0.0f &&
           isfinite(STEP_MOVE_RATE) && STEP_MOVE_RATE > 0.0f &&
           STEP_SUPPORT_MAX > (LEG_MASS_TOTAL - WHEEL_MASS) * GRAVITY &&
           STEP_WHEEL_MAX <= M3508_OUT_MAX / M3508_TO_TORQUE &&
           STEP_BRAKE_MAX <= STEP_WHEEL_MAX;
}

// 右倾为正，左髋高于中心。将支撑髋的竖直投影换算到中心。
static float App_Step_Height(unsigned int support, float *rate)
{
    const float angle = support == 0u ? body.leg_ang_lft : body.leg_ang_rgt;
    const float omega = support == 0u ? body.leg_omg_lft : body.leg_omg_rgt;
    const float len = step_state.length[support];
    const float offset = support == 0u ? -SLIP_HALF_TRACK : SLIP_HALF_TRACK;
    const float projection = len * cosf(angle);
    const float roll_sin = sinf(body.ang_roll);
    const float roll_cos = cosf(body.ang_roll);

    *rate = (step_state.rate[support] * cosf(angle) - len * sinf(angle) * omega) * roll_cos +
            (offset * roll_cos - projection * roll_sin) * body.omg_roll;
    return projection * roll_cos + offset * roll_sin;
}

// 阶段斜坡从最近有效接纳量接入，输入样本在切代前读取。
static void App_Step_Enter(step_phase_t phase, uint32_t now)
{
    step_state.phase = phase;
    step_state.elapsed_ms = 0u;
    step_enter_tick = now;
    if (step_last.valid_mask == EXECUTION_CHANNEL_MASK)
        step_seed = step_last;
}

// 发布控制任务拥有的完整快照。
static void App_Step_Publish(void)
{
    taskENTER_CRITICAL();
    step_published = step_state;
    taskEXIT_CRITICAL();
}

// 仅活动阶段持有专用分配权限；等待滑移消退时使用正常控制。
bool App_Step_Owns_Output(void)
{
    return step_state.phase >= STEP_RETRACT && step_state.phase <= STEP_RETURN;
}

// 控制层消费故障后交给既有安全锁存和失能链。
bool App_Step_Fault(void)
{
    return step_fault;
}

// 安全退出清除动作记忆，保留终止遥测。
void App_Step_Exit(step_reason_t reason)
{
    if (App_Step_Owns_Output() && !step_fault) step_state.terminated = reason;
    step_state.phase = STEP_IDLE;
    step_state.side = 0u;
    step_state.elapsed_ms = 0u;
    step_state.target_rate = 0.0f;
    step_state.accepted_mask = 0u;
    step_state.execution_valid = 0u;
    step_state.limit_mask = 0u;
    step_fault = false;
    step_bad_send = 0u;
    Air_Set_Step(0u);
    Slip_Set_Step(false, 0.0f, (float[2]){0.0f, 0.0f}, 0u, false);
    App_Step_Publish();
}

// 控制任务每拍在正常目标映射之后、驻车捕获之前仲裁。
bool App_Step_Update(bool follow, bool air, float intent, float yaw_target)
{
    const uint32_t now = Estimator_Frame_Tick();

    Execution_Latest(&step_last);
    Slip_Side_Status(&step_state.slip);
    step_state.observation = Slip_Observation();
    step_state.contact_valid = air_status.side_valid_mask;
    step_state.contact_mask = air_status.side_contact_mask;
    step_state.unloaded_mask = air_status.side_unloaded_mask;
    step_state.length[0] = leg_lft.len;
    step_state.length[1] = leg_rgt.len;
    step_state.rate[0] = leg_lft.d_len;
    step_state.rate[1] = leg_rgt.d_len;
    step_state.roll = body.ang_roll;
    step_state.roll_rate = body.omg_roll;
    step_state.elapsed_ms = now - step_enter_tick;

    const bool accepted = step_last.valid_mask == EXECUTION_CHANNEL_MASK &&
                          now - step_last.tick <= SLIP_FEEDBACK_MS;
    const float propulsion[2] = {accepted ? step_last.model_u[0] : 0.0f,
                                 accepted ? step_last.model_u[1] : 0.0f};
    const uint32_t feedback_tick = HAL_GetTick();
    const bool feedback = active_flag == APP_ENABLE && imu.online &&
                          feedback_tick - imu.last_rx_tick <= SLIP_FEEDBACK_MS &&
                          dm_f_lft.enabled && dm_b_lft.enabled && dm_f_rgt.enabled && dm_b_rgt.enabled &&
                          isfinite(leg_lft.d_len) && isfinite(leg_rgt.d_len) &&
                          isfinite(body.ang_pitch) && isfinite(body.ang_roll) &&
                          isfinite(body.leg_ang_lft) && isfinite(body.leg_ang_rgt) &&
                          isfinite(body.omg_pitch) && isfinite(body.omg_roll) && isfinite(body.vel_kalman) &&
                          isfinite(body.leg_omg_lft) && isfinite(body.leg_omg_rgt) &&
                          isfinite(body.whl_vel_lft) && isfinite(body.whl_vel_rgt) &&
                          (dji_wheel_lft.frame_seq != 0u && dji_wheel_rgt.frame_seq != 0u) &&
                          feedback_tick - dji_wheel_lft.frame_tick <= SLIP_FEEDBACK_MS &&
                          feedback_tick - dji_wheel_rgt.frame_tick <= SLIP_FEEDBACK_MS;
    (void)yaw_target;

    // NONE 由检测器完成消退确认，动作层不再重复计时。
    if (!App_Step_Owns_Output() && step_state.slip.side == SLIP_SIDE_NONE)
        step_auto_armed = true;
    if (step_state.phase == STEP_WAIT_CLEAR && step_auto_armed)
    {
        step_state.phase = STEP_IDLE;
        step_state.side = 0u;
    }
    if (step_auto_armed && step_state.phase == STEP_IDLE &&
        (step_state.slip.side == SLIP_SIDE_LEFT || step_state.slip.side == SLIP_SIDE_RIGHT))
    {
        const unsigned int action = step_state.slip.side == SLIP_SIDE_LEFT ? 0u : 1u;
        step_reason_t reject = STEP_REASON_NONE;

        if (!App_Step_Config_Ready()) reject = STEP_REASON_CONFIG;
        else if (!follow || air) reject = STEP_REASON_MODE;
        else if (!feedback || !accepted) reject = STEP_REASON_FEEDBACK;
        else if (!(leg_lft.len >= Leg_Len_Min && leg_lft.len <= Leg_Len_Max &&
                   leg_rgt.len >= Leg_Len_Min && leg_rgt.len <= Leg_Len_Max)) reject = STEP_REASON_FEEDBACK;

        step_state.rejected = reject;
        if (reject == STEP_REASON_NONE)
        {
            float rate;

            // 真正开始后才消费本次事件；暂时不满足入口条件时允许同一事件重试。
            step_auto_armed = false;
            step_state.side = (uint8_t)(1u << action);
            step_state.terminated = STEP_REASON_NONE;
            step_state.target[0] = step_state.length[0];
            step_state.target[1] = step_state.length[1];
            step_state.target_rate = 0.0f;
            step_state.height_ref = App_Step_Height(1u - action, &rate);
            step_initial_len = step_state.length[action];
            step_leg_angle[0] = body.leg_ang_lft;
            step_leg_angle[1] = body.leg_ang_rgt;
            step_start = step_initial_len;
            step_goal = fmaxf(Leg_Len_Min, step_initial_len - STEP_STROKE);
            step_duration = (step_start - step_goal) / STEP_MOVE_RATE;
            step_bad_send = 0u;
            App_Step_Enter(STEP_RETRACT, now);
        }
    }

    if (App_Step_Owns_Output())
    {
        const unsigned int action = step_state.side == 1u ? 0u : 1u;

        if (air)
        {
            step_state.terminated = STEP_REASON_AIR;
            App_Step_Enter(STEP_WAIT_CLEAR, now);
        }
        else if (!feedback || step_bad_send >= STEP_FAILURE_MS ||
                 !(leg_lft.len >= Leg_Len_Min && leg_lft.len <= Leg_Len_Max &&
                   leg_rgt.len >= Leg_Len_Min && leg_rgt.len <= Leg_Len_Max))
        {
            step_state.terminated = STEP_REASON_FEEDBACK;
            step_fault = true;
        }
        else
        {
            switch (step_state.phase)
            {
                case STEP_RETRACT: // 目标和实测均到位后才开始保持计时
                case STEP_RETURN:  // 回到入场实测长度后交接双侧承载
                    if (step_state.elapsed_ms >= 1000.0f * step_duration &&
                        fabsf(step_state.length[action] - step_goal) <= STEP_TRACK_ERROR)
                    {
                        step_state.target[action] = step_goal;
                        step_state.target_rate = 0.0f;
                        if (step_state.phase == STEP_RETURN)
                            step_state.terminated = STEP_REASON_COMPLETE;
                        App_Step_Enter(step_state.phase == STEP_RETRACT ? STEP_HOLD : STEP_WAIT_CLEAR, now);
                    }
                    else if (step_state.elapsed_ms >= 1000.0f * step_duration + STEP_SETTLE_MS)
                    {
                        step_state.terminated = STEP_REASON_TIMEOUT;
                        step_fault = true;
                    }
                    break;
                case STEP_HOLD: // 保持固定短腿目标，不依据承载或滑移状态改变行程
                    if (step_state.elapsed_ms >= STEP_HOLD_MS)
                    {
                        step_start = step_goal;
                        step_goal = step_initial_len;
                        App_Step_Enter(STEP_RETURN, now);
                    }
                    break;
                default: // 空闲和等待滑移消退不持有此分支
                    break;
            }
        }
    }

    const bool owns = App_Step_Owns_Output();
    step_motion = App_Step_Clamp(intent, STEP_VEL_MAX);
    Slip_Set_Step(follow && !air, follow && !air ? intent : 0.0f, propulsion,
                  owns ? step_state.side : 0u, false);
    Air_Set_Step(owns ? step_state.side : 0u);
    if (!owns) App_Step_Publish();
    return owns;
}

// 独立动作控制器，只有支撑髋承担机体俯仰反馈。
void App_Step_Output(const float gravity[2], const float hip_comp[2], float force[2], float hip[2], float wheel[2])
{
    const unsigned int action = step_state.side == 1u ? 0u : 1u;
    const unsigned int support = 1u - action;
    const float time = 0.001f * step_state.elapsed_ms;
    const float pitch = body.ang_pitch - BALANCE_PITCH_REF;
    const float angle = action == 0u ? body.leg_ang_lft : body.leg_ang_rgt;
    const float omega = action == 0u ? body.leg_omg_lft : body.leg_omg_rgt;
    const float wheel_speed = (action == 0u ? WHEEL_LFT_POLARITY * dji_wheel_lft.vel : WHEEL_RGT_POLARITY * dji_wheel_rgt.vel) / REDUCTION_RATIO;
    const bool supporting = (step_state.contact_mask & (1u << support)) != 0u;
    float height_rate;
    float blend;

    if (step_state.phase == STEP_RETRACT || step_state.phase == STEP_RETURN)
    {
        const float distance = step_goal - step_start;

        if (time >= step_duration)
        {
            step_state.target[action] = step_goal;
            step_state.target_rate = 0.0f;
        }
        else
        {
            step_state.target_rate = distance / step_duration;
            step_state.target[action] = step_start + time * step_state.target_rate;
        }
    }
    step_state.height = App_Step_Height(support, &height_rate);
    const float len_error = step_state.target[action] - step_state.length[action];
    const float axial = STEP_AXIAL_KP * len_error + STEP_AXIAL_KD * (step_state.target_rate - step_state.rate[action]);
    const float axial_limit = STEP_AXIAL_MAX;
    const float weight = (LEG_MASS_TOTAL - WHEEL_MASS) * GRAVITY;
    const float height_force = STEP_HEIGHT_KP * (step_state.height_ref - step_state.height) - STEP_HEIGHT_KD * height_rate;
    const float support_angle = support == 0u ? body.leg_ang_lft : body.leg_ang_rgt;
    const float support_force = (weight + height_force) / fmaxf(0.5f, cosf(support_angle) * cosf(body.ang_roll));
    const float pitch_hip = STEP_PITCH_KP * pitch + STEP_PITCH_KD * body.omg_pitch;
    const float action_hip = STEP_HIP_KP * (step_leg_angle[action] - angle) - STEP_HIP_KD * omega;
    const float drive = Slip_Prediction_Expired() ? 0.0f : STEP_WHEEL_KP * (body.vel_kalman - step_motion);
    const float brake = STEP_WHEEL_BRAKE * wheel_speed;

    force[action] = gravity[action] + App_Step_Clamp(axial, axial_limit);
    force[support] = fmaxf(0.0f, fminf(STEP_SUPPORT_MAX, support_force));
    hip[action] = hip_comp[action] + App_Step_Clamp(action_hip, STEP_HIP_MAX);
    hip[support] = hip_comp[support] + App_Step_Clamp(pitch_hip, STEP_HIP_MAX);
    wheel[action] = App_Step_Clamp(brake, STEP_BRAKE_MAX);
    wheel[support] = App_Step_Clamp(drive, STEP_WHEEL_MAX);
    step_state.limit_mask = 0u;
    if (fabsf(axial) > axial_limit || fabsf(action_hip) > STEP_HIP_MAX || fabsf(brake) > STEP_BRAKE_MAX)
        step_state.limit_mask |= (uint8_t)(1u << action);
    if (support_force > STEP_SUPPORT_MAX || fabsf(pitch_hip) > STEP_HIP_MAX || fabsf(drive) > STEP_WHEEL_MAX)
        step_state.limit_mask |= (uint8_t)(1u << support);
    // 每个阶段从最近接纳出力接入；腿长目标走直线，出力交接仍保持连续。
    blend = App_Step_Blend(time);

    for (unsigned int side = 0u; side < 2u; side++)
    {
        force[side] = step_seed.total_s[side] + blend * (force[side] - step_seed.total_s[side]);
        const float seed_hip = step_seed.model_u[side + 2u] + step_seed.hip_comp[side];

        hip[side] = seed_hip + blend * (hip[side] - seed_hip);
        wheel[side] = -step_seed.model_u[side] + blend * (wheel[side] + step_seed.model_u[side]);
    }
    if (!supporting)
    {
        force[support] = gravity[support];
        hip[support] = hip_comp[support];
        wheel[support] = 0.0f;
    }
    step_state.requested[0] = force[0];
    step_state.requested[1] = force[1];

}

// 从统一执行历史读取接纳码逆解量；仅作交接与遥测，不称为实测力。
void App_Step_Accepted(uint8_t limits)
{
    execution_sample_t sample;

    Execution_Latest(&sample);
    step_state.accepted_mask = sample.accepted_mask;
    step_state.execution_valid = sample.valid_mask;
    step_state.accepted[0] = sample.total_s[0];
    step_state.accepted[1] = sample.total_s[1];
    step_state.limit_mask |= limits;
    if (sample.accepted_mask == EXECUTION_CHANNEL_MASK)
        step_bad_send = 0u;
    else
    {
        step_bad_send += CTRL_PERIOD_MS;
        step_state.limit_mask |= 8u;
    }
    App_Step_Publish();
}

// 正常出力限幅仅作原控制器处理，不再阻止单侧动作入口。
void App_Step_Normal_Limits(uint8_t limits)
{
    (void)limits;
}
#endif
