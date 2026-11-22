/**
 * @file  slip.c
 * @brief 防打滑估计：去重力纵向加速度 + 均值通道 KF 融合机体速度
 * @version  v1.6  2026-11-16
 */
#include "slip.h"

#include "code_config.h"
#include "system_key.h"
#include "balance.h"
#include "body.h"
#include "imu.h"
#include "imu_to_world.h"
#include "kalman.h"
#include "air.h"
#include "dji_motor.h"

#include <math.h>
#include <stdbool.h>

/* ====== KF 参数（逐项含义见《防打滑机制框架讲解.md》附录 A） ====== */

//! 打滑遥测实例
volatile slip_status_t slip_status;

static kalman_accel_t slip_kf; // 均值通道单 KF：转向轮速差 ±ω·T/2 左右抵消，无需分侧补偿
static uint8_t slip_inited;
static uint8_t slip_air_prev;         // 上一拍已施加的离地电平
static volatile uint8_t slip_air_req; // 外部离地开关 1=离地
static uint8_t slip_observation;      // 位 0/1 本拍修正来源
#if SLIP_AIR_SLIP_ENABLED
static uint32_t slip_air_hold_ms;    // ms，滑移抑制请求的剩余时限
static float slip_air_reference;    // m/s，抑制期间仅按惯性推进的速度参考
static float slip_air_bias;         // m/s²，事件前锁存的加速度偏置
#endif
#if STEP_ENABLE
static slip_side_detector_t slip_detector;
static slip_side_input_t slip_side_input;
static float slip_weight[2];         // 0～1 可用轮速恢复权重
static uint32_t slip_used_frame[2];  // 最近消费的反馈序号
static bool slip_follow;             // 握手传入，无冲突的低速跟随
static bool slip_recovering;         // 握手传入，双侧接触恢复中
static bool slip_fusion_started;     // 先消费已确认接触的轮速，再执行缺观测预算
static uint32_t slip_prediction_ms;  // ms，接地连续无可用新观测时长
#endif

/* ====== static 辅助 ====== */

// 折回 (-π, π]：imu_world.pitch 是多圈累加值
static float Slip_Wrap_Pi(float ang)
{
    float w = fmodf(ang, TWO_PI);
    if (w > PI)
    {
        w -= TWO_PI;
    }
    else if (w <= -PI)
    {
        w += TWO_PI;
    }
    return w;
}

static void Slip_Bias_Clamp(kalman_accel_t *kf)
{
    if (kf->a_bias > SLIP_BIAS_CLAMP) kf->a_bias = SLIP_BIAS_CLAMP;
    if (kf->a_bias < -SLIP_BIAS_CLAMP) kf->a_bias = -SLIP_BIAS_CLAMP;
}

/* ====== 公有 API ====== */

// 打滑模块初始化函数
void Slip_Init(void)
{
    Kalman_Accel_Init(&slip_kf, body.vel, 0.02f, SLIP_KF_Q_A, SLIP_KF_Q_B, SLIP_KF_R); // p0=0 时首拍修正权重恒 0
    slip_kf.v_max = SLIP_KF_V_MAX;
    slip_kf.gate  = SLIP_KF_GATE;
    slip_air_req  = 0u;
    slip_air_prev = 0u;
#if SLIP_AIR_SLIP_ENABLED
    slip_air_hold_ms = 0u;
#endif
#if STEP_ENABLE
    slip_detector = (slip_side_detector_t){.status = {.side = SLIP_SIDE_UNKNOWN}};
    slip_side_input = (slip_side_input_t){0};
    slip_weight[0] = slip_weight[1] = 0.0f;
    slip_used_frame[0] = slip_used_frame[1] = 0u;
    slip_follow = slip_recovering = slip_fusion_started = false;
    slip_prediction_ms = 0u;
#endif
    // 兜底：控制环若早于本模块首拍读 x0[1]，也拿到运动学值而不是未初始化的 0
    body.vel_kalman = body.vel;
    //! 必须最后置位：它是"状态已就绪"的唯一判据，提前置位会让消费者读到半初始化的 KF
    slip_inited = 1u;
}

// 由控制任务写入下一估计拍所需的方向与动作上下文。
void Slip_Set_Step(bool follow, float intent, const float propulsion[2], uint8_t action_mask, bool recovering)
{
#if STEP_ENABLE
    slip_side_input.intent = intent;
    slip_side_input.propulsion[0] = propulsion[0];
    slip_side_input.propulsion[1] = propulsion[1];
    slip_side_input.action_mask = action_mask;
#else
    (void)intent;
    (void)propulsion;
    (void)action_mask;
#endif
#if STEP_ENABLE
    slip_follow = follow;
    slip_recovering = recovering;
#else
    (void)follow;
    (void)recovering;
#endif
}

// 握手窗口内读取估计快照，遥测通过控制任务发布的副本读取。
void Slip_Side_Status(slip_side_status_t *out)
{
#if STEP_ENABLE
    *out = slip_detector.status;
#else
    *out = (slip_side_status_t){.side = SLIP_SIDE_UNKNOWN};
#endif
}

// 返回本拍实际用于修正的来源位。
uint8_t Slip_Observation(void)
{
    return slip_observation;
}

// 缺观测不会永久依赖惯性量；控制任务使用该状态触发既有安全停机。
bool Slip_Prediction_Expired(void)
{
#if STEP_ENABLE
    return slip_prediction_ms >= SLIP_PREDICT_MAX_MS;
#else
    return false;
#endif
}

// 外部离地开关设置函数
void Slip_Set_Air(uint8_t air)
{
    slip_air_req = (air != 0u) ? 1u : 0u;
}

// KF 每拍只预测一次。有本拍支持力时，在修正前提交滑移与 AIR 仲裁。
static void Slip_Update_Frame(bool support_ready, bool evaluate_air)
{
    //! 兜底：init_task 必须先于估计任务首拍跑 Slip_Init，万一没跑就原地补一次
    if (!slip_inited)
    {
        Slip_Init();
    }

    // 1. 去重力：低头为正，a_lon = (f_x + sinθ)·g
    //! θ 必须用 imu_world.pitch 折回值：body.ang_pitch 过 90° 折返会让有效性门形同虚设
    float pitch = Slip_Wrap_Pi(imu_world.pitch);
    bool acc_ok = imu.online && (fabsf(pitch) <= DEG_TO_RAD(30.0f));
    float a_lon = 0.0f;
    if (acc_ok)
    {
        a_lon = (imu.acc[IMU_ACC_X] + sinf(pitch)) * GRAVITY;
    }
    if (!isfinite(a_lon))
    {
        a_lon = 0.0f;
        acc_ok = false;
    }

    // 2. 外部离地开关：只认电平和它的 1→0 降沿
    uint8_t air = slip_air_req;
#if STEP_ENABLE && SLIP_AIR_SLIP_ENABLED
    uint8_t air_guard_usable = 3u; // 位 0/1，与抗滑移惯性参考一致的轮速
#endif

    // 3. KF 预测 + 落地沿重定基：air=1 期间只 Predict；落地沿用轮速清账 a_bias 与 P
    float z = body.vel;
    if (air || !Slip_Prediction_Expired())
        Kalman_Accel_Predict(&slip_kf, a_lon, CTRL_DT);
#if SLIP_WHEEL_STAMP_ENABLED
    const uint32_t now = HAL_GetTick();
    const dji_motor_t *wheel[2] = {&dji_wheel_lft, &dji_wheel_rgt};
    uint8_t fresh_mask = 0u;

    for (unsigned int side = 0u; side < 2u; side++)
        if (wheel[side]->frame_seq != 0u && now - wheel[side]->frame_tick <= SLIP_FEEDBACK_MS)
            fresh_mask |= (uint8_t)(1u << side);
#endif
#if STEP_ENABLE
    const uint8_t contact = air_status.side_contact_mask & air_status.side_valid_mask;

    slip_side_input.velocity[0] = body.vel_lft;
    slip_side_input.velocity[1] = body.vel_rgt;
    slip_side_input.predicted = slip_kf.vel;
    slip_side_input.acceleration = a_lon;
    slip_side_input.yaw_rate = body.omg_yaw;
    slip_side_input.valid_mask = 0u;
    slip_side_input.now = now;
    slip_side_input.air = air != 0u;
    for (unsigned int side = 0u; side < 2u; side++)
        slip_side_input.frame[side] = wheel[side]->frame_seq;
    if (acc_ok && now - imu.last_rx_tick <= SLIP_FEEDBACK_MS)
        slip_side_input.valid_mask = fresh_mask & contact;
    Slip_Side_Update(&slip_detector, &slip_side_input);
#endif
#if SLIP_AIR_SLIP_ENABLED
    if (evaluate_air)
    {
        const float turn = SLIP_HALF_TRACK * body.omg_yaw;
        // 打滑期间按事件前的偏置推进参考，避免双轮同滑把参考拖到轮速上。
        if (slip_air_hold_ms == 0u)
        {
            slip_air_reference = slip_kf.vel;
            slip_air_bias = slip_kf.a_bias;
        }
        else
        {
            slip_air_reference = fmaxf(-SLIP_KF_V_MAX, fminf(SLIP_KF_V_MAX,
                slip_air_reference + CTRL_DT * (a_lon - slip_air_bias)));
        }
        const float lft = body.vel_lft + turn - slip_air_reference;
        const float rgt = body.vel_rgt - turn - slip_air_reference;

        // 该判据只屏蔽均值 AIR，不等待驱动方向、侧别或收腿确认。
        // 单侧或侧别不明表现为补偿后两侧分离；双侧同滑表现为共同偏离惯性预测。
        if (!support_ready || air || !acc_ok || now - imu.last_rx_tick > SLIP_FEEDBACK_MS ||
            fresh_mask != 3u || !isfinite(lft) || !isfinite(rgt))
            slip_air_hold_ms = 0u;
        else if ((air_status.fn_det_lft >= SLIP_CONTACT_LOW || air_status.fn_det_rgt >= SLIP_CONTACT_LOW) &&
                 (fabsf(lft - rgt) >= SLIP_ENTER ||
                  (fabsf(lft) >= SLIP_ENTER && fabsf(rgt) >= SLIP_ENTER)))
            slip_air_hold_ms = SLIP_AIR_SLIP_HOLD_MS;
        else if (slip_air_hold_ms > 0u)
            slip_air_hold_ms = slip_air_hold_ms > CTRL_PERIOD_MS ? slip_air_hold_ms - CTRL_PERIOD_MS : 0u;

        Air_Set_Slip_Inhibit(slip_air_hold_ms != 0u);
        Air_Update_Detect();
#if STEP_ENABLE
        if (air_status.slip_inhibited)
        {
            air_guard_usable = (uint8_t)((fabsf(lft) < SLIP_ENTER ? 1u : 0u) |
                                         (fabsf(rgt) < SLIP_ENTER ? 2u : 0u));
        }
#endif
        const uint8_t next_air = Air_Get();
        if (next_air && !air && Slip_Prediction_Expired())
            Kalman_Accel_Predict(&slip_kf, a_lon, CTRL_DT);
        air = next_air;
    }
#else
    (void)support_ready;
    (void)evaluate_air;
#endif
    const bool landed = slip_air_prev != 0u && air == 0u;
    if (landed)
    {
        Kalman_Accel_Reset(&slip_kf, z);
    }
    float diff = z - slip_kf.vel; // 纯遥测

    // 4. 融合：真 AIR 保留原有纯预测和整车触地重定基语义。
    slip_observation = 0u;
#if STEP_ENABLE
    const bool select_sides = !air && !landed &&
                              (slip_side_input.action_mask || (slip_follow && (contact != 0u || slip_fusion_started)));

    if (select_sides)
    {
        uint8_t usable = slip_recovering && contact == 3u ? slip_detector.status.valid_mask :
                         slip_detector.status.usable_mask;
#if SLIP_AIR_SLIP_ENABLED
        usable &= air_guard_usable;
#endif
        float sum = 0.0f;
        float total = 0.0f;
        uint8_t new_frames = 0u;

        for (unsigned int side = 0u; side < 2u; side++)
        {
            const uint8_t bit = (uint8_t)(1u << side);
            const bool fresh = slip_used_frame[side] != wheel[side]->frame_seq;

            slip_used_frame[side] = wheel[side]->frame_seq;
            if (fresh) new_frames |= bit;
            if (!(usable & bit))
                slip_weight[side] = 0.0f;
            else
            {
                slip_weight[side] = fminf(1.0f, slip_weight[side] + (float)CTRL_PERIOD_MS / SLIP_FUSION_RAMP_MS);
                if (fresh)
                {
                    sum += slip_detector.status.center[side] * slip_weight[side];
                    total += slip_weight[side];
                    slip_observation |= bit;
                }
            }
        }
        if (total > 0.0f)
        {
            slip_fusion_started = true;
            slip_prediction_ms = 0u;
            slip_kf.r = 2.0f * SLIP_KF_R / total;
            Kalman_Accel_Update(&slip_kf, sum / total);
            slip_kf.r = SLIP_KF_R;
        }
        else if (!slip_side_input.action_mask && (fresh_mask & new_frames) == 3u
#if SLIP_AIR_SLIP_ENABLED
                 && air_guard_usable == 3u
#endif
        )
        {
            // 非动作阶段沿用原均值 KF 的野值降权；打滑不冻结驾驶输入。
            Kalman_Accel_Update(&slip_kf, z);
            slip_observation = 3u;
            slip_prediction_ms = 0u;
        }
        else if (slip_fusion_started && slip_prediction_ms < SLIP_PREDICT_MAX_MS)
            slip_prediction_ms += CTRL_PERIOD_MS;
    }
    else
    {
        slip_weight[0] = slip_weight[1] = 0.0f;
        slip_used_frame[0] = wheel[0]->frame_seq;
        slip_used_frame[1] = wheel[1]->frame_seq;
        slip_fusion_started = false;
        slip_prediction_ms = 0u;
    }
    if (!air && !select_sides)
#else
    if (!air)
#endif
    {
        Kalman_Accel_Update(&slip_kf, z);
        slip_observation = 3u;
    }
    Slip_Bias_Clamp(&slip_kf);

    slip_air_prev         = air;
    slip_status.diff      = diff;
    slip_status.vel_fused = slip_kf.vel;
    slip_status.a_lon     = a_lon;
    slip_status.air       = air;

    // 控制环从 body 取 x0[1] 与 pos 积分，取源由 BALANCE_USE_SLIP 决定；关掉后 KF 与遥测照跑
#if BALANCE_USE_SLIP
    body.vel_kalman = slip_kf.vel;
#else
    body.vel_kalman = body.vel;
#endif
}

// 已有 AIR 电平由调用者提供时，仅更新速度融合。
void Slip_Update(void)
{
    Slip_Update_Frame(false, false);
}

// 生产估计入口。抑制关闭时按原顺序运行，保留既有 AIR/落地语义。
void Slip_Air_Update(void)
{
#if SLIP_AIR_SLIP_ENABLED
    const bool support_ready = Air_Update_Support();

    Slip_Set_Air(Air_Get());
    Slip_Update_Frame(support_ready, true);
#else
    Air_Update();
    Slip_Set_Air(Air_Get());
    Slip_Update();
#endif
}
