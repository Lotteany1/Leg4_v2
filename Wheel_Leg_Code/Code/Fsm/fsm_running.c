/** @file  fsm_running.c  @brief 平衡运行目标更新、落地恢复与倒地确认  @version  v4.17  2026-11-22 */
#include "fsm_running.h"
#include "air.h"
#include "app_gyro.h"
#include "app_rc.h"
#include "app_step.h"
#include "code_config.h"
#include "slip.h"
#include "body.h"
#include "imu_to_world.h"
#include "leg.h"
#include "music.h"
#include "songs.h"
#include "task_sync.h"
#include "turn_limit.h"
#include "vel_limit.h"
#include "system_key.h"
#include "wrap.h"
#include <math.h>

static fsm_running_behavior_t fsm_running_behavior = FSM_RUNNING_BEHAVIOR_FOLLOW; // 上拍行为 陀螺切入判定
static uint32_t fsm_running_fall_ms                = 0u;                          // ms 连续倾倒累计时长
static fsm_running_position_t fsm_running_position;
static float fsm_running_origin;     // m 本次停车点的累计里程
static uint32_t fsm_running_ramp_ms; // ms 姿态恢复后的权重爬升时间
static bool fsm_running_standing;
static bool fsm_running_entry;      // 平衡入场首拍使用行驶掩码
static bool fsm_running_yaw_active; // 转向开始后保持到松杆制动完成，再锁定实际航向
static uint32_t fsm_running_yaw_stop_ms; // ms 松杆后连续低偏航转速时长
static bool fsm_running_air_prev;   // 落地沿重新捕获
static bool fsm_running_pos_after_land; // 落地后的停车点捕获与权重恢复尚未完成
static fsm_landing_t fsm_running_landing = {.spring_scale = 1.0f};
#if STEP_ENABLE
static bool fsm_running_pos_after_step; // 动作退出后新停车点从零权重起坡
#endif
#if BALANCE_USE_AIR_CHECK
static bool fsm_running_land_contact_pending; // 驻车判稳前沿用双侧卸载重离地条件
static uint32_t fsm_running_land_tick;       // ms 确认落地的估计帧时标
static uint32_t fsm_running_land_stable_ms;  // ms 双腿接触及姿态稳定持续时间
#endif
#if BALANCE_USE_VEL_LIMIT
static vel_limit_t fsm_running_vel; // m/s、m/s² 平衡速度 S 曲线状态 仅控制任务推进
#endif

// 作废驻车原点，里程归估计任务管理。
static void Fsm_Running_Park_Clear(void)
{
    if (fsm_running_standing)
        fsm_running_position.events += 2u;
    fsm_running_position.events &= ~1u;
    fsm_running_position.error = 0.0f;
    fsm_running_position.weight = 0.0f;
    fsm_running_standing = false;
}

#if BALANCE_USE_AIR_CHECK
// AIR 或离开平衡时清除补偿恢复状态和双侧卸载判据。
static void Fsm_Running_Land_Clear(void)
{
    if (fsm_running_land_contact_pending) Air_Set_Landing(0u);
    fsm_running_land_contact_pending = false;
    fsm_running_land_stable_ms       = 0u;
    fsm_running_landing              = (fsm_landing_t){.spring_scale = 1.0f};
}

// 触地即起坡，两腿按同一时间恢复轴向气弹簧补偿，返回恢复窗口是否仍有效。
static bool Fsm_Running_Land_Update(uint32_t now)
{
    const uint32_t elapsed  = now - fsm_running_land_tick;
    const uint32_t duration = FSM_LAND_RECOVER_MS;
    const float start       = fminf(100.0f, fmaxf(0.0f, FSM_LAND_SPRING_START_PCT)) / 100.0f;
    const bool recovering  = elapsed < duration;

    fsm_running_landing.elapsed_ms    = elapsed;
    fsm_running_landing.spring_scale = recovering ? start + (1.0f - start) * ((float)elapsed / duration) : 1.0f;
    return recovering;
}

// 驻车前分别确认双腿接触和速度，避免异号伸缩的均值掩盖运动。
static bool Fsm_Running_Land_Stable(void)
{
    if (air_status.contact_mask != 3u ||
        !(fabsf(leg_lft.d_len) <= FSM_LAND_STABLE_SPEED && fabsf(leg_rgt.d_len) <= FSM_LAND_STABLE_SPEED &&
          fabsf(body.ang_pitch) <= FSM_LAND_STABLE_ANGLE && fabsf(body.ang_roll) <= FSM_LAND_STABLE_ANGLE &&
          fabsf(body.omg_pitch) <= FSM_LAND_STABLE_OMEGA && fabsf(body.omg_roll) <= FSM_LAND_STABLE_OMEGA))
        fsm_running_land_stable_ms = 0u;
    else if (fsm_running_land_stable_ms < FSM_LAND_STABLE_MS)
        fsm_running_land_stable_ms += CTRL_PERIOD_MS;
    return fsm_running_land_stable_ms >= FSM_LAND_STABLE_MS;
}
#endif

// 退出平衡或出力停止后，下次入场重新等待完整落地事件。
void Fsm_Running_Exit(void)
{
#if STEP_ENABLE
    App_Step_Exit(STEP_REASON_DISABLED);
    fsm_running_pos_after_step = false;
#endif
    Fsm_Running_Park_Clear();
    fsm_running_yaw_active     = false;
    fsm_running_yaw_stop_ms    = 0u;
    fsm_running_air_prev       = false;
    fsm_running_pos_after_land = false;
#if BALANCE_USE_AIR_CHECK
    Fsm_Running_Land_Clear();
#endif
}

fsm_landing_t Fsm_Running_Landing(void)
{
    fsm_landing_t landing = fsm_running_landing;
#if BALANCE_USE_AIR_CHECK
    landing.contact_pending = fsm_running_land_contact_pending;
#endif
    return landing;
}

fsm_running_position_t Fsm_Running_Position(void)
{
    return fsm_running_position;
}

// 入场保持实测腿长和高度，首拍使用行驶掩码。
void Fsm_Running_Enter(void)
{
    fsm_running_fall_ms      = 0u;
    fsm_running_behavior     = FSM_RUNNING_BEHAVIOR_FOLLOW;
    fsm_running_entry        = true;
    Fsm_Running_Exit();

    body.vel_tgt         = 0.0f;
    body.omg_yaw_tgt     = 0.0f;
    body.ang_yaw_tgt     = body.ang_yaw;
    body.leg_len_tgt_lft = LEG_LEN_CLAMP((leg_lft.len + leg_rgt.len) * 0.5f);
    body.leg_len_tgt_rgt = body.leg_len_tgt_lft;
    body.h_tgt           = LEG_LEN_CLAMP(body.h_body);
#if BALANCE_USE_VEL_LIMIT
    Vel_Limit_Init(&fsm_running_vel);
#endif
    App_Gyro_Reset();
}

// ! 平衡目标：先读摇杆，台阶接管就返回；其余拍更新陀螺和限速，再选空中、行进或驻车。
balance_ctrl_mode_t Fsm_Running_Tick(fsm_running_behavior_t behavior, const dr16_t *rc)
{
#if BALANCE_USE_AIR_CHECK
    const bool air = Air_Get() != 0u;
#else
    const bool air = false;
#endif
    const bool landed = fsm_running_air_prev && !air;
    const bool entering = fsm_running_entry;
#if STEP_ENABLE
    // 下面的 App_Step_Update 可能结束动作，先记住拍初是谁占用输出。
    const bool step_owned = App_Step_Owns_Output();
    const float held_height = body.h_tgt;
    const float held_len_lft = body.leg_len_tgt_lft;
    const float held_len_rgt = body.leg_len_tgt_rgt;
#endif

    fsm_running_entry = false;
    if (landed)
        body.ang_yaw_tgt = body.ang_yaw;
    fsm_running_air_prev = air;

    // 先按遥控更新目标，台阶用本拍驾驶意图判断能否开始。
    App_Rc_Update_Targets(&body, rc);

#if STEP_ENABLE
    const bool follow = !entering && behavior == FSM_RUNNING_BEHAVIOR_FOLLOW &&
                        !App_Gyro_Running() && !landed && !fsm_running_landing.active &&
                        !fsm_running_pos_after_land;

    if (App_Step_Update(follow, air, body.vel_tgt, body.omg_yaw_tgt))
    {
        Fsm_Running_Park_Clear();
        body.h_tgt = held_height;
        body.leg_len_tgt_lft = held_len_lft;
        body.leg_len_tgt_rgt = held_len_rgt;
        body.ang_yaw_tgt = body.ang_yaw;
        body.omg_yaw_tgt = 0.0f;
        fsm_running_yaw_active = false;
        fsm_running_yaw_stop_ms = 0u;
        body.vel_tgt = 0.0f;
#if BALANCE_USE_VEL_LIMIT
        Vel_Limit_Reset(&fsm_running_vel, 0.0f);
#endif
        return LEG_CTRL_MODE_STEP;
    }
    if (step_owned && !air)
    {
        // 从动作结束目标接回，避免恢复到动作前遗留的不同腿长目标。
        step_status_t step;

        App_Step_Status(&step);
        body.h_tgt = LEG_LEN_CLAMP(body.h_body);
        body.leg_len_tgt_lft = step.target[0];
        body.leg_len_tgt_rgt = step.target[1];
        body.vel_tgt = body.omg_yaw_tgt = 0.0f;
        body.ang_yaw_tgt = body.ang_yaw;
        fsm_running_yaw_active = false;
        fsm_running_yaw_stop_ms = 0u;
        fsm_running_pos_after_step = true;
        return LEG_CTRL_MODE_MOVING;
    }
#endif

    // 切入陀螺时锁方向和本拍高度，重新进入平衡时重新取值。
    if (behavior == FSM_RUNNING_BEHAVIOR_GYRO && fsm_running_behavior != FSM_RUNNING_BEHAVIOR_GYRO)
    {
        // 切入陀螺行为
        App_Gyro_Enter();
#if APP_USE_MUSIC
        Music_Play(SONG_IDX_LANHUACAO);
#endif
    }
    else if (behavior != FSM_RUNNING_BEHAVIOR_GYRO && fsm_running_behavior == FSM_RUNNING_BEHAVIOR_GYRO)
    {
#if APP_USE_MUSIC
        Music_Stop();
#endif
    }
    fsm_running_behavior = behavior;

    // 陀螺行为持续更新；切回跟随后仍逐拍推进退出斜坡，再交还遥控目标。
    App_Gyro_Update(behavior == FSM_RUNNING_BEHAVIOR_GYRO);

    // 按最终旋转指令缩放直行速度目标，保留前后方向与旋转目标。
    Turn_Limit_Update(&body);

#if BALANCE_USE_VEL_LIMIT
    // 对缩放后的最终速度目标做 S 曲线 零目标优先撤掉原方向加速再制动 不改变偏航目标
    Vel_Limit_Update(&fsm_running_vel, body.vel_tgt, CTRL_DT);
    body.vel_tgt = fsm_running_vel.vel;
#endif

    // 接地时只积分陀螺混合及转向限制后的最终转速；空中不累计无法执行的欠转角度。
    if (air)
    {
        body.ang_yaw_tgt = body.ang_yaw;
        fsm_running_yaw_active = false;
        fsm_running_yaw_stop_ms = 0u;
    }
    else
    {
        if (body.omg_yaw_tgt != 0.0f)
        {
            body.ang_yaw_tgt += body.omg_yaw_tgt * CTRL_DT;
            fsm_running_yaw_active = true;
            fsm_running_yaw_stop_ms = 0u;
        }
        else if (fsm_running_yaw_active)
        {
            // 松杆先撤掉欠转角度，由零转速目标制动；停稳前不锁点，避免继续追旧目标。
            const float cos_pitch = cosf(body.ang_pitch);
            const float yaw_numerator = sinf(body.ang_roll) * body.omg_pitch + cosf(body.ang_roll) * body.omg_yaw;

            body.ang_yaw_tgt = body.ang_yaw;
            if (cos_pitch > BODY_EULER_COS_MIN && fabsf(yaw_numerator) <= FSM_RUNNING_YAW_STOP_OMEGA * cos_pitch)
                fsm_running_yaw_stop_ms += CTRL_PERIOD_MS;
            else
                fsm_running_yaw_stop_ms = 0u;
            if (fsm_running_yaw_stop_ms >= FSM_RUNNING_YAW_STOP_MS)
                fsm_running_yaw_active = false;
        }

        // 连续航向不折返到 ±π；限制欠转量，避免卡住或打滑后积攒过大的追角需求。
        body.ang_yaw_tgt = fminf(body.ang_yaw + FSM_RUNNING_YAW_ERROR_MAX,
                                fmaxf(body.ang_yaw - FSM_RUNNING_YAW_ERROR_MAX, body.ang_yaw_tgt));
    }

#if BALANCE_USE_AIR_CHECK
    if (air)
    {
        Fsm_Running_Park_Clear();
        if (fsm_running_land_contact_pending) Fsm_Running_Land_Clear();
        return LEG_CTRL_MODE_AIR;
    }
    if (landed)
    {
        const float len_target = LEG_LEN_CLAMP(FSM_LAND_LEN_TGT);

        // 确认触地当拍直接写入接地高度和腿长目标。
        body.leg_len_tgt_lft = len_target;
        body.leg_len_tgt_rgt = len_target;
        body.h_tgt           = len_target;
        Fsm_Running_Park_Clear();
        fsm_running_land_tick            = Estimator_Frame_Tick();
        fsm_running_landing.active       = true;
        fsm_running_land_contact_pending = true;
        fsm_running_pos_after_land       = true;
        Air_Set_Landing(1u);
    }
    if (fsm_running_landing.active)
    {
        const bool recovering = Fsm_Running_Land_Update(Estimator_Frame_Tick());

        if (!landed && !recovering)
            fsm_running_landing = (fsm_landing_t){.spring_scale = 1.0f};
        // 恢复时间同时供高度合力交接和速度通道使用，之后累计驻车判稳。
        return LEG_CTRL_MODE_MOVING;
    }
    const bool land_park_ready = !fsm_running_pos_after_land || Fsm_Running_Land_Stable();
    if (land_park_ready && fsm_running_land_contact_pending)
    {
        Air_Set_Landing(0u);
        fsm_running_land_contact_pending = false;
    }
#else
    const bool land_park_ready = true;
#endif
#if STEP_ENABLE
    if (Slip_Prediction_Expired())
    {
        // 轮速无法校正惯性推算时，暂停纵向速度追踪和驻车锁点。
        body.vel_tgt = body.vel_kalman;
        Fsm_Running_Park_Clear();
        return LEG_CTRL_MODE_MOVING;
    }
#endif
    if (entering || behavior != FSM_RUNNING_BEHAVIOR_FOLLOW ||
        body.vel_tgt != 0.0f || body.omg_yaw_tgt != 0.0f)
    {
        Fsm_Running_Park_Clear();
        return LEG_CTRL_MODE_MOVING;
    }

    const bool legs_valid = fabsf(body.leg_ang_lft) <= FSM_RUNNING_POS_LEG_ANG_TH &&
                            fabsf(body.leg_ang_rgt) <= FSM_RUNNING_POS_LEG_ANG_TH;

    bool captured_after_land = false;
    if (!fsm_running_standing)
    {
#if BALANCE_USE_SLIP
        const float velocity = body.vel_kalman;
#else
        const float velocity = body.vel;
#endif

        if (!land_park_ready || !(fabsf(velocity) <= FSM_RUNNING_STOP_VEL_TH) || !legs_valid || !isfinite(body.pos))
            return LEG_CTRL_MODE_MOVING;

        fsm_running_origin = body.pos;
        fsm_running_position.capture++;
        captured_after_land = fsm_running_pos_after_land;
#if STEP_ENABLE
        captured_after_land = captured_after_land || fsm_running_pos_after_step;
#endif
        fsm_running_ramp_ms = captured_after_land ? 0u : FSM_RUNNING_POS_RAMP_MS;
        fsm_running_standing = true;
    }

    fsm_running_position.error = body.pos - fsm_running_origin;
    if (!(fabsf(fsm_running_position.error) < FSM_RUNNING_POS_RECAPTURE))
    {
        Fsm_Running_Park_Clear();
        return LEG_CTRL_MODE_MOVING;
    }

    // 腿角异常只暂停位置反馈，恢复后用原停车点逐步接回驻车权重；速度阻尼继续保留。
    if (!legs_valid)
    {
        fsm_running_ramp_ms = 0u;
        fsm_running_position.events |= 1u;
    }
    else
    {
        fsm_running_position.events &= ~1u;
        if (!captured_after_land && fsm_running_ramp_ms < FSM_RUNNING_POS_RAMP_MS)
            fsm_running_ramp_ms += CTRL_PERIOD_MS;
        if (fsm_running_ramp_ms >= FSM_RUNNING_POS_RAMP_MS)
        {
            fsm_running_pos_after_land = false;
#if STEP_ENABLE
            fsm_running_pos_after_step = false;
#endif
        }
    }
    fsm_running_position.weight = FSM_RUNNING_POS_WEIGHT * fminf(1.0f, (float)fsm_running_ramp_ms / FSM_RUNNING_POS_RAMP_MS);
    return LEG_CTRL_MODE_STANDING;
}

// 每次调用推进一拍倒地确认 pitch roll 与双腿世界系倾角联判 连续越阈满确认时长 回窗清零
bool Fsm_Running_Fall_Update(void)
{
    float pitch_wrap = Wrap_Pi(imu_world.pitch);
    bool tilted      = (fabsf(pitch_wrap) > FSM_RUNNING_FALL_PITCH_TH) ||
                       (fabsf(body.ang_roll) > FSM_RUNNING_FALL_ROLL_TH) ||
                       (fabsf(Imu_To_World_Leg_Ang(leg_lft.ang)) > FSM_RUNNING_FALL_LEG_TH) ||
                       (fabsf(Imu_To_World_Leg_Ang(leg_rgt.ang)) > FSM_RUNNING_FALL_LEG_TH);

    if (!tilted)
    {
        fsm_running_fall_ms = 0u;
        return false;
    }

    if (fsm_running_fall_ms < FSM_RUNNING_FALL_MS)
    {
        fsm_running_fall_ms += CTRL_PERIOD_MS;
    }

    return (fsm_running_fall_ms >= FSM_RUNNING_FALL_MS);
}
