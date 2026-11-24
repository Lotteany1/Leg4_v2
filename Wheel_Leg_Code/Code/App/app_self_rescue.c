/** @file  app_self_rescue.c  @brief 倒地自救阶段推进  @version  v4.5  2026-11-24 */
#include "app_self_rescue.h"
#include "balance.h"
#include "body.h"
#include "code_config.h"
#include "imu_to_world.h"
#include "leg.h"
#include "wrap.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

pid_t sr_pid_leg_len_lft, sr_pid_leg_len_rgt;
pid_t sr_pid_leg_symmetry;
pid_t sr_pid_leg_ang_lft, sr_pid_leg_ang_rgt;
pid_t sr_pid_leg_omg_swing_lft, sr_pid_leg_omg_swing_rgt;
pid_t sr_pid_leg_omg_stand_lft, sr_pid_leg_omg_stand_rgt;

static self_rescue_phase_t phase = SR_PHASE_INIT; // 当前阶段
static bool finished             = false;         // 成功交接标志

static uint16_t retract_ramp_ms = SR_RETRACT_TRQ_RAMP_MS; // ms 收腿力矩斜坡已运行时间
static uint16_t exit_hold = 0u; // 起立进窗累计 ms

static float ang_hold_lft, ang_hold_rgt; // RETRACT 摆角目标 电机系 rad
static float ang_tgt_lft, ang_tgt_rgt;   // ROTATE 摆角目标 电机系 rad
static float len_goal_lft, len_goal_rgt; // 腿长终点 m
static volatile float rescue_dir = -1.0f; //! 唯一初始路径配置 +1 正转扫腿 -1 反转摆腿 支持调试器修改 下次入场锁存

static float active_dir       = +1.0f; // 当前尝试路径 自动切换不覆盖初始配置
static float turnover_dir     = +1.0f; // 翻身扫腿方向，换向重试时反向
static uint8_t retry_count    = 0u;    // 已启动的换向重试次数
static uint32_t attempt_ms    = 0u;    // 当前尝试总时间 ms，阶段切换连续累计
static uint32_t turnover_ms   = 0u;    // 公共翻身累计时间 ms
static uint16_t turnover_hold = 0u;    // 翻正持续时间 ms

static uint16_t stall_ms[2];         // ms 左右腿有动作请求但缺少实测进展的连续时间
static uint16_t obstacle_retract_ms; // ms 换向前收腿时间

static uint16_t swing_cnt = 0u; // SWING 进窗稳持计数 ms
static uint16_t stand_cnt = 0u; // STAND 立腿进窗稳持计数 ms

// 腿长目标每拍朝终点限速走 阶跃经 kd 放大过流锁存
static float Self_Rescue_Slew(float cur, float goal)
{
    float speed = active_dir > 0.0f ? SR_FORWARD_LEG_LEN_RATE_MPS : SR_REVERSE_RETRACT_LEN_RATE_MPS;
    float step  = speed * CTRL_DT;
    float d     = goal - cur;
    if (d > step)
        return cur + step;
    if (d < -step)
        return cur - step;
    return goal;
}

// 对称限幅 立腿段角速度目标用
static float Self_Rescue_Clamp(float x, float max)
{
    if (x > max)
        return max;
    if (x < -max)
        return -max;
    return x;
}

// 反转立腿采用直接速度设定，按机构角最短方向回零；减速角窗内速度连续降到零。
// STAND 和 VERIFY 共用同一曲线，切阶段时不改变速度目标的生成规则。
static float Self_Rescue_Reverse_Stand_Speed(float ang)
{
    float fraction = Self_Rescue_Clamp(-Wrap_Pi(ang) / SR_REVERSE_STAND_SLOWDOWN_ANG_RAD, 1.0f);

    return SR_REVERSE_STAND_RATE_RADPS * fraction;
}

// 积分项限幅 err_sum 钳到积分输出恰为 ±i_max 的量 ki 为 0 直接返回
static void Self_Rescue_Limit_I(pid_t *pid, float i_max)
{
    if (pid->ki == 0.0f)
        return;

    float sum_max = i_max / (pid->ki * CTRL_DT);
    pid->err_sum  = Self_Rescue_Clamp(pid->err_sum, sum_max);
}

// 自救反馈复位
void App_Self_Rescue_Pid_Init(void)
{
    Pid_Init(&sr_pid_leg_len_lft, SR_PID_LEG_LEN_LFT_KP, SR_PID_LEG_LEN_LFT_KI, SR_PID_LEG_LEN_LFT_KD, ONE_MOTOR_OUT_MAX, -ONE_MOTOR_OUT_MAX);
    Pid_Init(&sr_pid_leg_len_rgt, SR_PID_LEG_LEN_RGT_KP, SR_PID_LEG_LEN_RGT_KI, SR_PID_LEG_LEN_RGT_KD, ONE_MOTOR_OUT_MAX, -ONE_MOTOR_OUT_MAX);

    Pid_Init(&sr_pid_leg_symmetry, SR_PID_LEG_SYMMETRY_KP, SR_PID_LEG_SYMMETRY_KI, SR_PID_LEG_SYMMETRY_KD, ONE_MOTOR_OUT_MAX, -ONE_MOTOR_OUT_MAX);

    Pid_Init(&sr_pid_leg_ang_lft, SR_PID_LEG_ANG_LFT_KP, SR_PID_LEG_ANG_LFT_KI, SR_PID_LEG_ANG_LFT_KD, ONE_MOTOR_OUT_MAX, -ONE_MOTOR_OUT_MAX);
    Pid_Init(&sr_pid_leg_ang_rgt, SR_PID_LEG_ANG_RGT_KP, SR_PID_LEG_ANG_RGT_KI, SR_PID_LEG_ANG_RGT_KD, ONE_MOTOR_OUT_MAX, -ONE_MOTOR_OUT_MAX);

    Pid_Init(&sr_pid_leg_omg_swing_lft, SR_REVERSE_SWING_RATE_PID_KP, SR_REVERSE_SWING_RATE_PID_KI, SR_REVERSE_SWING_RATE_PID_KD, SR_REVERSE_SWING_RATE_PID_OUT_MAX, -SR_REVERSE_SWING_RATE_PID_OUT_MAX);
    Pid_Init(&sr_pid_leg_omg_swing_rgt, SR_REVERSE_SWING_RATE_PID_KP, SR_REVERSE_SWING_RATE_PID_KI, SR_REVERSE_SWING_RATE_PID_KD, SR_REVERSE_SWING_RATE_PID_OUT_MAX, -SR_REVERSE_SWING_RATE_PID_OUT_MAX);

    Pid_Init(&sr_pid_leg_omg_stand_lft, SR_REVERSE_STAND_RATE_PID_KP, SR_REVERSE_STAND_RATE_PID_KI, SR_REVERSE_STAND_RATE_PID_KD, SR_REVERSE_STAND_RATE_PID_OUT_MAX, -SR_REVERSE_STAND_RATE_PID_OUT_MAX);
    Pid_Init(&sr_pid_leg_omg_stand_rgt, SR_REVERSE_STAND_RATE_PID_KP, SR_REVERSE_STAND_RATE_PID_KI, SR_REVERSE_STAND_RATE_PID_KD, SR_REVERSE_STAND_RATE_PID_OUT_MAX, -SR_REVERSE_STAND_RATE_PID_OUT_MAX);
}

// Balance 完成输入校验后更新自救 PID 不合成力或力矩
void App_Self_Rescue_Pid_Update(balance_ctrl_mode_t mode)
{
    if (mode != LEG_CTRL_MODE_SELF_RESCUE && mode != LEG_CTRL_MODE_SELF_RESCUE_SWING &&
        mode != LEG_CTRL_MODE_SELF_RESCUE_STAND && mode != LEG_CTRL_MODE_SELF_RESCUE_TURNOVER) return;

    Pid_Update(&sr_pid_leg_len_lft, LEG_LEN_CLAMP(body.leg_len_tgt_lft), body.leg_len_lft);
    Pid_Update(&sr_pid_leg_len_rgt, LEG_LEN_CLAMP(body.leg_len_tgt_rgt), body.leg_len_rgt);
    Pid_Update(&sr_pid_leg_symmetry, 0.0f, Wrap_Pi(leg_lft.ang - leg_rgt.ang));
    switch (mode)
    {
        case LEG_CTRL_MODE_SELF_RESCUE:          // 正转位置环
        case LEG_CTRL_MODE_SELF_RESCUE_TURNOVER: // 公共翻身位置环
            Pid_Update(&sr_pid_leg_ang_lft, body.leg_ang_tgt_lft, leg_lft.ang);
            Pid_Update(&sr_pid_leg_ang_rgt, body.leg_ang_tgt_rgt, leg_rgt.ang);
            break;
        case LEG_CTRL_MODE_SELF_RESCUE_SWING: // 反转摆腿速度环
            Pid_Update(&sr_pid_leg_omg_swing_lft, body.leg_omg_tgt_lft, body.leg_omg_lft);
            Pid_Update(&sr_pid_leg_omg_swing_rgt, body.leg_omg_tgt_rgt, body.leg_omg_rgt);
            Self_Rescue_Limit_I(&sr_pid_leg_omg_swing_lft, SR_REVERSE_SWING_RATE_PID_I_MAX);
            Self_Rescue_Limit_I(&sr_pid_leg_omg_swing_rgt, SR_REVERSE_SWING_RATE_PID_I_MAX);
            break;
        case LEG_CTRL_MODE_SELF_RESCUE_STAND: // 反转收腿与立腿速度环
            Pid_Update(&sr_pid_leg_omg_stand_lft, body.leg_omg_tgt_lft, body.leg_omg_lft);
            Pid_Update(&sr_pid_leg_omg_stand_rgt, body.leg_omg_tgt_rgt, body.leg_omg_rgt);
            break;
        default: // 正常模式不使用自救 PID
            break;
    }
}

// 目标每拍追平实测 降低机体下沉引起的反馈对抗
static void Self_Rescue_Lock_To_Measured(void)
{
    body.leg_len_tgt_lft = leg_lft.len;
    body.leg_len_tgt_rgt = leg_rgt.len;
    len_goal_lft         = leg_lft.len;
    len_goal_rgt         = leg_rgt.len;
    body.leg_ang_tgt_lft = leg_lft.ang;
    body.leg_ang_tgt_rgt = leg_rgt.ang;
}

// 退出判据 姿态进窗且双腿世界系摆角都在正下方 只查一腿 另一腿半空会顶翻
static bool Self_Rescue_Exit_Check(void)
{
    return (fabsf(Wrap_Pi(imu_world.pitch)) < SR_COMMON_EXIT_ANG) &&
           (fabsf(body.ang_roll) < SR_COMMON_EXIT_ANG) &&
           (fabsf(Imu_To_World_Leg_Ang(leg_lft.ang)) < SR_COMMON_LEG_ANG) &&
           (fabsf(Imu_To_World_Leg_Ang(leg_rgt.ang)) < SR_COMMON_LEG_ANG) &&
           (leg_lft.len < Leg_Len_Min + SR_COMMON_LEN_ERR) &&
           (leg_rgt.len < Leg_Len_Min + SR_COMMON_LEN_ERR) &&
           (fabsf(leg_lft.len - leg_rgt.len) < SR_COMMON_LEN_DIFF);
}

// 正转收腿入口 摆角目标重贴实测 腿长终点交给末尾限速
static void Self_Rescue_Forward_Retract_Enter(void)
{
    // 换向收腿直接接正转收腿时，沿用同一段斜坡。
    if (phase != SR_PHASE_OBSTACLE_RETRACT)
        retract_ramp_ms = 0u;
    ang_hold_lft         = leg_lft.ang;
    ang_hold_rgt         = leg_rgt.ang;
    body.leg_ang_tgt_lft = ang_hold_lft;
    body.leg_ang_tgt_rgt = ang_hold_rgt;
    phase                = SR_PHASE_FORWARD_RETRACT;
}

// 正转扫腿入口 角目标从实测起步 清进窗计时
static void Self_Rescue_Forward_Sweep_Enter(void)
{
    ang_tgt_lft = leg_lft.ang;
    ang_tgt_rgt = leg_rgt.ang;
    exit_hold   = 0u;
    stall_ms[0] = stall_ms[1] = 0u;
    phase                     = SR_PHASE_FORWARD_SWEEP;
}

// 摆腿位姿表给每腿角速度目标 roll_side 倒向本腿一侧为正 无匹配兜底与直立同向
static float Self_Rescue_Reverse_Swing_Speed(float leg_ang, float pitch, float roll_side)
{
    bool upright = fabsf(pitch) < SR_REVERSE_READY_PITCH_MAX_RAD;

    if (leg_ang > SR_REVERSE_READY_WORLD_LEG_ANG_MIN_RAD && leg_ang < SR_REVERSE_READY_WORLD_LEG_ANG_MAX_RAD && upright && roll_side < SR_REVERSE_READY_ROLL_MAX_RAD)
        return 0.0f;
    if (pitch < -SR_REVERSE_READY_PITCH_MAX_RAD && roll_side > SR_REVERSE_READY_ROLL_MAX_RAD)
        return -SR_REVERSE_SWING_RATE_RADPS;
    if (pitch > SR_REVERSE_READY_PITCH_MAX_RAD && roll_side > SR_REVERSE_READY_ROLL_MAX_RAD)
        return SR_REVERSE_SWING_RATE_RADPS;
    return -SR_REVERSE_SWING_RATE_RADPS;
}

// 双腿世界系角与 pitch roll 同时进窗 腿角使用还原 pitch 的折回值
static bool Self_Rescue_Stand_Check(float pitch)
{
    float ang_l = Imu_To_World_Leg_Ang(leg_lft.ang);
    float ang_r = Imu_To_World_Leg_Ang(leg_rgt.ang);

    return (ang_l > SR_REVERSE_READY_WORLD_LEG_ANG_MIN_RAD) && (ang_l < SR_REVERSE_READY_WORLD_LEG_ANG_MAX_RAD) &&
           (ang_r > SR_REVERSE_READY_WORLD_LEG_ANG_MIN_RAD) && (ang_r < SR_REVERSE_READY_WORLD_LEG_ANG_MAX_RAD) &&
           (fabsf(pitch) < SR_REVERSE_READY_PITCH_MAX_RAD) && (fabsf(body.ang_roll) < SR_REVERSE_READY_ROLL_MAX_RAD);
}

// 反转摆腿入口 清进窗时间 保留本次路径累计时间
static void Self_Rescue_Reverse_Swing_Enter(void)
{
    swing_cnt            = 0u;
    stand_cnt            = 0u;
    body.leg_omg_tgt_lft = 0.0f;
    body.leg_omg_tgt_rgt = 0.0f;
    stall_ms[0] = stall_ms[1] = 0u;
    Pid_Init(&sr_pid_leg_omg_swing_lft, SR_REVERSE_SWING_RATE_PID_KP, SR_REVERSE_SWING_RATE_PID_KI, SR_REVERSE_SWING_RATE_PID_KD, SR_REVERSE_SWING_RATE_PID_OUT_MAX, -SR_REVERSE_SWING_RATE_PID_OUT_MAX);
    Pid_Init(&sr_pid_leg_omg_swing_rgt, SR_REVERSE_SWING_RATE_PID_KP, SR_REVERSE_SWING_RATE_PID_KI, SR_REVERSE_SWING_RATE_PID_KD, SR_REVERSE_SWING_RATE_PID_OUT_MAX, -SR_REVERSE_SWING_RATE_PID_OUT_MAX);
    phase = SR_PHASE_REVERSE_SWING;
}

// 反转收腿入口 先收短后进入独立立腿阶段
static void Self_Rescue_Reverse_Retract_Enter(void)
{
    retract_ramp_ms      = 0u;
    stand_cnt            = 0u;
    body.leg_omg_tgt_lft = body.leg_omg_lft;
    body.leg_omg_tgt_rgt = body.leg_omg_rgt;
    phase                = SR_PHASE_REVERSE_RETRACT;
}

// 按本次方向选择起身动作，沿用本次尝试的总计时
static void Self_Rescue_Path_Enter(void)
{
    if (active_dir < 0.0f)
        Self_Rescue_Reverse_Swing_Enter();
    else
        Self_Rescue_Forward_Retract_Enter();
}

// 两腿相同角增量保持初始相位差 防劈叉反馈同步 首次按 pitch 选择就近翻身方向
static void Self_Rescue_Turnover_Enter(void)
{
    float pitch = Wrap_Pi(imu_world.pitch);

    Self_Rescue_Lock_To_Measured();
    if (retry_count == 0u)
        turnover_dir = pitch >= 0.0f ? +1.0f : -1.0f;
    ang_tgt_lft   = leg_lft.ang;
    ang_tgt_rgt   = leg_rgt.ang;
    turnover_ms   = 0u;
    turnover_hold = 0u;
    phase         = SR_PHASE_TURNOVER;
}

// 已翻正时绕过零腿力伸腿阶段，避免收短后又被气弹簧顶回障碍
static bool Self_Rescue_Upright(void)
{
    return fabsf(Wrap_Pi(imu_world.pitch)) < SR_TURNOVER_PITCH_TH &&
           fabsf(body.ang_roll) < SR_TURNOVER_ROLL_TH;
}

// 任一腿有动作请求且机构角持续几乎不动时累计诊断时间，到位腿不累计
static void Self_Rescue_Update_Stall_Diagnostic(void)
{
    float command[2]     = {body.leg_omg_tgt_lft, body.leg_omg_tgt_rgt};
    const float speed[2] = {leg_lft.d_ang, leg_rgt.d_ang};

    if (phase == SR_PHASE_FORWARD_SWEEP)
    {
        command[0] = fabsf(body.leg_ang_tgt_lft - leg_lft.ang) > SR_OBSTACLE_ANG_ERR_MIN_RAD ? SR_FORWARD_SWEEP_RATE_RADPS : 0.0f;
        command[1] = fabsf(body.leg_ang_tgt_rgt - leg_rgt.ang) > SR_OBSTACLE_ANG_ERR_MIN_RAD ? SR_FORWARD_SWEEP_RATE_RADPS : 0.0f;
    }
    for (unsigned int i = 0u; i < 2u; i++)
    {
        if (fabsf(command[i]) > SR_OBSTACLE_COMMAND_MIN_RADPS && fabsf(speed[i]) < SR_OBSTACLE_STALL_RATE_MAX_RADPS)
        {
            stall_ms[i] += CTRL_PERIOD_MS;
        }
        else
        {
            stall_ms[i] = 0u;
        }
    }
}

// 换向或反转失姿时先锁住摆角并收腿，实测收短后再启动当前方向的动作。
static void Self_Rescue_Obstacle_Retract_Enter(void)
{
    retract_ramp_ms = 0u;
    App_Self_Rescue_Pid_Init();
    Self_Rescue_Lock_To_Measured();
    ang_hold_lft         = leg_lft.ang;
    ang_hold_rgt         = leg_rgt.ang;
    body.leg_omg_tgt_lft = body.leg_omg_tgt_rgt = 0.0f;
    stall_ms[0] = stall_ms[1] = 0u;
    obstacle_retract_ms       = 0u;
    phase                     = SR_PHASE_OBSTACLE_RETRACT;
}

// 成功交接平衡 腿长锁均值 腿角折回 清目标并初始化模型后端
static void Self_Rescue_Handoff(void)
{
    float len = (leg_lft.len + leg_rgt.len) * 0.5f;

    if (!isfinite(len))
    {
        len = Leg_Len_Min;
    }

    body.leg_len_tgt_lft = LEG_LEN_CLAMP(len);
    body.leg_len_tgt_rgt = LEG_LEN_CLAMP(len);
    len_goal_lft         = body.leg_len_tgt_lft;
    len_goal_rgt         = body.leg_len_tgt_rgt;

    leg_lft.ang = leg_lft.ang_last = Wrap_Pi(leg_lft.ang);
    leg_rgt.ang = leg_rgt.ang_last = Wrap_Pi(leg_rgt.ang);
    body.leg_ang_lft               = leg_lft.ang + body.ang_pitch;
    body.leg_ang_rgt               = leg_rgt.ang + body.ang_pitch;

    body.vel_tgt     = 0.0f;
    body.omg_yaw_tgt = 0.0f;
    body.ang_yaw_tgt = body.ang_yaw;
    Balance_Mpc_Stop();
    finished = true;
}

// 入场锁存初始方向并清重试计数，先停稳再翻身。
void App_Self_Rescue_Enter(void)
{
    active_dir          = (rescue_dir < 0.0f) ? -1.0f : +1.0f;
    retry_count         = 0u;
    attempt_ms          = 0u;
    obstacle_retract_ms = 0u;
    retract_ramp_ms     = SR_RETRACT_TRQ_RAMP_MS;
    stall_ms[0] = stall_ms[1] = 0u;

    App_Self_Rescue_Pid_Init();
    Balance_Mpc_Stop();

    finished  = false;
    exit_hold = 0u;

    body.leg_omg_tgt_lft = 0.0f;
    body.leg_omg_tgt_rgt = 0.0f;

    body.vel_tgt     = 0.0f;
    body.omg_yaw_tgt = 0.0f;
    body.ang_yaw_tgt = body.ang_yaw;

    body.leg_len_tgt_lft = leg_lft.len;
    body.leg_len_tgt_rgt = leg_rgt.len;
    len_goal_lft         = leg_lft.len;
    len_goal_rgt         = leg_rgt.len;

    phase = SR_PHASE_PREPARE;
}

// 阶段机按 phase 推进目标与计时 末尾统一限速 出力交给 balance
void App_Self_Rescue_Tick(void)
{
    // 收腿提前到位时，下一自救阶段继续走完斜坡。
    if (retract_ramp_ms < SR_RETRACT_TRQ_RAMP_MS)
        retract_ramp_ms += CTRL_PERIOD_MS;

    if (phase >= SR_PHASE_PREPARE && phase <= SR_PHASE_VERIFY)
        attempt_ms += CTRL_PERIOD_MS;

    switch (phase)
    {
        case SR_PHASE_PREPARE: // 目标贴实测 等待机体停稳
            Self_Rescue_Lock_To_Measured();

            if (attempt_ms >= SR_COMMON_PREPARE_MS)
            {
                Self_Rescue_Turnover_Enter();
            }
            break;

        case SR_PHASE_TURNOVER: // 腿长不出力 自然伸腿后同步扫腿 已翻正则跳过
        {
            float pitch  = Wrap_Pi(imu_world.pitch);
            bool upright = fabsf(pitch) < SR_TURNOVER_PITCH_TH && fabsf(body.ang_roll) < SR_TURNOVER_ROLL_TH;

            Self_Rescue_Lock_To_Measured();
            turnover_ms += CTRL_PERIOD_MS;
            if (upright)
            {
                turnover_hold += CTRL_PERIOD_MS;
                if (turnover_hold >= SR_TURNOVER_HOLD_MS)
                {
                    if (Self_Rescue_Exit_Check())
                    {
                        exit_hold            = 0u;
                        phase                = SR_PHASE_VERIFY;
                        body.leg_omg_tgt_lft = 0.0f;
                        body.leg_omg_tgt_rgt = 0.0f;
                    }
                    else
                    {
                        Self_Rescue_Path_Enter();
                    }
                    break;
                }
            }
            else
            {
                turnover_hold = 0u;
                if (turnover_ms >= SR_TURNOVER_EXTEND_MS)
                {
                    ang_tgt_lft += turnover_dir * SR_TURNOVER_SPEED * CTRL_DT;
                    ang_tgt_rgt += turnover_dir * SR_TURNOVER_SPEED * CTRL_DT;
                    ang_tgt_lft = leg_lft.ang + Self_Rescue_Clamp(ang_tgt_lft - leg_lft.ang, SR_OBSTACLE_SWEEP_LEAD_MAX_RAD);
                    ang_tgt_rgt = leg_rgt.ang + Self_Rescue_Clamp(ang_tgt_rgt - leg_rgt.ang, SR_OBSTACLE_SWEEP_LEAD_MAX_RAD);
                }
            }
            body.leg_ang_tgt_lft = ang_tgt_lft;
            body.leg_ang_tgt_rgt = ang_tgt_rgt;
            break;
        }

        case SR_PHASE_OBSTACLE_RETRACT: // 先收腿离开台阶，已翻正时直接启动当前路径
            len_goal_lft = len_goal_rgt = Leg_Len_Min;
            body.leg_ang_tgt_lft        = ang_hold_lft;
            body.leg_ang_tgt_rgt        = ang_hold_rgt;
            obstacle_retract_ms += CTRL_PERIOD_MS;
            if (leg_lft.len < Leg_Len_Min + SR_OBSTACLE_RETRACT_LEN_TOL_M &&
                leg_rgt.len < Leg_Len_Min + SR_OBSTACLE_RETRACT_LEN_TOL_M)
            {
                if (Self_Rescue_Upright())
                    Self_Rescue_Path_Enter();
                else
                    Self_Rescue_Turnover_Enter();
            }
            break;

        case SR_PHASE_FORWARD_RETRACT: // 摆角保持实测 腿长限速收短
            len_goal_lft         = Leg_Len_Min;
            len_goal_rgt         = Leg_Len_Min;
            body.leg_ang_tgt_lft = ang_hold_lft;
            body.leg_ang_tgt_rgt = ang_hold_rgt;

            if (leg_lft.len < Leg_Len_Min + SR_FORWARD_RETRACT_LEN_TOL_M &&
                leg_rgt.len < Leg_Len_Min + SR_FORWARD_RETRACT_LEN_TOL_M)
            {
                Self_Rescue_Forward_Sweep_Enter();
            }
            break;

        case SR_PHASE_FORWARD_SWEEP: // 等速扫腿 进窗后转入唯一的稳持确认阶段
        {
            ang_tgt_lft += SR_FORWARD_SWEEP_DIRECTION * SR_FORWARD_SWEEP_RATE_RADPS * CTRL_DT;
            ang_tgt_rgt += SR_FORWARD_SWEEP_DIRECTION * SR_FORWARD_SWEEP_RATE_RADPS * CTRL_DT;
            ang_tgt_lft = leg_lft.ang + Self_Rescue_Clamp(ang_tgt_lft - leg_lft.ang, SR_OBSTACLE_SWEEP_LEAD_MAX_RAD);
            ang_tgt_rgt = leg_rgt.ang + Self_Rescue_Clamp(ang_tgt_rgt - leg_rgt.ang, SR_OBSTACLE_SWEEP_LEAD_MAX_RAD);

            len_goal_lft         = SR_FORWARD_SWEEP_LEG_LEN_M;
            len_goal_rgt         = SR_FORWARD_SWEEP_LEG_LEN_M;
            body.leg_ang_tgt_lft = ang_tgt_lft;
            body.leg_ang_tgt_rgt = ang_tgt_rgt;

            // 进窗即停止扫腿，连续稳持只在 VERIFY 累计一次。
            if (Self_Rescue_Exit_Check())
            {
                phase     = SR_PHASE_VERIFY;
                exit_hold = 0u;
            }
            break;
        }

        case SR_PHASE_REVERSE_SWING: // 按位姿给角速度目标 双腿进窗转立腿
        {
            float pitch = SR_COMMON_PITCH_SIGN * Wrap_Pi(imu_world.pitch);
            float roll  = SR_COMMON_ROLL_SIGN * body.ang_roll;

            body.leg_omg_tgt_lft = Self_Rescue_Reverse_Swing_Speed(Imu_To_World_Leg_Ang(leg_lft.ang), pitch, -roll);
            body.leg_omg_tgt_rgt = Self_Rescue_Reverse_Swing_Speed(Imu_To_World_Leg_Ang(leg_rgt.ang), pitch, roll);

            Self_Rescue_Lock_To_Measured();

            if (Self_Rescue_Stand_Check(pitch))
            {
                swing_cnt += CTRL_PERIOD_MS;
                if (swing_cnt > SR_REVERSE_READY_HOLD_MS)
                {
                    Self_Rescue_Reverse_Retract_Enter();
                }
            }
            else
            {
                swing_cnt = 0u;
            }

            break;
        }

        case SR_PHASE_REVERSE_RETRACT: // 先收腿 保持现有角速度
        case SR_PHASE_REVERSE_STAND:   // 再立腿 机体失姿先收腿脱困
        {
            float pitch = SR_COMMON_PITCH_SIGN * Wrap_Pi(imu_world.pitch);

            // 摆腿到位后已开始收腿／立腿，腿角不再受摆腿窗口约束；仅机体失姿退出。
            if (fabsf(pitch) >= SR_REVERSE_READY_PITCH_MAX_RAD || fabsf(body.ang_roll) >= SR_REVERSE_READY_ROLL_MAX_RAD)
            {
                Self_Rescue_Obstacle_Retract_Enter();
                break;
            }

            len_goal_lft         = SR_REVERSE_STAND_LEG_LEN_M;
            len_goal_rgt         = SR_REVERSE_STAND_LEG_LEN_M;
            body.leg_ang_tgt_lft = leg_lft.ang;
            body.leg_ang_tgt_rgt = leg_rgt.ang;

            if (phase == SR_PHASE_REVERSE_RETRACT)
            {
                body.leg_omg_tgt_lft = body.leg_omg_lft;
                body.leg_omg_tgt_rgt = body.leg_omg_rgt;

                if (body.leg_len_lft < SR_REVERSE_RETRACT_DONE_LEN_M && body.leg_len_rgt < SR_REVERSE_RETRACT_DONE_LEN_M)
                {
                    phase = SR_PHASE_REVERSE_STAND;

                    stand_cnt = 0u;
                }
            }
            else
            {
                float ang_l = Wrap_Pi(leg_lft.ang);
                float ang_r = Wrap_Pi(leg_rgt.ang);

                body.leg_omg_tgt_lft = Self_Rescue_Reverse_Stand_Speed(leg_lft.ang);
                body.leg_omg_tgt_rgt = Self_Rescue_Reverse_Stand_Speed(leg_rgt.ang);

                if (fabsf(ang_l) < SR_REVERSE_STAND_BODY_LEG_ANG_TOL_RAD && fabsf(ang_r) < SR_REVERSE_STAND_BODY_LEG_ANG_TOL_RAD)
                {
                    stand_cnt += CTRL_PERIOD_MS;
                    if (stand_cnt > SR_REVERSE_STAND_DONE_HOLD_MS)
                    {
                        phase     = SR_PHASE_VERIFY;
                        exit_hold = 0u;
                    }
                }
                else
                {
                    stand_cnt = 0u;
                }
            }
            break;
        }

        case SR_PHASE_VERIFY: // 保持起立姿态 等待角速度收敛 出窗清计时 总超时才换路径
            if (active_dir < 0.0f)
            {
                body.leg_omg_tgt_lft = Self_Rescue_Reverse_Stand_Speed(leg_lft.ang);
                body.leg_omg_tgt_rgt = Self_Rescue_Reverse_Stand_Speed(leg_rgt.ang);
            }
            if (Self_Rescue_Exit_Check() && fabsf(body.omg_pitch) < SR_COMMON_OMG_TH && fabsf(body.omg_roll) < SR_COMMON_OMG_TH)
            {
                exit_hold += CTRL_PERIOD_MS;
                if (exit_hold >= SR_COMMON_HOLD_MS)
                    phase = SR_PHASE_HANDOFF;
            }
            else
            {
                exit_hold = 0u;
            }
            break;

        case SR_PHASE_HANDOFF: // 锁目标并交接平衡
            Self_Rescue_Handoff();
            break;

        case SR_PHASE_FAILED: // 重试耗尽 不置完成 保持零出力
        default:              // 未知阶段停止自救
            phase = SR_PHASE_FAILED;
            break;
    }

    if (phase == SR_PHASE_FORWARD_SWEEP || phase == SR_PHASE_REVERSE_SWING || phase == SR_PHASE_REVERSE_STAND)
        Self_Rescue_Update_Stall_Diagnostic();

    // 本拍确认起立即可交接，否则到时换向；收腿和翻身也占用本次尝试时间。
    if (phase >= SR_PHASE_PREPARE && phase <= SR_PHASE_VERIFY && attempt_ms >= SR_ATTEMPT_TIMEOUT_MS)
    {
        if (retry_count >= SR_RETRY_MAX)
        {
            phase = SR_PHASE_FAILED;
            return;
        }
        retry_count++;
        active_dir   = -active_dir;
        turnover_dir = -turnover_dir;
        attempt_ms   = 0u;
        Self_Rescue_Obstacle_Retract_Enter();
    }

    body.leg_len_tgt_lft = Self_Rescue_Slew(body.leg_len_tgt_lft, len_goal_lft);
    body.leg_len_tgt_rgt = Self_Rescue_Slew(body.leg_len_tgt_rgt, len_goal_rgt);
}

// 仅成功交接置位 最终失败保持 false 由遥控失能再使能重启
bool App_Self_Rescue_Finished(void)
{
    return finished;
}

// 只拷贝内部进度，不清零、不推进计时；跨任务一致性由调用方临界区保证。
self_rescue_diag_t App_Self_Rescue_Diagnostic(void)
{
    self_rescue_diag_t diag = {
        .round_count  = (retry_count + (phase == SR_PHASE_FAILED)) / 2u,
        .second_path  = (retry_count % 2u) != 0u,
        .attempt_ms   = attempt_ms,
        .stall_lft_ms = stall_ms[0],
        .stall_rgt_ms = stall_ms[1],
        .retract_ms   = obstacle_retract_ms,
    };

    return diag;
}

// 当前自救阶段只读
self_rescue_phase_t App_Self_Rescue_Get_Phase(void)
{
    return phase;
}

// 本次入场锁存的自救方向只读
float App_Self_Rescue_Get_Dir(void)
{
    return active_dir;
}

// 直发模式在关节限幅后取本拍比例，离开自救后恢复全量。
float App_Self_Rescue_Trq_Scale(balance_ctrl_mode_t mode)
{
    if (retract_ramp_ms >= SR_RETRACT_TRQ_RAMP_MS || mode != App_Self_Rescue_Ctrl_Mode())
        return 1.0f;
    return (float)retract_ramp_ms / (float)SR_RETRACT_TRQ_RAMP_MS;
}

// 自救阶段派生出力 交接完成后返回正常通道
balance_ctrl_mode_t App_Self_Rescue_Ctrl_Mode(void)
{
    if (finished) return LEG_CTRL_MODE_MOVING;
    if (phase == SR_PHASE_FAILED || phase == SR_PHASE_INIT) return LEG_CTRL_MODE_NUM;

    switch (phase)
    {
        case SR_PHASE_REVERSE_SWING: // 摆腿角速度出力
            return LEG_CTRL_MODE_SELF_RESCUE_SWING;

        case SR_PHASE_TURNOVER: // 公共翻身 位置环与零轴向力
            return LEG_CTRL_MODE_SELF_RESCUE_TURNOVER;

        case SR_PHASE_VERIFY:  // 确认阶段保持当前路径的控制方式
        case SR_PHASE_HANDOFF: // 完成标志置位前仍保持原通道，反转路径不插入正转位置环
            return active_dir > 0.0f ? LEG_CTRL_MODE_SELF_RESCUE : LEG_CTRL_MODE_SELF_RESCUE_STAND;

        case SR_PHASE_REVERSE_RETRACT: // 反转收腿也启用腿长环
        case SR_PHASE_REVERSE_STAND:   // 立腿角速度与腿长出力
            return LEG_CTRL_MODE_SELF_RESCUE_STAND;

        default: // 未完成阶段默认普通自救出力
            return LEG_CTRL_MODE_SELF_RESCUE;
    }
}
