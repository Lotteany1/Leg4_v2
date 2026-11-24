/**
 * @file  air.c
 * @brief 腿端支持力离地检测，持续失重与压缩冲击辅助确认
 * @version  v1.12  2026-11-24
 */
#include "air.h"

#include "code_config.h"
#include "system_key.h"
#include "balance.h"
#include "body.h"
#include "dm_motor.h"
#include "fsm.h"
#include "imu.h"
#include "imu_to_world.h"
#include "leg.h"
#include "slip_side.h"
#include "lowpass.h"
#include "Leg_SpringF.h"

#include "arm_math.h"

#include <math.h>


typedef struct
{
    float fn;                     // N     含 Tp 项 观测用
    float fn_det;                 // N     剔除 Tp 项 判定用
    float d_len_last, d_ang_last; // m/s rad/s 上拍一阶导 数值微分用
    lowpass1d_t lp_dd_len, lp_dd_ang;
    lowpass1d_t lp_fn, lp_fn_det;
} air_leg_t;

volatile air_status_t air_status;

#if BALANCE_USE_AIR_CHECK

static air_leg_t air_lft, air_rgt;
static volatile uint8_t air_flag;
static uint8_t air_slip_inhibit;
static uint8_t air_landing; // 估计与控制任务握手交接的 LAND 标记
static uint8_t air_inited;
static uint8_t air_reseed;
static bool air_support_ready; // 本拍支持力有效，供离地状态判定
static uint16_t air_both_low_ms; // ms，两腿判定支持力同时低的持续时间
static uint16_t air_freefall_ms; // ms，失重独立消抖，确认后保留至触地或 IMU 失效
static float air_freefall_height; // m，失重期间的最近机体高度，用于确认随后的腿部压缩
static uint16_t air_land_cnt;
static uint16_t air_rearm;
static uint16_t air_stuck_cnt;
static uint16_t air_exit_freeze;
#if STEP_ENABLE
static uint16_t air_side_high[2], air_side_low[2]; // ms 各侧连续确认
static uint8_t air_step;                         // 位 0/1，动作所有权及侧别

// 中间带保留已确认状态，反馈失效立即回到未知。
static void Air_Side_Update(unsigned int side, float force, const dm_motor_t *front, const dm_motor_t *back, uint32_t now)
{
    const uint8_t bit = (uint8_t)(1u << side);
    const float high = SLIP_CONTACT_HIGH;

    if (now - front->frame_tick > SLIP_FEEDBACK_MS || now - back->frame_tick > SLIP_FEEDBACK_MS || !isfinite(force))
    {
        air_side_high[side] = 0u;
        air_side_low[side]  = 0u;
        air_status.side_contact_mask &= (uint8_t)~bit;
        air_status.side_unloaded_mask &= (uint8_t)~bit;
        return;
    }
    air_status.side_valid_mask |= bit;
    if (force <= SLIP_CONTACT_LOW && air_side_low[side] < SLIP_CONTACT_MS)
        air_side_low[side] += CTRL_PERIOD_MS;
    else if (force > SLIP_CONTACT_LOW)
        air_side_low[side] = 0u;
    if (force >= high && air_side_high[side] < SLIP_CONTACT_MS)
        air_side_high[side] += CTRL_PERIOD_MS;
    else if (force < high)
        air_side_high[side] = 0u;
    if (air_side_high[side] >= SLIP_CONTACT_MS)
    {
        air_status.side_contact_mask |= bit;
        air_status.side_unloaded_mask &= (uint8_t)~bit;
    }
    if (air_side_low[side] >= SLIP_CONTACT_MS)
    {
        air_status.side_contact_mask &= (uint8_t)~bit;
        air_status.side_unloaded_mask |= bit;
    }
}
#endif

// 电机反馈轴力加上被动气弹簧力，判定通道剔除 Tp 项。
static void Air_Leg_Estimate(air_leg_t *st, const leg_state_t *leg, float F, float Tp, float dd_z_m,
                             volatile air_force_terms_t *terms)
{
    const float len     = (leg->len > AIR_LEN_MIN) ? leg->len : AIR_LEN_MIN;
    const float spring  = Leg_SpringF(leg->len, leg->ang);
    const float F_all   = F + spring;
    const float sin_ang = arm_sin_f32(leg->ang);
    const float cos_ang = arm_cos_f32(leg->ang);
    const float P       = F_all * cos_ang + Tp * sin_ang / len;
    const float dd_len  = Lowpass_Update(&st->lp_dd_len, (leg->d_len - st->d_len_last) / CTRL_DT);
    const float dd_ang  = Lowpass_Update(&st->lp_dd_ang, (leg->d_ang - st->d_ang_last) / CTRL_DT);
    const float dd_z_w  = dd_z_m - dd_len * cos_ang + 2.0f * leg->d_len * leg->d_ang * sin_ang +
                          leg->len * dd_ang * sin_ang + leg->len * leg->d_ang * leg->d_ang * cos_ang;

    const float inertia = WHEEL_MASS * (GRAVITY + dd_z_w);

    terms->motor_z  = F * cos_ang;
    terms->spring_z = spring * cos_ang;
    terms->inertia_z = inertia;
    st->d_len_last = leg->d_len;
    st->d_ang_last = leg->d_ang;
    st->fn         = Lowpass_Update(&st->lp_fn, P + inertia);
    st->fn_det     = Lowpass_Update(&st->lp_fn_det, F_all * cos_ang + inertia);
}

// 正常离地要求两腿同时卸载，避免单腿负估值抵消另一腿的支撑。落地仍允许单腿先承重。
static void Air_Detect_Update(void)
{
    const float fn_avg = (air_lft.fn_det + air_rgt.fn_det) * 0.5f;
    const bool both_low = air_lft.fn_det < AIR_FN_TH_LOW && air_rgt.fn_det < AIR_FN_TH_LOW;
    const float half_weight = 0.5f * LEG_MASS_TOTAL * GRAVITY;
    const bool imu_fresh = imu.online && HAL_GetTick() - imu.last_rx_tick <= IMU_OFFLINE_MS;
    const bool weightless = imu_fresh && fabsf(1.0f + imu_world.acc_z) * half_weight < AIR_FN_TH_LOW;

    // 左右均无明确承重时，独立累计接近失重的时间。禁止用短暂卸载拼接失重计时。
    if (!imu_fresh || !isfinite(body.h_body) || (!air_flag && !air_landing && air_rearm > 0u))
        air_freefall_ms = 0u;
    else if (weightless && air_lft.fn_det < AIR_FN_TH_HIGH && air_rgt.fn_det < AIR_FN_TH_HIGH)
    {
        if (air_freefall_ms == 0u || air_freefall_ms < AIR_DEBOUNCE_MS) air_freefall_ms += CTRL_PERIOD_MS;
        air_freefall_height = body.h_body;
    }
    else if (!air_flag || air_freefall_ms < AIR_DEBOUNCE_MS)
        air_freefall_ms = 0u;
    const bool freefall = air_freefall_ms != 0u && air_freefall_ms >= AIR_DEBOUNCE_MS;

    if (both_low)
    {
        if (air_both_low_ms < AIR_LAND_RELIFT_MS || air_both_low_ms < AIR_DEBOUNCE_MS)
            air_both_low_ms += CTRL_PERIOD_MS;
    }
    else
        air_both_low_ms = 0u;

    if (air_flag)
    {
        uint8_t landed = 0u;

        // 已确认失重时屏蔽腾空中的假力峰；腿部压缩且向上惯性力足够大可确认硬着陆。
        const bool force_contact = fn_avg > AIR_FN_TH_HIGH && !(freefall && weightless);
        const bool impact = freefall && body.h_body < air_freefall_height &&
                            half_weight * imu_world.acc_z > AIR_FN_TH_HIGH;
        if (fn_avg > AIR_FN_TH_HIGH) air_status.land_high_samples++;
        if (force_contact || impact)
        {
            air_land_cnt += CTRL_PERIOD_MS;
            if (air_land_cnt >= AIR_LAND_MS)
            {
                landed = 1u;
            }
        }
        else
        {
            air_land_cnt = 0u;
        }

#if AIR_STUCK_MS > 0u
        air_stuck_cnt += CTRL_PERIOD_MS;
        if (air_stuck_cnt >= AIR_STUCK_MS)
        {
            landed = 1u;
        }
#endif

        if (landed)
        {
            air_status.land_events++;
            air_status.last_impact_landing = impact && !force_contact;
            air_freefall_ms = 0u;
            air_both_low_ms = 0u;
            air_flag      = 0u;
            air_land_cnt  = 0u;
            air_rearm     = AIR_REARM_MS;
            air_stuck_cnt = 0u;
        }
    }
#if STEP_ENABLE
    else if (air_step)
    {
        if (air_status.side_unloaded_mask == 3u)
            air_flag = 1u;
    }
#endif
    else if (freefall)
        air_flag = 1u;
    else if (air_landing != 0u)
    {
        // 冲击触地后的低力可能来自同一个负峰：短时再离地须同时失重，持续卸载仍可正常离地。
        air_rearm = 0u;
        if (air_both_low_ms >= AIR_LAND_RELIFT_MS &&
            (!air_status.last_impact_landing || !imu_fresh || air_freefall_ms >= AIR_LAND_RELIFT_MS ||
             air_both_low_ms >= AIR_DEBOUNCE_MS))
            air_flag = 1u;
    }
    else if (air_slip_inhibit != 0u)
    {
        // 打滑期间沿用双腿卸载计时，及时提交 AIR。
        if (air_both_low_ms >= AIR_DEBOUNCE_MS)
            air_flag = 1u;
        else
            air_status.slip_inhibited = 1u;
    }
    else if (air_rearm > 0u)
    {
        air_rearm = air_rearm > CTRL_PERIOD_MS ? air_rearm - CTRL_PERIOD_MS : 0u;
        air_both_low_ms = 0u;
    }
    else if (both_low && air_both_low_ms >= AIR_DEBOUNCE_MS)
        air_flag = 1u;
}

// 冻结清状态 消抖计数与标志归零 标记恢复需重播种 fn 保持末值观测
static void Air_Freeze_Clear(void)
{
    air_freefall_ms = 0u;
#if STEP_ENABLE
    air_status.side_contact_mask = 0u;
    air_status.side_unloaded_mask = 0u;
    air_side_high[0] = air_side_high[1] = 0u;
    air_side_low[0] = air_side_low[1] = 0u;
#endif
    air_both_low_ms = 0u;
    air_land_cnt  = 0u;
    air_rearm     = 0u;
    air_stuck_cnt = 0u;
    air_flag      = 0u;
    air_reseed    = 1u;
}

// 播种一阶导初值并清微分滤波记忆 防冻结后首拍微分打出假尖峰
static void Air_Seed_Derivative(void)
{
    Lowpass_Reset(&air_lft.lp_dd_len);
    Lowpass_Reset(&air_lft.lp_dd_ang);
    Lowpass_Reset(&air_rgt.lp_dd_len);
    Lowpass_Reset(&air_rgt.lp_dd_ang);
    air_lft.d_len_last = leg_lft.d_len;
    air_lft.d_ang_last = leg_lft.d_ang;
    air_rgt.d_len_last = leg_rgt.d_len;
    air_rgt.d_ang_last = leg_rgt.d_ang;
}

#endif

// 离地模块初始化函数
void Air_Init(void)
{
#if BALANCE_USE_AIR_CHECK
    Lowpass_Init(&air_lft.lp_dd_len, AIR_LPF_D2);
    Lowpass_Init(&air_lft.lp_dd_ang, AIR_LPF_D2);
    Lowpass_Init(&air_lft.lp_fn, AIR_LPF_FN);
    Lowpass_Init(&air_lft.lp_fn_det, AIR_LPF_FN);
    Lowpass_Init(&air_rgt.lp_dd_len, AIR_LPF_D2);
    Lowpass_Init(&air_rgt.lp_dd_ang, AIR_LPF_D2);
    Lowpass_Init(&air_rgt.lp_fn, AIR_LPF_FN);
    Lowpass_Init(&air_rgt.lp_fn_det, AIR_LPF_FN);
    Air_Seed_Derivative();
    air_flag        = 0u;
    air_slip_inhibit = 0u;
    air_landing      = 0u;
    air_reseed      = 0u;
    air_both_low_ms = 0u;
    air_support_ready = false;
    air_land_cnt    = 0u;
    air_rearm       = 0u;
    air_stuck_cnt   = 0u;
    air_exit_freeze = 0u;
#if STEP_ENABLE
    air_step = 0u;
    air_status.side_valid_mask = 0u;
    air_status.side_contact_mask = 0u;
    air_status.side_unloaded_mask = 0u;
    air_side_high[0] = air_side_high[1] = 0u;
    air_side_low[0] = air_side_low[1] = 0u;
#endif
    air_inited      = 1u;
    air_freefall_ms = 0u;
#endif
}

// 离地检测入口 关节离线/非平衡态/坏值冻结 遥测写 air_status 标志经 Air_Get 外发 总开关关掉时检测整套不跑只把遥测清零
bool Air_Update_Support(void)
{
#if STEP_ENABLE
    air_status.side_valid_mask = 0u;
#endif
    air_status.slip_inhibited = 0u;
    air_status.contact_mask   = 0u;
#if BALANCE_USE_AIR_CHECK
    air_support_ready = false;
    if (!air_inited)
    {
        Air_Init();
    }

    if (!dm_f_lft.enabled || !dm_b_lft.enabled || !dm_f_rgt.enabled || !dm_b_rgt.enabled ||
        !dm_f_lft.online || !dm_b_lft.online || !dm_f_rgt.online || !dm_b_rgt.online)
    {
        Air_Freeze_Clear();
    }
    else if (Fsm_State_Get() != FSM_STATE_BALANCE)
    {
        air_exit_freeze = AIR_EXIT_FREEZE_MS;
        Air_Freeze_Clear();
    }
    else if (air_exit_freeze > 0u)
    {
        air_exit_freeze = air_exit_freeze > CTRL_PERIOD_MS ? air_exit_freeze - CTRL_PERIOD_MS : 0u;
        Air_Freeze_Clear();
    }
    else if (!isfinite(body.F_lft) || !isfinite(body.Tp_lft) || !isfinite(body.F_rgt) || !isfinite(body.Tp_rgt) ||
             !isfinite(imu_world.acc_z) || !isfinite(leg_lft.len) || !isfinite(leg_rgt.len))
    {
        Air_Freeze_Clear();
    }
    else
    {
        if (air_reseed)
        {
            Air_Seed_Derivative();
            air_reseed = 0u;
        }
        const float dd_z_m = imu_world.acc_z * GRAVITY;
        Air_Leg_Estimate(&air_lft, &leg_lft, body.F_lft, body.Tp_lft, dd_z_m, &air_status.terms_lft);
        Air_Leg_Estimate(&air_rgt, &leg_rgt, body.F_rgt, body.Tp_rgt, dd_z_m, &air_status.terms_rgt);
#if STEP_ENABLE
        const uint32_t now = HAL_GetTick();
        Air_Side_Update(0u, air_lft.fn_det, &dm_f_lft, &dm_b_lft, now);
        Air_Side_Update(1u, air_rgt.fn_det, &dm_f_rgt, &dm_b_rgt, now);
#endif
        air_support_ready = isfinite(air_lft.fn_det) && isfinite(air_rgt.fn_det);
        if (air_lft.fn_det > AIR_FN_TH_HIGH) air_status.contact_mask |= 1u;
        if (air_rgt.fn_det > AIR_FN_TH_HIGH) air_status.contact_mask |= 2u;
    }

    air_status.fn_lft     = air_lft.fn;
    air_status.fn_rgt     = air_rgt.fn;
    air_status.fn_det_lft = air_lft.fn_det;
    air_status.fn_det_rgt = air_rgt.fn_det;
    air_status.fn_avg     = (air_lft.fn_det + air_rgt.fn_det) * 0.5f;
    air_status.flag       = air_flag;
    return air_support_ready;
#else
    air_status.fn_lft     = 0.0f;
    air_status.fn_rgt     = 0.0f;
    air_status.fn_det_lft = 0.0f;
    air_status.fn_det_rgt = 0.0f;
    air_status.fn_avg     = 0.0f;
    air_status.flag       = 0u;
    return false;
#endif
}

// 当前拍滑移已就绪，再从支持力提交整车 AIR。
void Air_Update_Detect(void)
{
#if BALANCE_USE_AIR_CHECK
    if (air_support_ready)
        Air_Detect_Update();
    air_status.support_valid = air_support_ready;
    air_status.freefall_confirmed = air_flag && air_freefall_ms != 0u && air_freefall_ms >= AIR_DEBOUNCE_MS;
    air_status.unload_ms = air_both_low_ms;
    air_status.flag = air_flag;
#endif
}

// 供原单阶段调用方使用。
void Air_Update(void)
{
    Air_Update_Support();
    Air_Update_Detect();
}

// 由控制任务交接，AIR 确立后仍走既有整车落地路径。
void Air_Set_Step(uint8_t side)
{
#if BALANCE_USE_AIR_CHECK && STEP_ENABLE
    air_step = side;
#else
    (void)side;
#endif
}

// 由控制拍设置；下一估计拍应用，AIR 与非平衡出场撤销。
void Air_Set_Landing(uint8_t active)
{
#if BALANCE_USE_AIR_CHECK
    air_landing = active;
#else
    (void)active;
#endif
}

// 实验适配器逐拍提供有时限的请求；关闭任何功能开关时接口仍可调用。
void Air_Set_Slip_Inhibit(uint8_t inhibit)
{
#if BALANCE_USE_AIR_CHECK && BALANCE_USE_SLIP_AIR_INHIBIT
    air_slip_inhibit = inhibit != 0u;
#else
    (void)inhibit;
#endif
}

// 离地标志只读 总开关关掉时恒 0
uint8_t Air_Get(void)
{
#if BALANCE_USE_AIR_CHECK
    return air_flag;
#else
    return 0u;
#endif
}
