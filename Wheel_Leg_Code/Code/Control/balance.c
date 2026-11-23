/** @file  balance.c  @brief 平衡出力控制  @version  v4.20  2026-11-22 */
#include "balance.h"
#include "air.h"
#include "app_self_rescue.h"
#include "code_config.h"
#if STEP_ENABLE
#include "app_step.h"
#include "app_safety.h"
#endif
#include "Leg_Comp.h"
#include "Leg_CoriolisComp.h"
#include "Leg_RollComp.h"
#include "Leg_SpringF.h"
#include "Leg_Trq.h"
#include "FreeRTOS.h"
#include "arm_math.h"
#include "body.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "fsm_running.h"
#include "leg.h"
#include "monitor.h"
#include "notch.h"
#include "system_key.h"
#include "task.h"
#include "task_sync.h"
#include "tinympc14.h"
#include "turn_limit.h"
#include <math.h>
#include <stdbool.h>

control_vector_t leg_control_lft, leg_control_rgt; // 左右腿出力向量 补偿预计算与出力合成共用
static balance_ctrl_mode_t balance_mode_prev;
static float balance_hip_comp[2];        // N·m 本拍实际叠加的髋补偿，冻结后不重读后台偏置
static float balance_force_requested[2]; // N 上一控制拍总支持力请求 已加回气弹簧 仅供诊断
#if STEP_ENABLE
static execution_sample_t balance_step_seed; // 恢复接地前最后一拍接纳码
static uint32_t balance_step_return_tick;    // ms，回到双侧接地的估计时标
static float balance_step_mix = 1.0f;       // 双支撑轮/髋交接权重
#endif
static float balance_force_exit_soft[2];
static uint16_t balance_force_exit_cnt;
static bool balance_landing_prev; // 上拍气弹簧补偿仍在恢复，退出时从接纳力开始撤力

// 6.5 Hz 陷波 抑制 v·ω 前馈链上行速度振荡经 RollComp 灌入差动力 (RBJ fs=1 kHz f0=6.5 Hz Q=2 DC 增益 1)
static const notch_coeff_t balance_notch_coeff = {
    .b0 = 0.9898916271f,
    .b1 = -1.978132381f,
    .a2 = 0.9797832541f,
};
static notch_state_t balance_notch_roll; // Roll_CompF 右腿侧陷波状态
static notch_state_t balance_notch_turn; // Turn_DistF 陷波状态

#if CTRL_PERIOD_MS != 1u || BALANCE_MPC_PERIOD_MS != TINYMPC14_ASYNC_PERIOD_MS
#error TinyMPC14 requires 1 ms control and 4 ms background solves
#endif

static volatile bool balance_mpc_ready;      // 后台任务已发布首个接地模型
static volatile bool balance_mpc_active;     // 当前执行代次允许后台求解
static volatile balance_mpc_diag_t balance_mpc_diag;
static tinympc14_comp_state_t balance_mpc_comp;
static turn_limit_leglen_t balance_mpc_turn;
static struct
{
    float bias, previous;   // N 接纳差力减反馈差力的慢变偏置和前一拍样本
    float integral;         // N·s 去偏差力的泄漏积分
    uint32_t tick;          // ms 上次双侧有效观测的估计拍
    bool ready;
} balance_force_history;
#if BALANCE_USE_HEIGHT_FILTER
static const notch_coeff_t balance_height_coeff = {
    .b0 = 0.9582897663f,
    .b1 = -1.9029384781f,
    .a2 = 0.9165795326f,
};
static notch_state_t balance_height_notch, balance_height_rate_notch;
static float balance_height_rate, balance_height_rate_input;
static uint32_t balance_height_tick;
#endif

static bool Balance_State_Check(balance_ctrl_mode_t mode);
static void Balance_Stop(void);
static void Balance_Comp_Update(bool stepping);
static bool Balance_Landing_Update(void);
static bool Balance_Ground_Update(balance_ctrl_mode_t mode, bool landing, const execution_sample_t *handoff,
                                  float hip_comp[2], volatile balance_mpc_force_t **force_record);
static void Balance_Vector_Update(balance_ctrl_mode_t mode);
static void Balance_Axial_Output_Update(balance_ctrl_mode_t mode, const float prev_force[2]);
static bool Balance_Send_Allocated(bool stepping, volatile balance_mpc_force_t *force_record);
static void Balance_Trq_Out(void);
#if STEP_ENABLE
static const execution_sample_t *Balance_Step_Handoff(bool grounded, bool stepping);
#endif

// ! 单拍出力入口：保存上拍力 → 校验 → 按模式算力 → 发送。
void Balance_Control(balance_ctrl_mode_t mode)
{
    const bool grounded = Balance_Mode_Grounded(mode);
#if STEP_ENABLE
    const bool stepping = mode == LEG_CTRL_MODE_STEP;
    const execution_sample_t *handoff = Balance_Step_Handoff(grounded, stepping);
#else
    const bool stepping = false;
    const execution_sample_t *handoff = NULL;
#endif
    volatile balance_mpc_force_t *force_record = NULL;
    // 保存上拍支持力，用于离开接地时的有界衰减。
    float prev_force[2] = {leg_control_lft.F, leg_control_rgt.F};

    if (balance_landing_prev && !grounded)
    {
        execution_sample_t last;

        // 切代次前读取最近接纳码，补偿恢复期间再次腾空时从该轴向力开始撤力。
        Execution_Latest(&last);
        prev_force[0] = last.valid_mask == EXECUTION_CHANNEL_MASK ? last.total_s[0] - last.spring_f[0] : body.F_lft;
        prev_force[1] = last.valid_mask == EXECUTION_CHANNEL_MASK ? last.total_s[1] - last.spring_f[1] : body.F_rgt;
    }
    Distributor_Mode((uint32_t)mode, grounded);
    leg_control_lft = (control_vector_t){.T = 0.0f};
    leg_control_rgt = (control_vector_t){.T = 0.0f};
    if ((unsigned int)mode >= LEG_CTRL_MODE_NUM || !Balance_State_Check(mode))
    {
        // 模式或反馈无效，本拍发送零出力后结束。
        Balance_Stop();
        return;
    }
    if (mode == LEG_CTRL_MODE_AIR && balance_mode_prev != LEG_CTRL_MODE_AIR)
        Dji_Motor_All_Stop();
    // 新接地代次以当前转弯需求播种。旧代次的滤波历史会在静止首拍产生虚假的左右外载。
    if (grounded && (!balance_mpc_active || balance_mpc_diag.epoch != Execution_Epoch()))
    {
        Notch_Reset(&balance_notch_roll);
        Notch_Reset(&balance_notch_turn);
    }
    if (mode != LEG_CTRL_MODE_AIR)
        Balance_Comp_Update(stepping);
    if (grounded)
    {
        const fsm_landing_t landing = Fsm_Running_Landing();
        bool ready;

        // 落地时两侧尚未都接触，先算过渡力；接触齐全后走接地模型。
        if (landing.contact_pending && air_status.contact_mask != 3u)
            ready = Balance_Landing_Update();
        else
            ready = Balance_Ground_Update(mode, landing.active, handoff, balance_hip_comp, &force_record);

        if (!ready)
        {
            Balance_Stop();
            return;
        }
        Balance_Axial_Output_Update(mode, prev_force);
        if (!Balance_Send_Allocated(false, force_record))
        {
            Balance_Stop();
            return;
        }
    }
#if STEP_ENABLE
    else if (stepping) // 台阶独占轮、髋和轴向力，仍由分配器发送
    {
        Balance_Mpc_Stop();
        App_Self_Rescue_Pid_Update(mode);

        const float gravity[2] = {Balance_Axial_Gravity(leg_lft.len, body.leg_ang_lft),
                                  Balance_Axial_Gravity(leg_rgt.len, body.leg_ang_rgt)};
        float force[2], hip[2], wheel[2];

        balance_hip_comp[0] = leg_control_lft.MgF;
        balance_hip_comp[1] = leg_control_rgt.MgF;
        // 动作腿和支撑腿分开算轴力、髋矩和轮矩，再交给同一个分配器。
        App_Step_Output(gravity, balance_hip_comp, force, hip, wheel);
        leg_control_lft.F = force[0] + leg_control_lft.SpringF;
        leg_control_rgt.F = force[1] + leg_control_rgt.SpringF;
        leg_control_lft.Tp = hip[0];
        leg_control_rgt.Tp = hip[1];
        leg_control_lft.T = wheel[0];
        leg_control_rgt.T = wheel[1];
        Balance_Axial_Output_Update(mode, prev_force);
        if (!Balance_Send_Allocated(true, NULL))
        {
            Balance_Stop();
            return;
        }
    }
#endif
    else // 空中、自救、测试：合成左右腿出力后直接发送
    {
        Balance_Mpc_Stop();
        App_Self_Rescue_Pid_Update(mode);
        Balance_Vector_Update(mode);
        Balance_Axial_Output_Update(mode, prev_force);
        balance_landing_prev = false;
        // 直发只保存出力请求；接纳记录由分配发送路径发布。
        Balance_Trq_Out();
    }
    balance_mode_prev = mode;
}

// 对称限幅 非有限值由输入检查与发送前清洗拦截
static inline float Balance_Clamp(float x, float max)
{
    if (x > max)
        return max;
    if (x < -max)
        return -max;
    return x;
}

#if STEP_ENABLE
// 把接地输出与台阶末拍已接纳的力混合。
static void Balance_Step_Blend(void)
{
    if (balance_step_mix < 1.0f)
    {
        const float remain = 1.0f - balance_step_mix;

        leg_control_lft.T = balance_step_mix * leg_control_lft.T - remain * balance_step_seed.model_u[0];
        leg_control_rgt.T = balance_step_mix * leg_control_rgt.T - remain * balance_step_seed.model_u[1];
        leg_control_lft.Tp = balance_step_mix * leg_control_lft.Tp +
                             remain * (balance_step_seed.model_u[2] + balance_step_seed.hip_comp[0]);
        leg_control_rgt.Tp = balance_step_mix * leg_control_rgt.Tp +
                             remain * (balance_step_seed.model_u[3] + balance_step_seed.hip_comp[1]);
        for (unsigned int side = 0u; side < 2u; side++)
            balance_hip_comp[side] = balance_step_mix * balance_hip_comp[side] + remain * balance_step_seed.hip_comp[side];
    }
}

// 台阶接回地面：从最近接纳的力开始混合；进入台阶时清除旧滤波历史。
static const execution_sample_t *Balance_Step_Handoff(bool grounded, bool stepping)
{
    const execution_sample_t *handoff = NULL;

    if (grounded)
    {
        if (balance_mode_prev == LEG_CTRL_MODE_STEP)
        {
            Execution_Latest(&balance_step_seed);
            if (balance_step_seed.valid_mask == EXECUTION_CHANNEL_MASK) handoff = &balance_step_seed;
            balance_step_return_tick = Estimator_Frame_Tick();
            balance_step_mix = handoff != NULL ? 0.0f : 1.0f;
        }
        if (balance_step_mix < 1.0f)
            balance_step_mix = App_Step_Blend(0.001f * (Estimator_Frame_Tick() - balance_step_return_tick));
    }
    else
        balance_step_mix = 1.0f;
    if (stepping && balance_mode_prev != LEG_CTRL_MODE_STEP)
    {
        Notch_Reset(&balance_notch_roll);
        Notch_Reset(&balance_notch_turn);
    }
    return handoff;
}
#endif

// 两腿补偿项预计算
static void Balance_Comp_Update(bool stepping)
{
    float turn_raw, roll_lft, roll_rgt;

    Leg_Comp(leg_lft.len, &leg_control_lft.Comp_Ang, &leg_control_lft.Comp_Len);
    Leg_Comp(leg_rgt.len, &leg_control_rgt.Comp_Ang, &leg_control_rgt.Comp_Len);

    // 沿腿被动弹簧力取反号后叠加到电机轴向请求
    leg_control_lft.SpringF = -Leg_SpringF(leg_lft.len, leg_lft.ang);
    leg_control_rgt.SpringF = -Leg_SpringF(leg_rgt.len, leg_rgt.ang);

    leg_control_lft.MgF = LEG_GRAV_FF * GRAVITY * arm_sin_f32(body.leg_ang_lft + leg_control_lft.Comp_Ang) * leg_control_lft.Comp_Len;
    leg_control_rgt.MgF = LEG_GRAV_FF * GRAVITY * arm_sin_f32(body.leg_ang_rgt + leg_control_rgt.Comp_Ang) * leg_control_rgt.Comp_Len;
    if (stepping) return;

#if BALANCE_USE_SLIP
    Leg_RollComp(body.h_body, body.omg_yaw_filt, body.vel_kalman, &roll_lft, &roll_rgt, &turn_raw);
#else
    Leg_RollComp(body.h_body, body.omg_yaw_filt, body.vel, &roll_lft, &roll_rgt, &turn_raw);
#endif
    // 前馈对经 6.5 Hz 陷波后使用 保持左=−右 的对称关系
    roll_rgt = Notch_Update(&balance_notch_roll, &balance_notch_coeff, roll_rgt);
    roll_lft = -roll_rgt;
    turn_raw = Notch_Update(&balance_notch_turn, &balance_notch_coeff, turn_raw);
#if BALANCE_USE_ROLL_COMP
    leg_control_lft.Roll_CompF = roll_lft;
    leg_control_rgt.Roll_CompF = roll_rgt;
#endif
    // 外部转弯力矩与执行器补偿使用同一次计算 关闭补偿时仍保留限幅前扰动到模型快照
    leg_control_lft.Turn_DistF = turn_raw;
    leg_control_rgt.Turn_DistF = -turn_raw;

#if BALANCE_USE_CORIOLIS_TP
    Leg_CoriolisComp(body.leg_len, body.omg_yaw, body.ang_pitch, body.ang_roll,
                     body.omg_pitch, body.omg_roll,
                     &leg_control_lft.Coriolis_T, &leg_control_lft.Coriolis_Tp,
                     &leg_control_lft.Coriolis_F, &leg_control_rgt.Coriolis_F);

    leg_control_rgt.Coriolis_T  = leg_control_lft.Coriolis_T;
    leg_control_rgt.Coriolis_Tp = leg_control_lft.Coriolis_Tp;
#endif
}

#if BALANCE_USE_HEIGHT_FILTER
static void Balance_Filter_Height(uint32_t tick, bool start, float *height, float *rate)
{
    const uint32_t elapsed = tick - balance_height_tick;

    if (start || elapsed > BALANCE_HEIGHT_HOLD_MS)
    {
        Notch_Reset(&balance_height_notch);
        Notch_Reset(&balance_height_rate_notch);
        balance_height_rate = *rate;
    }
    else if (elapsed == 0u)
    {
        *height = balance_height_notch.y1;
        *rate   = balance_height_rate_notch.y1;
        return;
    }
    else
    {
        for (uint32_t i = 1u; i < elapsed; i++)
        {
            balance_height_rate += BALANCE_HEIGHT_RATE_ALPHA * (balance_height_rate_input - balance_height_rate);
            Notch_Update(&balance_height_notch, &balance_height_coeff, balance_height_notch.x1);
            Notch_Update(&balance_height_rate_notch, &balance_height_coeff, balance_height_rate);
        }
    }
    balance_height_tick = tick;
    balance_height_rate_input = *rate;
    balance_height_rate += BALANCE_HEIGHT_RATE_ALPHA * (*rate - balance_height_rate);
    *height = Notch_Update(&balance_height_notch, &balance_height_coeff, *height);
    *rate   = Notch_Update(&balance_height_rate_notch, &balance_height_coeff, balance_height_rate);
}
#endif

// 检查已解析的力矩反馈是否在上次关节命令接纳后、本次估计前到达。两个时基分别判龄与先后。
static bool Balance_Force_Frame(const dm_motor_t *motor, uint32_t now, uint32_t cycle, uint32_t accepted_cycle)
{
    return motor->online && motor->enabled && now - motor->frame_tick <= TINYMPC14_NODE_TICKS &&
           (int32_t)(motor->frame_cycle - accepted_cycle) >= 0 && (int32_t)(cycle - motor->frame_cycle) >= 0;
}

// 积分尚未形成的瞬态差力。只跨实际连续的双侧配对观测积分，换代或失去任一侧后重新播种。
static float Balance_Force_Impulse(uint32_t tick, bool paired, float deficit)
{
    if (!paired)
    {
        balance_force_history.ready = false;
        return 0.0f;
    }
    const uint32_t elapsed = tick - balance_force_history.tick;
    balance_force_history.tick = tick;
    if (!balance_force_history.ready || elapsed != CTRL_PERIOD_MS)
    {
        balance_force_history.bias = deficit;
        balance_force_history.previous = deficit;
        balance_force_history.integral = 0.0f;
        balance_force_history.ready = true;
        return 0.0f;
    }

    const float dt = 0.001f * (float)elapsed;
    const float horizon = TINYMPC14_NN * TINYMPC14_TS;
    const float alpha = dt / (2.0f * horizon + dt);
    const float bound = 0.5f * ((tinympc14_u_max[4] - tinympc14_u_min[4]) +
                                (tinympc14_u_max[5] - tinympc14_u_min[5])) * horizon;
    balance_force_history.bias += alpha * (deficit - balance_force_history.bias);
    const float area = dt * (0.5f * (deficit + balance_force_history.previous) - balance_force_history.bias);
    balance_force_history.previous = deficit;
    balance_force_history.integral = Balance_Clamp((1.0f - alpha) * balance_force_history.integral + area, bound);

    // 首节点已含方向相反的反馈差力增量，将其动态部分抵消一次。
    return balance_force_history.integral + TINYMPC14_TS * (deficit - balance_force_history.bias);
}

// 用同一拍的状态、参考及接纳输入推进双轮接地控制。
// 入场和求解失效时，Tinympc14_Async_Step_Compensated 使用当前模型的终端反馈。
static bool Balance_Ground_Update(balance_ctrl_mode_t mode, bool landing, const execution_sample_t *handoff,
                                  float hip_comp[2], volatile balance_mpc_force_t **force_record)
{
    const fsm_running_position_t pos = Fsm_Running_Position();
    const uint32_t epoch = Execution_Epoch();
    const uint32_t sample_tick = Estimator_Frame_Tick();
    const float len[2] = {body.leg_len_lft, body.leg_len_rgt};
    const float d_len[2] = {leg_lft.d_len, leg_rgt.d_len};
    float x[TINYMPC14_NX] = {
        mode == LEG_CTRL_MODE_STANDING ? pos.weight * pos.error : 0.0f,
#if BALANCE_USE_SLIP
        body.vel_kalman,
#else
        body.vel,
#endif
        body.ang_yaw - body.ang_yaw_tgt, body.omg_yaw,
        body.leg_ang_lft, body.leg_omg_lft, body.leg_ang_rgt, body.leg_omg_rgt,
        body.ang_pitch, body.omg_pitch, body.ang_roll, body.omg_roll_filt,
        body.h_body - TINYMPC14_H_REF, 0.0f,
    };
    float xr[TINYMPC14_NX] = {0.0f};
    float applied[TINYMPC14_NU] = {0.0f};
    float force_error[2] = {0.0f, 0.0f};
    unsigned int force_valid = 0u;
    float plan[TINYMPC14_NU];
    float axial_height = 0.0f, axial_rate = 0.0f;
    execution_sample_t last;
    tinympc14_comp_t comp;
    tinympc14_output_t output;

    // 模型角速度是 ZYX 欧拉角导数。实车陀螺已标定为体轴 p/q/r，
    // 俯仰时纯绕世界竖直轴转向也会产生体轴 p 分量。
    const float cos_roll = cosf(x[10]);
    const float sin_roll = sinf(x[10]);
    const float cos_pitch = cosf(x[8]);

    if (cos_pitch <= BODY_EULER_COS_MIN) return false; // 接近 ±90° 时接地模型的欧拉坐标退化
    x[3] = (sin_roll * body.omg_pitch + cos_roll * body.omg_yaw) / cos_pitch;
    x[9] = cos_roll * body.omg_pitch - sin_roll * body.omg_yaw;
    const float roll_coupling = sinf(x[8]) * x[3];

    x[5] += x[9] - body.omg_pitch;
    x[7] += x[9] - body.omg_pitch;
    for (unsigned int side = 0u; side < 2u; side++)
    {
        const unsigned int angle = 4u + 2u * side;
        const float c = cosf(x[angle]);

        axial_height += len[side] * c;
        axial_rate += d_len[side] * c - len[side] * sinf(x[angle]) * x[angle + 1u];
    }
    // h = cos(roll) · mean(L cos(psi))。几何导数使用同拍未滤的欧拉角速度。
    x[13] = 0.5f * (cos_roll * axial_rate - sin_roll * (body.omg_roll + roll_coupling) * axial_height);
    xr[1] = landing ? x[1] : body.vel_tgt; // 气弹簧恢复期间暂停纵向速度反馈
    xr[3] = body.omg_yaw_tgt;
    xr[8] = TINYMPC14_PITCH_REF;
    xr[12] = body.h_tgt - TINYMPC14_H_REF;
    if (!Tinympc14_All_Finite(x, TINYMPC14_NX) || !Tinympc14_All_Finite(xr, TINYMPC14_NX))
        return false;

    const bool start = !balance_mpc_active || balance_mpc_diag.epoch != epoch;

    if (start)
    {
        Balance_Mpc_Stop();
        Tinympc14_Output_Reset(&balance_mpc_comp);
        balance_mpc_turn = (turn_limit_leglen_t){0};
    }
#if BALANCE_USE_HEIGHT_FILTER
    Balance_Filter_Height(sample_tick, start, &x[12], &x[13]);
#endif
    Tinympc14_Output_Calc_Ex(len[0], len[1], x[4], x[6], x[1], x[3], &balance_mpc_comp, &comp);
    const turn_limit_leglen_input_t turn = {
        .h = body.h_body, .h_target = body.h_tgt, .roll = x[10],
        .len = {len[0], len[1]}, .turn_force = comp.Fc[0],
        .force_min = tinympc14_u_min[4], .force_max = tinympc14_u_max[4], .dt = CTRL_DT,
    };
    Turn_Limit_Leglen_Update(&turn, &balance_mpc_turn);
    xr[10] = balance_mpc_turn.roll_ref;
    xr[12] = balance_mpc_turn.h_target - TINYMPC14_H_REF;
#if !BALANCE_USE_ROLL_COMP
    comp.Fc[0] = 0.0f;
    comp.Fc[1] = 0.0f;
#endif
#if BALANCE_USE_CORIOLIS_TP
    comp.MgF[0] -= leg_control_lft.Coriolis_Tp;
    comp.MgF[1] -= leg_control_rgt.Coriolis_Tp;
#endif
    Execution_Latest(&last);
    if (handoff != NULL)
        last = *handoff;
    if (last.valid_mask == EXECUTION_CHANNEL_MASK && (last.epoch == epoch || handoff != NULL))
    {
        for (unsigned int i = 0u; i < 4u; i++) applied[i] = last.model_u[i];
        applied[4] = last.total_s[0];
        applied[5] = last.total_s[1];
    }
    else
    {
        // 新代次和接纳码尚未集齐时，用轴力反馈加回弹簧播种总支持力。
        applied[4] = body.F_lft + comp.Fs[0];
        applied[5] = body.F_rgt + comp.Fs[1];
    }
    if (!Tinympc14_All_Finite(applied, TINYMPC14_NU)) return false;

    if (!start && !landing && handoff == NULL && last.epoch == epoch && last.valid_mask == EXECUTION_CHANNEL_MASK &&
        sample_tick - last.tick <= TINYMPC14_NODE_TICKS)
    {
        const uint32_t now = HAL_GetTick();
        const uint32_t cycle = Estimator_Frame_Cycle();
        const dm_motor_t *motor[4] = {&dm_f_lft, &dm_b_lft, &dm_f_rgt, &dm_b_rgt};
        const float measured[2] = {body.F_lft + comp.Fs[0], body.F_rgt + comp.Fs[1]};

        for (unsigned int side = 0u; side < 2u; side++)
        {
            const unsigned int j = side * 2u;
            const unsigned int pair = 3u << j;
            if ((last.accepted_mask & pair) == pair && isfinite(measured[side]) &&
                Balance_Force_Frame(motor[j], now, cycle, last.cycle + last.offset[j]) &&
                Balance_Force_Frame(motor[j + 1u], now, cycle, last.cycle + last.offset[j + 1u]))
            {
                const float span = tinympc14_u_max[4u + side] - tinympc14_u_min[4u + side];
                force_error[side] = Balance_Clamp(measured[side] - applied[4u + side], span);
                force_valid |= 1u << side;
            }
        }
    }

    // 转弯预补偿与倾覆按物理量冻结，各自用前台/后台实际采用的 LPV 模型投影。
    const tinympc14_ground_observation_t observation = {
        .turn = {leg_control_lft.Turn_DistF, leg_control_rgt.Turn_DistF},
        .comp = {comp.Fc[0], comp.Fc[1]}, .error = {force_error[0], force_error[1]},
        .height = body.h_body, .valid = force_valid,
        .pending_diff_impulse = Balance_Force_Impulse(sample_tick, force_valid == 3u, force_error[1] - force_error[0]),
    };
    taskENTER_CRITICAL();
    Tinympc14_Async_Applied(applied);
    Tinympc14_Async_Input_Ground_At(x, xr, len[0], len[1], sample_tick, &observation);
    Tinympc14_Async_Run();
    balance_mpc_active = true;
    taskEXIT_CRITICAL();
    const unsigned int old_fallbacks = tinympc14_counters.fallback;
    const unsigned int old_nan       = tinympc14_counters.nan;

    // 降高时协调合力与差力。用当前角和短时外推角的较大值，在转弯侧倾边界内逐渐让回横滚余量。
    float lower_share = 0.0f;
    if (!landing && x[12] > xr[12])
    {
        const float horizon = TINYMPC14_NN * TINYMPC14_TS;
        const float tilt = fmaxf(fabsf(x[10]), fabsf(x[10] + horizon * x[11]));
        lower_share = Tinympc14_Clip((LEGLEN_TURN_ROLL_STOP - tilt) /
                                     (LEGLEN_TURN_ROLL_STOP - LEGLEN_TURN_ROLL_ALLOW), 0.0f, 1.0f);
    }
    const float lowering_reduction = Tinympc14_Async_Step_Lowering(CTRL_PERIOD_MS, comp.Fc, lower_share);
    // 过期或无效输入只保持旧命令。新鲜输入以本次消费实际走过的回退分支分类。
    if (Tinympc14_Port_Tick() - sample_tick < TINYMPC14_NODE_TICKS * TINYMPC14_NN &&
        tinympc14_counters.nan == old_nan)
        *force_record = tinympc14_counters.fallback == old_fallbacks ?
                        &balance_mpc_diag.predicted_force : &balance_mpc_diag.terminal_force;
    for (unsigned int i = 0u; i < TINYMPC14_NU; i++) plan[i] = tinympc14_u[i];
    Tinympc14_Output_Apply_Period(plan, &comp, applied, CTRL_PERIOD_MS, &output);

    leg_control_lft.T = output.T_w[0];
    leg_control_rgt.T = output.T_w[1];
    leg_control_lft.Tp = output.Tp[0];
    leg_control_rgt.Tp = output.Tp[1];
    leg_control_lft.F = output.F_motor[0];
    leg_control_rgt.F = output.F_motor[1];
    leg_control_lft.SpringF = -comp.Fs[0];
    leg_control_rgt.SpringF = -comp.Fs[1];
    hip_comp[0] = comp.MgF[0];
    hip_comp[1] = comp.MgF[1];
#if STEP_ENABLE
    Balance_Step_Blend();
#endif

    taskENTER_CRITICAL();
    balance_mpc_diag.paired = false;
    balance_mpc_diag.epoch = epoch;
    balance_mpc_diag.cycle = Estimator_Frame_Cycle();
    balance_mpc_diag.sample_tick = sample_tick;
    balance_mpc_diag.roll_rate = x[11];
    balance_mpc_diag.height = x[12] + TINYMPC14_H_REF;
    balance_mpc_diag.height_rate = x[13];
    balance_mpc_diag.lowering_reduction = lowering_reduction;
    balance_mpc_diag.roll_prediction = Tinympc14_Async_Roll_Increment();
    balance_mpc_diag.roll_ref = xr[10];
    balance_mpc_diag.height_ref = balance_mpc_turn.h_target;
    balance_mpc_diag.leg_angle[0] = x[4];
    balance_mpc_diag.leg_angle[1] = x[6];
    balance_mpc_diag.accel_y = body.body_accel_y;
    balance_mpc_diag.pitch = x[8];
    balance_mpc_diag.force_error[0] = force_error[0];
    balance_mpc_diag.force_error[1] = force_error[1];
    balance_mpc_diag.force_valid = (uint8_t)force_valid;
    balance_mpc_diag.pending_diff_impulse = observation.pending_diff_impulse;
    balance_mpc_diag.fallbacks = tinympc14_counters.fallback;
    balance_mpc_diag.stale = tinympc14_counters.stale;
    for (unsigned int i = 0u; i < TINYMPC14_NU; i++)
        balance_mpc_diag.plan[i] = output.u_applied[i];
    taskEXIT_CRITICAL();
    return true;
}

// 离地只用终端反馈中两腿摆角和角速度的髋矩项，轮矩与接地预测均退出。
static void Balance_Air_Update(void)
{
    const float leg_state[4] = {body.leg_ang_lft, body.leg_omg_lft, body.leg_ang_rgt, body.leg_omg_rgt};
    const tinympc14_model_t *model = Tinympc14_Model();
    float hip[2] = {0.0f};

    if (!Tinympc14_All_Finite(leg_state, 4)) return;
    for (unsigned int side = 0u; side < 2u; side++)
    {
        const unsigned int channel = side + 2u;

        for (unsigned int i = 0u; i < 4u; i++)
            hip[side] -= model->Kinf[channel * TINYMPC14_NX + 4u + i] * leg_state[i];
        hip[side] = Tinympc14_Clip(hip[side], tinympc14_u_min[channel], tinympc14_u_max[channel]);
    }
    leg_control_lft.Tp = hip[0];
    leg_control_rgt.Tp = hip[1];
}

// 单侧先触地时，以本拍轴力为支撑种子并保留气弹簧恢复，不启动双支撑预测。
static bool Balance_Landing_Update(void)
{
    if (!isfinite(body.F_lft) || !isfinite(body.F_rgt)) return false;
    Balance_Mpc_Stop();
    Balance_Air_Update();
    leg_control_lft.F = Tinympc14_Clip(body.F_lft - leg_control_lft.SpringF, 0.0f, tinympc14_u_max[4]) + leg_control_lft.SpringF;
    leg_control_rgt.F = Tinympc14_Clip(body.F_rgt - leg_control_rgt.SpringF, 0.0f, tinympc14_u_max[5]) + leg_control_rgt.SpringF;
    return true;
}

// 自救摆角反馈 腿长与防劈叉出力
static void Balance_Rescue_Update(void)
{
    leg_control_lft.Tp = Balance_Clamp(leg_control_lft.MgF + sr_pid_leg_ang_lft.out + sr_pid_leg_symmetry.out, SR_PID_LEG_ANG_OUT_MAX);
    leg_control_rgt.Tp = Balance_Clamp(leg_control_rgt.MgF + sr_pid_leg_ang_rgt.out - sr_pid_leg_symmetry.out, SR_PID_LEG_ANG_OUT_MAX);

    leg_control_lft.F = leg_control_lft.SpringF + leg_control_lft.Roll_CompF + sr_pid_leg_len_lft.out;
    leg_control_rgt.F = leg_control_rgt.SpringF + leg_control_rgt.Roll_CompF + sr_pid_leg_len_rgt.out;
}

// 自救摆腿髋力矩合成
static void Balance_Rescue_Swing_Update(void)
{
    leg_control_lft.Tp = leg_control_lft.MgF + sr_pid_leg_omg_swing_lft.out;
    leg_control_rgt.Tp = leg_control_rgt.MgF + sr_pid_leg_omg_swing_rgt.out;
}

// 自救立腿角速度反馈 腿长反馈出力
static void Balance_Rescue_Stand_Update(void)
{
    leg_control_lft.Tp = leg_control_lft.MgF + sr_pid_leg_omg_stand_lft.out;
    leg_control_rgt.Tp = leg_control_rgt.MgF + sr_pid_leg_omg_stand_rgt.out;

    leg_control_lft.F = leg_control_lft.SpringF + sr_pid_leg_len_lft.out;
    leg_control_rgt.F = leg_control_rgt.SpringF + sr_pid_leg_len_rgt.out;
}

// 测试档悬空时补偿腿与轮自重。固定世界系腿角，对质心向下坐标求腿长导数。
float Balance_Axial_Gravity(float len, float ang)
{
    const float center = LEG_LEN_CLAMP(len);
    const float short_len = LEG_LEN_CLAMP(center - 0.005f);
    const float long_len = LEG_LEN_CLAMP(center + 0.005f);
    float short_ang, short_radius, long_ang, long_radius;

    Leg_Comp(short_len, &short_ang, &short_radius);
    Leg_Comp(long_len, &long_ang, &long_radius);
    const float short_down = short_radius * arm_cos_f32(ang + short_ang);
    const float long_down = long_radius * arm_cos_f32(ang + long_ang);
    const float leg_slope = (long_down - short_down) / (long_len - short_len);

    return -GRAVITY * (LEG_GRAV_FF * leg_slope + WHEEL_MASS * arm_cos_f32(ang));
}


// 模式出力选择 未使用通道保持本拍零值
static void Balance_Vector_Update(balance_ctrl_mode_t ctrl_mode)
{
    switch (ctrl_mode)
    {
        case LEG_CTRL_MODE_SELF_RESCUE_TURNOVER: // 翻身沿用摆角反馈 强制不抵消气弹簧
            Balance_Rescue_Update();
            leg_control_lft.F = 0.0f;
            leg_control_rgt.F = 0.0f;
            break;
        case LEG_CTRL_MODE_SELF_RESCUE: // 自救
            Balance_Rescue_Update();
            break;
        case LEG_CTRL_MODE_SELF_RESCUE_SWING: // 自救摆腿 腿长通道只抵消被动弹簧力
            Balance_Rescue_Swing_Update();
            leg_control_lft.F = leg_control_lft.SpringF;
            leg_control_rgt.F = leg_control_rgt.SpringF;
            break;
        case LEG_CTRL_MODE_SELF_RESCUE_STAND: // 自救立腿 纯 P 角速度环 + 自重前馈 腿长环照常
            Balance_Rescue_Stand_Update();
            break;
        case LEG_CTRL_MODE_AIR: // 离地腿姿保护，仅发髋矩
            Balance_Air_Update();
            break;
        case LEG_CTRL_MODE_TEST: // 悬空测试：髋通道与腿长通道分别抵消重力
            // MgF 只含非轮腿机构；悬空时轮子也绕髋轴产生重力矩。
            leg_control_lft.Tp = leg_control_lft.MgF + WHEEL_MASS * GRAVITY * leg_lft.len * arm_sin_f32(body.leg_ang_lft);
            leg_control_rgt.Tp = leg_control_rgt.MgF + WHEEL_MASS * GRAVITY * leg_rgt.len * arm_sin_f32(body.leg_ang_rgt);
            leg_control_lft.F  = leg_control_lft.SpringF + Balance_Axial_Gravity(leg_lft.len, body.leg_ang_lft);
            leg_control_rgt.F  = leg_control_rgt.SpringF + Balance_Axial_Gravity(leg_rgt.len, body.leg_ang_rgt);
            break;

        default: // 非法模式 未使用通道保持零
            break;
    }
}

// 先叠加腿向阻尼，再处理离开接地时的轴力过渡。
static void Balance_Axial_Output_Update(balance_ctrl_mode_t ctrl_mode, const float prev_force[2])
{
    if (ctrl_mode != LEG_CTRL_MODE_AIR && ctrl_mode != LEG_CTRL_MODE_SELF_RESCUE_TURNOVER)
    {
        // 伸腿为正；速度阻尼和其余轴力一起由 Leg_Trq 映射到关节。
        if (isfinite(leg_lft.d_len)) leg_control_lft.F -= BALANCE_LEG_DAMPING_NS_PER_M * leg_lft.d_len;
        if (isfinite(leg_rgt.d_len)) leg_control_rgt.F -= BALANCE_LEG_DAMPING_NS_PER_M * leg_rgt.d_len;
    }
    if (Balance_Mode_Grounded(ctrl_mode) || ctrl_mode == LEG_CTRL_MODE_SELF_RESCUE_TURNOVER
#if STEP_ENABLE
        || ctrl_mode == LEG_CTRL_MODE_STEP
#endif
    )
    {
        balance_force_exit_cnt = 0u;
        return;
    }
    if (Balance_Mode_Grounded(balance_mode_prev))
    {
        balance_force_exit_soft[0] = prev_force[0] - leg_control_lft.F;
        balance_force_exit_soft[1] = prev_force[1] - leg_control_rgt.F;
        leg_control_lft.F          = prev_force[0];
        leg_control_rgt.F          = prev_force[1];
        balance_force_exit_cnt     = BALANCE_FORCE_EXIT_TICKS;
        return;
    }
    if (balance_force_exit_cnt > 0u)
    {
        balance_force_exit_cnt--;
        const float k = (float)balance_force_exit_cnt / (float)BALANCE_FORCE_EXIT_TICKS;
        leg_control_lft.F += balance_force_exit_soft[0] * k;
        leg_control_rgt.F += balance_force_exit_soft[1] * k;
    }
}

// 关节映射与六路发送 右腿关节交换 右轮极性反转
static void Balance_Trq_Out(void)
{
    Leg_Trq(leg_control_lft.F, leg_control_lft.Tp, leg_lft.pitch_1, leg_lft.pitch_4, leg_control_lft.out);
    Leg_Trq(leg_control_rgt.F, LEG_MIRROR_CHANGE * leg_control_rgt.Tp, leg_rgt.pitch_1, leg_rgt.pitch_4, leg_control_rgt.out);

    float trq_l0 = leg_control_lft.out[0];
    float trq_l1 = leg_control_lft.out[1];
    float trq_r0 = leg_control_rgt.out[1];
    float trq_r1 = leg_control_rgt.out[0];
    float trq_wl = leg_control_lft.T;
    float trq_wr = leg_control_rgt.T;

    if (!isfinite(trq_l0))
        trq_l0 = 0.0f;
    if (!isfinite(trq_l1))
        trq_l1 = 0.0f;
    if (!isfinite(trq_r0))
        trq_r0 = 0.0f;
    if (!isfinite(trq_r1))
        trq_r1 = 0.0f;
    if (!isfinite(trq_wl))
        trq_wl = 0.0f;
    if (!isfinite(trq_wr))
        trq_wr = 0.0f;

    if (trq_l0 > DM_T_MAX || trq_l0 < -DM_T_MAX || trq_l1 > DM_T_MAX || trq_l1 < -DM_T_MAX)
        monitor_total.leg_clamp_lft_cnt++;
    if (trq_r0 > DM_T_MAX || trq_r0 < -DM_T_MAX || trq_r1 > DM_T_MAX || trq_r1 < -DM_T_MAX)
        monitor_total.leg_clamp_rgt_cnt++;

    trq_l0 = Balance_Clamp(trq_l0, DM_T_MAX);
    trq_l1 = Balance_Clamp(trq_l1, DM_T_MAX);
    trq_r0 = Balance_Clamp(trq_r0, DM_T_MAX);
    trq_r1 = Balance_Clamp(trq_r1, DM_T_MAX);
    trq_wl = WHEEL_LFT_CMD_POLARITY * Balance_Clamp(trq_wl, M3508_OUT_MAX / M3508_TO_TORQUE) * M3508_TO_TORQUE;
    trq_wr = WHEEL_RGT_CMD_POLARITY * Balance_Clamp(trq_wr, M3508_OUT_MAX / M3508_TO_TORQUE) * M3508_TO_TORQUE;

    Dm_Motor_Mit_Ctrl(dm_f_lft.handle, dm_f_lft.slave_id, 0.0f, 0.0f, 0.0f, 0.0f, trq_l0);
    Dm_Motor_Mit_Ctrl(dm_b_lft.handle, dm_b_lft.slave_id, 0.0f, 0.0f, 0.0f, 0.0f, trq_l1);
    Dm_Motor_Mit_Ctrl(dm_f_rgt.handle, dm_f_rgt.slave_id, 0.0f, 0.0f, 0.0f, 0.0f, trq_r0);
    Dm_Motor_Mit_Ctrl(dm_b_rgt.handle, dm_b_rgt.slave_id, 0.0f, 0.0f, 0.0f, 0.0f, trq_r1);
    Dji_Send_Group(DJI_GROUP_A, (float[4]){trq_wl, trq_wr, 0.0f, 0.0f});

    // 在向量被下一拍清空前保存完整请求；这是请求值，不代表电机已执行或限幅后的实际力。
    balance_force_requested[0] = leg_control_lft.F - leg_control_lft.SpringF;
    balance_force_requested[1] = leg_control_rgt.F - leg_control_rgt.SpringF;
}

// 冻结合成结果；映射、分配、编码、发送和接纳发布由同一模块管理。
static bool Balance_Distributor_Out(bool stepping)
{
    const fsm_landing_t landing = Fsm_Running_Landing();
    distributor_frame_t frame = {
        .tick     = Estimator_Frame_Tick(),
        .cycle    = Estimator_Frame_Cycle(),
        .phi      = {{leg_lft.pitch_1, leg_lft.pitch_4}, {leg_rgt.pitch_1, leg_rgt.pitch_4}},
        .force    = {leg_control_lft.F, leg_control_rgt.F},
        .hip      = {leg_control_lft.Tp, leg_control_rgt.Tp},
        .spring_f = {-leg_control_lft.SpringF, -leg_control_rgt.SpringF},
        .hip_comp = {balance_hip_comp[0], balance_hip_comp[1]},
        .turn     = {leg_control_lft.Turn_DistF, leg_control_rgt.Turn_DistF},
        .wheel    = {leg_control_lft.T, leg_control_rgt.T},
        .height_priority = stepping || ((balance_mpc_active && balance_mpc_turn.height_priority)),
        .stepping = stepping,
        .state    = {
               .pitch      = body.ang_pitch,
               .pitch_rate = body.omg_pitch,
               .roll       = body.ang_roll,
               .roll_rate  = body.omg_roll,
               .height     = body.h_body,
               .height_ref = body.h_tgt,
               .len        = {leg_lft.len, leg_rgt.len},
        },
    };
    float nominal[2][2];

    if (landing.active)
    {
        frame.land.active       = true;
        frame.land.elapsed_ms   = landing.elapsed_ms;
        frame.land.spring_scale = landing.spring_scale;
        for (unsigned int side = 0u; side < 2u; side++)
        {
            // 只撤去指定比例的轴向弹簧补偿，分配和执行历史继续使用真实 Fs。
            frame.land.nominal[side] = frame.force[side] + frame.spring_f[side];
            frame.force[side]       += (1.0f - landing.spring_scale) * frame.spring_f[side];
        }
    }

    if (!Distributor_Send(&frame, nominal)) return false;
    balance_landing_prev = landing.active;
    for (unsigned int joint = 0u; joint < 2u; joint++)
    {
        leg_control_lft.out[joint] = nominal[0][joint];
        leg_control_rgt.out[joint] = nominal[1][joint];
    }
    balance_force_requested[0] = frame.force[0] + frame.spring_f[0];
    balance_force_requested[1] = frame.force[1] + frame.spring_f[1];
    return true;
}

// 控制任务读取上一拍请求，与本拍反馈组成诊断快照，不参与控制与观测预测。
void Balance_Force_Diagnostic(float request[2])
{
    request[0] = balance_force_requested[0];
    request[1] = balance_force_requested[1];
}

// 停止模型并发送本拍零力矩
static void Balance_Stop(void)
{
    Balance_Mpc_Stop();
#if STEP_ENABLE
    if (App_Step_Owns_Output())
    {
        App_Step_Exit(STEP_REASON_FEEDBACK);
        App_Safety_Trip();
        App_Safety_Apply_Flag();
    }
    balance_step_mix = 1.0f;
#endif
    Fsm_Running_Exit();
    Distributor_Mode(LEG_CTRL_MODE_NUM, false);
    leg_control_lft   = (control_vector_t){.T = 0.0f};
    leg_control_rgt   = (control_vector_t){.T = 0.0f};
    balance_mode_prev = LEG_CTRL_MODE_NUM;
    balance_force_exit_cnt = 0u;
    balance_landing_prev   = false;
    Balance_Trq_Out();
}

// 补偿与自救输入检查 模型状态由后端校验
static bool Balance_State_Check(balance_ctrl_mode_t ctrl_mode)
{
    if (!isfinite(leg_lft.pitch_1) || !isfinite(leg_lft.pitch_4) ||
        !isfinite(leg_rgt.pitch_1) || !isfinite(leg_rgt.pitch_4))
        return false;
    if (ctrl_mode == LEG_CTRL_MODE_AIR)
        return true;
#if BALANCE_USE_SLIP
    const float vel = body.vel_kalman;
#else
    const float vel = body.vel;
#endif
    if (!isfinite(leg_lft.len) || !isfinite(leg_rgt.len) ||
        !isfinite(body.h_body) || !isfinite(body.omg_yaw_filt) || !isfinite(vel))
        return false;
    if (!isfinite(body.leg_ang_lft) || !isfinite(body.leg_ang_rgt))
        return false;
#if BALANCE_USE_CORIOLIS_TP
    if (!isfinite(body.leg_len) || !isfinite(body.omg_yaw) ||
        !isfinite(body.ang_pitch) || !isfinite(body.ang_roll) ||
        !isfinite(body.omg_pitch) || !isfinite(body.omg_roll))
        return false;
#endif
    if (Balance_Mode_Grounded(ctrl_mode) || ctrl_mode == LEG_CTRL_MODE_TEST)
        return true;
#if STEP_ENABLE
    if (ctrl_mode == LEG_CTRL_MODE_STEP)
        return true; // 行为入口已检查专有反馈与行程
#endif
    if (!isfinite(body.leg_len_lft) || !isfinite(body.leg_len_rgt) ||
        !isfinite(body.leg_len_tgt_lft) || !isfinite(body.leg_len_tgt_rgt) ||
        !isfinite(leg_lft.ang - leg_rgt.ang))
        return false;
    if (ctrl_mode == LEG_CTRL_MODE_SELF_RESCUE || ctrl_mode == LEG_CTRL_MODE_SELF_RESCUE_TURNOVER)
        return isfinite(body.leg_ang_tgt_lft) && isfinite(body.leg_ang_tgt_rgt) &&
               isfinite(leg_lft.ang) && isfinite(leg_rgt.ang);
    return isfinite(body.leg_omg_tgt_lft) && isfinite(body.leg_omg_tgt_rgt) &&
           isfinite(body.leg_omg_lft) && isfinite(body.leg_omg_rgt);
}

// 上电初始化模型 自救 PID 交由 App 初始化
void Balance_Control_Init(void)
{
    Distributor_Init();
    leg_control_lft   = (control_vector_t){.T = 0.0f};
    leg_control_rgt   = (control_vector_t){.T = 0.0f};
    balance_mode_prev = LEG_CTRL_MODE_NUM;
    balance_force_exit_cnt = 0u;
    balance_landing_prev   = false;
    App_Self_Rescue_Pid_Init();
}

// 同拍接纳记录回算下一拍锚点，诊断只配对完整记录。
static void Balance_Mpc_Accepted(volatile balance_mpc_force_t *force_record)
{
    if (!balance_mpc_active) return;
    execution_sample_t last;

    Execution_Latest(&last);
    if (last.epoch != balance_mpc_diag.epoch || last.cycle != balance_mpc_diag.cycle ||
        last.valid_mask != EXECUTION_CHANNEL_MASK) return;

    taskENTER_CRITICAL();
    for (unsigned int i = 0u; i < 4u; i++) balance_mpc_diag.accepted[i] = last.model_u[i];
    balance_mpc_diag.accepted[4] = last.total_s[0];
    balance_mpc_diag.accepted[5] = last.total_s[1];
    balance_mpc_diag.accepted_mask = last.accepted_mask;
    balance_mpc_diag.paired = true;
    // valid_mask 可含旧码；四个关节均在本拍入队才按本拍控制来源记录实际分配后轴力。
    const unsigned int joint_mask = (1u << DISTRIBUTOR_JOINT_NUM) - 1u;

    if (force_record != NULL && (last.accepted_mask & joint_mask) == joint_mask)
    {
        force_record->tick  = last.tick;
        force_record->sum   = last.total_s[0] + last.total_s[1];
        force_record->diff  = last.total_s[0] - last.total_s[1];
        force_record->valid = true;
    }
    taskEXIT_CRITICAL();
}

// 分配成功后，才把本拍实际接纳的结果交给 MPC 和台阶控制。
static bool Balance_Send_Allocated(bool stepping, volatile balance_mpc_force_t *force_record)
{
    if (!Balance_Distributor_Out(stepping)) return false;
    Balance_Mpc_Accepted(force_record);
#if STEP_ENABLE
    const uint8_t limits = Distributor_Output_Limits();

    if (stepping)
        App_Step_Accepted(limits);
    else
        App_Step_Normal_Limits(limits);
#endif
    return true;
}

// 首次建模使用求解任务的栈，完成后初始化任务才能开放估计与控制。
void Balance_Mpc_Init(void)
{
    Tinympc14_Init();
    Tinympc14_Async_Invalidate();
    balance_mpc_active = false;
    balance_mpc_diag = (balance_mpc_diag_t){0};
    balance_mpc_ready = true;
}

// 初始化任务读取首个接地模型的发布状态。
bool Balance_Mpc_Ready(void)
{
    return balance_mpc_ready;
}

// 取消当前接地代次的在途后台求解，下一次由控制拍发布新快照。
void Balance_Mpc_Stop(void)
{
    if (!balance_mpc_active) return;
    taskENTER_CRITICAL();
    balance_mpc_active = false;
    balance_mpc_diag.paired = false;
    balance_mpc_diag.lowering_reduction = 0.0f;
    balance_mpc_diag.predicted_force = (balance_mpc_force_t){0};
    balance_mpc_diag.terminal_force  = (balance_mpc_force_t){0};
    Tinympc14_Async_Invalidate();
    taskEXIT_CRITICAL();
}

// 后台独占更新模型与求解，发布失败时由控制拍继续使用终端反馈。
void Balance_Mpc_Solve(void)
{
    if (!balance_mpc_active) return;
    Tinympc14_Async_Solve();
    taskENTER_CRITICAL();
    if (balance_mpc_diag.commits != tinympc14_counters.commit)
    {
        balance_mpc_diag.publish_tick = osKernelGetTickCount();
        balance_mpc_diag.commits = tinympc14_counters.commit;
    }
    taskEXIT_CRITICAL();
}

// 遥测按临界区读取状态、请求和接纳的诊断快照。
void Balance_Mpc_Diagnostic(balance_mpc_diag_t *out)
{
    taskENTER_CRITICAL();
    *out = balance_mpc_diag;
    // 总线可在两个控制拍之间切代；观察时立即屏蔽前代的两类接纳记录。
    if (balance_mpc_active && out->epoch != Execution_Epoch())
    {
        out->lowering_reduction = 0.0f;
        out->predicted_force = (balance_mpc_force_t){0};
        out->terminal_force  = (balance_mpc_force_t){0};
    }
    taskEXIT_CRITICAL();
}
