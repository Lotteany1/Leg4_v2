/** @file  tinympc14_integration_test.c  @brief 真实求解器、分配和 CAN 接纳联测  @version  v1.2  2026-11-19 */
#include "tinympc14_ground_fixture.h"
#include "Leg_RollComp.h"

// 独立完整 CAD 的 dV/d[L,theta]：非轮腿和轮子都悬空，角度单位 rad。
// 保留两侧平均模型带来的偏差预算，能够检出原先漏掉的约 0.47 N·m 轮重矩。
static void Test_Suspended_Gravity(void)
{
    const float angle[3] = {-0.3f, 0.0f, 0.3f};
    const float cad_force[3][2] = {{-9.225710f, -9.465212f}, {-10.167431f, -10.462223f}, {-10.200925f, -10.524674f}};
    const float cad_torque[3][2] = {{0.164952f, 0.223028f}, {0.909104f, 0.986275f}, {1.572049f, 1.661421f}};

    for (unsigned int k = 0u; k < 3u; ++k)
    {
        Test_Reset();
        body.ang_pitch = TINYMPC14_PITCH_REF + angle[k];
        body.leg_ang_lft = body.leg_ang_rgt = angle[k];
        Test_Beat(LEG_CTRL_MODE_TEST);
        const control_vector_t *leg[2] = {&leg_control_lft, &leg_control_rgt};

        for (unsigned int side = 0u; side < 2u; ++side)
        {
            assert(fabsf(leg[side]->MotorF - leg[side]->SpringF - cad_force[k][side]) < 0.20f);
            assert(fabsf(leg[side]->Tp - cad_torque[k][side]) < 0.06f);
        }
    }
    puts("PASS: suspended TEST gravity agrees with independent per-side CAD, including wheel hip moment");
}

// 在同一运动状态和接纳历史下改变力矩反馈，分别核查终端、已发布序列及下一次后台求解。
static void Test_Force_Feedback(float error, bool publish, bool stale, float *now_diff, float *next_diff)
{
    dm_motor_t *motor[4] = {&dm_f_lft, &dm_b_lft, &dm_f_rgt, &dm_b_rgt};
    Test_Reset();
    const balance_mpc_diag_t first = Test_Beat(LEG_CTRL_MODE_STANDING);
    const float spring = Leg_SpringF(TINYMPC14_H_REF, 0.0f);
    const uint32_t next_tick = test_tick + CTRL_PERIOD_MS;
    if (publish) Balance_Mpc_Solve();
    for (unsigned int i = 0u; i < 4u; i++)
    {
        motor[i]->enabled = motor[i]->online = 1u;
        motor[i]->frame_tick = next_tick;
        motor[i]->frame_cycle = next_tick * (SystemCoreClock / 1000u);
    }
    if (stale)
    {
        motor[0]->frame_cycle = first.cycle - 1u; // 左前早于上一命令接纳
        motor[2]->frame_tick = next_tick - TINYMPC14_NODE_TICKS - 1u;
    }
    body.F_lft = first.accepted[4] - spring + error;
    body.F_rgt = first.accepted[5] - spring - error;
    const balance_mpc_diag_t beat = Test_Beat(LEG_CTRL_MODE_STANDING);
    assert(beat.force_valid == (stale ? 0u : 3u));
    assert(fabsf(beat.force_error[0] - (stale ? 0.0f : error)) < 1e-4f);
    assert(fabsf(beat.force_error[1] + (stale ? 0.0f : error)) < 1e-4f);
    assert(beat.paired && (publish ? beat.predicted_force.valid : beat.terminal_force.valid));
    for (unsigned int side = 0u; side < 2u; side++)
        assert(fabsf(beat.plan[4u + side] - first.accepted[4u + side]) <= TINYMPC14_DF_RATE / 2.0f + 1e-4f);
    *now_diff = beat.plan[4] - beat.plan[5];

    const unsigned int commits = tinympc14_counters.commit;
    Balance_Mpc_Solve();
    assert(tinympc14_counters.commit == commits + 1u);
    const float *seq = Tinympc14_Sequence();
    *next_diff = seq[4] - seq[5];
    for (unsigned int i = 0u; i < 4u; i++) motor[i]->enabled = motor[i]->online = 0u;
}

static void Test_Realized_Force(void)
{
    for (unsigned int mode = 0u; mode < 2u; mode++)
    {
        float base, base_next, changed, changed_next, stale, stale_next;
        Test_Force_Feedback(0.0f, mode != 0u, false, &base, &base_next);
        Test_Force_Feedback(25.0f, mode != 0u, false, &changed, &changed_next);
        Test_Force_Feedback(25.0f, mode != 0u, true, &stale, &stale_next);
        assert(changed < base - 0.05f && changed_next < base_next - 0.05f);
        assert(fabsf(stale - base) < 1e-4f && fabsf(stale_next - base_next) < 1e-3f);
    }
}

// 单独检查一拍、半节点和完整预测节点。沿计划运动时应复现原命令；
// 正反滚速偏差仅改变差力，过期估计不能再次校正旧命令。
static void Test_Committed_Roll_Rate(void)
{
    const float ref[TINYMPC14_NX] = {[8] = TINYMPC14_PITCH_REF};
    float x[TINYMPC14_NX] = {[8] = TINYMPC14_PITCH_REF, [11] = 0.004f};
    const float zero[TINYMPC14_NX] = {0.0f};
    const tinympc14_ground_observation_t obs = {.height = TINYMPC14_H_REF};
    float hold[TINYMPC14_NU], uff[TINYMPC14_NU];
    Test_Reset();
    const uint32_t base = test_tick;
    Tinympc14_Affine_Pub(ref, zero, zero, hold, uff);
    Tinympc14_Async_Applied(hold);
    Tinympc14_Async_Input_Ground_At(x, ref, obs.height, obs.height, base, &obs);
    Tinympc14_Async_Run();
    Tinympc14_Async_Solve();
    assert(tinympc14_counters.commit == 1u);
    const tinympc14_model_t *m = Tinympc14_Model();
    const float *seq = Tinympc14_Sequence();
    const float first_rate = x[11];
    float next_rate = m->fd[11];
    for (unsigned int j = 0; j < TINYMPC14_NX; j++) next_rate += m->Ad[11*TINYMPC14_NX+j]*x[j];
    for (unsigned int i = 0; i < TINYMPC14_NU; i++) next_rate += m->Bd[11*TINYMPC14_NU+i]*(seq[i]+uff[i]);
    assert(fabsf(next_rate-first_rate) > 1e-5f);
    const float gain = m->Kinf[4*TINYMPC14_NX+11]-m->Kinf[5*TINYMPC14_NX+11];
    for (unsigned int n = 0; n <= TINYMPC14_NODE_TICKS; n++)
    {
        const uint32_t stamp = base+n;
        const float rate = first_rate+(next_rate-first_rate)*(float)n/TINYMPC14_NODE_TICKS;
        for (int sign = -1; sign <= 1; sign++)
        {
            const float innovation = 0.001f*(float)sign;
            x[11] = rate+innovation;
            for (int stale = 0; stale <= 1; stale++)
            {
                test_tick = stamp+stale*TINYMPC14_NODE_TICKS;
                Tinympc14_Async_Applied(hold);
                Tinympc14_Async_Input_Ground_At(x, ref, obs.height, obs.height, stamp, &obs);
                Tinympc14_Async_Step_Compensated(1u, obs.comp);
                const float left = tinympc14_u[4], right = tinympc14_u[5];
                Tinympc14_Async_Applied(hold);
                Tinympc14_Async_Step_Lowering(1u, obs.comp, 0.0f);
                const float shift = stale ? 0.0f : 0.5f*gain*innovation;
                assert(fabsf(tinympc14_u[4]-left+shift) < 2e-4f);
                assert(fabsf(tinympc14_u[5]-right-shift) < 2e-4f);
            }
        }
    }
}

// 接地原始外扰在新工作点重投影。期间一次前台消费不能改变后台观测原配的接纳锚点。
static void Test_Force_Snapshot(void)
{
    const float length[2] = {0.23f, 0.32f};
    const float x[TINYMPC14_NX] = {[4] = 0.27f, [6] = 0.27f, [8] = TINYMPC14_PITCH_REF, [10] = -0.06f};
    const float ref[TINYMPC14_NX] = {[8] = TINYMPC14_PITCH_REF, [12] = 0.035f};
    const float anchor[TINYMPC14_NU] = {0.0f, 0.0f, 0.0f, 0.0f, 23.0f, 32.0f};
    const float zero[TINYMPC14_NX] = {0.0f};
    const tinympc14_ground_observation_t physical = {
        .turn = {8.0f, -8.0f}, .comp = {-3.0f, 3.0f}, .height = 0.30f,
    };
    float expected_fed[TINYMPC14_NX] = {0.0f};
    float expected[TINYMPC14_NUU];
    Test_Reset();
    Tinympc14_Model_Update(length[0], length[1], x[4]);
    const tinympc14_model_t *m = Tinympc14_Model();
    for (unsigned int i = 0u; i < TINYMPC14_NX; i++)
        expected_fed[i] = m->Bd[i * TINYMPC14_NU + 4u] * 5.0f - m->Bd[i * TINYMPC14_NU + 5u] * 5.0f;
    Tinympc14_Roll_Disturbance(x[10], physical.height, expected_fed);
    Tinympc14_Async_Applied(anchor);
    Tinympc14_Async_Input(x, ref, expected_fed, zero, length[0], length[1]);
    Tinympc14_Async_Run();
    // 通用接口沿用消费时传入的外部输出补偿。
    Tinympc14_Async_Step_Compensated(1u, physical.comp);
    Tinympc14_Async_Solve();
    assert(tinympc14_counters.commit == 1u);
    for (unsigned int i = 0u; i < TINYMPC14_NUU; i++) expected[i] = Tinympc14_Sequence()[i];

    Test_Reset();
    Tinympc14_Async_Applied(anchor);
    Tinympc14_Async_Input_Ground_At(x, ref, length[0], length[1], test_tick, &physical);
    Tinympc14_Async_Run();
    // 模拟采样之后另外一次消费或发送，不许改掉同拍接纳力。
    Tinympc14_Async_Step_Compensated(1u, physical.comp);
    const float later[TINYMPC14_NU] = {0.0f, 0.0f, 0.0f, 0.0f, 90.0f, 90.0f};
    Tinympc14_Async_Applied(later);
    Tinympc14_Async_Solve();
    assert(tinympc14_counters.commit == 1u);
    for (unsigned int i = 0u; i < TINYMPC14_NUU; i++)
        assert(fabsf(Tinympc14_Sequence()[i] - expected[i]) < 0.001f);
    Tinympc14_Async_Step_Compensated(1u, physical.comp);
    assert(fabsf(Tinympc14_Async_Roll_Increment() - expected_fed[11]) < 1e-6f);
}

// 独立控制接纳差力与实效差力之间的偏差，核查静态偏置、两侧时标及中断后的冲量。
static balance_mpc_diag_t Test_Impulse_Beat(balance_mpc_diag_t previous, float deficit, bool paired)
{
    dm_motor_t *motor[4] = {&dm_f_lft, &dm_b_lft, &dm_f_rgt, &dm_b_rgt};
    const uint32_t tick = test_tick + CTRL_PERIOD_MS;
    for (unsigned int i = 0; i < 4u; i++)
    {
        motor[i]->enabled = motor[i]->online = 1u;
        motor[i]->frame_tick = tick;
        motor[i]->frame_cycle = tick * (SystemCoreClock / 1000u);
    }
    if (!paired) motor[0]->frame_tick -= TINYMPC14_NODE_TICKS + 1u;
    const float spring = Leg_SpringF(TINYMPC14_H_REF, 0.0f);
    body.F_lft = previous.accepted[4] - spring - deficit / 2.0f;
    body.F_rgt = previous.accepted[5] - spring + deficit / 2.0f;
    return Test_Beat(LEG_CTRL_MODE_STANDING);
}

static void Test_Force_Impulse(void)
{
    Test_Reset();
    balance_mpc_diag_t d = Test_Beat(LEG_CTRL_MODE_STANDING);
    d = Test_Impulse_Beat(d, 8.0f, true);
    assert(d.force_valid == 3u && d.pending_diff_impulse == 0.0f);
    d = Test_Impulse_Beat(d, 28.0f, true);
    const float dt = 0.001f * CTRL_PERIOD_MS;
    const float bias_step = 20.0f * dt / (2.0f * TINYMPC14_NN * TINYMPC14_TS + dt);
    const float expected = dt * (10.0f - bias_step) + TINYMPC14_TS * (20.0f - bias_step);
    assert(fabsf(d.pending_diff_impulse - expected) < 1e-5f);
    for (unsigned int i = 0u; i < 350u; i++) d = Test_Impulse_Beat(d, 28.0f, true);
    assert(fabsf(d.pending_diff_impulse) < 1e-4f);
    d = Test_Impulse_Beat(d, -20.0f, false);
    assert(d.force_valid == 2u && d.pending_diff_impulse == 0.0f);
    d = Test_Impulse_Beat(d, -20.0f, true);
    assert(d.pending_diff_impulse == 0.0f);
    ++test_tick; // 两侧反馈都新鲜但相邻估计拍不连续
    d = Test_Impulse_Beat(d, 18.0f, true);
    assert(d.force_valid == 3u && d.pending_diff_impulse == 0.0f);
    dm_f_lft.enabled = dm_b_lft.enabled = dm_f_rgt.enabled = dm_b_rgt.enabled = 0u;
    dm_f_lft.online = dm_b_lft.online = dm_f_rgt.online = dm_b_rgt.online = 0u;
}

// 同一静载起点的首个着地控制拍，返回量化前的总轴力与左右差力。
static float Test_Lowering_Beat(float target, float roll, float rate, float vertical_rate, float *sum, float *diff)
{
    Test_Reset();
    body.h_tgt = target;
    body.ang_roll = roll;
    body.omg_roll = body.omg_roll_kalman = rate;
    body.omg_h_kalman = vertical_rate;
    const balance_mpc_diag_t beat = Test_Beat(LEG_CTRL_MODE_STANDING);

    assert(beat.paired);
    *sum = beat.plan[4] + beat.plan[5];
    *diff = beat.plan[4] - beat.plan[5];
    return beat.lowering_reduction;
}

// 降高冲突时两腿一起卸载。随着侧倾接近保护角，逐步恢复左右差力；快速下降时保留制动。
static void Test_Lowering_Allocation(void)
{
    for (int direction = -1; direction <= 1; direction += 2)
    {
        float level, partial, tilted, braking, raising, diff;
        const float speed = 4.0f * direction; // 001 惯量下触发降高与回正力竞争的角速度。
        const float low = TINYMPC14_H_REF - 0.10f;
        float reference_sum, reference_diff;
        Test_Lowering_Beat(low, 0.0f, 0.0f, 0.0f, &reference_sum, &reference_diff);

        const float full_cut = Test_Lowering_Beat(low, 0.0f, speed, 0.0f, &level, &diff);
        // 001 左右质量不同，卸载应保留静态差力，同时去掉角速度产生的瞬态差力。
        assert(full_cut > 1.0f && fabsf(diff - reference_diff) < 0.05f);
        const float partial_cut = Test_Lowering_Beat(low, 0.10f * direction, -speed, 0.0f, &partial, &diff);
        assert(partial_cut > 0.1f && partial_cut < full_cut && diff * direction > 0.1f);
        assert(Test_Lowering_Beat(low, 0.15f * direction, -speed, 0.0f, &tilted, &diff) == 0.0f);
        assert(tilted > partial + 0.1f && partial > level + 0.1f);
        // 同样的侧倾在向外发展时，当前能交付的回正差力应优先保留。
        assert(Test_Lowering_Beat(low, 0.10f * direction, speed, 0.0f, &tilted, &diff) < 0.05f);
        assert(diff * direction < -1.0f);
        Test_Lowering_Beat(low, 0.0f, speed, -1.0f, &braking, &diff);
        assert(braking > level + 1.0f && diff * direction < -1.0f);
        Test_Lowering_Beat(TINYMPC14_H_REF + 0.05f, 0.0f, speed, 0.0f, &raising, &diff);
        assert(raising > level + 1.0f && diff * direction < -1.0f);
    }
}

// 估计层横滚滤波必须进入真实控制状态，过期估计帧不能被处理时刻重新标新。
static void Test_Input_Regression(void)
{
    unsigned int failures = 0u;

    Test_Reset();
    body.omg_roll = 0.8f;
    body.omg_roll_kalman = 0.05f;
    body.leg_ang_lft = -0.02f;
    body.leg_ang_rgt = 0.03f;
    body.body_accel_y = -0.125f;
    const balance_mpc_diag_t filtered = Test_Beat(LEG_CTRL_MODE_STANDING);

    assert(filtered.leg_angle[0] == body.leg_ang_lft && filtered.leg_angle[1] == body.leg_ang_rgt);
    assert(filtered.accel_y == body.body_accel_y && filtered.pitch == body.ang_pitch);
    printf("roll snapshot: raw=%.3f filtered=%.3f consumed=%.3f\n",
           (double)body.omg_roll, (double)body.omg_roll_kalman, (double)filtered.roll_rate);
    if (fabsf(filtered.roll_rate - body.omg_roll_kalman) > 1e-6f) failures++;

    Test_Reset();
    test_tick = 100u;
    test_sample_delay = TINYMPC14_NN * TINYMPC14_NODE_TICKS;
    const balance_mpc_diag_t delayed = Test_Beat(LEG_CTRL_MODE_STANDING);

    assert(delayed.paired && !delayed.predicted_force.valid && !delayed.terminal_force.valid);
    Balance_Mpc_Solve();
    printf("delayed snapshot: sampled=%u handled=%u commits=%u\n",
           (unsigned int)delayed.sample_tick, (unsigned int)test_tick,
           (unsigned int)tinympc14_counters.commit);
    if (tinympc14_counters.commit != 0u) failures++;

    // 采样时标跨过 uint32_t 回绕时仍按真实解龄拒绝过期输入。
    Test_Reset();
    test_tick = 1u;
    (void)Test_Beat(LEG_CTRL_MODE_STANDING);
    Balance_Mpc_Solve();
    if (tinympc14_counters.commit != 0u) failures++;

    // 尚在时域内的调度延迟允许求解；恢复的新帧不受上一帧超龄影响。
    Test_Reset();
    test_sample_delay = 2u;
    (void)Test_Beat(LEG_CTRL_MODE_STANDING);
    Balance_Mpc_Solve();
    if (tinympc14_counters.commit != 1u) failures++;
    test_sample_delay = 0u;

    // 入场轴力偏离静平衡时，求解与回退都必须遵守同一接纳总力锚点及首拍变化率。
    Test_Reset();
    body.F_lft = body.F_rgt = 40.0f;
    const float preload = Tinympc14_Clip(40.0f + Leg_SpringF(TINYMPC14_H_REF, 0.0f), tinympc14_u_min[4], tinympc14_u_max[4]);
    const balance_mpc_diag_t loaded = Test_Beat(LEG_CTRL_MODE_STANDING);
    Balance_Mpc_Solve();
    const balance_mpc_diag_t follow = Test_Beat(LEG_CTRL_MODE_STANDING);
    for (unsigned int force = 4u; force < TINYMPC14_NU; force++)
    {
        const float anchor = Tinympc14_Clip(loaded.accepted[force], tinympc14_u_min[force], tinympc14_u_max[force]);
        assert(fabsf(loaded.plan[force] - preload) <= TINYMPC14_DF_RATE / 2.0f + 0.001f);
        assert(follow.paired && fabsf(follow.plan[force] - anchor) <= TINYMPC14_DF_RATE / 2.0f + 0.001f);
    }
    body.omg_roll_kalman = NAN;
    const balance_mpc_diag_t invalid = Test_Beat(LEG_CTRL_MODE_STANDING);

    if (invalid.paired || (leg_control_lft.F != 0.0f || leg_control_lft.MotorF != 0.0f) || leg_control_lft.T != 0.0f) failures++;
    fflush(stdout);
    assert(failures == 0u);
}

// 同一物理总力在补偿非零时应保持，不能被中间裸力限速推向相反方向。
static void Test_Compensation_Regression(void)
{
    float x[TINYMPC14_NX] = {0.0f};
    float zero[TINYMPC14_NX] = {0.0f};
    float ss[TINYMPC14_NU], ff[TINYMPC14_NU], prev[TINYMPC14_NU], plan[TINYMPC14_NU];
    tinympc14_comp_t comp = {0};
    tinympc14_output_t output;
    unsigned int failures = 0u;

    Test_Reset();
    x[8] = TINYMPC14_PITCH_REF;
    Tinympc14_Affine_Pub(x, zero, zero, ss, ff);
    for (unsigned int i = 0u; i < TINYMPC14_NU; i++) prev[i] = ss[i];
    comp.Fc[0] = 20.0f;
    comp.Fc[1] = -20.0f;
    prev[4] += comp.Fc[0];
    prev[5] += comp.Fc[1];
    Tinympc14_Async_Applied(prev);
    Tinympc14_Async_Input(x, x, zero, zero, TINYMPC14_H_REF, TINYMPC14_H_REF);
    Tinympc14_Async_Run();
    Tinympc14_Async_Step_Compensated(CTRL_PERIOD_MS, comp.Fc);
    for (unsigned int i = 0u; i < TINYMPC14_NU; i++) plan[i] = tinympc14_u[i];
    Tinympc14_Output_Apply_Period(plan, &comp, prev, CTRL_PERIOD_MS, &output);
    for (unsigned int side = 0u; side < 2u; side++)
    {
        printf("compensation side=%u previous=%.6f expected=%.6f actual=%.6f\n",
               side, (double)prev[4u+side], (double)(ss[4u+side]+comp.Fc[side]), (double)output.u_applied[4u+side]);
        if (fabsf(output.u_applied[4u+side]-prev[4u+side]) > 1e-4f) failures++;
    }
    // 后台规划也必须以同一补偿后的接纳总力约束首步。
    Tinympc14_Async_Solve();
    Tinympc14_Async_Step_Compensated(CTRL_PERIOD_MS, comp.Fc);
    for (unsigned int i = 0u; i < TINYMPC14_NU; i++) plan[i] = tinympc14_u[i];
    Tinympc14_Output_Apply_Period(plan, &comp, prev, CTRL_PERIOD_MS, &output);
    for (unsigned int side = 0u; side < 2u; side++)
    {
        printf("committed compensation side=%u expected=%.6f actual=%.6f commits=%u\n",
               side, (double)prev[4u+side], (double)output.u_applied[4u+side], tinympc14_counters.commit);
        if (fabsf(output.u_applied[4u+side]-prev[4u+side]) > 0.02f) failures++;
    }
    if (tinympc14_counters.commit != 1u) failures++;
    Tinympc14_Async_Invalidate();
    Tinympc14_Async_Run();

    // 观测超龄时保持已接纳总力，不把补偿重复加到保持值上。
    test_tick += TINYMPC14_NN * TINYMPC14_NODE_TICKS;
    Tinympc14_Async_Applied(prev);
    Tinympc14_Async_Step_Compensated(CTRL_PERIOD_MS, comp.Fc);
    for (unsigned int i = 0u; i < TINYMPC14_NU; i++) plan[i] = tinympc14_u[i];
    Tinympc14_Output_Apply_Period(plan, &comp, prev, CTRL_PERIOD_MS, &output);
    for (unsigned int i = 0u; i < TINYMPC14_NU; i++)
        if (fabsf(output.u_applied[i]-prev[i]) > 1e-4f) failures++;

    // 正负补偿、饱和与反向变化：用独立的物理总力限幅表达式检查真实消费链。
    for (int sign = -1; sign <= 1; sign++)
    {
        for (unsigned int edge = 0u; edge < 3u; edge++)
        {
            float feed[TINYMPC14_NU] = {1.0f, -1.0f, 2.0f, -2.0f, 0.0f, 0.0f};
            const float target = edge == 0u ? 0.0f : (edge == 1u ? 80.0f : 170.0f);

            comp.Fc[0] = 60.0f * sign;
            comp.Fc[1] = -comp.Fc[0];
            comp.MgF[0] = 2.0f;
            comp.MgF[1] = -2.0f;
            comp.Fs[0] = 25.0f;
            comp.Fs[1] = 35.0f;
            feed[4] = target - ss[4];
            feed[5] = target - ss[5];
            Tinympc14_Async_Applied(prev);
            Tinympc14_Async_Input(x, x, zero, feed, TINYMPC14_H_REF, TINYMPC14_H_REF);
            Tinympc14_Async_Step_Compensated(CTRL_PERIOD_MS, comp.Fc);
            for (unsigned int i = 0u; i < TINYMPC14_NU; i++) plan[i] = tinympc14_u[i];
            Tinympc14_Output_Apply_Period(plan, &comp, prev, CTRL_PERIOD_MS, &output);
            for (unsigned int side = 0u; side < 2u; side++)
            {
                const unsigned int i = 4u + side;
                const float lo = fmaxf(tinympc14_u_min[i], prev[i]-TINYMPC14_DF_RATE/2.0f);
                const float hi = fminf(tinympc14_u_max[i], prev[i]+TINYMPC14_DF_RATE/2.0f);
                const float expected = Tinympc14_Clip(target+comp.Fc[side], lo, hi);

                if (fabsf(output.u_applied[i]-expected) > 1e-4f) failures++;
                if (fabsf(output.F_motor[side]+comp.Fs[side]-expected) > 1e-4f) failures++;
                if (fabsf(output.T_w[side]+output.u_applied[side]) > 1e-6f) failures++;
                if (fabsf(output.Tp[side]-comp.MgF[side]-output.u_applied[side+2u]) > 1e-6f) failures++;
            }
        }
    }
    printf("compensation, stale hold, signs and total-force bounds: failures=%u\n", failures);
    fflush(stdout);
    assert(failures == 0u);
}

// 恢复旧横滚倾覆预测与高度滤波后，检查实际进入控制器的状态与扰动。
static void Test_Roll_Height_Regression(void)
{
    float expected[TINYMPC14_NX] = {0.0f};

    Test_Reset();
    body.ang_roll = 0.04f;
    Tinympc14_Roll_Disturbance(body.ang_roll, body.h_body, expected);
    const balance_mpc_diag_t tilted = Test_Beat(LEG_CTRL_MODE_STANDING);

    assert(fabsf(tilted.roll_prediction - expected[11]) < 1e-7f);
    assert(TINYMPC14_ROLL_TIP_GAIN == 0.0f || tilted.roll_prediction > 0.0f);
    Test_Reset();
    leg_lft.d_len = leg_rgt.d_len = 0.04f;
    body.omg_h_kalman = -0.01f;
    const balance_mpc_diag_t filtered = Test_Beat(LEG_CTRL_MODE_STANDING);

    assert(filtered.paired && fabsf(filtered.height_rate - body.omg_h_kalman) < 1e-7f);

    // 固定姿态绕世界竖直轴正反向转动。陀螺 p/q/r 均非零，欧拉横滚角与高度恒定。
    for (int sign = -1; sign <= 1; sign += 2)
    {
        Test_Reset();
        const float yaw_rate = sign * 2.0f;
        body.ang_pitch = -0.12f;
        body.ang_roll = 0.16f;
        body.omg_roll = -yaw_rate * sinf(body.ang_pitch);
        body.omg_roll_kalman = 0.0f; // 固定姿态的欧拉角导数已由估计器滤出
        body.omg_pitch = yaw_rate * sinf(body.ang_roll) * cosf(body.ang_pitch);
        body.omg_yaw = yaw_rate * cosf(body.ang_roll) * cosf(body.ang_pitch);
        body.leg_omg_lft = body.leg_omg_rgt = body.omg_pitch;
        body.leg_ang_lft = body.leg_ang_rgt = body.ang_pitch;
        const balance_mpc_diag_t turn = Test_Beat(LEG_CTRL_MODE_STANDING);

        assert(turn.paired && fabsf(turn.roll_rate) < 1e-6f && fabsf(turn.height_rate) < 1e-6f);
    }
    Test_Reset();
    // 直流速度保留，避免用滤波把真实升降速度抹掉。
    body.omg_h_kalman = 0.02f;
    balance_mpc_diag_t steady = {0};

    for (unsigned int i = 0u; i < 500u; i++) steady = Test_Beat(LEG_CTRL_MODE_STANDING);
    assert(fabsf(steady.height_rate - 0.02f) < 0.0001f);
}

// 固定同一传感器位姿，按固件周期求解；跨完整的预测时域平均已提交后的控制请求。
static void Test_Static_Support(float roll, float height_offset, float force[2])
{
    const unsigned int window = TINYMPC14_NN * TINYMPC14_NODE_TICKS / CTRL_PERIOD_MS;
    const float height = TINYMPC14_H_REF + height_offset;
    uint32_t initial_commits = 0u;
    balance_mpc_diag_t beat;

    Test_Reset();
    body.ang_roll = roll;
    body.h_body = height;
    body.leg_len_lft = leg_lft.len = height / cosf(roll) + TINYMPC14_HALF_TRACK * tanf(roll);
    body.leg_len_rgt = leg_rgt.len = height / cosf(roll) - TINYMPC14_HALF_TRACK * tanf(roll);
    force[0] = force[1] = 0.0f;
    for (unsigned int i = 0u; i < 2u * window; i++)
    {
        beat = Test_Beat(LEG_CTRL_MODE_STANDING);
        if (i * CTRL_PERIOD_MS % BALANCE_MPC_PERIOD_MS == 0u) Balance_Mpc_Solve();
        if (i == window) initial_commits = beat.commits;
        if (i < window) continue;
        assert(beat.paired && beat.commits > 0u);
        for (unsigned int side = 0u; side < 2u; side++)
        {
            const float request = beat.plan[4u + side];
            assert(request > tinympc14_u_min[4u + side] && request < tinympc14_u_max[4u + side]);
            force[side] += request / (float)window;
        }
    }
    assert(beat.commits > initial_commits);
}

// 双支撑：围绕 CAD 静载，横滚产生恢复差力，升降产生同向合力。
static void Test_Restoring_Support(void)
{
    float neutral[2], roll[2][2], height[2];
    const float symmetry_tol = 0.05f; // N，稳态求解与读数的对称性容差
    const float reference[TINYMPC14_NX] = {[8] = TINYMPC14_PITCH_REF};
    const float zero[TINYMPC14_NX] = {0.0f};
    float hold[TINYMPC14_NU], feedforward[TINYMPC14_NU];

    Test_Static_Support(0.0f, 0.0f, neutral);
    Tinympc14_Affine_Pub(reference, zero, zero, hold, feedforward);
    assert(fabsf(neutral[0] - hold[4]) < symmetry_tol && fabsf(neutral[1] - hold[5]) < symmetry_tol);
#ifdef TINYMPC14_M_LOAD
    // 生成模型的静力来自 dV/dH；每侧总轴力含弹簧，电机播种已在 Test_Reset 中扣除它。
    assert(fabsf(neutral[0] + neutral[1] - TINYMPC14_M_LOAD * TINYMPC14_G) < symmetry_tol);
#endif
    for (unsigned int direction = 0u; direction < 2u; direction++)
    {
        const float sign = direction == 0u ? -1.0f : 1.0f;
        Test_Static_Support(sign * 0.01f, 0.0f, roll[direction]);
        assert(sign * (roll[direction][0] - roll[direction][1] - neutral[0] + neutral[1]) < -symmetry_tol);
        assert(fabsf(roll[direction][0] + roll[direction][1] - neutral[0] - neutral[1]) < symmetry_tol);
        Test_Static_Support(0.0f, sign * 0.002f, height);
        assert(sign * (height[0] + height[1] - neutral[0] - neutral[1]) < -symmetry_tol);
        assert(fabsf(height[0] - height[1] - neutral[0] + neutral[1]) < symmetry_tol);
    }
    assert(fabsf(roll[0][0] - neutral[0] - roll[1][1] + neutral[1]) < symmetry_tol &&
           fabsf(roll[0][1] - neutral[1] - roll[1][0] + neutral[0]) < symmetry_tol);
    printf("PASS: static axial support %.3f / %.3f N, roll and common height restoring forces\n", (double)neutral[0], (double)neutral[1]);
}

// 只与控制拍实际接纳码对照，包括电机限幅、协议量化和被动弹簧。
static void Test_Accepted_Force(const balance_mpc_diag_t *beat, balance_mpc_force_t record)
{
    assert(beat->paired && record.valid && record.tick == beat->sample_tick);
    assert(record.sum == beat->accepted[4] + beat->accepted[5]);
    assert(record.diff == beat->accepted[4] - beat->accepted[5]);
}

// 模型优化裸轴力。转弯外扰与执行端已经补偿的力合计后才是预测器需要抵消的残量。
static void Test_Turn_Disturbance(void)
{
    Test_Reset();
    body.ang_pitch = 0.0f;
    body.vel = body.vel_kalman = 0.8f;
    body.omg_yaw = body.omg_yaw_filt = 1.0f;
    const balance_mpc_diag_t beat = Test_Beat(LEG_CTRL_MODE_MOVING);
    tinympc14_comp_t comp;
    Tinympc14_Output_Calc(leg_lft.len, leg_rgt.len, 0.0f, 0.0f, body.vel, body.omg_yaw, &comp);
    const tinympc14_model_t *model = Tinympc14_Model();
    float left = leg_control_lft.Turn_DistF, right = leg_control_rgt.Turn_DistF;
    assert(beat.paired && left > 1.0f && right < -1.0f);
#if BALANCE_USE_ROLL_COMP
    assert(comp.Fc[0] < -1.0f && comp.Fc[1] > 1.0f);
    left += comp.Fc[0];
    right += comp.Fc[1];
#endif
    const float predicted = model->Bd[11 * TINYMPC14_NU + 4] * left + model->Bd[11 * TINYMPC14_NU + 5] * right;
    assert(fabsf(beat.roll_prediction - predicted) < 1.0e-6f);

    // 转弯外扰直接使用本拍载荷，转向归零时同拍归零。
    float raw, outside, inside;
    Leg_RollComp(body.h_body, body.omg_yaw_filt, body.vel, &inside, &outside, &raw);
    assert(fabsf(leg_control_lft.Turn_DistF - raw) < 1.0e-4f);
    body.omg_yaw = body.omg_yaw_filt = 0.0f;
    Test_Beat(LEG_CTRL_MODE_MOVING);
    assert(fabsf(leg_control_lft.Turn_DistF) < 1.0e-6f);
#if BALANCE_USE_ROLL_COMP
    assert(fabsf(leg_control_lft.Roll_CompF) < 1.0e-6f && fabsf(leg_control_rgt.Roll_CompF) < 1.0e-6f);
#endif
    Test_Beat(LEG_CTRL_MODE_AIR);
    body.omg_yaw = body.omg_yaw_filt = -1.0f;
    Test_Beat(LEG_CTRL_MODE_MOVING);
    Leg_RollComp(body.h_body, body.omg_yaw_filt, body.vel, &inside, &outside, &raw);
    assert(fabsf(leg_control_lft.Turn_DistF - raw) < 1.0e-4f);
    test_bus_epoch++;
    body.omg_yaw = body.omg_yaw_filt = 0.0f;
    Test_Beat(LEG_CTRL_MODE_MOVING);
    assert(fabsf(leg_control_lft.Turn_DistF) < 1.0e-6f);
}

static void Test_Output_Isolation(void)
{
    Test_Reset();
    const balance_mpc_diag_t initial = Test_Beat(LEG_CTRL_MODE_MOVING);
    assert(initial.paired);
    const control_vector_t *legs[2] = {&leg_control_lft, &leg_control_rgt};
    for (unsigned int side = 0; side < 2; side++)
    {
        assert(legs[side]->F == tinympc14_u[4u + side]);
        assert(legs[side]->MpcT == tinympc14_u[side]);
        assert(legs[side]->MpcTp == tinympc14_u[2u + side]);
        assert(fabsf(legs[side]->MotorF - (initial.plan[4u + side] + legs[side]->SpringF)) < 1e-4f);
    }

    const float plan[TINYMPC14_NU] = {0, 0, 0, 0, 100, 100};
    const float *prev = plan;
    tinympc14_comp_t comp = {0};
    tinympc14_output_t output;
    Tinympc14_Output_Apply_Period(plan, &comp, prev, CTRL_PERIOD_MS, &output);
    // 固定轴力计划，改变弹簧及轮/髋请求，电机轴力只随弹簧变化。
    float mixed[TINYMPC14_NU];
    for (unsigned int i = 0u; i < TINYMPC14_NU; i++) mixed[i] = plan[i];
    tinympc14_comp_t changed = comp;
    for (unsigned int side = 0u; side < 2u; side++)
    {
        mixed[side] += 1.0f;
        mixed[side + 2u] -= 2.0f;
        changed.Fs[side] += 30.0f;
    }
    tinympc14_output_t changed_output;
    Tinympc14_Output_Apply_Period(mixed, &changed, prev, CTRL_PERIOD_MS, &changed_output);
    for (unsigned int side = 0u; side < 2u; side++)
    {
        assert(changed_output.u_applied[4u + side] == output.u_applied[4u + side]);
        assert(fabsf(changed_output.F_motor[side] - (output.F_motor[side] - 30.0f)) < 1e-4f);
    }
}

int main(void)
{
    Test_Output_Isolation();
    Test_Suspended_Gravity();
    Test_Realized_Force();
    Test_Force_Snapshot();
    Test_Committed_Roll_Rate();
    Test_Force_Impulse();
    Test_Turn_Disturbance();
    Test_Lowering_Allocation();
    Test_Restoring_Support();
    Test_Roll_Height_Regression();
    Test_Compensation_Regression();
    Test_Input_Regression();
    Test_Reset();

    const balance_mpc_diag_t initial = Test_Beat(LEG_CTRL_MODE_MOVING);

    assert(initial.paired && initial.commits == 0u && initial.fallbacks == 1u);
    assert(!initial.predicted_force.valid);
    Test_Accepted_Force(&initial, initial.terminal_force);
    Balance_Mpc_Solve();
    const balance_mpc_diag_t solved = Test_Beat(LEG_CTRL_MODE_STANDING);

    assert(solved.paired && solved.commits > 0u && solved.epoch == initial.epoch);
    assert(solved.publish_tick == initial.sample_tick);
    assert(solved.fallbacks == initial.fallbacks);
    Test_Accepted_Force(&solved, solved.predicted_force);
    assert(solved.terminal_force.tick == initial.sample_tick);

    // 一侧关节入队失败时，下一拍限速锚点来自真实保持码。
    test_fail_mask = 1u;
    body.h_tgt += 0.02f;
    const balance_mpc_diag_t partial = Test_Beat(LEG_CTRL_MODE_STANDING);

    assert(partial.paired && partial.accepted_mask == (EXECUTION_CHANNEL_MASK & ~1u));
    assert(partial.predicted_force.tick == solved.sample_tick);
    assert(partial.predicted_force.sum == solved.predicted_force.sum && partial.predicted_force.diff == solved.predicted_force.diff);
    assert(partial.terminal_force.tick == initial.sample_tick);
    test_fail_mask = 0u;
    const balance_mpc_diag_t next = Test_Beat(LEG_CTRL_MODE_STANDING);

    Test_Accepted_Force(&next, next.predicted_force);
    for (unsigned int side = 0u; side < 2u; side++)
    {
        const unsigned int force = 4u + side;
        const float anchor = Tinympc14_Clip(partial.accepted[force], tinympc14_u_min[force], tinympc14_u_max[force]);

        assert(fabsf(next.plan[force] - anchor) <= TINYMPC14_DF_RATE / 2.0f + 0.001f);
    }

    // 后台停更时序列过期，控制继续使用同一模型的六路终端反馈。
    balance_mpc_diag_t stale = next;

    for (unsigned int i = 0u; i <= TINYMPC14_NN * TINYMPC14_NODE_TICKS; i++)
        stale = Test_Beat(LEG_CTRL_MODE_STANDING);
    assert(stale.paired && stale.stale > solved.stale && stale.fallbacks > solved.fallbacks);
    Test_Accepted_Force(&stale, stale.terminal_force);
    assert(stale.predicted_force.tick != stale.sample_tick);
    test_fail_mask = 4u;
    const balance_mpc_diag_t partial_terminal = Test_Beat(LEG_CTRL_MODE_STANDING);

    assert(partial_terminal.paired && partial_terminal.terminal_force.tick == stale.sample_tick);
    assert(partial_terminal.terminal_force.sum == stale.terminal_force.sum && partial_terminal.terminal_force.diff == stale.terminal_force.diff);
    test_fail_mask = 0u;
    Balance_Mpc_Solve();

    // 总线切代与出地立即作废预测，AIR 期间不能提交新接地序列。
    test_bus_epoch++;
    balance_mpc_diag_t cancelled;

    Balance_Mpc_Diagnostic(&cancelled);
    assert(!cancelled.predicted_force.valid && !cancelled.terminal_force.valid);
    const balance_mpc_diag_t restarted = Test_Beat(LEG_CTRL_MODE_STANDING);

    assert(restarted.paired && restarted.epoch != stale.epoch && restarted.fallbacks == partial_terminal.fallbacks + 1u);
    assert(!restarted.predicted_force.valid);
    Test_Accepted_Force(&restarted, restarted.terminal_force);
    const balance_mpc_diag_t air = Test_Beat(LEG_CTRL_MODE_AIR);

    assert(!air.paired && leg_control_lft.T == 0.0f && leg_control_rgt.T == 0.0f);
    assert(!air.terminal_force.valid && !air.predicted_force.valid);
    Balance_Mpc_Solve();
    balance_mpc_diag_t stopped;

    Balance_Mpc_Diagnostic(&stopped);
    assert(stopped.commits == air.commits);

    // 无效状态撤销出力和代次，恢复后由新量测重新启动。
    body.omg_roll = NAN;
    const balance_mpc_diag_t fault = Test_Beat(LEG_CTRL_MODE_STANDING);

    assert(!fault.paired && leg_control_lft.T == 0.0f && leg_control_lft.F == 0.0f && leg_control_lft.MotorF == 0.0f);
    assert(!fault.terminal_force.valid && !fault.predicted_force.valid);
    body.omg_roll = 0.0f;
    assert(Test_Beat(LEG_CTRL_MODE_STANDING).paired);
    puts("PASS: TinyMPC14 startup, accepted force, stale recovery, bus epoch, AIR and invalid-state restart");
    return 0;
}
