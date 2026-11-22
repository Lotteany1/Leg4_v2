/** @file height_roll_closed_loop_test.c  @brief 实际固件在双轮等高接地、执行滞后下的升降与横滚回归 */
#include "tinympc14_ground_fixture.h"
#include "Leg_Pos.h"
#include "kalman.h"

// 在当前装配分支上反解两髋角，供实际关节映射、CAN 量化与力反算使用。
static void Set_Leg_Pose(leg_state_t *leg, float length, float angle)
{
    float lo = 0.4f, hi = 1.7f, pose[2], lo_pose[2];
    const float two_pi = 6.283185307f;

    Leg_Pos(lo, two_pi - lo, lo_pose);
    Leg_Pos(hi, two_pi - hi, pose);
    const bool increasing = pose[0] > lo_pose[0];
    for (int k = 0; k < 23; ++k)
    {
        const float mid = 0.5f * (lo + hi);
        Leg_Pos(mid, two_pi - mid, pose);
        if ((pose[0] < length) == increasing) lo = mid;
        else hi = mid;
    }
    const float joint = 0.5f * (lo + hi);
    Leg_Pos(joint, two_pi - joint, pose);
    leg->pitch_1 = joint + angle - pose[1];
    leg->pitch_4 = two_pi - joint + angle - pose[1];
    leg->len = length;
}

// 只积分 H 与 roll。俯仰和矢状腿角固定，实车滤波、消费、关节编码沿用生产入口。
// delay 与 tau 是独立选择的测试条件。17 cm 对应髋中点相对轮轴的高度。
static void Run_Closed_Loop(double inertia, unsigned int delay, double tau, int solve_period, double pulse)
{
    const double dt = 0.001, half_track = 0.212755, load = 144.53, mass = 14.2903;
    double height = 0.30, speed = 0.0, roll = 0.0, rate = 0.0;
    double motor[2], queue[8][2], late_height = 0.0, peak = 0.0, late_rate = 0.0;
    double force_last[2] = {load / 2.0, load / 2.0};
    unsigned int paired_force_samples = 0u;
    dm_motor_t *const joint[4] = {&dm_f_lft, &dm_b_lft, &dm_f_rgt, &dm_b_rgt};
    kalman_accel_t gyro;

    Test_Reset();
    assert(delay >= 1u && delay <= 8u);
    Kalman_Accel_Init(&gyro, 0.0f, BODY_ROLL_KF_R, BODY_ROLL_KF_Q_W, BODY_ROLL_KF_Q_A, BODY_ROLL_KF_R);
    gyro.gate = 0.0f;
    for (unsigned int side = 0; side < 2; ++side)
    {
        motor[side] = load / 2.0 - Leg_SpringF((float)height, 0.0f);
        for (unsigned int k = 0; k < delay; ++k) queue[k][side] = motor[side];
    }

    for (int k = 0; k < 2800; ++k)
    {
        if (k == 600) rate += copysign(1.0, pulse); // 降高途中施加角动量冲击，角速度变化 1 rad/s
        const double s = sin(roll), c = cos(roll);
        const double lengths[2] = {(height + half_track * s) / c, (height - half_track * s) / c};
        const double spring[2] = {Leg_SpringF((float)lengths[0], 0.0f), Leg_SpringF((float)lengths[1], 0.0f)};
        Set_Leg_Pose(&leg_lft, (float)lengths[0], 1.5707963f - TINYMPC14_PITCH_REF);
        Set_Leg_Pose(&leg_rgt, (float)lengths[1], 1.5707963f + TINYMPC14_PITCH_REF);
        body.leg_len_lft = leg_lft.len;
        body.leg_len_rgt = leg_rgt.len;
        leg_lft.d_len = (float)(speed / c + (height * s + half_track) * rate / (c * c));
        leg_rgt.d_len = (float)(speed / c + (height * s - half_track) * rate / (c * c));
        body.F_lft = (float)motor[0];
        body.F_rgt = (float)motor[1];
        body.h_body = (float)height;
        body.ang_roll = (float)roll;
        body.omg_roll = (float)rate;
        // 仿真 200 Hz 陀螺帧与 1 kHz 控制时钟，速度反馈在帧间保持。
        if (k % 5 == 0)
        {
            Kalman_Accel_Predict(&gyro, 0.0f, 0.005f);
            Kalman_Accel_Update(&gyro, (float)rate);
        }
        body.omg_roll_filt = gyro.vel;
        body.h_tgt = (float)fmax(0.17, 0.30 - 0.25 * dt * fmax(0, k - 250));
        for (unsigned int j = 0; j < 4; ++j)
        {
            joint[j]->online = 1u;
            joint[j]->enabled = 1u;
            joint[j]->frame_tick = test_tick + 1u;
            joint[j]->frame_cycle = (test_tick + 1u) * (SystemCoreClock / 1000u);
        }

        const balance_mpc_diag_t beat = Test_Beat(LEG_CTRL_MODE_STANDING);
        assert(beat.paired);
        if (beat.force_valid == 3u) ++paired_force_samples;
        if (solve_period > 0 && k % solve_period == 0) Balance_Mpc_Solve();
        const unsigned int pos = (unsigned int)k % delay;
        double established[2];
        for (unsigned int side = 0; side < 2; ++side)
        {
            const double sent = beat.accepted[4u + side];
            // 约束施加于未量化的总力，关节码反算会产生小于 0.2 N 的舍入差。
            assert(sent >= 19.8 && sent <= 150.2 && fabs(sent - force_last[side]) < 3.2);
            force_last[side] = sent;
            motor[side] += (1.0 - exp(-dt / tau)) * (queue[pos][side] - motor[side]);
            queue[pos][side] = sent - spring[side];
            established[side] = motor[side] + spring[side];
        }
        const double vertical = (established[0] + established[1]) / c;
        const double torque = half_track * (established[0] - established[1]) / (c * c)
                            + (vertical - load) * height * tan(roll) + 44.33 * s;
        const double disturbance = k >= 600 && k < 670 ? pulse : 0.0;
        speed += dt * (vertical - load - 8.0 * speed) / mass;
        rate += dt * (torque + disturbance) / inertia;
        height += dt * speed;
        roll += dt * rate;
        assert(isfinite(height) && isfinite(roll) && height >= 0.12 && height < 0.42 && fabs(roll) < 0.60);
        peak = fmax(peak, fabs(roll));
        if (k >= 2300)
        {
            late_height += height;
            late_rate += rate * rate;
        }
    }
    late_height /= 500;
    late_rate = sqrt(late_rate / 500);
    printf("plant I=%.3f delay=%u ms tau=%.3f s solve=%d ms pulse=%+.1f Nm: H=%.5f m, peak=%.3f deg, late roll speed RMS=%.4f rad/s, fresh pairs=%u\n",
           inertia, delay, tau, solve_period, pulse, late_height, peak * 57.2957795, late_rate, paired_force_samples);
    fflush(stdout);
    assert(paired_force_samples >= 2790u);
    assert(fabs(late_height - 0.17) < 0.01 && peak < 0.12 && late_rate < 0.15);
}

int main(void)
{
    Run_Closed_Loop(0.578, 2u, 0.008, 4, 2.0);
    Run_Closed_Loop(0.578, 4u, 0.008, 0, -2.0);
    Run_Closed_Loop(1.371, 2u, 0.012, 16, 2.0);
    Run_Closed_Loop(0.232, 2u, 0.008, 4, -2.0);
    // 该组合在 10 Hz 约落后 44°，覆盖实车低目标录波的基波相位。
    Run_Closed_Loop(0.232, 4u, 0.009, 4, 2.0);
    Run_Closed_Loop(0.232, 6u, 0.006, 16, -2.0);
    return 0;
}
