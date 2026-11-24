/* Host regression: execute the real recovery state machine with measured-state fixtures. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../Code/App/app_self_rescue.c"

body_state_t body;
leg_state_t leg_lft, leg_rgt;
imu_world_t imu_world;

void Balance_Mpc_Stop(void)
{
}
float Imu_To_World_Leg_Ang(float ang)
{
    return Wrap_Pi(ang + imu_world.pitch);
}

static void Reset(float dir, float pitch, float angle)
{
    memset(&body, 0, sizeof(body));
    memset(&leg_lft, 0, sizeof(leg_lft));
    memset(&leg_rgt, 0, sizeof(leg_rgt));
    memset(&imu_world, 0, sizeof(imu_world));
    rescue_dir      = dir;
    imu_world.pitch = pitch;
    leg_lft.ang = leg_rgt.ang = angle;
    leg_lft.len = leg_rgt.len = Leg_Len_Min;
    body.leg_len_lft = body.leg_len_rgt = Leg_Len_Min;
    App_Self_Rescue_Enter();
}

static void Tick(unsigned ms)
{
    for (unsigned i = 0; i < ms / CTRL_PERIOD_MS; i++) App_Self_Rescue_Tick();
}

static void Test_Retract_Ramp(void)
{
    for (int dir = -1; dir <= 1; dir += 2)
    {
        Reset((float)dir, 0.0f, 1.0f);
        leg_lft.len = leg_rgt.len = body.leg_len_lft = body.leg_len_rgt = Leg_Len_Max;
        assert(App_Self_Rescue_Trq_Scale(App_Self_Rescue_Ctrl_Mode()) == 1.0f);
        Tick(SR_COMMON_PREPARE_MS + SR_TURNOVER_HOLD_MS);
        if (dir < 0) Tick(SR_REVERSE_READY_HOLD_MS + CTRL_PERIOD_MS);
        assert(phase == (dir < 0 ? SR_PHASE_REVERSE_RETRACT : SR_PHASE_FORWARD_RETRACT));
        for (unsigned ms = 0u; ms <= 301u; ms += CTRL_PERIOD_MS)
        {
            float expected = ms < 300u ? (float)ms / 300.0f : 1.0f;
            assert(fabsf(App_Self_Rescue_Trq_Scale(App_Self_Rescue_Ctrl_Mode()) - expected) < 0.00001f);
            assert(App_Self_Rescue_Trq_Scale(LEG_CTRL_MODE_TEST) == 1.0f);
            Tick(CTRL_PERIOD_MS);
        }
    }

    /* Retry retraction starts at zero. Short legs advance immediately, and the
       first forward sweep uses the remaining ramp without a second reset. */
    Reset(-1.0f, 0.0f, 2.0f);
    Tick(SR_ATTEMPT_TIMEOUT_MS);
    assert(phase == SR_PHASE_OBSTACLE_RETRACT && App_Self_Rescue_Trq_Scale(App_Self_Rescue_Ctrl_Mode()) == 0.0f);
    Tick(CTRL_PERIOD_MS);
    assert(phase == SR_PHASE_FORWARD_RETRACT);
    Tick(CTRL_PERIOD_MS);
    assert(phase == SR_PHASE_FORWARD_SWEEP);
    Tick(150u - 2u * CTRL_PERIOD_MS);
    assert(App_Self_Rescue_Trq_Scale(App_Self_Rescue_Ctrl_Mode()) == 0.5f);
    App_Self_Rescue_Enter();
    assert(App_Self_Rescue_Trq_Scale(App_Self_Rescue_Ctrl_Mode()) == 1.0f);
}

int main(void)
{
    Test_Retract_Ramp();
    /* Rate is direct and bounded in either direction, with a continuous linear slowdown.
       Continuous multi-turn angles preserve the physical shortest direction to zero. */
    float slow = SR_REVERSE_STAND_SLOWDOWN_ANG_RAD;
    assert(Self_Rescue_Reverse_Stand_Speed(0.0f) == 0.0f);
    assert(fabsf(Self_Rescue_Reverse_Stand_Speed(2.0f * slow) + SR_REVERSE_STAND_RATE_RADPS) < 0.0001f);
    assert(fabsf(Self_Rescue_Reverse_Stand_Speed(-2.0f * slow) - SR_REVERSE_STAND_RATE_RADPS) < 0.0001f);
    assert(fabsf(Self_Rescue_Reverse_Stand_Speed(0.5f * slow) + 0.5f * SR_REVERSE_STAND_RATE_RADPS) < 0.0001f);
    assert(fabsf(Self_Rescue_Reverse_Stand_Speed(TWO_PI + 0.5f * slow) - Self_Rescue_Reverse_Stand_Speed(0.5f * slow)) < 0.001f);
    assert(fabsf(Self_Rescue_Reverse_Stand_Speed(slow + 0.00001f) - Self_Rescue_Reverse_Stand_Speed(slow - 0.00001f)) < 0.001f);
    Reset(-1.0f, 0.0f, 0.05f);
    phase = SR_PHASE_REVERSE_STAND;
    Tick(1u);
    float stand_rate = body.leg_omg_tgt_lft;
    phase            = SR_PHASE_VERIFY;
    Tick(1u);
    assert(body.leg_omg_tgt_lft == stand_rate);
    assert(fabsf(body.leg_omg_tgt_lft) <= SR_REVERSE_STAND_RATE_RADPS);

    /* A frozen mechanism gets one full budget per direction, including preparation,
       turnover, retraction and sweep. A debugger edit only takes effect on re-entry. */
    for (int first = -1; first <= 1; first += 2)
    {
        float dir = (float)first;
        Reset(dir, 0.0f, 2.0f);
        rescue_dir = -dir;
        for (unsigned attempt = 0u; attempt <= 5u; attempt++)
        {
            Tick(SR_ATTEMPT_TIMEOUT_MS - CTRL_PERIOD_MS);
            assert(active_dir == dir && retry_count == attempt);
            assert(phase == (dir > 0.0f ? SR_PHASE_FORWARD_SWEEP : SR_PHASE_REVERSE_SWING));
            assert(App_Self_Rescue_Diagnostic().attempt_ms == SR_ATTEMPT_TIMEOUT_MS - CTRL_PERIOD_MS);
            Tick(CTRL_PERIOD_MS);
            if (attempt < 5u)
            {
                dir = -dir;
                assert(active_dir == dir && retry_count == attempt + 1u && attempt_ms == 0u);
                assert(phase == SR_PHASE_OBSTACLE_RETRACT);
                assert(body.leg_omg_tgt_lft == 0.0f && body.leg_omg_tgt_rgt == 0.0f);
            }
        }
        assert(phase == SR_PHASE_FAILED && retry_count == 5u);
        assert(!App_Self_Rescue_Finished() && App_Self_Rescue_Ctrl_Mode() == LEG_CTRL_MODE_NUM);
        Tick(SR_ATTEMPT_TIMEOUT_MS);
        assert(phase == SR_PHASE_FAILED && attempt_ms == SR_ATTEMPT_TIMEOUT_MS);
        App_Self_Rescue_Enter();
        assert(phase == SR_PHASE_PREPARE && retry_count == 0u && attempt_ms == 0u);
        assert(active_dir == rescue_dir);
    }

    /* A persistent topple reverses the actual turnover sweep after the timeout. */
    Reset(-1.0f, 2.5f, 0.0f);
    Tick(SR_ATTEMPT_TIMEOUT_MS - CTRL_PERIOD_MS);
    assert(phase == SR_PHASE_TURNOVER && body.leg_ang_tgt_lft > 0.0f);
    assert(App_Self_Rescue_Ctrl_Mode() == LEG_CTRL_MODE_SELF_RESCUE_TURNOVER);
    Tick(CTRL_PERIOD_MS + SR_TURNOVER_EXTEND_MS + 1u);
    assert(phase == SR_PHASE_TURNOVER && active_dir > 0.0f && body.leg_ang_tgt_lft < 0.0f);
    Reset(1.0f, -2.5f, 0.0f);
    Tick(SR_COMMON_PREPARE_MS + SR_TURNOVER_EXTEND_MS + 1u);
    assert(turnover_dir < 0.0f && body.leg_ang_tgt_lft < 0.0f);

    /* Actual retraction, not elapsed time, gates the forward sweep. */
    Reset(1.0f, 0.0f, 2.0f);
    leg_lft.len = leg_rgt.len = Leg_Len_Max;
    Tick(SR_COMMON_PREPARE_MS + SR_TURNOVER_HOLD_MS + 200u);
    assert(phase == SR_PHASE_FORWARD_RETRACT);
    leg_lft.len = leg_rgt.len = Leg_Len_Min;
    Tick(1u);
    assert(phase == SR_PHASE_FORWARD_SWEEP);

    /* A blocked retraction in either direction uses the same deadline. */
    Reset(1.0f, 0.0f, 2.0f);
    leg_lft.len = leg_rgt.len = Leg_Len_Max;
    Tick(SR_ATTEMPT_TIMEOUT_MS - CTRL_PERIOD_MS);
    assert(phase == SR_PHASE_FORWARD_RETRACT && active_dir > 0.0f);
    Tick(CTRL_PERIOD_MS);
    Tick(SR_ATTEMPT_TIMEOUT_MS - CTRL_PERIOD_MS);
    assert(phase == SR_PHASE_OBSTACLE_RETRACT && active_dir < 0.0f && retry_count == 1u);
    Tick(CTRL_PERIOD_MS);
    assert(phase == SR_PHASE_OBSTACLE_RETRACT && active_dir > 0.0f && retry_count == 2u);
    leg_lft.len = leg_rgt.len = Leg_Len_Min;
    Tick(2u);
    assert(phase == SR_PHASE_FORWARD_SWEEP);
    Tick(100u);
    assert(fabsf(body.leg_ang_tgt_lft - leg_lft.ang) <= SR_OBSTACLE_SWEEP_LEAD_MAX_RAD + 0.001f);

    /* Restore the previous 1.30-rad ready window: the new recording at 1.157/1.153
       must advance to retraction rather than remain on the narrowed 1.15 boundary. */
    Reset(-1.0f, 0.151507f, 1.157317f - 0.151507f);
    leg_rgt.ang = 1.153335f - 0.151507f;
    leg_lft.len = leg_rgt.len = Leg_Len_Max;
    phase                     = SR_PHASE_REVERSE_SWING;
    Tick(SR_REVERSE_READY_HOLD_MS + 1u);
    assert(phase == SR_PHASE_REVERSE_RETRACT);

    /* Recorded 1.295 -> 1.301 rad crossing after retraction starts is not a failure. */
    Reset(-1.0f, -0.101505f, 1.257603f + 0.101505f);
    leg_rgt.ang = 1.300985f + 0.101505f;
    leg_lft.len = body.leg_len_lft = 0.332f;
    leg_rgt.len = body.leg_len_rgt = 0.327f;
    phase                          = SR_PHASE_REVERSE_RETRACT;
    Tick(1u);
    assert(phase == SR_PHASE_REVERSE_RETRACT && active_dir < 0.0f && retry_count == 0u);
    leg_lft.len = body.leg_len_lft = Leg_Len_Min;
    leg_rgt.len = body.leg_len_rgt = Leg_Len_Min;
    Tick(1u);
    assert(phase == SR_PHASE_REVERSE_STAND);
    body.ang_roll = SR_REVERSE_READY_ROLL_MAX_RAD + 0.01f;
    Tick(1u);
    assert(phase == SR_PHASE_OBSTACLE_RETRACT && active_dir < 0.0f && retry_count == 0u);

    /* Forward geometric entry uses only one common hold, not two consecutive waits. */
    Reset(1.0f, 0.0f, 0.0f);
    Self_Rescue_Forward_Sweep_Enter();
    Tick(1u);
    assert(phase == SR_PHASE_VERIFY && exit_hold == 0u);
    Tick(SR_COMMON_HOLD_MS - 1u);
    assert(phase == SR_PHASE_VERIFY && !finished);
    Tick(1u);
    assert(phase == SR_PHASE_HANDOFF && !finished);
    Tick(1u);
    assert(finished);

    /* VERIFY still requires a continuous window: a rate excursion resets the hold,
       and an out-of-window pose cannot hand off even after the configured duration. */
    Reset(1.0f, 0.0f, 0.0f);
    phase = SR_PHASE_VERIFY;
    Tick(SR_COMMON_HOLD_MS - 1u);
    assert(phase == SR_PHASE_VERIFY && !finished);
    body.omg_pitch = SR_COMMON_OMG_TH + 0.1f;
    Tick(1u);
    assert(exit_hold == 0u && phase == SR_PHASE_VERIFY);
    body.omg_pitch = 0.0f;
    Tick(SR_COMMON_HOLD_MS - 1u);
    assert(phase == SR_PHASE_VERIFY && !finished);
    Tick(1u);
    assert(phase == SR_PHASE_HANDOFF);
    Reset(-1.0f, SR_COMMON_EXIT_ANG + 0.05f, 0.0f);
    phase = SR_PHASE_VERIFY;
    Tick(SR_COMMON_HOLD_MS + 5u);
    assert(phase == SR_PHASE_VERIFY && !finished);

    /* Replay the measured stand dropout: pitch 0.471, body leg angles -0.255/-0.185.
       Loss of posture retracts immediately, with the same attempt deadline. */
    Reset(-1.0f, 0.471227f, -0.254794f);
    leg_rgt.ang          = -0.184528f;
    phase                = SR_PHASE_REVERSE_STAND;
    body.leg_omg_tgt_lft = 11.57f;
    body.leg_omg_tgt_rgt = 6.35f;
    Tick(1u);
    assert(phase == SR_PHASE_OBSTACLE_RETRACT && active_dir < 0.0f && retry_count == 0u);
    assert(body.leg_omg_tgt_lft == 0.0f && body.leg_omg_tgt_rgt == 0.0f);
    Tick(2u);
    assert(phase == SR_PHASE_REVERSE_SWING && attempt_ms == 3u);

    /* A stable final tick wins at the deadline, even on the fifth retry. */
    Reset(1.0f, 0.0f, 0.0f);
    body.omg_pitch = SR_COMMON_OMG_TH + 0.1f;
    Tick(5u * SR_ATTEMPT_TIMEOUT_MS + SR_ATTEMPT_TIMEOUT_MS - SR_COMMON_HOLD_MS);
    assert(phase == SR_PHASE_VERIFY && retry_count == 5u);
    body.omg_pitch = 0.0f;
    Tick(SR_COMMON_HOLD_MS + CTRL_PERIOD_MS);
    assert(App_Self_Rescue_Finished());

    /* High rate blocks success; once stationary the already upright robot hands off. */
    Reset(1.0f, 0.0f, 0.0f);
    body.omg_pitch = 2.0f;
    Tick(SR_COMMON_PREPARE_MS + SR_TURNOVER_HOLD_MS + SR_COMMON_HOLD_MS + 5u);
    assert(!App_Self_Rescue_Finished());
    body.omg_pitch = 0.0f;
    Tick(SR_COMMON_HOLD_MS + 2u);
    assert(App_Self_Rescue_Finished());
    /* The last reverse VERIFY tick must not insert the forward position loop. */
    Reset(-1.0f, 0.0f, 0.0f);
    phase = SR_PHASE_VERIFY;
    Tick(SR_COMMON_HOLD_MS);
    assert(phase == SR_PHASE_HANDOFF);
    assert(App_Self_Rescue_Ctrl_Mode() == LEG_CTRL_MODE_SELF_RESCUE_STAND);
    Tick(1u);
    assert(App_Self_Rescue_Finished());
    /* Complete forward and reverse recovery paths through VERIFY and HANDOFF.
       A multi-turn encoder representation must fold once without changing physical posture. */
    for (unsigned path = 0u; path < 2u; path++)
    {
        float direction = path == 0u ? 1.0f : -1.0f;
        Reset(direction, 0.0f, path == 0u ? 2.0f : 1.0f);
        Tick(SR_COMMON_PREPARE_MS + SR_TURNOVER_HOLD_MS + 2u);
        if (path == 0u)
        {
            assert(phase == SR_PHASE_FORWARD_SWEEP);
        }
        else
        {
            Tick(SR_REVERSE_READY_HOLD_MS + 3u);
            assert(phase == SR_PHASE_REVERSE_STAND);
        }
        leg_lft.ang = leg_lft.ang_last = TWO_PI + 0.04f;
        leg_rgt.ang = leg_rgt.ang_last = TWO_PI + 0.05f;
        leg_lft.len = body.leg_len_lft = Leg_Len_Min + 0.01f;
        leg_rgt.len = body.leg_len_rgt = Leg_Len_Min + 0.02f;
        body.ang_pitch = imu_world.pitch = 0.02f;
        body.omg_pitch = body.omg_roll = 0.0f;
        for (unsigned ms = 0u; phase != SR_PHASE_HANDOFF && ms < 500u; ms++) Tick(1u);
        assert(phase == SR_PHASE_HANDOFF && !App_Self_Rescue_Finished());
        assert(App_Self_Rescue_Ctrl_Mode() == (path == 0u ? LEG_CTRL_MODE_SELF_RESCUE : LEG_CTRL_MODE_SELF_RESCUE_STAND));
        Tick(1u);
        assert(App_Self_Rescue_Finished() && App_Self_Rescue_Ctrl_Mode() == LEG_CTRL_MODE_MOVING);
        assert(fabsf(leg_lft.ang - 0.04f) < 0.001f && fabsf(leg_rgt.ang - 0.05f) < 0.001f);
        assert(fabsf(body.leg_len_tgt_lft - (Leg_Len_Min + 0.015f)) < 0.001f);
        assert(body.leg_len_tgt_lft == body.leg_len_tgt_rgt);
    }
    puts("self rescue: shared timeout, five alternating retries, latched stop, measured retract, handoff PASS");
    return 0;
}
