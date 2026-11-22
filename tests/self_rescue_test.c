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

int main(void)
{
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

    /* Initial selection is latched; later debugger edits apply only on re-entry. */
    Reset(-1.0f, 0.0f, 2.0f);
    rescue_dir = 1.0f;
    Tick(SR_COMMON_PREPARE_MS + SR_TURNOVER_HOLD_MS + 1u);
    assert(phase == SR_PHASE_REVERSE_SWING && active_dir == -1.0f);
    /* Exercise total timeout independently of the faster obstacle detector. */
    leg_lft.d_ang = leg_rgt.d_ang = 2.0f;
    /* A full round visits both paths in the requested order. */
    Tick(SR_REVERSE_PATH_TIMEOUT_MS + SR_TURNOVER_HOLD_MS + 5u);
    assert(active_dir == 1.0f && second_path && round_count == 0u);
    Tick(60000u);
    assert(phase == SR_PHASE_FAILED && round_count == SR_COMMON_ROUND_MAX);
    assert(!App_Self_Rescue_Finished() && App_Self_Rescue_Ctrl_Mode() == LEG_CTRL_MODE_NUM);
    Tick(1000u);
    assert(phase == SR_PHASE_FAILED);

    Reset(1.0f, 0.0f, 2.0f);
    Tick(SR_COMMON_PREPARE_MS + SR_TURNOVER_HOLD_MS + 1u);
    assert(phase == SR_PHASE_FORWARD_RETRACT || phase == SR_PHASE_FORWARD_SWEEP);
    Tick(60000u);
    assert(phase == SR_PHASE_FAILED && round_count == SR_COMMON_ROUND_MAX);

    /* Opposite pitch produces opposite turnover directions; initial path is irrelevant. */
    Reset(-1.0f, 2.5f, 0.0f);
    Tick(SR_COMMON_PREPARE_MS + SR_TURNOVER_EXTEND_MS + 1u);
    assert(turnover_dir > 0.0f && body.leg_ang_tgt_lft > 0.0f);
    assert(App_Self_Rescue_Ctrl_Mode() == LEG_CTRL_MODE_SELF_RESCUE_TURNOVER);
    Tick(30000u);
    assert(phase == SR_PHASE_FAILED && round_count == SR_COMMON_ROUND_MAX);
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

    /* A blocked reverse swing retracts before positive sweep, without re-extending in turnover. */
    Reset(-1.0f, 0.0f, 2.0f);
    leg_lft.len = leg_rgt.len = Leg_Len_Max;
    Tick(SR_COMMON_PREPARE_MS + SR_TURNOVER_HOLD_MS + SR_OBSTACLE_STALL_HOLD_MS + 2u);
    assert(phase == SR_PHASE_OBSTACLE_RETRACT && active_dir > 0.0f);
    assert(round_count == 0u && second_path);
    assert(body.leg_omg_tgt_lft == 0.0f && body.leg_omg_tgt_rgt == 0.0f);
    Tick(100u);
    assert(phase == SR_PHASE_OBSTACLE_RETRACT);
    leg_lft.len = leg_rgt.len = Leg_Len_Min;
    Tick(2u);
    assert(phase == SR_PHASE_FORWARD_SWEEP);
    assert(App_Self_Rescue_Ctrl_Mode() == LEG_CTRL_MODE_SELF_RESCUE);
    Tick(100u);
    assert(fabsf(body.leg_ang_tgt_lft - leg_lft.ang) <= SR_OBSTACLE_SWEEP_LEAD_MAX_RAD + 0.001f);

    /* An already positioned leg must not trigger stall detection. */
    Reset(-1.0f, 0.0f, 0.0f);
    phase                = SR_PHASE_REVERSE_SWING;
    body.leg_omg_tgt_lft = body.leg_omg_tgt_rgt = 0.0f;
    for (unsigned i = 0; i < SR_OBSTACLE_STALL_HOLD_MS + 5u; i++)
        assert(!Self_Rescue_Stalled());

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
    assert(phase == SR_PHASE_REVERSE_RETRACT && active_dir < 0.0f && !second_path);
    leg_lft.len = body.leg_len_lft = Leg_Len_Min;
    leg_rgt.len = body.leg_len_rgt = Leg_Len_Min;
    Tick(1u);
    assert(phase == SR_PHASE_REVERSE_STAND);
    body.ang_roll = SR_REVERSE_READY_ROLL_MAX_RAD + 0.01f;
    Tick(1u);
    assert(phase == SR_PHASE_OBSTACLE_RETRACT && active_dir > 0.0f);

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
       It must retract and change path, never command continued reverse swinging. */
    Reset(-1.0f, 0.471227f, -0.254794f);
    leg_rgt.ang          = -0.184528f;
    phase                = SR_PHASE_REVERSE_STAND;
    body.leg_omg_tgt_lft = 11.57f;
    body.leg_omg_tgt_rgt = 6.35f;
    Tick(1u);
    assert(phase == SR_PHASE_OBSTACLE_RETRACT && active_dir > 0.0f);
    assert(body.leg_omg_tgt_lft == 0.0f && body.leg_omg_tgt_rgt == 0.0f);
    Tick(2u);
    assert(phase == SR_PHASE_FORWARD_SWEEP);

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
    puts("self rescue: path ordering, three-round limit, turnover timeout/direction, measured retract, handoff PASS");
    return 0;
}
