#include "app_step.h"
#include "app_safety.h"
#include "air.h"
#include "body.h"
#include "code_config.h"
#include "distributor.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "imu.h"
#include "leg.h"
#include "slip.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

body_state_t body;
leg_state_t leg_lft, leg_rgt;
volatile air_status_t air_status;
volatile uint8_t active_flag;
dji_motor_t dji_wheel_lft, dji_wheel_rgt;
dm_motor_t dm_f_lft, dm_b_lft, dm_f_rgt, dm_b_rgt;
imu_t imu;

static uint32_t tick;
static uint8_t action_mask;
static execution_sample_t latest;
static slip_side_status_t slip;
static step_status_t status;

void Test_Enter_Critical(void) {}
void Test_Exit_Critical(void) {}
uint32_t HAL_GetTick(void) { return tick; }
uint32_t Estimator_Frame_Tick(void) { return tick; }
void Air_Set_Step(uint8_t side) { (void)side; }
void Slip_Set_Step(bool follow, float intent, const float propulsion[2], uint8_t mask, bool recovering)
{
    (void)follow;
    (void)intent;
    (void)propulsion;
    (void)recovering;
    action_mask = mask;
}
void Slip_Side_Status(slip_side_status_t *out) { *out = slip; }
uint8_t Slip_Observation(void) { return action_mask ? 3u ^ action_mask : 3u; }
bool Slip_Prediction_Expired(void) { return false; }
void Execution_Latest(execution_sample_t *sample) { *sample = latest; }

static void Feed_Slip(slip_side_detector_t *detector, slip_side_input_t *input, uint32_t milliseconds, bool fresh)
{
    for (uint32_t i = 0u; i < milliseconds; i++)
    {
        input->now++;
        if (fresh)
        {
            input->frame[0]++;
            input->frame[1]++;
        }
        Slip_Side_Update(detector, input);
    }
}

static void Test_Slip(void)
{
    slip_side_input_t input = {.predicted = 0.12f, .intent = 0.12f, .valid_mask = 3u,
                               .propulsion = {1.0f, 1.0f}};
    slip_side_detector_t detector = {0};

    for (int turn = -1; turn <= 1; turn += 2)
    {
        input.yaw_rate = 0.8f * turn;
        input.velocity[0] = input.predicted - SLIP_HALF_TRACK * input.yaw_rate;
        input.velocity[1] = input.predicted + SLIP_HALF_TRACK * input.yaw_rate;
        Feed_Slip(&detector, &input, SLIP_EXIT_MS + 2u, true);
        assert(detector.status.side == SLIP_SIDE_NONE && detector.status.usable_mask == 3u);
        assert(fabsf(detector.status.center[0] - input.predicted) < 1e-6f);
        assert(fabsf(detector.status.center[1] - input.predicted) < 1e-6f);
    }
    input.yaw_rate = 0.0f;
    input.velocity[0] = 0.42f;
    input.velocity[1] = 0.12f;
    Feed_Slip(&detector, &input, 1u, true);
    Feed_Slip(&detector, &input, SLIP_ENTER_MS + 2u, false);
    assert(detector.status.side == SLIP_SIDE_UNKNOWN && detector.status.duration[0] == 0u);
    Feed_Slip(&detector, &input, SLIP_ENTER_MS + 2u, true);
    assert(detector.status.side == SLIP_SIDE_LEFT && detector.status.usable_mask == 2u);
    input.velocity[0] = 0.24f;
    Feed_Slip(&detector, &input, 1u, true);
    assert(detector.status.side == SLIP_SIDE_LEFT);
    input.velocity[0] = 0.12f;
    Feed_Slip(&detector, &input, SLIP_EXIT_MS - 1u, true);
    assert(detector.status.side == SLIP_SIDE_UNKNOWN);
    Feed_Slip(&detector, &input, 1u, true);
    assert(detector.status.side == SLIP_SIDE_NONE);

    input.velocity[1] = 0.42f;
    input.acceleration = 1.4f;
    Feed_Slip(&detector, &input, SLIP_ENTER_MS + 2u, true);
    assert(detector.status.side == SLIP_SIDE_UNKNOWN);
    input.acceleration = -1.4f;
    Feed_Slip(&detector, &input, SLIP_ENTER_MS + 2u, true);
    assert(detector.status.side == SLIP_SIDE_UNKNOWN);
    input.acceleration = 0.0f;
    input.propulsion[1] = -0.1f;
    Feed_Slip(&detector, &input, SLIP_ENTER_MS + 2u, true);
    assert(detector.status.side == SLIP_SIDE_UNKNOWN);
    input.propulsion[1] = 1.0f;
    Feed_Slip(&detector, &input, SLIP_ENTER_MS + 2u, true);
    assert(detector.status.side == SLIP_SIDE_RIGHT && detector.status.usable_mask == 1u);
    input.action_mask = 2u;
    Feed_Slip(&detector, &input, SLIP_ENTER_MS + 2u, true);
    assert(detector.status.side == SLIP_SIDE_UNKNOWN && detector.status.usable_mask == 1u);
    input.action_mask = 0u;
    input.velocity[0] = 0.43f;
    Feed_Slip(&detector, &input, 1u, true);
    assert(detector.status.side == SLIP_SIDE_BOTH && detector.status.usable_mask == 0u);
    input.valid_mask = 0u;
    Feed_Slip(&detector, &input, 1u, true);
    assert(detector.status.side == SLIP_SIDE_UNKNOWN && detector.status.usable_mask == 0u);

    detector = (slip_side_detector_t){0};
    input.valid_mask = 3u;
    input.acceleration = 1.4f;
    Feed_Slip(&detector, &input, SLIP_ENTER_MS + 2u, true);
    assert(detector.status.side == SLIP_SIDE_UNKNOWN && detector.status.usable_mask == 3u);
    input.acceleration = 0.0f;
    input.intent = -0.12f;
    input.propulsion[0] = input.propulsion[1] = -1.0f;
    input.predicted = -0.12f;
    input.velocity[0] = -0.12f;
    input.velocity[1] = -0.42f;
    Feed_Slip(&detector, &input, SLIP_ENTER_MS + 2u, true);
    assert(detector.status.side == SLIP_SIDE_RIGHT && detector.status.usable_mask == 1u);

    // 摆腿和伸缩项由 Body_State_Update 补到速度输入；镜像运动时两髋速度保持一致。
    detector = (slip_side_detector_t){0};
    input.velocity[0] = input.velocity[1] = input.predicted;
    Feed_Slip(&detector, &input, SLIP_EXIT_MS + 2u, true);
    for (unsigned int i = 0u; i < SLIP_ENTER_MS + 2u; i++)
    {
        const float leg_velocity = 0.05f * sinf(0.1f * i);

        input.velocity[0] = (input.predicted - leg_velocity) + leg_velocity;
        input.velocity[1] = (input.predicted + leg_velocity) - leg_velocity;
        Feed_Slip(&detector, &input, 1u, true);
        assert(detector.status.side == SLIP_SIDE_NONE);
    }
}

static void Test_Profile(void)
{
    float length, rate;
    const float start = 0.30f;
    const float target = start - STEP_STROKE;
    const float duration = 1.875f * STEP_STROKE / STEP_MOVE_RATE;

    for (unsigned int i = 0u; i <= (unsigned int)(1000.0f * duration) + 1u; i++)
    {
        App_Step_Profile(start, target, 0.0f, duration, 0.001f * i, &length, &rate);
        assert(length >= target && length <= start && rate <= 0.0f);
        assert(fabsf(rate) <= STEP_MOVE_RATE + 1e-6f);
    }
    assert(length == target && rate == 0.0f);
    App_Step_Profile(target, start, -0.01f, 0.5f, 0.0f, &length, &rate);
    assert(length == target && rate == -0.01f);
    App_Step_Profile(target, start, -0.01f, 0.5f, 0.0001f, &length, &rate);
    assert(fabsf(rate + 0.01f) < 1e-5f);
}

#if STEP_ENABLE
static void Reset(void)
{
    App_Step_Exit(STEP_REASON_DISABLED);
    tick = 1u;
    active_flag = APP_ENABLE;
    imu.online = true;
    dm_f_lft.enabled = dm_b_lft.enabled = dm_f_rgt.enabled = dm_b_rgt.enabled = 1u;
    dji_wheel_lft.frame_seq = dji_wheel_rgt.frame_seq = 1u;
    air_status = (air_status_t){.side_valid_mask = 3u, .side_contact_mask = 3u};
    body = (body_state_t){.ang_pitch = BALANCE_PITCH_REF};
    leg_lft = leg_rgt = (leg_state_t){.len = 0.300f};
    latest = (execution_sample_t){.model_u = {0.15f, 0.20f, 0.10f, -0.10f},
                                  .total_s = {80.0f, 80.0f}, .accepted_mask = EXECUTION_CHANNEL_MASK,
                                  .valid_mask = EXECUTION_CHANNEL_MASK};
    slip = (slip_side_status_t){.side = SLIP_SIDE_NONE, .valid_mask = 3u, .usable_mask = 3u};
}

static void Tick(bool follow, bool air, float intent, uint8_t limits)
{
    tick += CTRL_PERIOD_MS;
    latest.tick = imu.last_rx_tick = dji_wheel_lft.frame_tick = dji_wheel_rgt.frame_tick = tick;
    if (App_Step_Update(follow, air, intent, 0.0f))
    {
        float force[2], hip[2], wheel[2];

        App_Step_Output((float[2]){-12.0f, -12.0f}, (float[2]){0.0f, 0.0f}, force, hip, wheel);
        assert(isfinite(force[0]) && isfinite(force[1]));
        latest.total_s[0] = force[0];
        latest.total_s[1] = force[1];
        for (unsigned int side = 0u; side < 2u; side++)
        {
            latest.model_u[side] = -wheel[side];
            latest.model_u[side + 2u] = hip[side];
        }
        App_Step_Accepted(limits);
    }
    App_Step_Status(&status);
}

static void Advance(uint32_t ms, float intent)
{
    for (uint32_t i = 0u; i < ms; i += CTRL_PERIOD_MS)
        Tick(true, false, intent, 0u);
}

static void Trigger(uint8_t side, bool follow)
{
    slip.side = SLIP_SIDE_NONE;
    Advance(1u, 0.10f);
    slip.side = side == 1u ? SLIP_SIDE_LEFT : SLIP_SIDE_RIGHT;
    Tick(follow, false, 0.10f, 0u);
}

static void Test_Request(void)
{
    Reset();
    leg_lft.len = Leg_Len_Min + 0.001f;
    Trigger(1u, true);
    assert(status.phase == STEP_RETRACT && status.rejected == STEP_REASON_NONE);
    for (unsigned int i = 0u; i < 8u && status.phase == STEP_RETRACT; i++)
    {
        leg_lft.len = status.target[0];
        Advance(1u, 0.10f);
    }
    assert(status.phase == STEP_HOLD && status.target[0] == Leg_Len_Min);

    Reset();
    Trigger(2u, false);
    assert(status.phase == STEP_IDLE && status.rejected == STEP_REASON_MODE);
    latest.valid_mask = 0u;
    Trigger(2u, true);
    assert(status.phase == STEP_IDLE && status.rejected == STEP_REASON_FEEDBACK);
    latest.valid_mask = EXECUTION_CHANNEL_MASK;
    slip.side = SLIP_SIDE_NONE;
    Advance(1u, 0.10f);
    slip.side = SLIP_SIDE_BOTH;
    Advance(1u, 0.10f);
    assert(status.phase == STEP_IDLE);
    slip.side = SLIP_SIDE_UNKNOWN;
    Advance(1u, 0.10f);
    assert(status.phase == STEP_IDLE);
}

// 入口暂时拒绝不等于已经执行；同一持续事件条件恢复后应立即重试。
static void Test_Rejected_Retry(void)
{
    Reset();
    Trigger(1u, false);
    assert(status.phase == STEP_IDLE && status.rejected == STEP_REASON_MODE);
    Tick(true, false, 0.10f, 0u);
    assert(status.phase == STEP_RETRACT && status.rejected == STEP_REASON_NONE);

    Reset();
    leg_lft.len = Leg_Len_Min - 0.001f;
    Trigger(1u, true);
    assert(status.phase == STEP_IDLE && status.rejected == STEP_REASON_FEEDBACK);
    leg_lft.len = 0.30f;
    Tick(true, false, 0.10f, 0u);
    assert(status.phase == STEP_RETRACT && status.rejected == STEP_REASON_NONE);
}

// 堵转时驾驶请求可以超过动作速度上限，出力使用限速后的请求。
static void Test_Blocked_Drive(void)
{
    Reset();
    slip.side = SLIP_SIDE_NONE;
    Advance(1u, 1.0f);
    slip.side = SLIP_SIDE_LEFT;
    Tick(true, false, 1.0f, 0u);
    assert(status.phase == STEP_RETRACT && !App_Step_Fault());
    for (uint32_t i = 0u; i < 100u; i++)
    {
        leg_lft.len = status.target[0];
        leg_lft.d_len = status.target_rate;
        Advance(1u, 1.0f);
    }
    assert(status.target[0] < 0.30f && !App_Step_Fault());
    assert(fabsf(latest.model_u[1] - 0.4f) < 1e-5f);
}

// 逐拍跟随目标，覆盖承载与卸载两种反馈，动作行程必须一致。
static void Test_Mirror(uint8_t side, bool unloaded)
{
    leg_state_t *leg = side == 1u ? &leg_lft : &leg_rgt;
    const unsigned int action = side == 1u ? 0u : 1u;
    const float initial = 0.307f;
    float min_target = initial;
    uint32_t hold_tick = 0u;
    bool returned = false;

    Reset();
    leg->len = initial;
    Trigger(side, true);
    assert(status.phase == STEP_RETRACT && status.target[action] == initial);
    assert(latest.total_s[0] == 80.0f && latest.total_s[1] == 80.0f);
    for (uint32_t i = 0u; i < 4000u; i++)
    {
        const step_phase_t previous = status.phase;
        const float previous_target = status.target[action];
        const execution_sample_t previous_output = latest;

        leg->len = status.target[action];
        leg->d_len = status.target_rate;
        air_status.side_contact_mask = unloaded && status.phase < STEP_RETURN ? 3u ^ side : 3u;
        air_status.side_unloaded_mask = 3u ^ air_status.side_contact_mask;
        // 松开指令不打断已确认的单次动作。
        Advance(1u, 0.0f);
        min_target = fminf(min_target, status.target[action]);
        assert(fabsf(status.target[action] - previous_target) <= STEP_MOVE_RATE * 0.001f + 1e-7f);
        assert(fabsf(status.target_rate) <= STEP_MOVE_RATE + 1e-6f);
        if (status.phase != previous && App_Step_Owns_Output())
        {
            for (unsigned int j = 0u; j < 2u; j++)
            {
                assert(fabsf(latest.total_s[j] - previous_output.total_s[j]) < 1e-5f);
                assert(fabsf(latest.model_u[j] - previous_output.model_u[j]) < 1e-5f);
                assert(fabsf(latest.model_u[j + 2u] - previous_output.model_u[j + 2u]) < 1e-5f);
            }
        }
        if (status.phase == STEP_HOLD && previous != STEP_HOLD) hold_tick = tick;
        if (status.phase == STEP_RETURN && previous != STEP_RETURN)
        {
            assert(hold_tick != 0u && tick - hold_tick == STEP_HOLD_MS);
            returned = true;
        }
        if (status.phase == STEP_WAIT_CLEAR) break;
    }
    assert(returned && status.phase == STEP_WAIT_CLEAR && !App_Step_Fault());
    assert(status.terminated == STEP_REASON_COMPLETE && action_mask == 0u);
    assert(fabsf(min_target - (initial - STEP_STROKE)) < 1e-6f);
    assert(status.target[action] == initial);
    Advance(2000u, 0.10f);
    assert(status.phase == STEP_WAIT_CLEAR);
    slip.side = SLIP_SIDE_UNKNOWN;
    Advance(100u, 0.10f);
    assert(status.phase == STEP_WAIT_CLEAR);
    slip.side = SLIP_SIDE_BOTH;
    Advance(100u, 0.10f);
    assert(status.phase == STEP_WAIT_CLEAR);
    Trigger(side, true);
    assert(status.phase == STEP_RETRACT);
}

// 目标到位后须等待实测长度进入跟踪误差范围。
static void Test_Measured_Arrival(void)
{
    Reset();
    Trigger(1u, true);
    const uint32_t duration_ms = (uint32_t)(1000.0f * STEP_STROKE / STEP_MOVE_RATE) + 2u;
    Advance(duration_ms, 0.10f);
    assert(status.phase == STEP_RETRACT && !App_Step_Fault());
    leg_lft.len = status.target[0];
    Advance(1u, 0.10f);
    assert(status.phase == STEP_HOLD && status.elapsed_ms == 0u);
    Advance(STEP_HOLD_MS - 1u, 0.10f);
    assert(status.phase == STEP_HOLD);
    Advance(1u, 0.10f);
    assert(status.phase == STEP_RETURN);
}

// 轨迹时间结束不能代替实际到位；故障交给既有安全链。
static void Test_Failures(void)
{
    Reset();
    Trigger(1u, true);
    Advance(1500u, 0.10f);
    assert(App_Step_Fault() && status.phase != STEP_HOLD);

    Reset();
    Trigger(2u, true);
    latest.accepted_mask &= (uint8_t)~4u;
    Advance(100u, 0.10f);
    assert(App_Step_Fault() && status.terminated == STEP_REASON_FEEDBACK);

    Reset();
    Trigger(2u, true);
    air_status.side_contact_mask = 2u;
    Advance(1u, 0.10f);
    assert(latest.total_s[0] == -12.0f && latest.model_u[0] == 0.0f);
    App_Step_Exit(STEP_REASON_DISABLED);
    assert(!App_Step_Fault() && !App_Step_Owns_Output());

    Reset();
    Trigger(1u, true);
    Tick(true, true, 0.10f, 0u);
    assert(status.phase == STEP_WAIT_CLEAR && status.terminated == STEP_REASON_AIR);

    Reset();
    Trigger(1u, true);
    active_flag = APP_DISABLE;
    Advance(1u, 0.10f);
    assert(App_Step_Fault());
}

#endif

int main(void)
{
    Test_Slip();
    Test_Profile();
#if STEP_ENABLE
    Test_Request();
    Test_Rejected_Retry();
    Test_Blocked_Drive();
    Test_Mirror(1u, false);
    Test_Mirror(2u, true);
    Test_Mirror(1u, true);
    Test_Mirror(2u, false);
    Test_Measured_Arrival();
    Test_Failures();
#else
    App_Step_Init();
    App_Step_Status(&status);
    assert(status.phase == STEP_IDLE && status.slip.side == SLIP_SIDE_UNKNOWN);
#endif
    puts(STEP_ENABLE ? "step enabled: PASS" : "step disabled: PASS");
    return 0;
}
