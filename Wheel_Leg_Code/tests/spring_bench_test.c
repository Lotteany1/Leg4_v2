/** @file  spring_bench_test.c  @brief 右腿台架控制任务与驱动接口故障注入  @version  v1.0  2026-11-22 */
#include "app_spring_bench.h"
#include "app_safety.h"
#include "app_self_rescue.h"
#include "balance.h"
#include "body.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "fsm_running.h"
#include "imu.h"
#include "leg.h"
#include "Leg_Comp.h"
#include "Leg_Pos.h"
#include "Leg_Spd.h"
#include "Leg_Trq.h"
#include "monitor.h"
#include "task_sync.h"
#include <assert.h>
#include <math.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { LEFT_FRONT = 1, RIGHT_FRONT, LEFT_REAR, RIGHT_REAR };
enum { FRONT_FRAME = 1u, REAR_FRAME = 2u, BOTH_FRAMES = 3u };
static const float observed_length = 0.3662f, old_horizontal_angle = -1.907f, observed_roll = 0.0055f;
static float entry_angle;
static const float com_angle = 0.62135589f, com_radius = 0.10470625f; // 当前补偿表的 340 mm 端点
dm_motor_t dm_f_lft = {.slave_id = LEFT_FRONT}, dm_b_lft = {.slave_id = LEFT_REAR};
dm_motor_t dm_f_rgt = {.slave_id = RIGHT_FRONT}, dm_b_rgt = {.slave_id = RIGHT_REAR};
leg_state_t leg_rgt;
body_state_t body;
volatile bool task_ready = true;
volatile uint8_t active_flag;
volatile monitor_can_t monitor_can[3];
osThreadId_t EstimatorTaskHandle;

static struct
{
    unsigned int clear[5], enable[5], disable[5], drive[5], disable_order[4], disable_count, wheels, acknowledgements;
    float torque[5];
} io;
static dr16_t remote;
static bool radio_online = true, imu_online = true, pending_frame, saturated;
static uint16_t fail_drive, fail_disable, fail_enable;
static uint32_t clock_ms, flag_result;
static jmp_buf end_tick;
static float length, velocity, body_leg_angle, force_to_vmc, hip_to_vmc;
static spring_bench_frame_t frame;
void StartControlTask(void *argument);

uint32_t HAL_GetTick(void) { return clock_ms; }
bool Dr16_Online(void) { return radio_online; }
bool Imu_Online(void) { return imu_online; }
dr16_t Dr16_Snapshot(void) { return remote; }
void Test_Enter_Critical(void) {}
void Test_Exit_Critical(void) {}
uint32_t Monitor_Begin(void) { return clock_ms; }
void Monitor_End(monitor_slot_t slot, uint32_t begin) { assert(slot == MONITOR_SLOT_TICK && begin == clock_ms); }
int osDelay(uint32_t ticks) { assert(!task_ready); return 0; }

// 真实控制任务执行一次循环，下一次等待时返回测试程序。
uint32_t osThreadFlagsWait(uint32_t flags, uint32_t options, uint32_t timeout)
{
    assert(flags == TASK_ESTIMATE_READY && options == osFlagsWaitAny && timeout == 5u);
    if (!pending_frame) longjmp(end_tick, 1);
    pending_frame = false;
    return flag_result;
}

uint32_t osThreadFlagsSet(osThreadId_t thread, uint32_t flags)
{
    assert(thread == EstimatorTaskHandle && flags == TASK_CONTROL_DONE);
    io.acknowledgements++;
    return flags;
}

HAL_StatusTypeDef Dm_Motor_Clear_Error(FDCAN_HandleTypeDef *bus, uint16_t id)
{
    assert(id >= LEFT_FRONT && id <= RIGHT_REAR);
    io.clear[id]++;
    return HAL_OK;
}

HAL_StatusTypeDef Dm_Motor_Enable(FDCAN_HandleTypeDef *bus, uint16_t id)
{
    assert(id >= LEFT_FRONT && id <= RIGHT_REAR);
    io.enable[id]++;
    return id == fail_enable ? HAL_BUSY : HAL_OK;
}

HAL_StatusTypeDef Dm_Motor_Disable(FDCAN_HandleTypeDef *bus, uint16_t id)
{
    assert(id >= LEFT_FRONT && id <= RIGHT_REAR && io.disable_count < 4u);
    io.disable[id]++;
    io.disable_order[io.disable_count++] = id;
    return id == fail_disable ? HAL_BUSY : HAL_OK;
}

HAL_StatusTypeDef Dm_Motor_Mit_Ctrl(FDCAN_HandleTypeDef *bus, uint16_t id,
                                    float pos, float vel, float kp, float kd, float trq)
{
    assert(id >= LEFT_FRONT && id <= RIGHT_REAR);
    assert(pos == 0.0f && vel == 0.0f && kp == 0.0f && kd == 0.0f);
    if (id == LEFT_FRONT || id == LEFT_REAR) assert(trq == 0.0f);
    io.drive[id]++;
    io.torque[id] = trq;
    return id == fail_drive ? HAL_BUSY : HAL_OK;
}

void Dji_Motor_All_Stop(void) { io.wheels++; }
void Leg_Pos(float q1, float q4, float pos[2])
{
    pos[0] = length;
    pos[1] = atan2f(sinf(PI_2 - body_leg_angle), cosf(PI_2 - body_leg_angle));
}
void Leg_Spd(float q1, float q4, float w1, float w4, float speed[2]) { speed[0] = velocity; speed[1] = 0.0f; }
void Leg_Comp(float len, float *offset, float *radius) { *offset = com_angle; *radius = com_radius; }
float Balance_Axial_Gravity(float len, float angle) { return -WHEEL_MASS * GRAVITY * cosf(angle); }

// 使用生产拨杆译码。台架入口若意外执行常规 FSM、安全使能或自救，立即令测试失败。
bool App_Safety_Latched(void) { abort(); }
void App_Safety_Reset(void) { abort(); }
void App_Self_Rescue_Enter(void) { abort(); }
void App_Self_Rescue_Tick(void) { abort(); }
bool App_Self_Rescue_Finished(void) { abort(); }
self_rescue_phase_t App_Self_Rescue_Get_Phase(void) { abort(); }
balance_ctrl_mode_t App_Self_Rescue_Ctrl_Mode(void) { abort(); }
void Fsm_Running_Enter(void) { abort(); }
void Fsm_Running_Exit(void) { abort(); }
bool Fsm_Running_Fall_Update(void) { abort(); }
balance_ctrl_mode_t Fsm_Running_Tick(fsm_running_behavior_t behavior, const dr16_t *rc) { abort(); }
void Balance_Control(balance_ctrl_mode_t mode) { abort(); }
void Monitor_Win_Clear(void) { abort(); }
void Dm_Motor_All_Clear_Error(void) { abort(); }
uint32_t Estimator_Frame_Tick(void) { abort(); }
uint32_t osKernelGetTickCount(void) { abort(); }

// 两个不同的测试力臂辨别右侧前后关节出力映射；算法仅接受真实回读的 motor_force。
void Leg_Trq(float force, float hip, float q1, float q4, float torque[2])
{
    force_to_vmc = force;
    hip_to_vmc = hip;
    torque[0] = saturated ? DM_T_MAX : 0.1f * force + 0.03f * hip;
    torque[1] = -0.15f * force - 0.06f * hip;
}

// 单步真实控制任务，独立决定每个电机是否返回新帧，也可注入估计超时。
static void Tick(uint8_t left, uint8_t right, unsigned int updated, uint32_t flags)
{
    const spring_cal_phase_t old_phase = frame.phase;
    const float old_load = frame.load;
    clock_ms++;
    dm_f_lft.frame_tick = dm_b_lft.frame_tick = clock_ms;
    dm_f_lft.frame_cycle++;
    dm_b_lft.frame_cycle++;
    remote.sw_left = left;
    remote.sw_right = right;
    if (updated & FRONT_FRAME)
    {
        dm_f_rgt.frame_tick = clock_ms;
        dm_f_rgt.frame_cycle++;
    }
    if (updated & REAR_FRAME)
    {
        dm_b_rgt.frame_tick = clock_ms;
        dm_b_rgt.frame_cycle++;
    }
    memset(&io, 0, sizeof(io));
    flag_result = flags;
    pending_frame = true;
    if (setjmp(end_tick) == 0) StartControlTask(NULL);
    App_Spring_Bench_Frame(&frame);
    assert(io.acknowledgements == (flags == TASK_ESTIMATE_READY ? 1u : 0u));
    if (old_phase == SPRING_CAL_RUNNING && frame.phase == SPRING_CAL_RUNNING)
        assert(frame.load - old_load <= 0.001f * SPRING_CAL_FORCE_RATE + 0.00002f);
}

static void Test_Tick(unsigned int updated)
{
    Tick(DR16_SW_MID, DR16_SW_DOWN, updated, TASK_ESTIMATE_READY);
}

static void Check_Disabled(void)
{
    assert(active_flag == APP_DISABLE && frame.command == 0.0f && !frame.sample_ok);
    assert(io.disable_count == 4u && io.wheels == 1u);
    assert(io.disable_order[0] == RIGHT_FRONT && io.disable_order[1] == RIGHT_REAR);
    assert(io.disable[LEFT_FRONT] == 1u && io.disable[LEFT_REAR] == 1u);
}

// 上一趟状态任意，仅双下确认后再拨测试，且必须收到两侧本趟使能回报。
static void Arm(void)
{
    length = observed_length;
    velocity = 0.0f;
    body_leg_angle = entry_angle;
    body.ang_pitch = 0.0f;
    body.ang_roll = observed_roll;
    body.F_rgt = 0.0f;
    leg_rgt.len = NAN; // 未连接的左腿可能使常规双腿估计保留坏值
    dm_f_lft.err_code = dm_b_lft.err_code = DM_ERR_ENABLE;
    dm_f_rgt.err_code = DM_ERR_ENABLE;
    dm_b_rgt.err_code = DM_ERR_ENABLE;
    Tick(DR16_SW_DOWN, DR16_SW_DOWN, BOTH_FRAMES, TASK_ESTIMATE_READY);
    const uint32_t old_run = frame.run;
    Test_Tick(BOTH_FRAMES);
    assert(frame.run == old_run + 1u && frame.phase == SPRING_CAL_IDLE);
    for (int id = 1; id <= 4; id++) assert(io.clear[id] && !io.enable[id]);
    Test_Tick(BOTH_FRAMES);
    assert(io.enable[LEFT_FRONT] && io.enable[LEFT_REAR]);
    assert(io.enable[RIGHT_FRONT] && io.enable[RIGHT_REAR] && !io.drive[RIGHT_FRONT] && !io.drive[RIGHT_REAR]);
}

static void Start(void)
{
    Arm();
    Test_Tick(FRONT_FRAME);
    assert(frame.phase == SPRING_CAL_IDLE && !io.drive[RIGHT_FRONT] && !io.drive[RIGHT_REAR]);
    Test_Tick(REAR_FRAME);
    assert(frame.phase == SPRING_CAL_RUNNING && frame.pair == 1u && active_flag == APP_ENABLE);
    assert(frame.enabled_mask == 15u && io.drive[LEFT_FRONT] && io.drive[LEFT_REAR]);
    assert(frame.command == -SPRING_CAL_START_N && force_to_vmc == frame.command);
    assert(fabsf(frame.angle - entry_angle) < 0.00001f && frame.len == observed_length && frame.roll == observed_roll);
    const float wheel_hip = WHEEL_MASS * GRAVITY * frame.len * sinf(frame.angle);
    const float leg_hip = LEG_GRAV_FF * GRAVITY * com_radius * sinf(frame.angle + com_angle);
    assert(fabsf(frame.hip_gravity - leg_hip - wheel_hip) < 0.0002f);
    assert(fabsf(frame.hip_gravity - frame.hip_command) < 0.00001f);
    assert(fabsf(hip_to_vmc + frame.hip_gravity) < 0.00001f);
    assert(fabsf(io.torque[RIGHT_FRONT] - (-0.15f * frame.command - 0.06f * hip_to_vmc)) < 0.00001f);
    assert(fabsf(io.torque[RIGHT_REAR] - (0.1f * frame.command + 0.03f * hip_to_vmc)) < 0.00001f);
}

static void Check_Latched(spring_cal_reason_t why)
{
    const uint32_t run = frame.run;
    for (unsigned int k = 0; k < 12u; k++)
    {
        Test_Tick(BOTH_FRAMES);
        assert(frame.reason == why && frame.run == run && frame.command == 0.0f);
        assert(!io.enable[RIGHT_FRONT] && !io.enable[RIGHT_REAR] && !io.drive[RIGHT_FRONT] && !io.drive[RIGHT_REAR]);
    }
}

// 固定机身的合成电机、非线性弹簧和重力：用运动反馈走完整趟，覆盖高预载起步。
static void Test_Loaded_Stroke(void)
{
    Start();
    float measured_length = length;
    unsigned int usable = 0u, breakaway_ms = 0u, uncalibrated_motion = 0u;
    float first_sample = 0.0f, last_sample = 0.0f;
    for (unsigned int k = 1u; k <= 300000u && length > Leg_Len_Min + SPRING_CAL_STOP_MARGIN_M; k++)
    {
        const float compressed = (observed_length - measured_length) / (observed_length - Leg_Len_Min);
        const float passive = SPRING_CAL_START_N + frame.gravity - 0.85f + 0.15f * compressed + 0.05f * sinf(2.0f * PI * compressed);
        const float acceleration = (frame.command + passive - frame.gravity - 85.0f * velocity) / 1.8f;
        body.F_rgt = frame.command;
        velocity += 0.001f * acceleration;
        measured_length += 0.001f * velocity;
        if (measured_length >= observed_length)
        {
            measured_length = observed_length;
            velocity = 0.0f;
        }
        length = measured_length;
        Test_Tick(BOTH_FRAMES);
        if (!breakaway_ms && length < observed_length - 0.002f) breakaway_ms = k;
        if (frame.len > Leg_Len_Max)
        {
            assert(!frame.sample_ok);
            if (frame.rate < -0.001f && frame.len < observed_length - 0.008f) uncalibrated_motion++;
        }
        if (frame.sample_ok)
        {
            if (!usable) first_sample = frame.len;
            last_sample = frame.len;
            usable++;
        }
    }
    printf("synthetic loaded stroke: breakaway %.1f s, finish %.1f s, %u usable samples, %.1f..%.1f mm\n",
           breakaway_ms * 0.001, frame.elapsed_ms * 0.001, usable, last_sample * 1000.0, first_sample * 1000.0);
    assert(uncalibrated_motion > 100u && breakaway_ms > 0u && usable > 1000u);
    assert(frame.phase == SPRING_CAL_RUNNING && length <= Leg_Len_Min + SPRING_CAL_STOP_MARGIN_M);
    assert(first_sample > Leg_Len_Max - 0.02f && last_sample < Leg_Len_Min + 0.02f);
}

int main(void)
{
    assert(fabsf(SPRING_CAL_BENCH_START_M - observed_length) < 0.001f);
    assert(SPRING_CAL_BENCH_ANGLE_RAD == 0.0f);
    length = observed_length;
    Test_Tick(BOTH_FRAMES);
    Check_Disabled();
    assert(!frame.run && frame.phase == SPRING_CAL_IDLE && !io.enable[RIGHT_FRONT] && !io.drive[RIGHT_REAR]);
    for (unsigned int k = 0; k < 20u; k++) Test_Tick(BOTH_FRAMES);
    assert(frame.run == 0 && active_flag == APP_DISABLE);

    Arm();
    length = 0.36182749f;
    body_leg_angle = -0.099449f;
    Test_Tick(BOTH_FRAMES);
    assert(frame.phase == SPRING_CAL_RUNNING && frame.command == -SPRING_CAL_START_N);
    Tick(DR16_SW_DOWN, DR16_SW_DOWN, BOTH_FRAMES, TASK_ESTIMATE_READY);
    Check_Disabled();

    Start();
    for (unsigned int k = 0; k < 200u; k++) Test_Tick(BOTH_FRAMES);
    assert(fabsf(frame.command + SPRING_CAL_START_N + 0.2f * SPRING_CAL_FORCE_RATE) < 0.0001f);
    const float held_before_speed = frame.load;
    velocity = -0.026f;
    Test_Tick(BOTH_FRAMES);
    assert(frame.load == held_before_speed);
    velocity = 0.0f;
    for (unsigned int k = 0; k < 1000u; k++) Test_Tick(BOTH_FRAMES);
    assert(frame.load > held_before_speed && frame.load <= held_before_speed + SPRING_CAL_FORCE_RATE + 0.00002f);
    Tick(DR16_SW_MID, DR16_SW_MID, BOTH_FRAMES, TASK_ESTIMATE_READY);
    Check_Disabled();
    Check_Latched(SPRING_CAL_REASON_RELEASE);

    Start();
    fail_drive = RIGHT_REAR;
    fail_disable = RIGHT_FRONT;
    Test_Tick(BOTH_FRAMES);
    Check_Disabled();
    assert(io.drive[RIGHT_FRONT] == 1u && io.drive[RIGHT_REAR] == 1u && frame.reason == SPRING_CAL_REASON_LIMIT);
    fail_drive = fail_disable = 0;
    Test_Tick(BOTH_FRAMES);
    Check_Disabled(); // 上一拍撤力未入队，本拍即重试
    Check_Latched(SPRING_CAL_REASON_LIMIT);

    Start();
    Tick(DR16_SW_MID, DR16_SW_DOWN, 0, osFlagsError);
    Check_Disabled();
    Check_Latched(SPRING_CAL_REASON_FEEDBACK);

    Start();
    for (unsigned int k = 0; k < 5u; k++) Test_Tick(FRONT_FRAME);
    assert(frame.phase == SPRING_CAL_RUNNING);
    Test_Tick(FRONT_FRAME);
    Check_Disabled();
    Check_Latched(SPRING_CAL_REASON_FEEDBACK);

    Start();
    imu_online = false;
    Test_Tick(BOTH_FRAMES);
    Check_Disabled();
    imu_online = true;
    Check_Latched(SPRING_CAL_REASON_FEEDBACK);

    Start();
    radio_online = false;
    Test_Tick(BOTH_FRAMES);
    Check_Disabled();
    radio_online = true;
    Check_Latched(SPRING_CAL_REASON_FEEDBACK);

    Tick(DR16_SW_DOWN, DR16_SW_DOWN, BOTH_FRAMES, TASK_ESTIMATE_READY);
    Test_Tick(BOTH_FRAMES);
    fail_enable = LEFT_REAR;
    Test_Tick(BOTH_FRAMES);
    assert(frame.phase == SPRING_CAL_IDLE && frame.gate == 4u && !io.drive[RIGHT_FRONT]);
    fail_enable = 0;
    Test_Tick(BOTH_FRAMES);
    assert(io.enable[LEFT_REAR] && !io.enable[RIGHT_FRONT]);
    Test_Tick(BOTH_FRAMES);
    assert(frame.phase == SPRING_CAL_RUNNING);
    Tick(DR16_SW_MID, DR16_SW_MID, BOTH_FRAMES, TASK_ESTIMATE_READY);
    Check_Disabled();

    // 清错应答丢失时仅重试未应答关节，先确认的关节保持零力矩。
    Tick(DR16_SW_DOWN, DR16_SW_MID, BOTH_FRAMES, TASK_ESTIMATE_READY);
    Tick(DR16_SW_MID, DR16_SW_MID, BOTH_FRAMES, TASK_ESTIMATE_READY);
    Test_Tick(BOTH_FRAMES);
    for (unsigned int k = 0; k < 10u; k++) Test_Tick(FRONT_FRAME);
    assert(io.clear[RIGHT_REAR] && !io.clear[RIGHT_FRONT] && frame.gate == 4u);
    Test_Tick(BOTH_FRAMES);
    Test_Tick(BOTH_FRAMES);
    assert(frame.phase == SPRING_CAL_RUNNING && frame.enabled_mask == 15u);
    Tick(DR16_SW_MID, DR16_SW_MID, BOTH_FRAMES, TASK_ESTIMATE_READY);
    Check_Disabled();

    Arm();
    dm_b_lft.err_code = DM_ERR_DISABLE;
    for (unsigned int k = 0; k < 119u; k++)
    {
        Test_Tick(BOTH_FRAMES);
        assert(!io.drive[RIGHT_FRONT] && !io.drive[RIGHT_REAR]);
    }
    Check_Disabled();
    Check_Latched(SPRING_CAL_REASON_FEEDBACK);

    Start();
    saturated = true;
    Test_Tick(BOTH_FRAMES);
    assert(!io.drive[RIGHT_FRONT] && !io.drive[RIGHT_REAR]);
    Check_Disabled();
    saturated = false;
    Check_Latched(SPRING_CAL_REASON_LIMIT);

    Start();
    velocity = -SPRING_CAL_OVERSPEED_MPS;
    Test_Tick(BOTH_FRAMES);
    assert(frame.phase == SPRING_CAL_RUNNING && frame.command == -SPRING_CAL_START_N);

    Start();
    body.F_rgt = -SPRING_CAL_FORCE_MAX_N;
    Test_Tick(BOTH_FRAMES);
    Check_Disabled();
    Check_Latched(SPRING_CAL_REASON_FORCE);

    Start();
    length = Leg_Len_Max - 0.003f;
    Test_Tick(BOTH_FRAMES);
    for (unsigned int k = 0; k < 7000u; k++) Test_Tick(BOTH_FRAMES);
    assert(frame.phase == SPRING_CAL_RUNNING && frame.load > SPRING_CAL_START_N);

    Start();
    for (int sign = -1; sign <= 1; sign += 2)
    {
        body.ang_pitch = (float)sign * 1.57079632679f;
        Test_Tick(BOTH_FRAMES);
        assert(frame.phase == SPRING_CAL_RUNNING);
    }
    body.ang_pitch = 1.571f;
    Test_Tick(BOTH_FRAMES);
    Check_Disabled();
    Check_Latched(SPRING_CAL_REASON_ANGLE);

    Start();
    body.ang_roll = 0.4f;
    Test_Tick(BOTH_FRAMES);
    Check_Disabled();
    Check_Latched(SPRING_CAL_REASON_ANGLE);

    Arm();
    length = Leg_Len_Max; // 日常控制长端 340 mm，距实测台架满伸仍有 26.2 mm
    Test_Tick(BOTH_FRAMES);
    Check_Disabled();
    Check_Latched(SPRING_CAL_REASON_START);

    Arm();
    body_leg_angle = old_horizontal_angle; // 之前的斜上安装姿态，与椅子上向下悬空相区分
    Test_Tick(BOTH_FRAMES);
    Check_Disabled();
    Check_Latched(SPRING_CAL_REASON_ANGLE);

    entry_angle = -0.28328943f; // 允许的初始小倾角也应包含轮重力矩并正确镜像
    Start();
    Tick(DR16_SW_MID, DR16_SW_MID, BOTH_FRAMES, TASK_ESTIMATE_READY);
    Check_Disabled();
    Check_Latched(SPRING_CAL_REASON_RELEASE);
    entry_angle = 0.0f;

    Start();
    length = Leg_Len_Min + 0.0039f;
    Test_Tick(BOTH_FRAMES);
    const float held = frame.command;
    for (unsigned int k = 0; k < 1000u; k++) Test_Tick(BOTH_FRAMES);
    assert(held < 0 && frame.command == held && !frame.sample_ok && frame.phase == SPRING_CAL_RUNNING);
    Tick(DR16_SW_MID, DR16_SW_MID, BOTH_FRAMES, TASK_ESTIMATE_READY);
    Check_Disabled();
    Check_Latched(SPRING_CAL_REASON_RELEASE);
    spring_cal_t cal = {0};
    spring_cal_input_t input = {.valid = true, .length = observed_length,
        .min_length = Leg_Len_Min, .max_length = observed_length};
    App_Spring_Cal_Update(&cal, &input);
    assert(cal.motor_command == -SPRING_CAL_START_N);
    for (unsigned int k = 1; k <= 10000u; k++)
    {
        input.tick_ms = input.sample_tick = k;
        App_Spring_Cal_Update(&cal, &input);
    }
    assert(fabs(cal.compression - SPRING_CAL_START_N - 10.0 * SPRING_CAL_FORCE_RATE) < 0.00001);
    Test_Loaded_Stroke();
    puts("PASS right spring bench gesture, paired feedback, force slope, mirrored output, same-tick shutdown, retry and latching");
    return 0;
}
