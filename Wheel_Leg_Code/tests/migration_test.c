#include "task_sync.h"
#include "body.h"
#include "leg.h"
#include "imu_to_world.h"
#include "air.h"
#include "app_gyro.h"
#include "fsm_running.h"
#include "app_rc.h"
#include "app_safety.h"
#include "app_self_rescue.h"
#include "balance.h"
#include "code_config.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "dr16.h"
#include "fsm.h"
#include "imu.h"
#include "monitor.h"
#include "pid.h"
#include "slip.h"
#include "system_key.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;
dm_motor_t dm_f_lft, dm_b_lft, dm_f_rgt, dm_b_rgt;
dji_motor_t dji_wheel_lft, dji_wheel_rgt;
imu_t imu;
dr16_t dr16;
volatile monitor_total_t monitor_total;
volatile monitor_win_t monitor_win;
volatile bool task_ready;
osThreadId_t ControlTaskHandle;

static uint32_t tick, notifications, critical_depth;
static uint32_t output_count, stop_count, disable_count;
static uint32_t joint_frames, clear_count;
static uint32_t rescue_reset_count, model_reset_count;
static bool online = true;
static balance_ctrl_mode_t output_mode;

void Test_Enter_Critical(void)
{
    critical_depth++;
}
void Test_Exit_Critical(void)
{
    assert(critical_depth);
    critical_depth--;
}
uint32_t HAL_GetTick(void)
{
    return tick;
}
uint32_t osKernelGetTickCount(void)
{
    return tick;
}
int osDelay(uint32_t delay)
{
    tick += delay;
    return 0;
}
int osDelayUntil(uint32_t next)
{
    tick = next;
    return 0;
}
void Dr16_Update(void)
{
}
bool Imu_Parse(void)
{
    return false;
}
void Dm_Motor_Parse(void)
{
}
void Dji_Motor_Parse(void)
{
}
uint32_t osThreadFlagsSet(osThreadId_t id, uint32_t flags)
{
    (void)id;
    assert(critical_depth == 0u);
    notifications |= flags;
    return notifications;
}
uint32_t Estimator_Frame_Tick(void)
{
    return tick;
}
void Monitor_Win_Clear(void)
{
}
dr16_t Dr16_Snapshot(void)
{
    return dr16;
}
int16_t Dr16_Deadline(int16_t input, uint16_t deadline)
{
    return abs(input) > deadline ? input : 0;
}
bool Dr16_Online(void)
{
    return online;
}
bool Imu_Online(void)
{
    return online;
}
bool Dm_Motor_Online(void)
{
    return online;
}
HAL_StatusTypeDef Dm_Motor_All_Enable(void)
{
    joint_frames += 4u;
    return HAL_OK;
}
HAL_StatusTypeDef Dm_Motor_All_Disable(void)
{
    disable_count++;
    joint_frames += 4u;
    dm_f_lft.enabled = dm_b_lft.enabled = dm_f_rgt.enabled = dm_b_rgt.enabled = 0u;
    return HAL_OK;
}
HAL_StatusTypeDef Dm_Motor_All_Set_Zero(void)
{
    return HAL_OK;
}

void Dm_Motor_All_Clear_Error(void)
{
    joint_frames += 4u;
    clear_count++;
}
void Dm_Send_Zero_All(void)
{
    joint_frames += 4u;
}
void Dji_Motor_All_Stop(void)
{
    stop_count++;
}
void Balance_Rescue_Init(void)
{
    rescue_reset_count++;
}
void Balance_Mpc_Stop(void)
{
    model_reset_count++;
}
void Balance_Control(balance_ctrl_mode_t mode)
{
    if (mode == LEG_CTRL_MODE_MOVING)
        assert(Fsm_State_Get() == FSM_STATE_BALANCE && body.h_tgt >= Leg_Len_Min);
    output_count++;
    output_mode = mode;
    joint_frames += 4u;
}

static void Test_Close(float actual, float expected)
{
    assert(isfinite(actual));
    assert(fabsf(actual - expected) < 1e-5f * (1.0f + fabsf(expected)));
}

static void Test_Pose(void)
{
    memset(&body, 0, sizeof(body));
    memset(&leg_lft, 0, sizeof(leg_lft));
    memset(&leg_rgt, 0, sizeof(leg_rgt));
    memset(&imu_world, 0, sizeof(imu_world));
    body.leg_len_lft = body.leg_len_rgt = leg_lft.len = leg_rgt.len = 0.17f;
}

static void Test_Pid_Period(void)
{
    pid_t pid;
    Pid_Init(&pid, 0.0f, 1.0f, 4.0f, 1000.0f, -1000.0f);
    Pid_Update(&pid, 0.002f, 0.0f);
    Test_Close(pid.d_out, 0.004f);
    Pid_Init(&pid, 0.0f, 1.0f, 0.0f, 1000.0f, -1000.0f);
    for (uint32_t i = 0u; i < 500u; i++) Pid_Update(&pid, 1.0f, 0.0f);
    Test_Close(pid.i_out, 0.998f);
}

static void Test_Rescue(void)
{
    Test_Pose();
    imu_world.pitch = 2.0f;
    App_Self_Rescue_Enter(1.0f);
    assert(App_Self_Rescue_Get_Phase() == SR_PHASE_ZERO);
    for (uint32_t ms = 0u; ms < SR_ZERO_MS - CTRL_PERIOD_MS; ms += CTRL_PERIOD_MS) App_Self_Rescue_Tick();
    assert(App_Self_Rescue_Get_Phase() == SR_PHASE_ZERO);
    App_Self_Rescue_Tick();
    assert(App_Self_Rescue_Get_Phase() == SR_PHASE_RETRACT);
    for (uint32_t ms = 0u; ms < SR_RETRACT_MS; ms += CTRL_PERIOD_MS) App_Self_Rescue_Tick();
    assert(App_Self_Rescue_Get_Phase() == SR_PHASE_ROTATE);
    App_Self_Rescue_Tick();
    Test_Close(body.leg_ang_tgt_lft, SR_ROTATE_SPEED);
    imu_world.pitch = 0.0f;
    for (uint32_t ms = 0u; ms < SR_EXIT_HOLD_MS; ms += CTRL_PERIOD_MS) App_Self_Rescue_Tick();
    assert(App_Self_Rescue_Get_Phase() == SR_PHASE_RETURN);
    App_Self_Rescue_Tick();
    assert(App_Self_Rescue_Finished());

    Test_Pose();
    App_Self_Rescue_Enter(-1.0f);
    assert(App_Self_Rescue_Ctrl_Mode() == LEG_CTRL_MODE_SELF_RESCUE_SWING);
    for (uint32_t ms = 0u; ms <= SR_SWING_HOLD_MS; ms += CTRL_PERIOD_MS) App_Self_Rescue_Tick();
    assert(App_Self_Rescue_Get_Phase() == SR_PHASE_STAND);
    assert(App_Self_Rescue_Ctrl_Mode() == LEG_CTRL_MODE_SELF_RESCUE_STAND);
    App_Self_Rescue_Tick();
    for (uint32_t ms = 0u; ms <= SR_STAND_HOLD_MS; ms += CTRL_PERIOD_MS) App_Self_Rescue_Tick();
    assert(App_Self_Rescue_Get_Phase() == SR_PHASE_RETURN);
    App_Self_Rescue_Tick();
    assert(App_Self_Rescue_Finished());

    Test_Pose();
    imu_world.pitch = 2.0f;
    App_Self_Rescue_Enter(-1.0f);
    for (uint32_t ms = 0u; ms < SR_SWING_TIMEOUT_MS; ms += CTRL_PERIOD_MS) App_Self_Rescue_Tick();
    assert(App_Self_Rescue_Get_Phase() == SR_PHASE_FAILED);
    App_Self_Rescue_Tick();
    assert(App_Self_Rescue_Finished());
    assert(rescue_reset_count == 3u && model_reset_count == 6u);

    Test_Pose();
    Fsm_Running_Enter();
    imu_world.pitch = 2.0f;
    for (uint32_t ms = 0u; ms < 200u - CTRL_PERIOD_MS; ms += CTRL_PERIOD_MS) assert(!Fsm_Running_Fall_Update());
    assert(Fsm_Running_Fall_Update());
    imu_world.pitch = 0.0f;
    assert(!Fsm_Running_Fall_Update());
}

static void Test_Estimator_Period(void)
{
    Test_Pose();
    Body_State_Init();
    memset(&imu, 0, sizeof(imu));
    dji_wheel_lft.vel = -0.5f * REDUCTION_RATIO / WHEEL_R;
    dji_wheel_rgt.vel = 0.5f * REDUCTION_RATIO / WHEEL_R;
    Body_State_Update();
    Test_Close(body.pos, 0.001f);
    body.vel_tgt         = 1.0f;
    body.omg_yaw_tgt     = 1.0f;
    imu.eul[IMU_EUL_YAW] = RAD_TO_DEG(0.7f);
    Body_State_Update();
    Test_Close(body.pos, 0.0f);
    Test_Close(body.ang_yaw_tgt, 0.7f);

    imu.online         = true;
    imu.acc[IMU_ACC_X] = 0.0f;
    Slip_Init();
    Slip_Update();
    Test_Close(body.vel_kalman, 0.5f);
    Slip_Set_Air(1u);
    body.vel = 2.0f;
    Slip_Update();
    Test_Close(body.vel_kalman, 0.5f);
    Slip_Set_Air(0u);
    Slip_Update();
    Test_Close(body.vel_kalman, 2.0f);
}

static void Test_Fsm_Tick(void)
{
    const fsm_request_t req = Fsm_Read_Request(&dr16);
    Fsm_Tick(&dr16, &req);
}

static void Test_Control_Tick(void)
{
    joint_frames = 0u;
    tick += CTRL_PERIOD_MS;
    const fsm_request_t req = Fsm_Read_Request(&dr16);
    App_Safety_Online_Check(req.run);
    App_Safety_Apply_Flag();
    Fsm_Tick(&dr16, &req);
    assert(joint_frames <= 8u);
}

static void Test_Running_Targets(void)
{
    Test_Pose();
    Fsm_Running_Enter();
    dr16.ch2 = 132;
    dr16.ch3 = 0;
    dr16.ch5 = 0;
    Fsm_Running_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, &dr16);
    Test_Close(body.vel_tgt, 0.57f);
    Test_Close(body.h_tgt, Leg_Len_Min);

    // 前后满杆配合左右零、半、满旋转 验证每拍重新映射后按比例缩放且不改变旋转指令
    for (int forward = -660; forward <= 660; forward += 1320)
    {
        for (int turn = -660; turn <= 660; turn += 330)
        {
            float expected_vel = (float)forward / 660.0f * 2.85f;

#if BALANCE_USE_TURN_LIMIT
            expected_vel *= 1.0f - fabsf((float)turn) / 660.0f;
#endif
            dr16.ch2 = forward;
            dr16.ch3 = turn;
            Fsm_Running_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, &dr16);
            Test_Close(body.vel_tgt, expected_vel);
            Test_Close(body.omg_yaw_tgt, -(float)turn / 660.0f * 6.0f);
        }
    }
    dr16.ch2 = 132;
    dr16.ch3 = 0;

    // 陀螺运行中重进平衡后 普通跟随保留入场高度 再切陀螺仍从新包络起步
    Fsm_Running_Tick(FSM_RUNNING_BEHAVIOR_GYRO, &dr16);
    assert(body.omg_yaw_tgt != 0.0f);
    Fsm_Running_Enter();
    Fsm_Running_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, &dr16);
    Test_Close(body.h_tgt, Leg_Len_Min);
    Fsm_Running_Tick(FSM_RUNNING_BEHAVIOR_GYRO, &dr16);
    assert(body.omg_yaw_tgt != 0.0f);
    Fsm_Running_Enter();
    Fsm_Running_Tick(FSM_RUNNING_BEHAVIOR_GYRO, &dr16);
    assert(body.omg_yaw_tgt != 0.0f);
    Fsm_Running_Enter();
    dr16.ch5 = -660;
    Fsm_Running_Tick(FSM_RUNNING_BEHAVIOR_FOLLOW, &dr16);
    Test_Close(body.h_tgt, Leg_Len_Min + 0.3f * CTRL_DT);

    body.h_tgt = body.leg_len_tgt_lft = body.leg_len_tgt_rgt = 0.27f;
    dr16.ch5                                                 = 660;
    App_Rc_Update_Targets(&body, &dr16);
    Test_Close(body.h_tgt, 0.27f - 0.3f * CTRL_DT);
    Test_Close(body.leg_len_tgt_lft, 0.27f);
    dr16.ch2 = dr16.ch5 = 0;
}

static void Test_Fsm(void)
{
    Test_Pose();
    Fsm_Rescue_Dir_Set(1.0f);
    dr16.sw_left  = DR16_SW_DOWN;
    dr16.sw_right = DR16_SW_MID;
    Test_Control_Tick();
    assert(active_flag == APP_DISABLE);
    assert(Fsm_State_Get() == FSM_STATE_DISABLE);
    dr16.sw_left = DR16_SW_MID;
    Test_Control_Tick();
    assert(active_flag == APP_ENABLE);
    assert(Fsm_State_Get() == FSM_STATE_WAIT_ENABLE);
    uint32_t before = output_count;
    Test_Control_Tick();
    assert(output_count == before);
    dm_f_lft.enabled = dm_b_lft.enabled = dm_f_rgt.enabled = 1u;
    Test_Control_Tick();
    assert(output_count == before);
    dm_b_rgt.enabled = 1u;
    Test_Control_Tick();
    assert(Fsm_State_Get() == FSM_STATE_WAIT_FRAME);
    assert(output_count == before);
    Test_Fsm_Tick();
    assert(output_count == before);
    Test_Control_Tick();
    assert(output_count == before + 1u && output_mode == LEG_CTRL_MODE_MOVING);
    assert(Fsm_State_Get() == FSM_STATE_BALANCE);
    uint32_t rescue_entries = rescue_reset_count;
    Test_Control_Tick();
    assert(Fsm_State_Get() == FSM_STATE_BALANCE);
    assert(rescue_reset_count == rescue_entries);
    dr16.sw_right = DR16_SW_DOWN;
    Test_Control_Tick();
    assert(Fsm_State_Get() == FSM_STATE_TEST && output_mode == LEG_CTRL_MODE_TEST);
    dr16.sw_right = DR16_SW_MID;
    body.h_tgt = 0.32f;
    Test_Control_Tick();
    assert(Fsm_State_Get() == FSM_STATE_BALANCE);
    Test_Close(body.h_tgt, Leg_Len_Min);
    assert(rescue_reset_count == rescue_entries + 1u);
    Test_Control_Tick();
    assert(Fsm_State_Get() == FSM_STATE_BALANCE);
    imu_world.pitch = 2.0f;
    for (uint32_t ms = 0u; ms < 200u; ms += CTRL_PERIOD_MS) Test_Fsm_Tick();
    assert(Fsm_State_Get() == FSM_STATE_RECOVERY && output_mode == LEG_CTRL_MODE_SELF_RESCUE);
    imu_world.pitch          = 0.0f;
    dr16.sw_left             = DR16_SW_UP;
    uint32_t disables_before = disable_count;
    Test_Control_Tick();
    assert(Fsm_State_Get() == FSM_STATE_DISABLE && output_mode == LEG_CTRL_MODE_NUM);
    assert(active_flag == APP_DISABLE && disable_count == disables_before + 1u);
    uint32_t clears_before = clear_count;
    Test_Control_Tick();
    assert(clear_count == clears_before + 1u);
    assert(active_flag == APP_DISABLE && stop_count && disable_count);
    dr16.sw_right = DR16_SW_DOWN;
    for (uint32_t ms = 0u; ms < 200u; ms += CTRL_PERIOD_MS) Test_Control_Tick();
    dr16.sw_right = DR16_SW_MID;

    dr16.sw_left = DR16_SW_MID;
    Test_Control_Tick();
    online = false;
    Test_Control_Tick();
    assert(App_Safety_Latched() && active_flag == APP_DISABLE);
    online = true;
    Test_Control_Tick();
    assert(App_Safety_Latched() && Fsm_State_Get() == FSM_STATE_DISABLE);
    dr16.sw_left = DR16_SW_UP;
    Test_Control_Tick();
    assert(App_Safety_Latched());
    dr16.sw_left = DR16_SW_DOWN;
    Test_Control_Tick();
    assert(!App_Safety_Latched());

    dr16.sw_left   = DR16_SW_MID;
    dr16.sw_right  = DR16_SW_DOWN;
    rescue_entries = rescue_reset_count;
    Test_Control_Tick();
    assert(Fsm_State_Get() == FSM_STATE_WAIT_ENABLE);
    dm_f_lft.enabled = dm_b_lft.enabled = dm_f_rgt.enabled = dm_b_rgt.enabled = 1u;
    Test_Control_Tick();
    assert(Fsm_State_Get() == FSM_STATE_WAIT_FRAME);
    Test_Control_Tick();
    assert(Fsm_State_Get() == FSM_STATE_TEST && output_mode == LEG_CTRL_MODE_TEST);
    assert(rescue_reset_count == rescue_entries);
}

int main(int argc, char **argv)
{
    if (argc > 1 && strcmp(argv[1], "--height-handoff") == 0)
    {
        Test_Running_Targets();
        Test_Fsm();
        assert(critical_depth == 0u);
        puts("Height handoff, FOLLOW target ownership and gyro re-entry PASS");
        return 0;
    }
    Test_Pid_Period();
    Test_Rescue();
    Test_Estimator_Period();
    Test_Running_Targets();
    Test_Fsm();
    assert(critical_depth == 0u);
    puts("Migration global state, 2 ms timing, rescue, handshake, PC disable and safety latch PASS");
    return 0;
}
