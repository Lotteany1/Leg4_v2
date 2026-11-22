/** @file  fsm_behavior_test.c  @brief 主状态机逐拍握手、安全与出力交接回归  @version  v1.3  2026-11-02 */
#include "fsm.h"
#include "air.h"
#include "fsm_running.h"
#include "app_safety.h"
#include "app_self_rescue.h"
#include "app_rc.h"
#include "app_step.h"
#include "app_gyro.h"
#include "air.h"
#include "balance.h"
#include "body.h"
#include "leg.h"
#include "imu_to_world.h"
#include "dm_motor.h"
#include "dji_motor.h"
#include "dr16.h"
#include "imu.h"
#include "monitor.h"
#include "task_sync.h"
#include "distributor.h"
#include "music.h"
#include "turn_limit.h"
#include "code_config.h"
#include "system_key.h"
#include "wrap.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

body_state_t body;
leg_state_t leg_lft, leg_rgt;
imu_world_t imu_world;
dm_motor_t dm_f_lft, dm_b_lft, dm_f_rgt, dm_b_rgt;
dr16_t dr16;
volatile monitor_total_t monitor_total;

static uint32_t test_tick = 100u; // ms 控制拍时钟，与估计帧时间独立
static uint32_t test_frame_tick;  // ms 最近完整估计帧的时间
static uint32_t test_enable_count, test_disable_count;
static uint32_t test_history_invalidations;
static uint32_t test_clear_count, test_window_count;
static uint32_t test_rescue_enters, test_rescue_ticks;
static uint32_t test_running_enters, test_rc_ticks;
static uint32_t test_output_count, test_joint_frames;
static uint32_t test_leg_conversions; // 本拍世界系腿角转换次数，用于观察倒地确认是否调用
static uint8_t test_air;              // 0 落地，1 离地；关闭离地检测时忽略
static bool test_online = true;
static bool test_finished, test_finish_on_tick;
static self_rescue_phase_t test_phase = SR_PHASE_INIT;
static balance_ctrl_mode_t test_output_mode;
static char test_events[64]; // 本拍调用顺序，只记录模块边界
static uint32_t test_event_count;

// 本夹具只检查 FSM 的安全边界；历史内容由执行链集成测试验证。
void Execution_Invalidate(void)
{
    test_history_invalidations++;
}

// 蜂鸣器不参与出力握手，保留真实运行模块所需的外围接口。
void Music_Play(uint8_t idx)
{
    (void)idx;
}

void Music_Stop(void)
{
}

// 记录驱动、入场和目标更新顺序，保留字符串结尾供断言交接后缀。
static void Test_Record(char event)
{
    assert(test_event_count + 1u < sizeof(test_events));
    test_events[test_event_count++] = event;
    test_events[test_event_count]   = '\0';
}

// 浮点断言按目标物理量的绝对误差比较。
static void Test_Close(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.00001f);
}

// HAL 与 RTOS 使用同一个毫秒时钟，估计帧 getter 独立返回帧时间。
uint32_t HAL_GetTick(void)
{
    return test_tick;
}

// RTOS tick 按毫秒返回，与安全模块的 HAL 时钟保持一致。
uint32_t osKernelGetTickCount(void)
{
    return test_tick;
}

// 返回独立设置的完整估计帧时间，用于区分旧帧、同时间帧与新帧。
uint32_t Estimator_Frame_Tick(void)
{
    return test_frame_tick;
}

// 监测窗口只计成功离开 WAIT_FRAME 的次数。
void Monitor_Win_Clear(void)
{
    test_window_count++;
    Test_Record('m');
}

// 遥控与在线状态由各场景直接给定，不引入驱动解析或 ISR。
dr16_t Dr16_Snapshot(void)
{
    return dr16;
}

// 遥控在线判据由场景给定，供真实安全模块汇聚。
bool Dr16_Online(void)
{
    return test_online;
}

// IMU 在线判据与场景的设备在线输入保持一致。
bool Imu_Online(void)
{
    return test_online;
}

// 关节在线判据独立于 enabled 应答，避免使能等待被当成离线。
bool Dm_Motor_Online(void)
{
    return test_online;
}

// 使能命令本身不伪造应答，四台关节确认由用例逐个设置。
HAL_StatusTypeDef Dm_Motor_All_Enable(void)
{
    test_enable_count++;
    test_joint_frames += 4u;
    Test_Record('e');
    return HAL_OK;
}

// 失能应答同步落地，便于后续重新使能时验证握手。
HAL_StatusTypeDef Dm_Motor_All_Disable(void)
{
    test_disable_count++;
    test_joint_frames += 4u;
    dm_f_lft.enabled = dm_b_lft.enabled = dm_f_rgt.enabled = dm_b_rgt.enabled = 0u;
    Test_Record('d');
    return HAL_OK;
}

// 清错占四帧；本拍不得再叠加关节控制帧。
static unsigned test_zero_count;

HAL_StatusTypeDef Dm_Motor_All_Set_Zero(void)
{
    test_zero_count++;
    return HAL_OK;
}

void Dm_Motor_All_Clear_Error(void)
{
    test_clear_count++;
    test_joint_frames += 4u;
    Test_Record('c');
}

// 停轮只记录顺序，不计入关节 CAN 帧预算。
void Dji_Motor_All_Stop(void)
{
    Test_Record('s');
}

// 灯效不参与状态转移，在线检查仍调用真实安全模块。
// 自救入场清完成标志，模拟当前公共接口的生命周期。
void App_Self_Rescue_Enter(void)
{
    test_rescue_enters++;
    test_finished = false;
    test_phase    = SR_PHASE_PREPARE;
    Test_Record('r');
}

// 仅未完成的自救允许推进；用例控制本拍是否交接成功。
void App_Self_Rescue_Tick(void)
{
    assert(!test_finished);
    test_rescue_ticks++;
    Test_Record('t');
    if (test_finish_on_tick && test_phase != SR_PHASE_FAILED)
    {
        test_finished = true;
        test_phase    = SR_PHASE_HANDOFF;
    }
}

// 只读自救完成标志，不推进阶段或清除交接结果。
bool App_Self_Rescue_Finished(void)
{
    return test_finished;
}

// 只读当前自救阶段，供主 FSM 判断最终失败锁定。
self_rescue_phase_t App_Self_Rescue_Get_Phase(void)
{
    return test_phase;
}

// 最终失败使用零出力通道，其余未完成阶段使用普通自救通道。
balance_ctrl_mode_t App_Self_Rescue_Ctrl_Mode(void)
{
    return test_phase == SR_PHASE_FAILED ? LEG_CTRL_MODE_NUM : LEG_CTRL_MODE_SELF_RESCUE;
}

// 真实运行入场必须先锁实测站姿，再复位陀螺上下文。
void App_Gyro_Reset(void)
{
    assert(Fsm_State_Get() == FSM_STATE_BALANCE);
    Test_Close(body.vel_tgt, 0.0f);
    Test_Close(body.omg_yaw_tgt, 0.0f);
    Test_Close(body.leg_len_tgt_lft, 0.23f);
    Test_Close(body.leg_len_tgt_rgt, 0.23f);
    Test_Close(body.h_tgt, 0.20f);
    test_running_enters++;
    Test_Record('i');
}

// 摇杆目标仅在平衡态推进；输出标记用于检查更新先于控制出力。
void App_Rc_Update_Targets(body_state_t *state, const dr16_t *rc)
{
    (void)rc;
    assert(Fsm_State_Get() == FSM_STATE_BALANCE);
    state->vel_tgt     = 2.0f;
    state->omg_yaw_tgt = 1.0f;
    test_rc_ticks++;
    Test_Record('u');
}

// 记录陀螺切入，验证同拍遥控目标已经先行更新。
void App_Gyro_Enter(void)
{
    Test_Close(body.vel_tgt, 2.0f);
    Test_Record('g');
}

// 分别记录陀螺运行与退出推进，验证跟随态仍调用退出更新。
void App_Gyro_Update(bool active)
{
    Test_Record(active ? 'a' : 'o');
}

// 记录目标限幅边界，控制出力必须排在这一步之后。
void Turn_Limit_Update(body_state_t *state)
{
    assert(state == &body);
    Test_Record('l');
}

// 关闭离地检测时，保持真实 Air_Get 恒为零的契约。
uint8_t Air_Get(void)
{
#if BALANCE_USE_AIR_CHECK
    return test_air;
#else
    return 0u;
#endif
}

volatile air_status_t air_status;

void Air_Set_Landing(uint8_t active)
{
    (void)active;
}

// 倒地检测入口使用真实算法，这里只提供世界系角度变换。
float Imu_To_World_Leg_Ang(float ang)
{
    test_leg_conversions++;
    return Wrap_Pi(ang + imu_world.pitch);
}

// 每拍记录唯一出力与四台关节控制帧，平衡出力必须已经完成目标更新。
void Balance_Control(balance_ctrl_mode_t mode)
{
    if (Balance_Mode_Grounded(mode) || mode == LEG_CTRL_MODE_AIR)
    {
        assert(Fsm_State_Get() == FSM_STATE_BALANCE);
        assert(body.vel_tgt > 0.0f && body.vel_tgt <= 2.0f);
        assert(test_event_count != 0u && test_events[test_event_count - 1u] == 'l');
    }
    test_output_count++;
    test_joint_frames += 4u;
    test_output_mode = mode;
    Test_Record('b');
}

// 每拍遵循控制任务的安全检查、使能执行、FSM 顺序；帧是否更新由用例选择。
static void Test_Beat(bool fresh_frame)
{
    test_tick += CTRL_PERIOD_MS;
    if (fresh_frame)
        test_frame_tick = test_tick;
    test_event_count     = 0u;
    test_events[0]       = '\0';
    test_output_count    = 0u;
    test_joint_frames    = 0u;
    test_leg_conversions = 0u;

    const fsm_request_t req = Fsm_Read_Request(&dr16);
    App_Safety_Online_Check(req.run);
    App_Safety_Apply_Flag();
    Fsm_Tick(&dr16, &req);

    assert(test_output_count <= 1u);
    assert(test_joint_frames <= 8u);
}

// 每个场景在独立进程运行，从上电失能态设置实测站姿。
static void Test_Init(void)
{
    dr16.sw_left  = DR16_SW_DOWN;
    dr16.sw_right = DR16_SW_MID;
    leg_lft.len   = 0.22f;
    leg_rgt.len   = 0.24f;
    body.h_body   = 0.20f;
    body.ang_yaw  = 0.4f;
    Test_Beat(true);
    assert(Fsm_State_Get() == FSM_STATE_DISABLE);
    assert(test_enable_count == 0u && active_flag == APP_DISABLE);
}

// 走到使能确认后的等待帧状态，检查请求与实际发帧隔一拍。
static void Test_Wait_Frame(void)
{
    uint32_t enable_before = test_enable_count;

    dr16.sw_left = DR16_SW_MID;
    Test_Beat(true);
    assert(Fsm_State_Get() == FSM_STATE_WAIT_ENABLE);
    assert(active_flag == APP_ENABLE && test_output_count == 0u);
    assert(test_enable_count == enable_before);
    Test_Beat(true);
    assert(test_enable_count == enable_before + 1u);
    assert(Fsm_State_Get() == FSM_STATE_WAIT_ENABLE && test_output_count == 0u);

    dm_f_lft.enabled = dm_b_lft.enabled = dm_f_rgt.enabled = 1u;
    Test_Beat(true);
    assert(Fsm_State_Get() == FSM_STATE_WAIT_ENABLE && test_output_count == 0u);
    dm_b_rgt.enabled = 1u;
    Test_Beat(true);
    assert(Fsm_State_Get() == FSM_STATE_WAIT_FRAME && test_output_count == 0u);
}

// 新完整估计帧才允许进入自救，并且该拍只推进一次自救。
static void Test_Recovery(void)
{
    Test_Wait_Frame();
    Test_Beat(true);
    assert(Fsm_State_Get() == FSM_STATE_RECOVERY);
    assert(test_rescue_enters == 1u && test_rescue_ticks == 1u);
    assert(test_window_count == 1u && test_output_count == 1u);
    assert(test_output_mode == LEG_CTRL_MODE_SELF_RESCUE);
}

// 自救完成当拍必须先平衡入场，再更新目标并出力。
static void Test_Balance(void)
{
    Test_Recovery();
    test_finish_on_tick = true;
    Test_Beat(true);
    test_finish_on_tick = false;
    assert(Fsm_State_Get() == FSM_STATE_BALANCE);
    assert(test_running_enters == 1u && test_rc_ticks == 1u);
    assert(strcmp(test_events, "tiuolb") == 0);
}

// 估计帧早于或等于使能确认时刻均不能放行；成功进入 TEST 才清窗口。
static void Test_Handshake(void)
{
    uint32_t enable_tick;

    Test_Init();
    dr16.sw_right = DR16_SW_DOWN;
    Test_Wait_Frame();
    enable_tick     = test_tick;
    test_frame_tick = enable_tick - CTRL_PERIOD_MS;
    Test_Beat(false);
    assert(Fsm_State_Get() == FSM_STATE_WAIT_FRAME && test_output_count == 0u);
    assert(test_window_count == 0u);
    test_frame_tick = enable_tick;
    Test_Beat(false);
    assert(Fsm_State_Get() == FSM_STATE_WAIT_FRAME && test_output_count == 0u);
    assert(test_window_count == 0u);
    Test_Beat(true);
    assert(Fsm_State_Get() == FSM_STATE_TEST && test_output_mode == LEG_CTRL_MODE_TEST);
    assert(test_window_count == 1u && test_rescue_enters == 0u);
    Test_Beat(true);
    assert(test_window_count == 1u && test_rescue_enters == 0u);
}

// 已完成自救不应再推进，直接平衡入场；陀螺切入必须在 RC 更新之后。
static void Test_Handoff(void)
{
    uint32_t rescue_ticks, running_enters;

    Test_Init();
    Test_Balance();
    running_enters = test_running_enters;
    Test_Beat(true);
    assert(test_running_enters == running_enters);
    assert(test_leg_conversions == 2u);
    dr16.sw_right = DR16_SW_DOWN;
    Test_Beat(true);
    assert(Fsm_State_Get() == FSM_STATE_TEST && test_leg_conversions == 0u);
    dr16.sw_right = DR16_SW_UP;
    Test_Beat(true);
    assert(Fsm_State_Get() == FSM_STATE_RECOVERY && test_rescue_enters == 2u);
    assert(test_running_enters == running_enters && test_leg_conversions == 0u);
    rescue_ticks  = test_rescue_ticks;
    test_finished = true;
    test_phase    = SR_PHASE_HANDOFF;
    test_air      = 1u;
    Test_Beat(true);
    assert(test_rescue_ticks == rescue_ticks);
    assert(test_running_enters == running_enters + 1u);
    assert(strcmp(test_events, "iugalb") == 0);
#if BALANCE_USE_AIR_CHECK
    assert(test_output_mode == LEG_CTRL_MODE_AIR);
    test_air = 0u;
    Test_Beat(true);
    assert(test_output_mode == LEG_CTRL_MODE_MOVING && Fsm_Running_Landing().active && Fsm_Running_Landing().elapsed_ms == 0u);
    dr16.sw_right = DR16_SW_DOWN;
    Test_Beat(true);
    assert(Fsm_State_Get() == FSM_STATE_TEST && test_output_mode == LEG_CTRL_MODE_TEST);
    assert(Fsm_Running_Landing().spring_scale == 1.0f);
#else
    assert(test_output_mode == LEG_CTRL_MODE_MOVING);
#endif
}

// 新失能拍发零，下一拍只清错；右 DOWN 保持时重复清错。
static void Test_Disable(void)
{
    uint32_t clears, windows;

    Test_Init();
    Test_Balance();
#if BALANCE_USE_AIR_CHECK
    test_air = 1u;
    Test_Beat(true);
    test_air = 0u;
    Test_Beat(true);
    assert(test_output_mode == LEG_CTRL_MODE_MOVING && Fsm_Running_Landing().active && Fsm_Running_Landing().elapsed_ms == 0u);
#endif
    clears       = test_clear_count;
    dr16.sw_left = DR16_SW_UP;
    Test_Beat(true);
    assert(Fsm_State_Get() == FSM_STATE_DISABLE && active_flag == APP_DISABLE);
    assert(test_output_count == 1u && test_output_mode == LEG_CTRL_MODE_NUM);
    assert(Fsm_Running_Landing().spring_scale == 1.0f);
    assert(test_clear_count == clears && test_leg_conversions == 0u);
    assert(strstr(test_events, "dsb") != NULL);
    Test_Beat(true);
    assert(test_clear_count == clears + 1u && test_output_count == 0u);
    Test_Beat(true);
    assert(test_clear_count == clears + 1u && test_output_count == 1u);
    dr16.sw_right = DR16_SW_DOWN;
    Test_Beat(true);
    assert(test_clear_count == clears + 2u && test_output_count == 0u);
    Test_Beat(true);
    assert(test_clear_count == clears + 3u && test_output_count == 0u);

    Test_Wait_Frame();
    windows      = test_window_count;
    dr16.sw_left = DR16_SW_DOWN;
    Test_Beat(true);
    assert(Fsm_State_Get() == FSM_STATE_DISABLE && test_window_count == windows);
}

// 离线恢复不会自动解除锁存；左杆 UP 仍失能，只有 DOWN 清锁存。
static void Test_Safety(void)
{
    Test_Init();
    Test_Balance();
    test_online = false;
    Test_Beat(true);
    assert(App_Safety_Latched() && active_flag == APP_DISABLE);
    assert(test_history_invalidations > 0u);
    assert(Fsm_State_Get() == FSM_STATE_DISABLE && test_leg_conversions == 0u);
    test_online = true;
    Test_Beat(true);
    assert(App_Safety_Latched() && Fsm_State_Get() == FSM_STATE_DISABLE);
    dr16.sw_left = DR16_SW_UP;
    Test_Beat(true);
    assert(App_Safety_Latched());
    dr16.sw_left = DR16_SW_DOWN;
    Test_Beat(true);
    assert(!App_Safety_Latched() && Fsm_State_Get() == FSM_STATE_DISABLE);
    Test_Wait_Frame();
}

// 正常自救允许切 TEST，最终失败保持零出力直到失能再使能。
static void Test_Failed(void)
{
    uint32_t rescue_ticks, running_enters;

    Test_Init();
    Test_Recovery();
    dr16.sw_right = DR16_SW_DOWN;
    rescue_ticks  = test_rescue_ticks;
    Test_Beat(true);
    assert(Fsm_State_Get() == FSM_STATE_TEST && test_rescue_ticks == rescue_ticks);
    dr16.sw_right = DR16_SW_MID;
    Test_Beat(true);
    assert(Fsm_State_Get() == FSM_STATE_RECOVERY && test_rescue_enters == 2u);
    test_phase     = SR_PHASE_FAILED;
    dr16.sw_right  = DR16_SW_DOWN;
    running_enters = test_running_enters;
    Test_Beat(true);
    assert(Fsm_State_Get() == FSM_STATE_RECOVERY && test_output_mode == LEG_CTRL_MODE_NUM);
    assert(test_running_enters == running_enters && test_leg_conversions == 0u);
    Test_Beat(true);
    assert(Fsm_State_Get() == FSM_STATE_RECOVERY);
    dr16.sw_left = DR16_SW_DOWN;
    Test_Beat(true);
    assert(Fsm_State_Get() == FSM_STATE_DISABLE);
    dr16.sw_right = DR16_SW_MID;
    Test_Wait_Frame();
    Test_Beat(true);
    assert(Fsm_State_Get() == FSM_STATE_RECOVERY && test_rescue_enters == 3u);
    assert(test_phase == SR_PHASE_PREPARE);
}

// 倒地必须连续满 200 ms，回窗后重新计时；右 DOWN 优先于倒地确认。
static void Test_Fall(void)
{
    Test_Init();
    Test_Balance();
    leg_lft.ang = 1.0f;
    for (uint32_t ms = 0u; ms < 200u - CTRL_PERIOD_MS; ms += CTRL_PERIOD_MS)
    {
        Test_Beat(true);
        assert(Fsm_State_Get() == FSM_STATE_BALANCE && test_leg_conversions == 1u);
    }
    leg_lft.ang = 0.0f;
    Test_Beat(true);
    assert(Fsm_State_Get() == FSM_STATE_BALANCE && test_leg_conversions == 2u);
    leg_lft.ang = 1.0f;
    for (uint32_t ms = 0u; ms < 200u - CTRL_PERIOD_MS; ms += CTRL_PERIOD_MS)
    {
        Test_Beat(true);
        assert(Fsm_State_Get() == FSM_STATE_BALANCE && test_leg_conversions == 1u);
    }
    Test_Beat(true);
    assert(Fsm_State_Get() == FSM_STATE_RECOVERY && test_leg_conversions == 1u);
    assert(test_rescue_enters == 2u && test_output_mode == LEG_CTRL_MODE_SELF_RESCUE);
    Test_Beat(true);
    assert(test_leg_conversions == 0u);

    leg_lft.ang   = 0.0f;
    test_finished = true;
    test_phase    = SR_PHASE_HANDOFF;
    Test_Beat(true);
    leg_lft.ang   = 1.0f;
    dr16.sw_right = DR16_SW_DOWN;
    Test_Beat(true);
    assert(Fsm_State_Get() == FSM_STATE_TEST && test_leg_conversions == 0u);
    for (uint32_t ms = 0u; ms < 200u; ms += CTRL_PERIOD_MS)
    {
        Test_Beat(true);
        assert(Fsm_State_Get() == FSM_STATE_TEST && test_leg_conversions == 0u);
    }
}

// 每组拨杆组合从平衡态开始，验证运行请求、模式选择、失能清错与安全锁存各自生效的条件。
static void Test_Mapping(uint8_t sw_left, uint8_t sw_right)
{
    uint32_t clears, rescue_enters, running_enters;

    Test_Init();
    Test_Balance();
    clears         = test_clear_count;
    rescue_enters  = test_rescue_enters;
    running_enters = test_running_enters;
    dr16.sw_left   = sw_left;
    dr16.sw_right  = sw_right;
    const fsm_request_t req = Fsm_Read_Request(&dr16);
    assert(req.run == (sw_left == DR16_SW_MID));
    assert(req.test == (req.run && sw_right == DR16_SW_DOWN));
    assert(req.gyro == (req.run && sw_right == DR16_SW_UP));
    assert(req.reset == (sw_left == DR16_SW_DOWN));
    assert(req.clear_err == (sw_right == DR16_SW_DOWN));
    assert(req.set_zero == (req.reset && sw_right == DR16_SW_UP));
    Test_Beat(true);
    assert(test_clear_count == clears);

    if (sw_left == DR16_SW_MID)
    {
        assert(active_flag == APP_ENABLE);
        assert(test_running_enters == running_enters);
        if (sw_right == DR16_SW_DOWN)
        {
            assert(Fsm_State_Get() == FSM_STATE_TEST && test_output_mode == LEG_CTRL_MODE_TEST);
            assert(test_leg_conversions == 0u && test_rc_ticks == 1u);
        }
        else
        {
            assert(Fsm_State_Get() == FSM_STATE_BALANCE);
            assert(strcmp(test_events, sw_right == DR16_SW_UP ? "ugalb" : "uolb") == 0);
        }
        Test_Beat(true);
        assert(test_clear_count == clears);
        if (sw_right != DR16_SW_DOWN)
        {
            assert(strcmp(test_events, sw_right == DR16_SW_UP ? "ualb" : "uolb") == 0);

            // 测试退出包含未知右档位，必须先自救，再按当前档位交接平衡行为。
            dr16.sw_right = DR16_SW_DOWN;
            Test_Beat(true);
            assert(Fsm_State_Get() == FSM_STATE_TEST && test_clear_count == clears);
            dr16.sw_right = sw_right;
            Test_Beat(true);
            assert(Fsm_State_Get() == FSM_STATE_RECOVERY && test_rescue_enters == rescue_enters + 1u);
            assert(test_running_enters == running_enters);
            test_finished = true;
            test_phase    = SR_PHASE_HANDOFF;
            Test_Beat(true);
            assert(Fsm_State_Get() == FSM_STATE_BALANCE && test_running_enters == running_enters + 1u);
            assert(strcmp(test_events, sw_right == DR16_SW_UP ? "iugalb" : "iuolb") == 0);
        }
        assert(test_clear_count == clears);
    }
    else
    {
        assert(Fsm_State_Get() == FSM_STATE_DISABLE && active_flag == APP_DISABLE);
        assert(test_output_count == 1u && test_output_mode == LEG_CTRL_MODE_NUM);
        assert(test_running_enters == running_enters && test_rescue_enters == rescue_enters);
        assert(test_leg_conversions == 0u && test_rc_ticks == 1u);

        // 失能首拍不清错，下一拍仅清一次；持续清错只由右下档位请求。
        Test_Beat(true);
        assert(test_clear_count == clears + 1u && test_output_count == 0u);
        Test_Beat(true);
        assert(test_clear_count == clears + (sw_right == DR16_SW_DOWN ? 2u : 1u));
        bool calibrate = sw_left == DR16_SW_DOWN && sw_right == DR16_SW_UP;

        assert(test_output_count == ((sw_right == DR16_SW_DOWN || calibrate) ? 0u : 1u));
        assert(test_zero_count == (calibrate ? 1u : 0u));
        if (calibrate)
        {
            Test_Beat(true);
            assert(test_zero_count == 1u);
            dr16.sw_right = DR16_SW_MID;
            Test_Beat(true);
            dr16.sw_right = DR16_SW_UP;
            Test_Beat(true);
            assert(test_zero_count == 2u);
            dr16.sw_left = DR16_SW_MID;
            Test_Beat(true);
            assert(Fsm_State_Get() == FSM_STATE_WAIT_ENABLE);
            dr16.sw_left = DR16_SW_DOWN;
            Test_Beat(true);
            assert(test_output_count == 1u && test_zero_count == 2u);
            Test_Beat(true);
            assert(test_output_count == 0u && test_zero_count == 2u);
            Test_Beat(true);
            assert(test_output_count == 0u && test_zero_count == 3u);
        }
    }

    // 离线条件与拨杆请求分开仲裁：仅左下清锁，左上、左中及未知档位均保持锁存。
    test_online = false;
    Test_Beat(true);
    assert(Fsm_State_Get() == FSM_STATE_DISABLE && active_flag == APP_DISABLE);
    assert(App_Safety_Latched() == (sw_left != DR16_SW_DOWN));
    test_online = true;
    Test_Beat(true);
    assert(Fsm_State_Get() == FSM_STATE_DISABLE && active_flag == APP_DISABLE);
    assert(App_Safety_Latched() == (sw_left != DR16_SW_DOWN));
}

#if STEP_ENABLE
static bool step_owned, step_fault;

bool Slip_Prediction_Expired(void) { return false; }
bool App_Gyro_Running(void) { return false; }
bool App_Step_Owns_Output(void) { return step_owned; }
bool App_Step_Fault(void) { return step_fault; }
void App_Step_Exit(step_reason_t reason) { (void)reason; step_owned = false; }
void App_Step_Status(step_status_t *status) { *status = (step_status_t){0}; }
bool App_Step_Update(bool follow, bool air, float intent, float yaw)
{
    (void)follow;
    (void)air;
    (void)intent;
    (void)yaw;
    return step_owned;
}

// 保持右下：台阶占用挡住测试，倒地仍可接管；台阶更新报错则本拍停机。
static void Test_Step(bool fault)
{
    Test_Init();
    Test_Balance();
    step_owned = true;
    dr16.sw_right = DR16_SW_DOWN;
    Test_Beat(true);
    assert(Fsm_State_Get() == FSM_STATE_BALANCE && test_output_mode == LEG_CTRL_MODE_STEP);
    if (fault)
    {
        step_fault = true;
        Test_Beat(true);
        assert(Fsm_State_Get() == FSM_STATE_DISABLE && App_Safety_Latched());
        assert(test_output_count == 1u && test_output_mode == LEG_CTRL_MODE_NUM);
        return;
    }
    leg_lft.ang = 1.0f;
    for (uint32_t ms = 0u; ms < 200u; ms += CTRL_PERIOD_MS)
        Test_Beat(true);
    assert(Fsm_State_Get() == FSM_STATE_RECOVERY && test_output_mode == LEG_CTRL_MODE_SELF_RESCUE);
}
#endif

// 单场景单进程保证真实模块 static 状态从上电初值开始，不增加产品复位接口。
int main(int argc, char **argv)
{
#ifdef _MSC_VER
    // 断言仍非零退出并输出证据，避免 WER 对每个失败场景弹窗/等待。
    _set_abort_behavior(0, _CALL_REPORTFAULT);
#endif
    assert(argc == 2 || argc == 4);
#if STEP_ENABLE
    if (strcmp(argv[1], "step_fall") == 0 || strcmp(argv[1], "step_fault") == 0)
    {
        Test_Step(strcmp(argv[1], "step_fault") == 0);
        return 0;
    }
#endif
    if (strcmp(argv[1], "handshake") == 0)
        Test_Handshake();
    else if (strcmp(argv[1], "handoff") == 0)
        Test_Handoff();
    else if (strcmp(argv[1], "disable") == 0)
        Test_Disable();
    else if (strcmp(argv[1], "safety") == 0)
        Test_Safety();
    else if (strcmp(argv[1], "failed") == 0)
        Test_Failed();
    else if (strcmp(argv[1], "fall") == 0)
        Test_Fall();
    else if (strcmp(argv[1], "mapping") == 0)
    {
        assert(argc == 4);
        Test_Mapping((uint8_t)(argv[2][0] - '0'), (uint8_t)(argv[3][0] - '0'));
        printf("FSM mapping LEFT=%s RIGHT=%s PASS\n", argv[2], argv[3]);
        return 0;
    }
    else
        return 2;
    printf("FSM %s PASS\n", argv[1]);
    return 0;
}
