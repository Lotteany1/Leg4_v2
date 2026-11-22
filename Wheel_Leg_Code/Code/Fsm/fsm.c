/** @file  fsm.c  @brief 主控制状态仲裁与出力调度  @version  v4.3  2026-11-21 */
#include "fsm.h"
#include "system_key.h"
#include "air.h"
#include "fsm_running.h"
#include "app_safety.h"
#include "app_step.h"
#include "code_config.h"
#include "app_self_rescue.h"
#include "balance.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "dr16.h"
#include "monitor.h"
#include "slip.h"
#include "task_sync.h"

static volatile fsm_state_t fsm_state = FSM_STATE_DISABLE; // 当前控制状态
static bool fsm_clear_due;
static bool fsm_zero_attempted; // 本次保持已尝试发送置零命令
static uint32_t fsm_enable_tick; // ms 四台关节使能确认时刻

// ! 拨杆请求：左中允许运行，右上陀螺，右中跟随，右下测试。左下右上置零。
fsm_request_t Fsm_Read_Request(const dr16_t *rc)
{
    const bool run = MOTOR_OUTPUT_ENABLE && rc->sw_left == DR16_SW_MID;
    return (fsm_request_t){
        .run       = run,
        .test      = run && rc->sw_right == DR16_SW_DOWN,
        .gyro      = run && rc->sw_right == DR16_SW_UP,
        .reset     = rc->sw_left == DR16_SW_DOWN,
        .clear_err = rc->sw_right == DR16_SW_DOWN,
        .set_zero  = rc->sw_left == DR16_SW_DOWN && rc->sw_right == DR16_SW_UP,
    };
}

// 四台关节电机使能确认
static bool Fsm_All_Dm_Enabled(void)
{
    return dm_f_lft.enabled && dm_b_lft.enabled && dm_f_rgt.enabled && dm_b_rgt.enabled;
}

// 换态执行一次入场 等帧起点锁在使能确认拍
static void Fsm_Enter(fsm_state_t next)
{
    if (next == fsm_state)
        return;
    if (fsm_state == FSM_STATE_WAIT_FRAME && next != FSM_STATE_DISABLE)
        Monitor_Win_Clear();

    if (fsm_state == FSM_STATE_BALANCE)
        Fsm_Running_Exit();

    fsm_state = next;
    switch (next)
    {
        case FSM_STATE_DISABLE: // 停轮 清错留到下一拍
            Dji_Motor_All_Stop();
            fsm_clear_due = true;
            break;

        case FSM_STATE_WAIT_ENABLE: // 请求关节使能
            fsm_zero_attempted = false;
            // 本拍安全层已经执行过，下拍才向四台 DM 发使能。
            active_flag = APP_ENABLE;
            break;

        case FSM_STATE_WAIT_FRAME: // 锁使能确认时刻
            fsm_enable_tick = osKernelGetTickCount();
            break;

        case FSM_STATE_RECOVERY: // 自救入场锁方向
            App_Self_Rescue_Enter();
            break;

        case FSM_STATE_BALANCE: // 平衡入场锁目标
            Fsm_Running_Enter();
            break;

        case FSM_STATE_TEST: // 测试入场无附加动作
            break;
    }
}

// 失能首拍发零出力，后续清错或置零各占一拍。
static void Fsm_Disabled_Tick(const fsm_request_t *req, bool entering)
{
    // 调试时每个控制拍都向四台失能关节要反馈，拨杆维护请求不能占掉这一拍。
    if (!MOTOR_OUTPUT_ENABLE)
    {
        Dm_Send_Zero_All();
        return;
    }
    if (!req->set_zero) fsm_zero_attempted = false;
    if (req->reset) App_Safety_Reset();
    if (entering)
    {
        Balance_Control(LEG_CTRL_MODE_NUM);
        return;
    }
    if (fsm_clear_due || req->clear_err)
    {
        Dm_Motor_All_Clear_Error();
        fsm_clear_due = false;
        return;
    }
    if (req->set_zero && !fsm_zero_attempted)
    {
        // 每次保持只尝试一次；松开组合后才允许再发。
        fsm_zero_attempted = true;
        //Dm_Motor_All_Set_Zero();
        return;
    }
    Balance_Control(LEG_CTRL_MODE_NUM);
}

// ! 单拍仲裁：先按请求和故障决定去哪，再由选中的状态接管本拍。
void Fsm_Tick(const dr16_t *rc, const fsm_request_t *req)
{
    const fsm_state_t prev = fsm_state;

    // 仲裁转移并执行入场：无运行请求或安全锁存时优先失能。
    // 每拍只仲裁一次，不连跳使能与等帧阶段；测试请求仍需先通过握手。
    if (!req->run || App_Safety_Latched())
    {
        Fsm_Enter(FSM_STATE_DISABLE);
    }
    else
    {
        switch (prev)
        {
            case FSM_STATE_DISABLE: // 有运行请求且无安全锁存，进入使能握手，本拍仍不出力
                Fsm_Enter(FSM_STATE_WAIT_ENABLE);
                break;

            case FSM_STATE_WAIT_ENABLE: // 四台关节确认后锁定等待新帧的起点
                if (Fsm_All_Dm_Enabled())
                    Fsm_Enter(FSM_STATE_WAIT_FRAME);
                break;

            case FSM_STATE_WAIT_FRAME: // 新帧严格更新后，按请求进入测试或自救
                if ((int32_t)(Estimator_Frame_Tick() - fsm_enable_tick) > 0)
                    Fsm_Enter(req->test ? FSM_STATE_TEST : FSM_STATE_RECOVERY);
                break;

            case FSM_STATE_RECOVERY: // 最终失败等待失能；正常自救允许测试请求切换
                if (App_Self_Rescue_Get_Phase() != SR_PHASE_FAILED && req->test)
                    Fsm_Enter(FSM_STATE_TEST);
                break;

            case FSM_STATE_BALANCE: // 测试请求优先，其余平衡拍只推进一次倒地确认
                if (req->test
#if STEP_ENABLE
                    && !App_Step_Owns_Output()
#endif
                )
                    Fsm_Enter(FSM_STATE_TEST);
                else if (Fsm_Running_Fall_Update())
                    Fsm_Enter(FSM_STATE_RECOVERY);
                break;

            case FSM_STATE_TEST: // 测试请求撤销后先进入自救
                if (!req->test)
                    Fsm_Enter(FSM_STATE_RECOVERY);
                break;

            default: // 未知状态回失能
                Fsm_Enter(FSM_STATE_DISABLE);
                break;
        }
    }

    // 执行失能、等待或自救动作；等待拍与清错拍在此返回，不进入本拍出力。
#if STEP_ENABLE
    if (fsm_state != FSM_STATE_BALANCE) App_Step_Exit(fsm_state == FSM_STATE_DISABLE ? STEP_REASON_DISABLED : STEP_REASON_MODE);
#endif
    switch (fsm_state)
    {
        case FSM_STATE_DISABLE: // 刚失能时先零出力，后续处理维护请求
            Fsm_Disabled_Tick(req, prev != FSM_STATE_DISABLE);
            return;

        case FSM_STATE_WAIT_ENABLE: // 等四台关节确认，本拍无控制输出
        case FSM_STATE_WAIT_FRAME:  // 等确认后的新帧，本拍无控制输出
            return;

        case FSM_STATE_RECOVERY: // 自救完成当拍接平衡
            // 拍初还没完成，就推进一步。
            if (!App_Self_Rescue_Finished())
                App_Self_Rescue_Tick();

            // 推进后仍未完成，本拍由自救出力。
            if (!App_Self_Rescue_Finished())
            {
                Balance_Control(App_Self_Rescue_Ctrl_Mode());
                return;
            }
            // 已经完成，本拍直接更新平衡目标并出力。
            Fsm_Enter(FSM_STATE_BALANCE);
            break;

        case FSM_STATE_BALANCE: // 当前已经在平衡，继续下面的目标更新
            break;

        case FSM_STATE_TEST: // 测试直接接管本拍输出
            Balance_Control(LEG_CTRL_MODE_TEST);
            return;

        default: // 异常状态零出力
            Balance_Control(LEG_CTRL_MODE_NUM);
            return;
    }

    bool gyro = req->gyro;
#if STEP_ENABLE
    // 台阶占用挡住了测试请求时，右下仍沿用陀螺目标。
    gyro = gyro || req->test;
#endif
    balance_ctrl_mode_t mode = Fsm_Running_Tick(gyro ? FSM_RUNNING_BEHAVIOR_GYRO : FSM_RUNNING_BEHAVIOR_FOLLOW, rc);
#if STEP_ENABLE
    // 更新台阶动作可能刚触发故障，这一拍就失能并走零出力。
    if (App_Step_Fault())
    {
        App_Safety_Trip();
        App_Safety_Apply_Flag();
        Fsm_Enter(FSM_STATE_DISABLE);
        Balance_Control(LEG_CTRL_MODE_NUM);
        return;
    }
#endif
    Balance_Control(mode);
}

// 当前统一状态只读
fsm_state_t Fsm_State_Get(void)
{
    return fsm_state;
}
