/** @file  fsm.h  @brief 主控制状态仲裁与出力调度  @version  v4.1  2026-11-21 */
#ifndef FSM_H
#define FSM_H

#include "dr16.h"
#include <stdbool.h>

// 控制状态枚举表，枚举值用于状态观测，与拨杆协议编码及单拍执行序号无关。
typedef enum
{
    FSM_STATE_DISABLE = 0, // 失能：无运行请求或安全锁存；按独立请求清锁存与清错
    FSM_STATE_WAIT_ENABLE, // 使能握手：有运行请求，等待四台关节使能确认
    FSM_STATE_WAIT_FRAME,  // 等帧：等待使能确认后的完整估计帧
    FSM_STATE_RECOVERY,    // 自救：启动、测试退出或倒地后进入，执行扫腿或摆腿
    FSM_STATE_BALANCE,     // 平衡：按译码行为更新运行目标，连续倒地后进入自救
    FSM_STATE_TEST,        // 测试：运行与测试请求通过握手；测试请求撤销先回自救
} fsm_state_t;

// 同一拍的拨杆请求，安全层与状态机共用。
typedef struct
{
    bool run;
    bool test;
    bool gyro;
    bool reset;     // 解除安全锁存
    bool clear_err; // 清四台 DM 故障
    bool set_zero;  // 标定四台 DM 零点
} fsm_request_t;

fsm_request_t Fsm_Read_Request(const dr16_t *rc);
void Fsm_Tick(const dr16_t *rc, const fsm_request_t *req);
fsm_state_t Fsm_State_Get(void);  // 当前控制状态 只读

#endif
