#ifndef TEST_FSM_POSITION_STUB_H
#define TEST_FSM_POSITION_STUB_H

#include "fsm_running.h"
#include "body.h"

#ifndef TEST_FSM_POSITION_WEIGHT
#define TEST_FSM_POSITION_WEIGHT 1.0f // 无量纲 后端独立用例默认满权重，可按实际驻车配置覆盖
#endif

// 后端独立用例直接提供相对误差。运行 FSM 另有真实捕获与保护用例。
fsm_running_position_t Fsm_Running_Position(void)
{
    return (fsm_running_position_t){.error = body.pos, .weight = TEST_FSM_POSITION_WEIGHT};
}

void Fsm_Running_Exit(void)
{
}

fsm_landing_t Fsm_Running_Landing(void)
{
    return (fsm_landing_t){0};
}

#endif
