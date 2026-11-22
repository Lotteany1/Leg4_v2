/** @file  app_rc.h  @brief 遥控目标映射  @version  v3.1  2026-11-21 */
#ifndef APP_RC_H
#define APP_RC_H

#include "body.h"
#include "dr16.h"

// 遥控通道死区（计数）及高度变化率
#define RC_VEL_DEADZONE    20
#define RC_YAW_DEADZONE    25
#define RC_HEIGHT_DEADZONE 20
#define RC_HEIGHT_RATE     0.3f // m/s 高度滚轮满量程

// 摇杆映射目标量 高度与腿长按步长累加 起点由 Fsm_Running_Enter 给
void App_Rc_Update_Targets(body_state_t *body, const dr16_t *rc);

#endif
