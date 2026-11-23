/** @file  app_rc.c  @brief 遥控目标映射  @version  v3.3  2026-11-21 */
#include "app_rc.h"
#include "system_key.h"
#include "code_config.h"
#include "dr16.h"
#include "turn_limit.h"

// ! 摇杆目标：CH2 前后速度，CH3 偏航角速度，CH5 调节高度。
void App_Rc_Update_Targets(body_state_t *body, const dr16_t *rc)
{
    body->vel_tgt     = Dr16_Deadline(rc->ch2, RC_VEL_DEADZONE) / DR16_Max * TURN_LIMIT_VEL_FULL;
    body->omg_yaw_tgt = -Dr16_Deadline(rc->ch3, RC_YAW_DEADZONE) / DR16_Max * TURN_LIMIT_YAW_FULL;

    // 高度从平衡入场值逐拍累加，摇杆回中保持当前目标。
    body->h_tgt -= Dr16_Deadline(rc->ch5, RC_HEIGHT_DEADZONE) / DR16_Max * CTRL_DT * RC_HEIGHT_RATE;
    body->h_tgt = LEG_LEN_CLAMP(body->h_tgt);
}
