/** @file  turn_coordination_limit_test.c  @brief 有限侧倾下的驾驶需求限制验证  @version  v1.0  2026-11-02 */
#include "turn_limit.h"
#include "system_key.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void)
{
    for (int sign = -1; sign <= 1; sign += 2)
    {
        body_state_t state = {0};
        state.vel = state.vel_kalman = 1.0f * sign;
        state.vel_tgt = 2.0f * sign;
        state.omg_yaw_tgt = 3.0f * sign;
        Turn_Limit_Update(&state);
        assert(fabsf(state.vel_tgt - sign) < 1e-5f);
        assert(state.omg_yaw_tgt == 3.0f * sign);
        state.vel_tgt = 2.0f * sign;
        state.ang_roll = 0.11f * sign;
        Turn_Limit_Update(&state);
#if LEGLEN_TURN_ENABLE
        assert(fabsf(state.vel_tgt - 0.5f * sign) < 1e-5f);
        assert(fabsf(state.omg_yaw_tgt - 1.5f * sign) < 1e-5f);
        state.vel_tgt = 2.0f * sign;
        state.omg_yaw_tgt = 3.0f * sign;
        state.ang_roll = 0.15f * sign;
        Turn_Limit_Update(&state);
        assert(state.vel_tgt == 0.0f && state.omg_yaw_tgt == 0.0f);
        state.vel = state.vel_kalman = 0.0f;
        state.omg_yaw_tgt = 3.0f * sign;
        Turn_Limit_Update(&state);
        assert(state.omg_yaw_tgt == 3.0f * sign);
#else
        assert(fabsf(state.vel_tgt - sign) < 1e-5f && state.omg_yaw_tgt == 3.0f * sign);
#endif
    }
    puts("PASS: tilt-based turn demand reduction, both signs, standstill yaw and disabled bypass");
    return 0;
}
