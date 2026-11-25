/** @file  monitor.h  @brief MPC test monitor counters  @version  v1.1  2026-10-26 */
#ifndef TEST_MONITOR_H
#define TEST_MONITOR_H
#include <stdint.h>
typedef struct
{
    uint32_t mpc_commit_cnt, mpc_fallback_cnt, mpc_nan_cnt, mpc_stale_cnt, mpc_async_fb_cnt;
    uint32_t leglen_fallback_cnt, leglen_nan_cnt, leglen_stale_cnt;
    uint32_t leg_clamp_lft_cnt, leg_clamp_rgt_cnt;
    uint32_t dm_disable_fail_cnt;
} monitor_total_t;
extern volatile monitor_total_t monitor_total;
typedef struct { uint32_t heartbeat; } monitor_can_t;
extern volatile monitor_can_t monitor_can[];
typedef struct { float tick_us, period_us_max; uint32_t overrun_cnt; } monitor_win_t;
extern volatile monitor_win_t monitor_win;
typedef enum { MONITOR_SLOT_TICK, MONITOR_SLOT_MPC, MONITOR_SLOT_LEGLEN, MONITOR_SLOT_MAX } monitor_slot_t;
void Monitor_Init(void);
void Monitor_Can_Sample(void);
uint32_t Monitor_Begin(void);
void Monitor_End(monitor_slot_t slot, uint32_t begin);
void Monitor_Win_Clear(void);
#endif
