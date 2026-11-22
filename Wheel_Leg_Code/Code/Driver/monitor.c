/** @file  monitor.c  @brief 全局性能与健康检测集中出口  @version  v2.0  2026-10-22 */
#include "monitor.h"
#include "stm32h7xx.h"

#define MONITOR_CAN_WIN_MS          100u  // ms CAN 负载统计窗口
#define MONITOR_CAN_CLASSIC_BASE_US 47.0f // µs 经典帧每帧基础耗时
#define MONITOR_CAN_CLASSIC_BYTE_US 8.0f  // µs 经典帧每字节耗时
#define MONITOR_CAN_FD_BASE_US      34.0f // µs FD+BRS 每帧基础耗时
#define MONITOR_CAN_FD_BYTE_US      2.0f  // µs FD+BRS 每字节耗时

// CAN 负载窗口状态 上一窗口的计数快照与起点
typedef struct
{
    uint32_t tick;
    uint32_t tx_frames;
    uint32_t rx_frames;
    uint32_t tx_bytes;
    uint32_t rx_bytes;
} monitor_can_win_t;

volatile monitor_win_t monitor_win;
volatile monitor_total_t monitor_total;
volatile monitor_can_t monitor_can[MONITOR_CAN_NUM];

static monitor_can_win_t monitor_can_win[MONITOR_CAN_NUM];

static float monitor_us_per_cyc;   // µs 每 CPU 周期
static uint32_t monitor_tick_prev; // 上一控制拍起点周期数
static uint8_t monitor_tick_begun; // 首拍无基准 拍间隔不计
static uint8_t monitor_win_clear;  // 拍尾清窗口请求

// 槽位本拍值表 下标同 monitor_slot_t
static volatile float *const monitor_slot_now[MONITOR_SLOT_MAX] =
    {
        [MONITOR_SLOT_TICK]   = &monitor_win.tick_us,
        [MONITOR_SLOT_MPC]    = &monitor_win.mpc_us,
};

// 槽位峰值表 下标同 monitor_slot_t
static volatile float *const monitor_slot_max[MONITOR_SLOT_MAX] =
    {
        [MONITOR_SLOT_TICK]   = &monitor_win.tick_us_max,
        [MONITOR_SLOT_MPC]    = &monitor_win.mpc_us_max,
};

// 单槽收尾 写本拍值与峰值保持
static void Monitor_Us_Keep(float us, volatile float *now, volatile float *max)
{
    *now = us;
    if (us > *max)
    {
        *max = us;
    }
}

// 开 DWT 周期计数 三个表与窗口基准全清零 上电一次
void Monitor_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    monitor_us_per_cyc = 1e6f / (float)SystemCoreClock;

    monitor_win        = (monitor_win_t){0};
    monitor_total      = (monitor_total_t){0};
    monitor_tick_begun = 0u;
    monitor_win_clear  = 0u;

    for (uint32_t i = 0u; i < MONITOR_CAN_NUM; i++)
    {
        monitor_can[i]     = (monitor_can_t){0};
        monitor_can_win[i] = (monitor_can_win_t){0};
    }
}

// 取控制拍起点周期数 拍间隔与清账都留到拍尾消费
uint32_t Monitor_Begin(void)
{
    return DWT->CYCCNT;
}

// 槽位收尾 写本拍值与峰值 控制拍顺带更新均值与拍间隔 拍尾消费清账
void Monitor_End(monitor_slot_t slot, uint32_t begin)
{
    if ((uint32_t)slot >= (uint32_t)MONITOR_SLOT_MAX)
    {
        return;
    }

    float us = (float)(DWT->CYCCNT - begin) * monitor_us_per_cyc;
    float period_us;

    Monitor_Us_Keep(us, monitor_slot_now[slot], monitor_slot_max[slot]);

    if (slot == MONITOR_SLOT_TICK)
    {
        monitor_win.tick_us_avg += MONITOR_AVG_ALPHA * (us - monitor_win.tick_us_avg);

        if (monitor_tick_begun)
        {
            period_us = (float)(begin - monitor_tick_prev) * monitor_us_per_cyc;

            if (period_us > monitor_win.period_us_max)
            {
                monitor_win.period_us_max = period_us;
            }
        }

        monitor_tick_prev  = begin;
        monitor_tick_begun = 1u;
    }

    if (monitor_win_clear)
    {
        monitor_win_clear = 0u;
        monitor_win       = (monitor_win_t){0};
    }
}

// 请求清账窗口 本拍不动 拍尾整体清零窗口量 累计量不动
void Monitor_Win_Clear(void)
{
    monitor_win_clear = 1u;
}

// CAN 负载窗口折算 遥测任务按固定周期调 首拍只立基准 窗口未满保持上一窗口值
void Monitor_Can_Sample(void)
{
    uint32_t now = HAL_GetTick();

    for (uint32_t i = 0u; i < MONITOR_CAN_NUM; i++)
    {
        monitor_can_win_t *win = &monitor_can_win[i];
        can_bus_raw_t raw;

        Can_Bus_Raw_Count(i, &raw);

        if (win->tick == 0u)
        {
            win->tick      = now;
            win->tx_frames = raw.tx_frames;
            win->rx_frames = raw.rx_frames;
            win->tx_bytes  = raw.tx_bytes;
            win->rx_bytes  = raw.rx_bytes;
            continue;
        }

        uint32_t dt = now - win->tick;

        if (dt < MONITOR_CAN_WIN_MS)
        {
            continue;
        }

        uint32_t n     = (raw.rx_frames - win->rx_frames) + (raw.tx_frames - win->tx_frames);
        uint32_t bytes = (raw.rx_bytes - win->rx_bytes) + (raw.tx_bytes - win->tx_bytes);
        float us;

        if (raw.fd_brs)
        {
            us = MONITOR_CAN_FD_BASE_US * (float)n + MONITOR_CAN_FD_BYTE_US * (float)bytes;
        }
        else
        {
            us = MONITOR_CAN_CLASSIC_BASE_US * (float)n + MONITOR_CAN_CLASSIC_BYTE_US * (float)bytes;
        }

        monitor_can[i].load_us_per_ms = us / (float)dt;
        monitor_can[i].tx_frames      = raw.tx_frames - win->tx_frames;
        monitor_can[i].rx_frames      = raw.rx_frames - win->rx_frames;

        win->tick      = now;
        win->tx_frames = raw.tx_frames;
        win->rx_frames = raw.rx_frames;
        win->tx_bytes  = raw.tx_bytes;
        win->rx_bytes  = raw.rx_bytes;
    }
}
