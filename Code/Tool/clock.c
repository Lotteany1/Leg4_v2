/**
 * @file  clock.c
 * @brief 软件定时器
 * @version  v1.0  2026-09-16
 */
#include "clock.h"
#include "FreeRTOS.h"
#include "task.h"

#define CLOCK_MAX_TIMERS 1

typedef struct
{
    uint8_t *flag;            /* 到期置位标志地址 由调用方清零 */
    TickType_t start_tick;    /* 拍，登记时刻 */
    TickType_t timeout_ticks; /* 拍，超时长度 */
    uint8_t active;           /* 槽位占用 到期不释放 */
} clock_timer_t;

static clock_timer_t clock_timers[CLOCK_MAX_TIMERS];

// 槽位用尽即静默丢弃，调用方需保证不超过 CLOCK_MAX_TIMERS
void Clock_Set_Timeout(uint8_t *flag, uint32_t timeout_ms)
{
    *flag = 0;

    for (int i = 0; i < CLOCK_MAX_TIMERS; i++)
    {
        if (!clock_timers[i].active)
        {
            clock_timers[i].flag          = flag;
            clock_timers[i].start_tick    = xTaskGetTickCount();
            clock_timers[i].timeout_ticks = pdMS_TO_TICKS(timeout_ms);
            clock_timers[i].active        = 1;
            return;
        }
    }
}

// 定时器扫描 到期只置标志不释放槽位 外部清零后下次扫描再次置位
void Clock_Scan(void)
{
    TickType_t now = xTaskGetTickCount();

    for (int i = 0; i < CLOCK_MAX_TIMERS; i++)
    {
        if (clock_timers[i].active)
        {
            if ((now - clock_timers[i].start_tick) >= clock_timers[i].timeout_ticks)
            {
                *clock_timers[i].flag = 1;
            }
        }
    }
}
