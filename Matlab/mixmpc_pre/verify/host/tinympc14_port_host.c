/**
 * @file  tinympc14_port_host.c
 * @brief 主机端口：手动推进毫秒 tick
 * @version  v0.1  2026-10-20
 */

#include "tinympc14.h"
#include "tinympc14_port_host.h"

static volatile uint32_t tinympc14_port_tick_ms;
static void (*tinympc14_port_tick_hook)(void);

uint32_t Tinympc14_Port_Tick(void)
{
    if(tinympc14_port_tick_hook)
    {
        void (*hook)(void)=tinympc14_port_tick_hook;
        tinympc14_port_tick_hook=0;
        hook();
    }
    return tinympc14_port_tick_ms;
}

void Tinympc14_Port_Enter_Critical(void)
{
}

void Tinympc14_Port_Exit_Critical(void)
{
}

void Tinympc14_Port_Host_Set_Tick(uint32_t tick_ms)
{
    tinympc14_port_tick_ms = tick_ms;
}
void Tinympc14_Port_Host_On_Tick(void (*hook)(void)) { tinympc14_port_tick_hook=hook; }
