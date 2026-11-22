/**
 * @file  tinympc14_port_host.h
 * @brief 主机端口：手动推进毫秒 tick
 * @version  v0.1  2026-10-20
 */

#ifndef TINYMPC14_PORT_HOST_H
#define TINYMPC14_PORT_HOST_H

#include <stdint.h>

void Tinympc14_Port_Host_Set_Tick(uint32_t tick_ms);
void Tinympc14_Port_Host_On_Tick(void (*hook)(void));

#endif
