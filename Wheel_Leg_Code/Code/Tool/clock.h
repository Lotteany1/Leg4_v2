/**
 * @file  clock.h
 * @brief 软件定时器头文件
 * @version  v1.0  2026-09-16
 */
#ifndef CLOCK_H
#define CLOCK_H

#include <stdint.h>

void Clock_Set_Timeout(uint8_t *flag, uint32_t timeout_ms); /* 登记一个定时器 到期置 flag 槽位用尽静默丢弃 */
void Clock_Scan(void);                                      /* 周期扫描 到期槽位置 flag 不清槽位 外部清零后再次触发 */

#endif /* CLOCK_H */
