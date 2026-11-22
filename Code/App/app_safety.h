/** @file  app_safety.h  @brief 安全锁存与电机使能执行  @version  v3.3  2026-11-21 */
#ifndef APP_SAFETY_H
#define APP_SAFETY_H

#include <stdbool.h>
#include <stdint.h>

#define APP_DISABLE 0u
#define APP_ENABLE  1u

#define APP_ENABLE_RETRY_MS  10u  // ms 使能重试间隔
#define APP_DISABLE_RETRY_MS 100u // ms 失能重发间隔

extern volatile uint8_t active_flag;

void App_Safety_Online_Check(bool control_requested); // 输入本拍运行请求 左杆退出 RC 同拍拉低使能  控制任务每拍直调 排在 Apply_Flag 之前
void App_Safety_Apply_Flag(void);   // 控制任务每拍直调 排在 Online_Check 之后
bool App_Safety_Latched(void);      // 安全门遥测 只读 FSM 每拍查
void App_Safety_Trip(void);         // 控制任务锁存故障并拉低使能，调用方同拍 Apply
void App_Safety_Reset(void);        // 仅 FSM 失能态拨 DOWN 时调用 清锁存

#endif
