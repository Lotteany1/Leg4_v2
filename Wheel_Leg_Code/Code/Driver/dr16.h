/** @file  dr16.h  @brief 遥控接收接口  @version  v3.3  2026-11-08 */
#ifndef DR16_H
#define DR16_H

#include "usart.h"
#include "code_config.h"
#include <stdbool.h>

// HAL库 DR16 句柄
#define DR16_HUART (&huart2)

#define DR16_OFFLINE_MS 100u // 超过该时长没有有效帧视为离线

// DR16 协议常量
#define DR16_FRAME_LEN  18
#define DR16_CH_LIMIT   660  // 通道值合法范围 ±660

// DR16 拨杆位置 (DR16 协议原始编码)
#define DR16_SW_UP   1
#define DR16_SW_MID  3
#define DR16_SW_DOWN 2

// DR16 类型定义
typedef enum
{
    DR16_KEY_W     = 1u << 0,
    DR16_KEY_S     = 1u << 1,
    DR16_KEY_A     = 1u << 2,
    DR16_KEY_D     = 1u << 3,
    DR16_KEY_SHIFT = 1u << 4,
    DR16_KEY_CTRL  = 1u << 5,
    DR16_KEY_Q     = 1u << 6,
    DR16_KEY_E     = 1u << 7,
    DR16_KEY_R     = 1u << 8,
    DR16_KEY_F     = 1u << 9,
    DR16_KEY_G     = 1u << 10,
    DR16_KEY_Z     = 1u << 11,
    DR16_KEY_X     = 1u << 12,
    DR16_KEY_C     = 1u << 13,
    DR16_KEY_V     = 1u << 14,
    DR16_KEY_B     = 1u << 15,
} dr16_key_t;

typedef enum
{
    DR16_MODE_RC = 0,
    DR16_MODE_PC = 1,
} dr16_mode_t;

// DR16 接收统计（独立全局，不放 dr16_t：避免被 Dr16_Clear / dr16 = parsed 覆写清零）
typedef struct
{
    volatile uint32_t frame_cnt;  // 校验通过的有效帧
    volatile uint32_t err_cnt;    // 校验失败丢弃的帧
    volatile uint32_t resync_cnt; // DMA 重对齐次数
    volatile uint32_t ore_cnt;    // 溢出（丢字节）
    volatile uint32_t fe_cnt;     // 帧错误
    volatile uint32_t pe_cnt;     // 校验错误
    volatile uint32_t ne_cnt;     // 噪声错误
    volatile uint32_t irq_cnt;    // ! 临时诊断：UART9 中断进入次数（任何标志）
    volatile uint32_t idle_cnt;   // ! 临时诊断：IDLE 标志命中次数
} dr16_stats_t;

// DR16 数据结构
typedef struct
{
    int16_t ch1; // 通道 -660 ~ 660
    int16_t ch2;
    int16_t ch3;
    int16_t ch4;
    int16_t ch5;     // 滚轮通道 (部分版本固件支持)
    uint8_t sw_left; // 拨杆 1/2/3
    uint8_t sw_right;
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_l;
    uint8_t mouse_r;
    uint16_t key_code; // 键盘位图，配合 dr16_key_t 使用
    volatile bool online;
    volatile uint32_t last_rx_tick;
} dr16_t;

// DR16 实例
extern dr16_t dr16;
extern dr16_stats_t dr16_stats;

// DR16 API
void Dr16_Init(void);
void Dr16_Update(void);
void Dr16_Idle_Isr(UART_HandleTypeDef *huart);
int16_t Dr16_Deadline(int16_t input, uint16_t deadline);
dr16_t Dr16_Snapshot(void);

bool Dr16_Online(void);

////void    Dr16_Led_Show_Status(void);

// 内联键盘查询，例如 Dr16_Key(DR16_KEY_W)
static inline bool Dr16_Key(dr16_key_t key)
{
    return (dr16.key_code & (uint16_t)key) != 0;
}

#define RC_SW_LEFT_UP    (dr16.sw_left == DR16_SW_UP)
#define RC_SW_LEFT_MID   (dr16.sw_left == DR16_SW_MID)
#define RC_SW_LEFT_DOWN  (dr16.sw_left == DR16_SW_DOWN)
#define RC_SW_RIGHT_UP   (dr16.sw_right == DR16_SW_UP)
#define RC_SW_RIGHT_MID  (dr16.sw_right == DR16_SW_MID)
#define RC_SW_RIGHT_DOWN (dr16.sw_right == DR16_SW_DOWN)

#endif
