/** @file  monitor.h  @brief 全局性能与健康检测集中出口  @version  v2.3  2026-11-22 */
#ifndef MONITOR_H
#define MONITOR_H

#include "can_bus.h"
#include <stdint.h>

#define MONITOR_CAN_NUM   CAN_BUS_NUM // 路数随 CAN_BUS_NUM
#define MONITOR_AVG_ALPHA 0.001999f   // EMA 系数 时间常数约 1 s

// 计时槽位枚举表
typedef enum
{
    MONITOR_SLOT_TICK = 0, // 0 控制拍循环体
    MONITOR_SLOT_MPC,      // 1 TinyMPC14 一次求解
    MONITOR_SLOT_MAX,      // 2 哨兵 加测量段只加成员与表行
} monitor_slot_t;

// 窗口量结构体 随清账窗口整体清零
typedef struct
{
    float tick_us;        // µs 控制拍循环体本拍耗时
    float tick_us_max;    // µs 控制拍耗时窗口峰值
    float tick_us_avg;    // µs 控制拍耗时指数滑动均值
    float period_us_max;  // µs 相邻控制拍起点间隔峰值 检测忙等与阻塞
    float mpc_us;         // µs TinyMPC14 一次求解本拍耗时
    float mpc_us_max;     // µs 平衡 MPC 耗时窗口峰值
    uint32_t overrun_cnt; // 控制拍掉队丢拍次数
} monitor_win_t;

// 累计计数结构体 只在 Monitor_Init 清零
typedef struct
{
    uint32_t mpc_deadline_misses; // TinyMPC14 后台整拍未赶上下一次截止的次数
    uint32_t leg_clamp_lft_cnt;   // 左腿关节力矩出口钳位
    uint32_t leg_clamp_rgt_cnt;   // 右腿关节力矩出口钳位
    uint32_t dm_disable_fail_cnt; // 失能帧入队失败
} monitor_total_t;

// CAN 每总线性能与健康结构体 下标同 can_bus_handles[] 可直接 watch
typedef struct
{
    float load_us_per_ms;        // µs/ms 窗口内总线占用
    uint32_t tx_frames;          // 窗口内本机发帧数 口径 = 入队成功
    uint32_t rx_frames;          // 窗口内收帧数 滤波器全通 含他机全部流量
    uint32_t tx_full_cnt;        // 发不出去累计 对端不回 ACK 也算
    uint32_t rx_loss_cnt;        // 读不走累计
    uint32_t bus_off_cnt;        // 进 bus-off 累计
    uint32_t bus_off_psr;        // 最近一次 bus-off 中断的协议状态寄存器
    uint32_t bus_off_ecr;        // 同次中断的错误计数寄存器
    uint32_t bo_recover_cnt;     // 退出 bus-off 累计
    uint32_t active_passive_cnt; // active-passive 翻转累计
    uint32_t heartbeat;          // 已完成控制拍数，仅 CAN1 槽使用，每拍加 1
    uint32_t tx_stall_cnt;       // 错误被动且发送队列超时撤销次数
    uint32_t stall_psr;          // 最近一次队列堵塞时协议状态
    uint32_t stall_ecr;          // 同次故障错误计数
    uint8_t state;               // 总线状态 取值同 can_bus_state_t
} monitor_can_t;

extern volatile monitor_win_t monitor_win;
extern volatile monitor_total_t monitor_total;
extern volatile monitor_can_t monitor_can[MONITOR_CAN_NUM];

void Monitor_Init(void);
uint32_t Monitor_Begin(void);
void Monitor_End(monitor_slot_t slot, uint32_t begin);
void Monitor_Win_Clear(void);
void Monitor_Can_Sample(void);

#endif
