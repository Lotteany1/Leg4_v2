/** @file  can_bus.h  @brief CAN 总线帧收发与恢复代次接口  @version  v3.3  2026-11-01 */
#ifndef CAN_BUS_H
#define CAN_BUS_H
#include "fdcan.h"
#include <stdbool.h>

//! --- CAN 总线参数配置区 --- //
#define CAN_BUS_NUM       3
#define CAN_BUS_ROUTE_MAX 8
//! ------------------------- //
// CAN 总线回调函数类型定义    ctx 是上下文 dlc 是实际字节数
typedef void (*can_parse_fn_t)(void *ctx, uint32_t can_id, const uint8_t *data, uint8_t dlc);

// CAN 总线核心 API
// 先注册后启动：Dji_Motor_Init()/Dm_Motor_Init() 必须早于 Can_Bus_Init()
void Can_Bus_Init(void);

HAL_StatusTypeDef Can_Bus_Send(FDCAN_HandleTypeDef *hfdcan, uint32_t can_id, const uint8_t *data, uint8_t len);

bool Can_Bus_Tx_Ready(FDCAN_HandleTypeDef *hfdcan);

bool Can_Bus_Register(FDCAN_HandleTypeDef *hfdcan, uint32_t can_id, can_parse_fn_t fn, void *ctx);

// CAN 总线模式枚举表
typedef enum
{
    CAN_BUS_MODE_CLASSIC = 0, // 经典 CAN ≤8 字节
    CAN_BUS_MODE_FD,          // CAN FD   恒速（数据段不加速），≤64 字节
    CAN_BUS_MODE_FD_BRS,      // CAN FD + 数据段变速，数据段时序由 CubeMX 配
} can_bus_mode_t;

// CAN 总线状态枚举表
typedef enum
{
    CAN_BUS_STATE_ACTIVE = 0, // CAN 总线 正常收发
    CAN_BUS_STATE_RECOVERING, // CAN 总线 bus—off 退避恢复中
    CAN_BUS_STATE_DEAD,       // CAN 总线 永久停机
} can_bus_state_t;

// Can_Bus_Init 失败原因枚举表 对应总线内部 init_err 非 0 即初始化未走完
typedef enum
{
    CAN_BUS_ERR_NONE = 0,       // 0 正常
    CAN_BUS_ERR_HANDLE,         // 1 句柄不在句柄表内
    CAN_BUS_ERR_INIT,           // 2 HAL_FDCAN_Init 失败 参数非法或报文 RAM 越界
    CAN_BUS_ERR_RAM_OVERLAP,    // 3 多实例报文 RAM 区间重叠 查 CubeMX 的 Message RAM Offset
    CAN_BUS_ERR_START,          // 4 HAL_FDCAN_Start 失败
    CAN_BUS_ERR_RAM_FIFO_ZERO,  // 5 TX FIFO / RX FIFO0 元素数为 0 CubeMX 报文 RAM 未配
    CAN_BUS_ERR_RAM_INCOMPLETE, // 6 报文 RAM 未配全 无硬件滤波器 或 元素尺寸非法
    CAN_BUS_ERR_FILTER,         // 7 滤波器/全局滤波器配置失败
} can_bus_err_t;

// CAN 总线原始计数快照 负载折算与诊断的取数口
typedef struct
{
    uint32_t tx_frames; // 入队成功发帧累计
    uint32_t rx_frames; // 收帧累计 兼判活心跳
    uint32_t tx_bytes;  // 发帧字节累计
    uint32_t rx_bytes;  // 收帧字节累计
    uint8_t fd_brs;     // 1 数据段变速 负载折算按 FD 口径
} can_bus_raw_t;

// CAN 总线监测 API
void Can_Bus_Raw_Count(uint32_t idx, can_bus_raw_t *out); // 原始计数快照 四计数非原子读
bool Can_Bus_Online(bool expect_traffic);                 // expect_traffic 代表当前期望 CAN 总线是否该有声音

uint32_t Can_Bus_Tx_Epoch(void); // bus-off/恢复的命令可信度代次，仅诊断任务读取

#endif
