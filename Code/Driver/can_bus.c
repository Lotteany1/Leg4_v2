/** @file  can_bus.c  @brief CAN 总线帧收发与恢复可信度交接  @version  v3.5  2026-11-22 */
#include "can_bus.h"
#include "system_key.h"
#include <string.h>
#include "monitor.h"
#include "FreeRTOS.h"
#include "task.h"

// CAN 总线健康状态宏定义参数
#define CAN_BUS_BO_BASE_MS   10u  // bus-off 退避基数 时间翻倍增长
#define CAN_BUS_BO_MAX_MS    500u // bus-off 退避时间上限
#define CAN_BUS_BO_RETRY_MAX 6u   // 退避指数上限，到顶后继续定期恢复
#define CAN_BUS_DEAD_MS      100u // RX 看门狗时长
#define CAN_BUS_TX_DEAD_MS   50u  // TX 看门狗时长

static uint32_t can1_pending_tick;          // 上次抽样挂起请求的时刻，ms
static volatile uint32_t can1_pending_mask; // 跨两次抽样仍挂起的候选槽，槽位重新入队即清位

static volatile uint32_t can_bus_tx_epoch; // bus-off 中断只递增计数，任务据此作废接纳保持假设

// CAN 总线句柄表 —— 波特率/模式/位时序/消息 RAM 元素个数全部以 CubeMX 配置为准
// 下标 == 硬件索引 勿重排
static FDCAN_HandleTypeDef *const can_bus_handles[CAN_BUS_NUM] =
    {
        &hfdcan1,
        &hfdcan2,
        &hfdcan3,
};

//! ---------------------------------------------------------------
// CAN 总线路由表
typedef struct
{
    uint32_t can_id;      // CAN ID
    can_parse_fn_t parse; // CAN 数据解析回调函数
    void *ctx;            // 上下文指针
} can_route_t;

// CAN 总线结构体
typedef struct
{
    // 0. 本槽位句柄 中断与恢复状态机直取
    FDCAN_HandleTypeDef *hfdcan;

    // 1. 收帧分发
    can_route_t route[CAN_BUS_ROUTE_MAX];
    uint32_t route_cnt;

    // 2. 发送定型
    uint8_t max_len; // 单帧上限 经典 8 字节 FD 由 TxElmtSize 决定
    FDCAN_TxHeaderTypeDef tx_template;

    // 3. bus—off状态机
    volatile can_bus_state_t bus_state; // 总线状态
    uint8_t init_err;          // 初始化失败原因 取值同 can_bus_err_t
    uint8_t bus_off_retry_cnt; // bus—off 重试次数
    uint32_t handled_bus_off;  // 后台任务已处理的 bus-off 中断计数
    uint32_t bus_off_tick;     // 进入 bus—off 的时刻

    // 4. 心跳监测
    volatile uint32_t rx_cnt;       // RX 心跳 收到一帧 +1
    volatile uint32_t tx_feed_tick; // TX 喂狗 最近发送时刻
    uint32_t alive_base;            // rx_cnt 的冻结基准 故障时快照
    uint32_t dead_tick;             // 判死计时起点 心跳停摆后记录

    // 5. 负载统计 ISR 只累加 折算与诊断在 monitor
    volatile uint32_t tx_cnt;   // 发帧计数 口径 = 入队成功
    volatile uint32_t rx_bytes; // 收帧字节累计 按 DLC 折算
    volatile uint32_t tx_bytes; // 发帧字节累计
} can_bus_t;

// CAN 总线结构体声明
static can_bus_t can_bus[CAN_BUS_NUM] = {0};

// CAN 总线工具函数
////--------------------------------------工具区范围--------------------------------------////
// 由句柄反查槽位下标 遍历句柄表 保证"表下标 == 硬件索引"
static int32_t Can_Bus_Hw_Index(const FDCAN_HandleTypeDef *hfdcan)
{
    for (uint32_t i = 0; i < CAN_BUS_NUM; i++)
    {
        if (can_bus_handles[i] == hfdcan) return (int32_t)i;
    }
    return -1;
}

// 字节长度转 CAN FD DLC 非法长度返回 -1
static int32_t Can_Bus_Len_To_Dlc(uint8_t len)
{
    if (len <= 8)
    {
        return (int32_t)len;
    }
    switch (len)
    {
        case 12: // 12 字节 CAN FD 编码
            return 9;
        case 16: // 16 字节 CAN FD 编码
            return 10;
        case 20: // 20 字节 CAN FD 编码
            return 11;
        case 24: // 24 字节 CAN FD 编码
            return 12;
        case 32: // 32 字节 CAN FD 编码
            return 13;
        case 48: // 48 字节 CAN FD 编码
            return 14;
        case 64: // 64 字节 CAN FD 编码
            return 15;
        default: // 非法长度或元素尺寸 返回无效值
            return -1;
    }
}

// DLC 低四位查表还原有效字节数
static inline uint8_t Can_Bus_Dlc_To_Bytes(uint32_t dlc)
{
    static const uint8_t dlc_table[16] = {
        0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
    return dlc_table[dlc & 0x0F];
}

// bus-off 重试指数退避 延时封顶
static uint32_t Can_Bus_Bo_Backoff(uint8_t retry)
{
    uint32_t delay = CAN_BUS_BO_BASE_MS << retry; // ovo 左移运算符，等价于乘以 2 的 n次方 n为 retry 值
    return (delay > CAN_BUS_BO_MAX_MS) ? CAN_BUS_BO_MAX_MS : delay;
}
////--------------------------------------工具区范围--------------------------------------////

// 从 CubeMX 写好的 Init 结构体回读模式
static can_bus_mode_t Can_Bus_Mode_Of(const FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan->Init.FrameFormat == FDCAN_FRAME_FD_BRS) return CAN_BUS_MODE_FD_BRS;
    if (hfdcan->Init.FrameFormat == FDCAN_FRAME_FD_NO_BRS) return CAN_BUS_MODE_FD;
    return CAN_BUS_MODE_CLASSIC;
}

// 消息 RAM 元素尺寸(字) → 数据字节数 非法尺寸返回 0
// 表项与 HAL 的 FDCAN_DATA_BYTES_x 一一对应 元素 = 2 字头 + 数据字 勿改用算术硬算
static uint8_t Can_Bus_Elmt_Bytes(uint32_t elmt_words)
{
    switch (elmt_words)
    {
        case FDCAN_DATA_BYTES_8: // 8 字节消息 RAM 数据区
            return 8u;
        case FDCAN_DATA_BYTES_12: // 12 字节消息 RAM 数据区
            return 12u;
        case FDCAN_DATA_BYTES_16: // 16 字节消息 RAM 数据区
            return 16u;
        case FDCAN_DATA_BYTES_20: // 20 字节消息 RAM 数据区
            return 20u;
        case FDCAN_DATA_BYTES_24: // 24 字节消息 RAM 数据区
            return 24u;
        case FDCAN_DATA_BYTES_32: // 32 字节消息 RAM 数据区
            return 32u;
        case FDCAN_DATA_BYTES_48: // 48 字节消息 RAM 数据区
            return 48u;
        case FDCAN_DATA_BYTES_64: // 64 字节消息 RAM 数据区
            return 64u;
        default: // 非法长度或元素尺寸 返回无效值
            return 0u;
    }
}

// CAN 总线初始化函数
// 第 1 遍 HAL_FDCAN_Init，第 2 遍配过滤器+TDC 再 Start
// FDCAN 参数一律以 CubeMX 为准 代码不覆写任何一项
void Can_Bus_Init(void)
{
    ////------------------------ 第 1 遍：初始化外设 ------------------------////
    for (uint32_t i = 0; i < CAN_BUS_NUM; i++)
    {
        FDCAN_HandleTypeDef *hfdcan = can_bus_handles[i];
        int32_t index               = Can_Bus_Hw_Index(hfdcan);
        if (index < 0)
        {
            can_bus[i].init_err = CAN_BUS_ERR_HANDLE; // 句柄查不到，理论上不可达
            continue;
        }

        can_bus_mode_t mode = Can_Bus_Mode_Of(hfdcan);
        can_bus_t *bus      = &can_bus[index];

        // max_len 不能超过消息 RAM 元素尺寸 否则发长帧会串写相邻元素
        uint8_t elmt_bytes = Can_Bus_Elmt_Bytes(hfdcan->Init.TxElmtSize);

        bus->hfdcan  = hfdcan;
        bus->max_len = (mode == CAN_BUS_MODE_CLASSIC) ? 8u : elmt_bytes;

        monitor_can[index].state = (uint8_t)CAN_BUS_STATE_ACTIVE;
        can_bus[index].init_err  = CAN_BUS_ERR_NONE;

        if (HAL_FDCAN_Init(hfdcan) != HAL_OK)
        {
            can_bus[index].init_err = CAN_BUS_ERR_INIT; // 参数非法或消息 RAM 越界
            continue;
        }

        // 报文 RAM 全由 CubeMX 配置 代码只读不写；没配全就在这里暴露 不让总线静默不通
        if ((hfdcan->Instance->TXBC & FDCAN_TXBC_TFQS) == 0u ||
            (hfdcan->Instance->RXF0C & FDCAN_RXF0C_F0S) == 0u)
        {
            can_bus[index].init_err = CAN_BUS_ERR_RAM_FIFO_ZERO; // TX FIFO / RX FIFO0 元素数为 0
        }
        else if (hfdcan->Init.StdFiltersNbr == 0u ||
                 Can_Bus_Elmt_Bytes(hfdcan->Init.TxElmtSize) == 0u ||
                 Can_Bus_Elmt_Bytes(hfdcan->Init.RxFifo0ElmtSize) == 0u)
        {
            can_bus[index].init_err = CAN_BUS_ERR_RAM_INCOMPLETE; // 无硬件滤波器 或 元素尺寸非法
        }
    }

    for (uint32_t a = 0; a < CAN_BUS_NUM - 1u; a++)
    {
        if (can_bus[a].init_err != 0) continue;
        for (uint32_t b = a + 1u; b < CAN_BUS_NUM; b++)
        {
            if (can_bus[b].init_err != 0) continue;
            const FDCAN_MsgRamAddressTypeDef *ma = &can_bus_handles[a]->msgRam;
            const FDCAN_MsgRamAddressTypeDef *mb = &can_bus_handles[b]->msgRam;

            // 三个 FDCAN 共用同一段报文 RAM 靠 Message RAM Offset 分区 撞区间就谁都别用
            if (ma->StandardFilterSA < mb->EndAddress && mb->StandardFilterSA < ma->EndAddress)
            {
                can_bus[a].init_err = CAN_BUS_ERR_RAM_OVERLAP;
                can_bus[b].init_err = CAN_BUS_ERR_RAM_OVERLAP;
            }
        }
    }

    ////------------------------ 第 2 遍：过滤器 + 启动 + 通知 ------------------------////
    for (uint32_t i = 0; i < CAN_BUS_NUM; i++)
    {
        FDCAN_HandleTypeDef *hfdcan = can_bus_handles[i];
        int32_t index               = Can_Bus_Hw_Index(hfdcan);
        if (index < 0 || can_bus[index].init_err != 0) continue;

        can_bus_mode_t mode = Can_Bus_Mode_Of(hfdcan);
        can_bus_t *bus      = &can_bus[index];

        // 发送头文件模板配置
        bus->tx_template = (FDCAN_TxHeaderTypeDef){
            .Identifier          = 0,
            .IdType              = FDCAN_STANDARD_ID,
            .TxFrameType         = FDCAN_DATA_FRAME,
            .DataLength          = FDCAN_DLC_BYTES_8,
            .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
            .BitRateSwitch       = (mode == CAN_BUS_MODE_FD_BRS) ? FDCAN_BRS_ON : FDCAN_BRS_OFF,
            .FDFormat            = (mode == CAN_BUS_MODE_CLASSIC) ? FDCAN_CLASSIC_CAN : FDCAN_FD_CAN,
            .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
            .MessageMarker       = 0};

        // 过滤器配置
        // 注意 此处允许接收所有报文
        HAL_StatusTypeDef cfg_state = HAL_FDCAN_ConfigFilter(hfdcan, &(FDCAN_FilterTypeDef){
                                                                         .IdType       = FDCAN_STANDARD_ID,
                                                                         .FilterIndex  = 0, // 需 CubeMX 里 StdFiltersNbr >= 1
                                                                         .FilterType   = FDCAN_FILTER_MASK,
                                                                         .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
                                                                         .FilterID1    = 0x000,
                                                                         .FilterID2    = 0x000});

        // 全局过滤器配置
        if (cfg_state == HAL_OK)
        {
            cfg_state = HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                                     FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
                                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
        }
        if (cfg_state != HAL_OK)
        {
            can_bus[index].init_err = CAN_BUS_ERR_FILTER;
            continue;
        }

        // 变速模式下需要配置 TDC 补偿 CubeMX 不生成这步 须在 Init 之后 Start 之前
        // TDCO 取数据段 TSeg1（SSP 对齐采样点） 寄存器只有 7 位 超出会被静默截断
        if (mode == CAN_BUS_MODE_FD_BRS)
        {
            uint32_t tdco = (hfdcan->Init.DataTimeSeg1 > 127u) ? 127u : hfdcan->Init.DataTimeSeg1;
            HAL_FDCAN_ConfigTxDelayCompensation(hfdcan, tdco, 0);
            HAL_FDCAN_EnableTxDelayCompensation(hfdcan);
        }

        // 启动 FDCAN
        if (HAL_FDCAN_Start(hfdcan) != HAL_OK)
        {
            can_bus[index].init_err = CAN_BUS_ERR_START;
            continue;
        }
        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

        // 激活总线错误通知
        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_MESSAGE_LOST, 0);
        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_BUS_OFF | FDCAN_IT_ERROR_PASSIVE, 0);

        // 初始化总线状态
        bus->bus_state         = CAN_BUS_STATE_ACTIVE;
        bus->bus_off_retry_cnt = 0;
        bus->rx_cnt            = 0;
        bus->alive_base        = 0;
        bus->tx_feed_tick      = HAL_GetTick();
        bus->dead_tick         = HAL_GetTick();
    }
}

// CAN 总线发送与注册函数
HAL_StatusTypeDef Can_Bus_Send(FDCAN_HandleTypeDef *hfdcan, uint32_t can_id, const uint8_t *data, uint8_t len)
{
    // 获取数组下标与长度
    int32_t index = Can_Bus_Hw_Index(hfdcan);
    int32_t dlc   = Can_Bus_Len_To_Dlc(len);

    // 索引与长度错误检测
    if (index < 0 || dlc < 0 || len > can_bus[index].max_len)
    {
        return HAL_ERROR;
    }

    if (can_bus[index].bus_state == CAN_BUS_STATE_RECOVERING ||
        (hfdcan->Instance->CCCR & FDCAN_CCCR_INIT) != 0u) return HAL_ERROR;

#if !MOTOR_OUTPUT_ENABLE
    // MIT 零值沿用 Dm_Float_To_Uint 的截断编码：P=0x7fff，V/T=0x7ff，KP/KD=0。
    static const uint8_t mit_zero[8] = {0x7f, 0xff, 0x7f, 0xf0, 0x00, 0x00, 0x07, 0xff};
    static const uint8_t disable[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
    static const uint8_t zero[8] = {0};
    if (len != 8u || data == NULL) return HAL_ERROR;
    if (hfdcan == &hfdcan1 && can_id >= 1u && can_id <= 4u)
    {
        if (memcmp(data, disable, sizeof(disable)) != 0 &&
            memcmp(data, mit_zero, sizeof(mit_zero)) != 0) return HAL_ERROR;
    }
    else if (hfdcan == &hfdcan3 && (can_id == 0x200u || can_id == 0x1ffu))
    {
        if (memcmp(data, zero, sizeof(zero)) != 0) return HAL_ERROR;
    }
    else
    {
        return HAL_ERROR;
    }
#endif

    // 满即丢帧 不忙等
    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0)
    {
        monitor_can[index].tx_full_cnt++;
        return HAL_ERROR;
    }

    // 模板拷贝
    FDCAN_TxHeaderTypeDef tx_header = can_bus[index].tx_template;
    tx_header.Identifier            = can_id;
    tx_header.IdType                = (can_id > 0x7FF) ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    tx_header.DataLength            = (uint32_t)dlc; //? 为什么不使用uin32_t类型 因为Can_Bus_Len_To_Dlc用 -1 表示错误 qaq

    // 入队并更新槽位代次，避免恢复任务将复用槽内的新帧当作旧帧。
    taskENTER_CRITICAL();
    HAL_StatusTypeDef state = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, (uint8_t *)data);
    if (state == HAL_OK)
    {
        if (index == 0) can1_pending_mask &= ~HAL_FDCAN_GetLatestTxFifoQRequestBuffer(hfdcan);
        can_bus[index].tx_feed_tick = HAL_GetTick(); // TX 看门狗喂狗
        can_bus[index].tx_cnt++;                     // 口径 = 入队成功 bus—off 被 Abort 的帧也计入
        can_bus[index].tx_bytes += len;
    }
    taskEXIT_CRITICAL();
    return state;
}

// CAN 总线 Tx FIFO 空位查询 给"宁可等也不丢"的关键命令用
bool Can_Bus_Tx_Ready(FDCAN_HandleTypeDef *hfdcan)
{
    int32_t index = Can_Bus_Hw_Index(hfdcan);
    if (index < 0)
    {
        return false;
    }
    return can_bus[index].bus_state != CAN_BUS_STATE_RECOVERING &&
           (hfdcan->Instance->CCCR & FDCAN_CCCR_INIT) == 0u && HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0;
}

// CAN 总线注册回调函数
bool Can_Bus_Register(FDCAN_HandleTypeDef *hfdcan, uint32_t can_id, can_parse_fn_t fn, void *ctx)
{
    // 空指针检测
    if (hfdcan == NULL || fn == NULL)
    {
        return false;
    }

    // 获取数组下标
    int32_t index = Can_Bus_Hw_Index(hfdcan);
    if (index < 0)
    {
        return false;
    }

    can_bus_t *bus = &can_bus[index];

    // 路由表溢出检测
    if (bus->route_cnt >= CAN_BUS_ROUTE_MAX)
    {
        return false;
    }

    // 检查是否已注册相同的 CAN ID
    for (uint32_t i = 0; i < bus->route_cnt; i++)
    {
        if (bus->route[i].can_id == can_id)
        {
            return false;
        }
    }

    // 注册回调函数
    bus->route[bus->route_cnt++] = (can_route_t){can_id, fn, ctx};
    return true;
}

// CAN 总线接收中断回调
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    // 获取数组下标
    int32_t index = Can_Bus_Hw_Index(hfdcan);
    if (index < 0)
    {
        return;
    }

    // 是否有消息丢失
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST)
    {
        monitor_can[index].rx_loss_cnt++;
    }

    // 如果没有新消息，则直接返回
    if (!(RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE))
    {
        return;
    }

    // 读取接收消息
    can_bus_t *bus = &can_bus[index];
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t data[64];

    // 循环读取 FIFO 中的所有消息
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0)
    {
        // 读取消息头和数据 如果失败就跳出循环
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, data) != HAL_OK)
        {
            break;
        }

        // 心跳监测喂狗
        can_bus[index].rx_cnt++;
        can_bus[index].rx_bytes += Can_Bus_Dlc_To_Bytes(rx_header.DataLength);

        // 遍历路由表，查找匹配的 CAN ID 并调用对应的解析函数
        for (uint32_t i = 0; i < bus->route_cnt; i++)
        {
            if (bus->route[i].can_id == rx_header.Identifier)
            {
                bus->route[i].parse(bus->route[i].ctx, rx_header.Identifier, data, Can_Bus_Dlc_To_Bytes(rx_header.DataLength));
                break;
            }
        }
    }
}

// CAN 总线错误中断回调
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    // 获取数组下标
    int32_t index = Can_Bus_Hw_Index(hfdcan);
    if (index < 0)
    {
        return;
    }

    if (ErrorStatusITs & FDCAN_IT_BUS_OFF)
    {
        uint32_t psr = hfdcan->Instance->PSR;
        if (psr & FDCAN_PSR_BO)
        {
            monitor_can[index].bus_off_psr = psr;
            monitor_can[index].bus_off_ecr = hfdcan->Instance->ECR;
            monitor_can[index].bus_off_cnt++;
            can_bus_tx_epoch++;
        }
    }

    if (ErrorStatusITs & FDCAN_IT_ERROR_PASSIVE)
    {
        monitor_can[index].active_passive_cnt++;
    }
}

// bus-off 重新上线 硬件进 bus-off 时会自动置 CCCR.INIT 这里显式走一遍 HAL 状态机
// 必须成对 Stop/Start 只 Stop 会让 State 停在 READY 之后就再也发不出去
static void Can_Bus_BusOff_Recover(FDCAN_HandleTypeDef *hfdcan)
{
    // Abort 会撤销 bus-off 后仍被 API 接纳的排队帧；恢复边界再次切断可信历史。
    taskENTER_CRITICAL();
    can_bus_tx_epoch++;
    taskEXIT_CRITICAL();
    HAL_FDCAN_AbortTxRequest(hfdcan, 0xFFFFFFFFu); // 取消所有挂起发送请求
    HAL_FDCAN_Stop(hfdcan);                        // 进初始化模式 State → READY
    HAL_FDCAN_Start(hfdcan);                       // 退出初始化重新入网 State → BUSY
    // 恢复过程可被控制任务抢占；其中接纳的帧也可能已被 Abort，退出时再切断一次。
    taskENTER_CRITICAL();
    can_bus_tx_epoch++;
    taskEXIT_CRITICAL();
}

// CAN 总线原始计数快照 负载折算与诊断的取数口 四计数非原子读
void Can_Bus_Raw_Count(uint32_t idx, can_bus_raw_t *out)
{
    *out = (can_bus_raw_t){0};

    if (idx >= CAN_BUS_NUM)
    {
        return;
    }

    const can_bus_t *bus = &can_bus[idx];

    out->tx_frames = bus->tx_cnt;
    out->rx_frames = bus->rx_cnt;
    out->tx_bytes  = bus->tx_bytes;
    out->rx_bytes  = bus->rx_bytes;
    out->fd_brs    = (Can_Bus_Mode_Of(bus->hfdcan) == CAN_BUS_MODE_FD_BRS) ? 1u : 0u;
}

// CAN 总线状态监测函数
// 判活原理：收帧心跳(rx_cnt 与冻结基准 alive_base 比对)判活，RX/TX 看门狗判死，bus-off 状态机自动恢复
// expect_traffic=false(期望静默)时只跟随刷新不判死，但永久故障(DEAD)不受门控豁免
bool Can_Bus_Online(bool expect_traffic) // 输入参数 expect_traffic 代表当前期望 CAN 总线是否该有声音 true 则期望有声音 false 则期望无声音
{
    // 获取当前时间
    uint32_t now = HAL_GetTick();
    bool all_ok  = true; // 总体健康标志 默认健康

    // 遍历所有 CAN 总线 句柄表下标即槽位下标
    for (uint32_t i = 0; i < CAN_BUS_NUM; i++)
    {
        can_bus_t *bus = &can_bus[i];

        bool required      = expect_traffic;
        bool alive_changed = (bus->rx_cnt != bus->alive_base); // 心跳是否在跳：本轮是否有新收帧

        // 中断只记事件。恢复状态由本任务维护，失能静默时也推进。
        uint32_t events = monitor_can[i].bus_off_cnt;
        if (events != bus->handled_bus_off)
        {
            bus->handled_bus_off = events;
            bus->bus_state       = CAN_BUS_STATE_RECOVERING;
            bus->bus_off_tick    = now;
        }
        if (bus->bus_state == CAN_BUS_STATE_RECOVERING)
        {
            FDCAN_GlobalTypeDef *hw = bus->hfdcan->Instance;
            if ((hw->PSR & FDCAN_PSR_BO) == 0u && (hw->CCCR & FDCAN_CCCR_INIT) == 0u)
            {
                bus->bus_state         = CAN_BUS_STATE_ACTIVE;
                bus->bus_off_retry_cnt = 0u;
                bus->dead_tick         = now;
                monitor_can[i].bo_recover_cnt++;
            }
            else if ((hw->CCCR & FDCAN_CCCR_INIT) != 0u &&
                     now - bus->bus_off_tick >= Can_Bus_Bo_Backoff(bus->bus_off_retry_cnt))
            {
                Can_Bus_BusOff_Recover(bus->hfdcan);
                if (bus->bus_off_retry_cnt < CAN_BUS_BO_RETRY_MAX) bus->bus_off_retry_cnt++;
                bus->bus_off_tick = now;
            }
        }
        else if (bus->bus_state == CAN_BUS_STATE_DEAD && alive_changed)
        {
            // 判死后心跳仍在跳 物理链路自行恢复 直接复活
            bus->bus_state         = CAN_BUS_STATE_ACTIVE;
            bus->bus_off_retry_cnt = 0;
        }

        // DM 失能时每 100 ms 只发四帧。即使 FIFO 尚有空位，旧请求也可能一直重发。
        if (i == 0u && bus->init_err == CAN_BUS_ERR_NONE &&
            now - can1_pending_tick >= CAN_BUS_TX_DEAD_MS)
        {
            taskENTER_CRITICAL();
            FDCAN_GlobalTypeDef *hw = bus->hfdcan->Instance;
            uint32_t pending = hw->TXBRP;
            uint32_t expired = can1_pending_mask & pending & ~hw->TXBCR;
            can1_pending_mask = pending;
            can1_pending_tick = now;
            if (expired != 0u && bus->bus_state != CAN_BUS_STATE_RECOVERING)
            {
                uint32_t psr = hw->PSR;
                if ((psr & (FDCAN_PSR_BO | FDCAN_PSR_EP)) == FDCAN_PSR_EP &&
                    HAL_FDCAN_AbortTxRequest(bus->hfdcan, expired) == HAL_OK)
                {
                    monitor_can[i].stall_psr = psr;
                    monitor_can[i].stall_ecr = hw->ECR;
                    can1_pending_mask &= ~expired;
                    can_bus_tx_epoch++;
                    monitor_can[i].tx_stall_cnt++;
                    bus->bus_state = CAN_BUS_STATE_DEAD;
                }
            }
            taskEXIT_CRITICAL();
        }

        // 记录总线状态
        monitor_can[i].state = (uint8_t)bus->bus_state;

        // 总线不期望有声音
        if (!required)
        {
            // 静默门控 只跟随刷新心跳基准 不做超时判死
            bus->alive_base = bus->rx_cnt;
            bus->dead_tick  = now;
            if (bus->bus_state == CAN_BUS_STATE_DEAD)
            {
                all_ok = false; // 永久故障不受门控豁免
            }
            continue;
        }

        if (alive_changed)
        {
            // 心跳正常 刷新基准与判死计时
            bus->alive_base = bus->rx_cnt;
            bus->dead_tick  = now;
        }
        else if (now - bus->dead_tick > CAN_BUS_DEAD_MS)
        {
            // RX 看门狗超时 判死
            all_ok = false;
            if (bus->bus_state == CAN_BUS_STATE_ACTIVE)
            {
                bus->bus_state       = CAN_BUS_STATE_DEAD;
                monitor_can[i].state = (uint8_t)bus->bus_state;
            }
        }

        // TX 看门狗 tx_feed_tick 只在发送成功时刷新 此处 expect_traffic 必为 true
        if (now - bus->tx_feed_tick > CAN_BUS_TX_DEAD_MS)
        {
            all_ok = false; // TX 看门狗超时 仅健康告警 不下沉状态
        }

        if (bus->bus_state == CAN_BUS_STATE_DEAD)
        {
            all_ok = false; // DEAD 状态一票否决
        }
    }
    return all_ok;
}

// 任一总线 bus-off 后递增；恢复包含 Abort，不把旧 FIFO 接纳当成继续执行。
uint32_t Can_Bus_Tx_Epoch(void)
{
    return can_bus_tx_epoch;
}
