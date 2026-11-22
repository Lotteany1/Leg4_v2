/** @file  radar.h  @brief COIN-D6 串口点云与完整角区墙距测量，雷达仅用来测距，无法为控制提供帮助  @version  v1.6  2026-11-02 */
#ifndef RADAR_H
#define RADAR_H

#include "code_config.h"
#include "usart.h"
#include <stdbool.h>
#include <stdint.h>

// 雷达角区与测距
#ifndef RADAR_CENTER_DEG
#define RADAR_CENTER_DEG 90.0f // 测量方向：0° 前方，90° 对应雷达的 90° 方向
#endif
#ifndef RADAR_ANGLE_DEG
#define RADAR_ANGLE_DEG 60.0f // 总角宽：前方 20° 就是 350°～10°，取值 (0,180)
#endif
#define RADAR_FILTER_M         0.10f // 偏离拟合墙面超过 10 cm 的点不参与平均
#define RADAR_MIN_INLIER_RATIO 0.50f // 过滤后至少保留一半有效点即可输出
#define RADAR_MAX_TILT_DEG     30.0f // 墙线拟合最大偏角；小角区方向不稳时用投影
#define RADAR_MIN_RANGE_M      0.05f // 忽略小于此距离的回波，单位 m
#define RADAR_MAX_RANGE_M      3.0f  // 忽略大于此距离的回波，单位 m
#define RADAR_MIN_POINTS       3u    // 投影与墙线拟合均至少需要 3 个有效点
#define RADAR_PERIOD_MS        100u  // 雷达处理和墙距发送周期，10 Hz
#define RADAR_TIMEOUT_MS       500u  // 超过多久没有新结果就判无效，单位 ms

#define RADAR_HUART (&huart1) // 启停发送与接收共用 USART1

// 完整角区测量结果；距离从雷达旋转中心量起，法线方向使用雷达原始角度。
typedef struct
{
    float distance_m;      // 内点沿拟合墙面法线投影后的平均距离 m
    float normal_deg;      // 拟合墙面朝向的法线角 [0,360) deg
    float rms_m;           // 内点到墙线的法向残差 RMS m
    uint32_t sequence;     // 有效角区结果序号，只有新测量成功才递增
    uint32_t timestamp_ms; // 完成角区时的 HAL tick ms
    uint16_t point_count;  // 参与过滤的有效点数
    uint16_t inlier_count; // 最终用于平均的墙面内点数
    bool fitted;           // true 为墙线拟合；false 为沿设置方向投影的近似距离
    bool valid;            // 完整角区、内点比例、拟合形状与时效均满足要求
} radar_result_t;

// 最近一个完整角区的处理状态，仅用于调试，不占用 VOFA 通道。
typedef enum
{
    RADAR_WAITING = 0, // 尚未采集到完整角区
    RADAR_FITTED,      // 墙线拟合成功
    RADAR_PROJECTED,   // 方向不易拟合，使用设置方向的投影近似值
    RADAR_FEW_POINTS,  // 有效点不足
    RADAR_FILTERED,    // 过滤后未保留足够的墙面点
} radar_status_t;

// 调用方 更新这些计数，可在调试器中直接观察，以区分接收故障与过滤拒绝。
typedef struct
{
    uint32_t valid_frames;    // 校验通过的扫描帧总数
    uint32_t checksum_errors; // 校验失败次数
    uint32_t rx_losses;       // UART 错误或队列溢出造成的断流次数
    uint32_t sector_count;    // 完整角区处理次数，含无效角区
    uint16_t point_count;     // 最近完整角区的有效点数
    uint16_t inlier_count;    // 最近完整角区过滤后的点数
    radar_status_t status;    // 最近完整角区的处理结果
} radar_diag_t;

extern volatile radar_diag_t radar_diag;

// 先在 CubeMX 配好 RADAR_HUART；本函数清状态并启动普通 DMA 接收，不自动发送启动命令。
HAL_StatusTypeDef Radar_Init(void);
// 任务侧启停电机与扫描，先 Init 成功再 Start；不允许在 ISR 调用。
HAL_StatusTypeDef Radar_Start(void);
HAL_StatusTypeDef Radar_Stop(void);
// 驱动提供 HAL UART RxEvent/Error 回调，沿用生成的 UART/DMA IRQ 到 HAL 的分发。
// 调用方 每 100 ms 调用，执行解帧、收齐角区、拟合与过滤；不要放进控制 ISR。
void Radar_Update(void);
// 自定义接收链或离线回放入口；只在同一个任务调用，与 Update 二选一，tick 单位 ms。
void Radar_Feed(const uint8_t *data, uint16_t size, uint32_t tick_ms);
// 读取最近有效测量；偶发失败保留旧值但不刷新时间，超时或未就绪时 valid=false。
radar_result_t Radar_Snapshot(void);
// 调用方 用有效扫描帧判断在线，不以墙距 valid 作为启动成功条件。
bool Radar_Online(void);

#endif
