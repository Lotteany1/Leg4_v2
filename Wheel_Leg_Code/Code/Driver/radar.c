/** @file  radar.c  @brief COIN-D6 完整角区收集、鲁棒墙线拟合与投影平均，雷达仅用来测距，无法为控制提供帮助  @version  v1.6  2026-11-20 */
#include "radar.h"
#include "code_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "arm_math.h"
#include <math.h>
#include <string.h>

#define RADAR_BAUD_RATE      230400u // 串口波特率
#define RADAR_MAX_GAP_DEG    5.0f    // 角度间隔过大时丢弃未完成的一次采集
#define RADAR_MIN_BASELINE_M 0.01f   // 拟合所需的点云展宽
#define RADAR_TX_TIMEOUT_MS  20u     // 启停命令发送超时 ms
#define RADAR_MAX_POINTS     128u    // 只缓存选中角区，最多 128 点
#define RADAR_DMA_SIZE       256u    // DMA 接收块大小，单位 byte
#define RADAR_RX_FIFO_SIZE   4096u   // 中断到任务的字节队列大小
#define RADAR_MAX_FRAME_SIZE 775u    // 协议最大帧长 10 + 3 * 255

volatile radar_diag_t radar_diag;

static uint8_t radar_dma_buf[RADAR_DMA_SIZE] __attribute__((section(".dma_buffer"), aligned(32)));
static volatile uint8_t radar_rx_fifo[RADAR_RX_FIFO_SIZE];
static volatile uint16_t radar_rx_read;
static volatile uint16_t radar_rx_write;
static volatile uint16_t radar_dma_pos;
static volatile bool radar_rx_lost;
static uint8_t radar_frame[RADAR_MAX_FRAME_SIZE];
static uint16_t radar_frame_size;
static float radar_x[RADAR_MAX_POINTS];
static float radar_y[RADAR_MAX_POINTS];
static float radar_work[RADAR_MAX_POINTS];
static uint8_t radar_inlier[RADAR_MAX_POINTS];
static uint16_t radar_point_count;
static bool radar_angle_known;
static bool radar_collecting;
static float radar_previous_angle;
static uint32_t radar_last_rx_ms;
static uint32_t radar_last_scan_ms;
static bool radar_scan_seen;
static radar_result_t radar_result;

static UART_HandleTypeDef *const radar_huart = RADAR_HUART;
static const uint8_t radar_start_cmd[4]      = {0xAAu, 0x55u, 0xF0u, 0x0Fu};
static const uint8_t radar_stop_cmd[4]       = {0xAAu, 0x55u, 0xF5u, 0x0Au};

// 将有限角度归一化到雷达原始 [0,360) 坐标，区间和法线使用同一角度方向。
static float Radar_Wrap(float angle)
{
    if (angle >= 360.0f) angle -= 360.0f;
    if (angle < 0.0f) angle += 360.0f;
    return angle;
}

// 失去码流连续性后丢弃残帧和未收齐的角区，必须重新从角区起点进入。
static void Radar_Reset_Stream(void)
{
    radar_frame_size  = 0u;
    radar_angle_known = false;
    radar_collecting  = false;
    radar_point_count = 0u;
}

// 角区点数较少，直接插入排序取中位数，用来排除明显过近或过远的点。
static float Radar_Median(uint16_t count)
{
    for (uint16_t i = 1u; i < count; ++i)
    {
        float value = radar_work[i];
        uint16_t j  = i;

        while (j > 0u && radar_work[j - 1u] > value)
        {
            radar_work[j] = radar_work[j - 1u];
            --j;
        }
        radar_work[j] = value;
    }
    return count & 1u ? radar_work[count / 2u] : 0.5f * (radar_work[count / 2u - 1u] + radar_work[count / 2u]);
}

// 对已选内点作正交最小二乘拟合；局部 X 轴朝向角区中心，法线须朝向前方。
static bool Radar_Fit(float *nx, float *ny, float *distance, float *rms, uint16_t *count)
{
    float mx = 0.0f, my = 0.0f;
    float xx = 0.0f, xy = 0.0f, yy = 0.0f;
    float tangent, along, across;
    float max_tilt = fminf(RADAR_MAX_TILT_DEG, RADAR_ANGLE_DEG);
    uint16_t n     = 0u;

    for (uint16_t i = 0u; i < radar_point_count; ++i)
    {
        if (!radar_inlier[i]) continue;
        mx += radar_x[i];
        my += radar_y[i];
        ++n;
    }
    if (n < 3u || n < RADAR_MIN_POINTS || (float)n < RADAR_MIN_INLIER_RATIO * radar_point_count) return false;
    mx /= n;
    my /= n;
    for (uint16_t i = 0u; i < radar_point_count; ++i)
    {
        if (!radar_inlier[i]) continue;
        float dx = radar_x[i] - mx;
        float dy = radar_y[i] - my;

        xx += dx * dx;
        xy += dx * dy;
        yy += dy * dy;
    }

    // 最大特征向量沿墙面；方向不够稳定时交给投影分支，不因此拒绝整个角区。
    tangent = 0.5f * atan2f(2.0f * xy, xx - yy);
    *nx     = -sinf(tangent);
    *ny     = cosf(tangent);
    if (*nx < 0.0f)
    {
        *nx = -*nx;
        *ny = -*ny;
    }
    along  = *ny * *ny * xx - 2.0f * *nx * *ny * xy + *nx * *nx * yy;
    across = *nx * *nx * xx + 2.0f * *nx * *ny * xy + *ny * *ny * yy;
    // 窄角区无法可靠支持大幅法线修正，超过角宽时使用投影近似，仍可输出距离。
    if (*nx < cosf(DEG_TO_RAD(max_tilt)) || along <= 4.0f * across ||
        along < RADAR_MIN_BASELINE_M * RADAR_MIN_BASELINE_M) return false;

    // n·均值点 等价于逐点按真实拟合法线作 r*cos(theta-normal) 后求平均。
    *distance = *nx * mx + *ny * my;
    *rms      = sqrtf(fmaxf(across, 0.0f) / n);
    *count    = n;
    return *distance > 0.0f;
}

// 收齐角区后，先按中位数预筛，再拟合墙面、剔除离群点并求平均距离。
static void Radar_Finish(uint32_t tick_ms)
{
    radar_result_t result = {0};
    uint16_t count        = 0u;
    float nx, ny, distance, rms;
    float median = 0.0f;
    bool fitted  = false;

    ++radar_diag.sector_count;
    radar_diag.point_count  = radar_point_count;
    radar_diag.inlier_count = 0u;
    radar_diag.status       = RADAR_FEW_POINTS;
    result.sequence         = radar_result.sequence + 1u;
    result.timestamp_ms     = tick_ms;
    result.point_count      = radar_point_count;
    if (radar_point_count >= RADAR_MIN_POINTS)
    {
        radar_diag.status = RADAR_FILTERED;
        float gate;

        // 用实际点云的中位数绝对偏差决定预筛范围，放宽允许倾角不会连带放进远处杂点。
        memcpy(radar_work, radar_x, radar_point_count * sizeof(float));
        median = Radar_Median(radar_point_count);
        for (uint16_t i = 0u; i < radar_point_count; ++i)
            radar_work[i] = fabsf(radar_x[i] - median);
        gate = fmaxf(RADAR_FILTER_M, 3.0f * Radar_Median(radar_point_count));
        for (uint16_t i = 0u; i < radar_point_count; ++i)
            radar_inlier[i] = fabsf(radar_x[i] - median) <= gate;
        // 首尾两组分别取坐标中位数得到初始墙线，避免倾斜墙面上的正常距离变化被误删。
        // 每组的离群点少于一半时，中位数种子比直接用所有点拟合更稳，计算量固定。
        if (radar_point_count >= 6u)
        {
            float mx[2], my[2];
            uint16_t half = radar_point_count / 2u;

            for (uint16_t group = 0u; group < 2u; ++group)
            {
                uint16_t begin = group == 0u ? 0u : half;
                uint16_t n     = group == 0u ? half : radar_point_count - half;

                memcpy(radar_work, radar_x + begin, n * sizeof(float));
                mx[group] = Radar_Median(n);
                memcpy(radar_work, radar_y + begin, n * sizeof(float));
                my[group] = Radar_Median(n);
            }
            float dx     = mx[1] - mx[0];
            float dy     = my[1] - my[0];
            float length = sqrtf(dx * dx + dy * dy);

            if (length > RADAR_MIN_BASELINE_M)
            {
                nx = dy / length;
                ny = -dx / length;
                if (nx < 0.0f)
                {
                    nx = -nx;
                    ny = -ny;
                }
                if (nx >= cosf(DEG_TO_RAD(fminf(RADAR_MAX_TILT_DEG, RADAR_ANGLE_DEG))))
                {
                    distance = 0.5f * (nx * (mx[0] + mx[1]) + ny * (my[0] + my[1]));
                    for (uint16_t i = 0u; i < radar_point_count; ++i)
                        radar_inlier[i] = fabsf(nx * radar_x[i] + ny * radar_y[i] - distance) <= RADAR_FILTER_M;
                }
            }
        }
        if (Radar_Fit(&nx, &ny, &distance, &rms, &count))
        {
            // 已找到墙面方向，按点到墙面的距离剔除离群点，再求最终平均距离。
            for (uint16_t i = 0u; i < radar_point_count; ++i)
                radar_inlier[i] = radar_inlier[i] && fabsf(nx * radar_x[i] + ny * radar_y[i] - distance) <= RADAR_FILTER_M;
            fitted = Radar_Fit(&nx, &ny, &distance, &rms, &count);
        }
        if (fitted && rms <= RADAR_FILTER_M)
        {
            result.distance_m   = distance;
            result.normal_deg   = Radar_Wrap(RADAR_CENTER_DEG + RAD_TO_DEG(atan2f(ny, nx)));
            result.rms_m        = rms;
            result.inlier_count = count;
            result.valid        = true;
            result.fitted       = true;
            radar_diag.status   = RADAR_FITTED;
        }
    }
    // 窄角区或近墙时，点云展开可能小于测距波动，墙面方向无法可靠拟合。
    // 这时只按设置方向投影并平均，标记 fitted=false，不能当作精确垂直墙距。
    if (!result.valid && radar_point_count >= RADAR_MIN_POINTS)
    {
        float mean = 0.0f, variance = 0.0f;
        // 方向不稳定时按中位数周围的点求投影平均，不依赖墙线拟合是否成功。
        float gate = RADAR_FILTER_M;

        count = 0u;
        for (uint16_t i = 0u; i < radar_point_count; ++i)
        {
            radar_inlier[i] = fabsf(radar_x[i] - median) <= gate;
            if (radar_inlier[i])
            {
                mean += radar_x[i];
                ++count;
            }
        }
        if (count >= RADAR_MIN_POINTS && (float)count >= RADAR_MIN_INLIER_RATIO * radar_point_count)
        {
            mean /= count;
            for (uint16_t i = 0u; i < radar_point_count; ++i)
            {
                if (!radar_inlier[i]) continue;
                float residual = radar_x[i] - mean;

                variance += residual * residual;
            }
            result.distance_m   = mean;
            result.normal_deg   = RADAR_CENTER_DEG;
            result.rms_m        = sqrtf(variance / count);
            result.inlier_count = count;
            result.valid        = true;
            radar_diag.status   = RADAR_PROJECTED;
        }
    }
    radar_diag.inlier_count = count;
    // 一次丢点或拟合失败不立即清掉上一有效距离；其时间戳不刷新，超过 500 ms 仍会无效。
    if (!result.valid) return;
    taskENTER_CRITICAL();
    radar_result = result;
    taskEXIT_CRITICAL();
}

// 先按原始扫描方向确认完整进入和退出角区，再存相对角区中心的平面坐标，距离单位 m。
static void Radar_Point(float angle, uint16_t distance_mm, uint32_t tick_ms)
{
    float width  = RADAR_ANGLE_DEG;
    float start  = Radar_Wrap(RADAR_CENTER_DEG - 0.5f * width);
    float offset = Radar_Wrap(angle - start);
    bool inside  = offset <= width;

    if (radar_angle_known)
    {
        float step            = Radar_Wrap(angle - radar_previous_angle);
        float previous_offset = Radar_Wrap(radar_previous_angle - start);

        if (step == 0.0f) return;
        if (step > RADAR_MAX_GAP_DEG)
        {
            radar_collecting  = false;
            radar_point_count = 0u;
        }
        else if (inside && previous_offset > width)
        {
            radar_collecting  = true;
            radar_point_count = 0u;
        }
        else if (!inside && radar_collecting)
        {
            Radar_Finish(tick_ms);
            radar_collecting  = false;
            radar_point_count = 0u;
        }
    }
    radar_angle_known    = true;
    radar_previous_angle = angle;
    if (inside && radar_collecting)
    {
        float range = distance_mm * 0.001f;
        float delta = offset - 0.5f * width;
        float sn, cs;

        if (range < RADAR_MIN_RANGE_M || range > RADAR_MAX_RANGE_M) return;
        if (radar_point_count == RADAR_MAX_POINTS)
        {
            radar_collecting  = false;
            radar_point_count = 0u;
            return;
        }
        arm_sin_cos_f32(delta, &sn, &cs);
        radar_x[radar_point_count] = range * cs;
        radar_y[radar_point_count] = range * sn;
        ++radar_point_count;
    }
}

// 协议多字节字段均为小端，采样点 XOR 以 b0 和 (b1 | b2<<8) 配对。
static uint16_t Radar_Le16(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

// 任务侧尝试解出缓存中的全部完整帧；噪声和坏帧逐字节重同步，不使用串口空闲作为帧尾。
static void Radar_Parse(uint32_t tick_ms)
{
    while (radar_frame_size >= 2u)
    {
        uint16_t drop = 1u;

        if (radar_frame[0] == 0xAAu && radar_frame[1] == 0x55u)
        {
            if (radar_frame_size < 10u) return;
            uint16_t n      = radar_frame[3];
            uint16_t fsa    = Radar_Le16(radar_frame + 4u);
            uint16_t lsa    = Radar_Le16(radar_frame + 6u);
            uint16_t length = 10u + 3u * n;

            if (n != 0u && (fsa & 1u) && (lsa & 1u) && (fsa >> 1) < 23040u && (lsa >> 1) < 23040u)
            {
                if (radar_frame_size < length) return;
                uint16_t cs = 0x55AAu ^ Radar_Le16(radar_frame + 2u) ^ fsa ^ lsa;

                for (uint16_t i = 0u; i < n; ++i)
                    cs ^= radar_frame[10u + 3u * i] ^ Radar_Le16(radar_frame + 11u + 3u * i);
                if (cs == Radar_Le16(radar_frame + 8u))
                {
                    ++radar_diag.valid_frames;
                    radar_last_scan_ms = tick_ms;
                    radar_scan_seen    = true;
                    float first        = (fsa >> 1) / 64.0f;
                    float span         = Radar_Wrap((lsa >> 1) / 64.0f - first);

                    for (uint16_t i = 0u; i < n; ++i)
                    {
                        const uint8_t *s  = radar_frame + 10u + 3u * i;
                        uint16_t distance = ((uint16_t)s[2] << 6) | (s[1] >> 2);
                        float angle       = first + (n > 1u ? span * i / (n - 1u) : 0.0f);

                        Radar_Point(Radar_Wrap(angle), distance, tick_ms);
                    }
                    drop = length;
                }
                else
                {
                    ++radar_diag.checksum_errors;
                    // 校验错误时丢弃本次角区，避免跨坏包拼出貌似完整的墙距。
                    radar_angle_known = false;
                    radar_collecting  = false;
                    radar_point_count = 0u;
                }
            }
        }
        radar_frame_size -= drop;
        memmove(radar_frame, radar_frame + drop, radar_frame_size);
    }
}

// 初始化前在任务侧调用，接收缓冲区放入工程非缓存 DMA 区，不改写 CubeMX 的串口配置。
HAL_StatusTypeDef Radar_Init(void)
{
    if (!(RADAR_CENTER_DEG >= 0.0f && RADAR_CENTER_DEG < 360.0f && RADAR_ANGLE_DEG > 0.0f && RADAR_ANGLE_DEG < 180.0f) ||
        radar_huart->hdmarx == NULL ||
        radar_huart->Init.BaudRate != RADAR_BAUD_RATE || radar_huart->Init.WordLength != UART_WORDLENGTH_8B ||
        radar_huart->Init.StopBits != UART_STOPBITS_1 || radar_huart->Init.Parity != UART_PARITY_NONE ||
        radar_huart->Init.HwFlowCtl != UART_HWCONTROL_NONE) return HAL_ERROR;
    if (radar_huart->hdmarx->Init.Mode != DMA_NORMAL) return HAL_ERROR;
    Radar_Reset_Stream();
    radar_scan_seen    = false;
    radar_diag_t empty = {0};

    radar_diag = empty;
    memset(&radar_result, 0, sizeof(radar_result));
    radar_rx_read    = 0u;
    radar_rx_write   = 0u;
    radar_dma_pos    = 0u;
    radar_rx_lost    = false;
    radar_last_rx_ms = HAL_GetTick();
    return HAL_UARTEx_ReceiveToIdle_DMA(radar_huart, radar_dma_buf, RADAR_DMA_SIZE);
}

// 任务侧同步发送 4 字节启动命令，返回 HAL 的发送状态。
HAL_StatusTypeDef Radar_Start(void)
{
    return HAL_UART_Transmit(radar_huart, (uint8_t *)radar_start_cmd, sizeof(radar_start_cmd), RADAR_TX_TIMEOUT_MS);
}

// 任务侧同步发送停止命令；停止后旧结果由快照超时判为无效。
HAL_StatusTypeDef Radar_Stop(void)
{
    return HAL_UART_Transmit(radar_huart, (uint8_t *)radar_stop_cmd, sizeof(radar_stop_cmd), RADAR_TX_TIMEOUT_MS);
}

// HAL 的 Size 是 DMA 缓冲区写入位置而非本次长度；HT、TC、IDLE 均转成新增字节快照。
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart != radar_huart) return;
    if (size > RADAR_DMA_SIZE) return;
    uint16_t pos       = radar_dma_pos;
    uint16_t end       = size == RADAR_DMA_SIZE ? 0u : size;
    uint16_t count     = size >= pos ? (uint16_t)(size - pos) : (uint16_t)(RADAR_DMA_SIZE - pos + size);
    uint16_t write     = radar_rx_write;
    uint16_t free_size = (radar_rx_read + RADAR_RX_FIFO_SIZE - write - 1u) % RADAR_RX_FIFO_SIZE;

    if (count > free_size)
        radar_rx_lost = true;
    else
    {
        for (uint16_t i = 0u; i < count; ++i)
        {
            radar_rx_fifo[write] = radar_dma_buf[pos];
            write                = (write + 1u) % RADAR_RX_FIFO_SIZE;
            pos                  = (pos + 1u) % RADAR_DMA_SIZE;
        }
        radar_rx_write = write;
    }
    radar_dma_pos = end;
    if (huart->RxState == HAL_UART_STATE_READY)
    {
        radar_dma_pos = 0u;
        if (HAL_UARTEx_ReceiveToIdle_DMA(huart, radar_dma_buf, RADAR_DMA_SIZE) != HAL_OK)
            radar_rx_lost = true;
    }
}

// HAL UART/DMA 错误中断只记接收断流，避免在 ISR 内进行协议解析与墙面计算。
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == radar_huart) radar_rx_lost = true;
}

// 自定义链路在单一任务中按到达顺序喂字节，接收间隔超时后不延续上次未完成角区。
void Radar_Feed(const uint8_t *data, uint16_t size, uint32_t tick_ms)
{
    if (data == NULL || size == 0u) return;
    if (tick_ms - radar_last_rx_ms > RADAR_TIMEOUT_MS) Radar_Reset_Stream();
    radar_last_rx_ms = tick_ms;
    for (uint16_t i = 0u; i < size; ++i)
    {
        radar_frame[radar_frame_size++] = data[i];
        Radar_Parse(tick_ms);
    }
}

// 任务先获取最多一个 DMA 块的队列快照，退出临界区后才解码和计算，ISR 不参与浮点运算。
void Radar_Update(void)
{
    uint8_t data[RADAR_DMA_SIZE];
    uint16_t count;
    bool lost;
    uint32_t tick_ms = HAL_GetTick();

    do
    {
        count = 0u;
        taskENTER_CRITICAL();
        lost = radar_rx_lost;
        if (lost)
        {
            radar_rx_read = radar_rx_write;
            radar_rx_lost = false;
        }
        while (radar_rx_read != radar_rx_write && count < sizeof(data))
        {
            data[count++] = radar_rx_fifo[radar_rx_read];
            radar_rx_read = (radar_rx_read + 1u) % RADAR_RX_FIFO_SIZE;
        }
        taskEXIT_CRITICAL();
        if (lost)
        {
            ++radar_diag.rx_losses;
            Radar_Reset_Stream();
        }
        Radar_Feed(data, count, tick_ms);
    } while (count == sizeof(data));
    if (tick_ms - radar_last_rx_ms > RADAR_TIMEOUT_MS) Radar_Reset_Stream();
    taskENTER_CRITICAL();
    if (radar_huart->RxState == HAL_UART_STATE_READY)
    {
        radar_dma_pos = 0u;
        if (HAL_UARTEx_ReceiveToIdle_DMA(radar_huart, radar_dma_buf, RADAR_DMA_SIZE) != HAL_OK)
            radar_rx_lost = true;
    }
    taskEXIT_CRITICAL();
}

// 读取方只取完整结果，超时仅影响返回快照，不把旧距离伪装成新一圈测量。
radar_result_t Radar_Snapshot(void)
{
    radar_result_t result;

    taskENTER_CRITICAL();
    result = radar_result;
    taskEXIT_CRITICAL();
    if (HAL_GetTick() - result.timestamp_ms > RADAR_TIMEOUT_MS) result.valid = false;
    return result;
}

// 由调用方在任务侧调用；校验通过的扫描帧证明启动成功，墙面是否拟合成功不影响在线判断。
bool Radar_Online(void)
{
    return radar_scan_seen && HAL_GetTick() - radar_last_scan_ms <= RADAR_TIMEOUT_MS;
}
