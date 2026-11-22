/** @file  radar_test.c  @brief 雷达角区完整性、倾斜墙体与串口接收回归测试  @version  v1.2  2026-11-02 */
#include "radar.h"

#define RADAR_TEST_DMA_SIZE  256u
#define RADAR_TEST_FIFO_SIZE 4096u
#define RADAR_TEST_START_DEG (RADAR_CENTER_DEG - 0.5f * RADAR_ANGLE_DEG)
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static DMA_HandleTypeDef dma;
UART_HandleTypeDef huart1 = {{230400u, 0u, 0u, 0u, 0u}, &dma, HAL_UART_STATE_READY};
static uint32_t tick;
static uint8_t *dma_target;
static uint16_t dma_offset;
static unsigned command_count;
static float radar_test_distance_m = 1.2f;
static float radar_test_noise_m;
static float radar_test_tilt_deg = 8.0f;
static uint32_t radar_test_seed  = 17u;

uint32_t HAL_GetTick(void)
{
    return tick;
}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size)
{
    assert(huart == &huart1 && size == RADAR_TEST_DMA_SIZE);
    dma_target     = data;
    dma_offset     = 0u;
    huart->RxState = HAL_UART_STATE_BUSY_RX;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size, uint32_t timeout)
{
    assert(huart == &huart1 && size == 4u && timeout == 20u);
    assert(data[0] == 0xAA && data[1] == 0x55);
    ++command_count;
    return HAL_OK;
}
static float Wrap(float x)
{
    x = fmodf(x, 360.0f);
    return x < 0.0f ? x + 360.0f : x;
}
static float Width(void)
{
    return RADAR_ANGLE_DEG;
}
static float Center(void)
{
    return RADAR_CENTER_DEG;
}
static void Deliver(const uint8_t *data, uint16_t size, int transport)
{
    if (!transport)
    {
        // 每个字节单独输入，覆盖分批接收与缓存状态。
        for (uint16_t i = 0; i < size; ++i) Radar_Feed(data + i, 1u, tick);
        return;
    }
    for (uint16_t i = 0; i < size; ++i)
    {
        dma_target[dma_offset++] = data[i];
        if (dma_offset == RADAR_TEST_DMA_SIZE / 2u)
            HAL_UARTEx_RxEventCallback(&huart1, dma_offset);
        else if (dma_offset == RADAR_TEST_DMA_SIZE)
        {
            huart1.RxState = HAL_UART_STATE_READY;
            HAL_UARTEx_RxEventCallback(&huart1, RADAR_TEST_DMA_SIZE);
        }
    }
}
static void Flush(void)
{
    if (dma_offset)
    {
        uint16_t size  = dma_offset;
        huart1.RxState = HAL_UART_STATE_READY;
        HAL_UARTEx_RxEventCallback(&huart1, size);
    }
    Radar_Update();
}
static void Emit(float start, unsigned count, int mode, int transport, int corrupt)
{
    uint8_t frame[10 + 3 * 25] = {0xAA, 0x55, 0};
    uint16_t fsa               = ((uint16_t)lroundf(Wrap(start) * 64.0f) << 1) | 1u;
    uint16_t lsa               = ((uint16_t)lroundf(Wrap(start + 0.5f * (count - 1u)) * 64.0f) << 1) | 1u;
    uint16_t cs;

    assert(count > 0 && count <= 25);
    frame[3] = (uint8_t)count;
    frame[4] = (uint8_t)fsa;
    frame[5] = fsa >> 8;
    frame[6] = (uint8_t)lsa;
    frame[7] = lsa >> 8;
    cs       = 0x55AAu ^ (count << 8) ^ fsa ^ lsa;
    for (unsigned i = 0; i < count; ++i)
    {
        float angle    = start + 0.5f * i;
        float delta    = Wrap(angle - Center() + 180.0f) - 180.0f;
        unsigned index = (unsigned)lroundf(Wrap(angle) * 2.0f);
        float range    = radar_test_distance_m / cosf((delta - radar_test_tilt_deg) * 0.017453292519943295f);
        uint16_t mm;
        uint8_t *s = frame + 10 + 3 * i;

        range += ((int)(index % 5u) - 2) * 0.0007f;
        radar_test_seed = radar_test_seed * 1664525u + 1013904223u;
        range += (((radar_test_seed >> 8) & 65535u) / 32767.5f - 1.0f) * radar_test_noise_m;
        if (mode == 1 && index % 4u == 0u) range += 0.8f;
        if (mode == 2) range = 0.0f;
        if (mode == 3 && index % 5u != 0u) range += (index % 2u ? 0.5f : -0.4f);
        mm   = (uint16_t)lroundf(range * 1000.0f);
        s[0] = (100u & 63u) << 2;
        s[1] = (uint8_t)((mm & 63u) << 2) | (100u >> 6);
        s[2] = mm >> 6;
        cs ^= s[0] ^ (s[1] | ((uint16_t)s[2] << 8));
    }
    frame[8] = (uint8_t)cs;
    frame[9] = cs >> 8;
    if (corrupt) frame[12] ^= 0x10;
    Deliver(frame, (uint16_t)(10 + 3 * count), transport);
}
static void Sweep(int mode, int transport)
{
    unsigned total = (unsigned)lroundf((Width() + 10.0f) * 2.0f) + 1u;
    for (unsigned i = 0; i < total;)
    {
        unsigned count = total - i < 25u ? total - i : 25u;
        Emit(RADAR_TEST_START_DEG - 5.0f + i * 0.5f, count, mode, transport, 0);
        if (transport) Flush();
        i += count;
        ++tick;
    }
}
static void Init(void)
{
    tick           = 10;
    huart1.RxState = HAL_UART_STATE_READY;
    assert(Radar_Init() == HAL_OK);
    assert(!Radar_Online());
}
static void CheckWall(void)
{
    radar_result_t result = Radar_Snapshot();
    assert(result.valid);
    assert(fabsf(result.distance_m - 1.2f) < 0.004f);
    assert(fabsf(Wrap(result.normal_deg - Center() + 180.0f) - (180.0f + radar_test_tilt_deg)) < 0.5f);
    assert(result.inlier_count >= RADAR_MIN_POINTS);
    printf("wall %.6f m, normal %.3f deg, inliers %u/%u, rms %.6f m\n",
           result.distance_m, result.normal_deg, result.inlier_count, result.point_count, result.rms_m);
}
int main(void)
{
    Init();
    // 启动时已在区间内部，只收到后半段不能发布结果。
    for (unsigned i = 0; i < (unsigned)(Width() + 4.0f); ++i)
        Emit(RADAR_TEST_START_DEG + Width() * 0.5f + i * 0.5f, 1u, 0, 0, 0);
    assert(Radar_Snapshot().sequence == 0u);
    Sweep(0, 0);
    CheckWall();
    assert(Radar_Snapshot().sequence == 1u);
    Sweep(1, 0);
    CheckWall();
    assert(Radar_Snapshot().inlier_count < Radar_Snapshot().point_count);
    uint32_t good_timestamp = Radar_Snapshot().timestamp_ms;
    uint32_t good_sequence  = Radar_Snapshot().sequence;
    Sweep(2, 0);
    assert(Radar_Snapshot().valid);
    assert(Radar_Snapshot().timestamp_ms == good_timestamp && Radar_Snapshot().sequence == good_sequence);
    assert(radar_diag.status == RADAR_FEW_POINTS);
    assert(Radar_Online());
    Sweep(0, 0);
    CheckWall();
    Sweep(3, 0);
    assert(Radar_Snapshot().valid);
    assert(radar_diag.status == RADAR_FILTERED);
    Sweep(0, 0);
    tick += RADAR_TIMEOUT_MS + 1u;
    assert(!Radar_Online());
    assert(!Radar_Snapshot().valid);
    Sweep(0, 0);
    CheckWall();

    Init();
    Emit(RADAR_TEST_START_DEG, 1u, 0, 0, 1);
    assert(!Radar_Online());
    // 区间结束前不发布，坏包切断本次采集，不能跨坏包拼接平均值。
    Emit(RADAR_TEST_START_DEG - 1.0f, 20, 0, 0, 0);
    assert(Radar_Snapshot().sequence == 0u);
    Emit(RADAR_TEST_START_DEG + 9.0f, 4, 0, 0, 1);
    for (float a = RADAR_TEST_START_DEG + 11.0f; a <= RADAR_TEST_START_DEG + Width() + 1.0f; a += 0.5f)
        Emit(a, 1, 0, 0, 0);
    assert(Radar_Snapshot().sequence == 0u);
    Sweep(1, 0);
    CheckWall();

    Init();
    Sweep(1, 1); // Normal DMA 的 HT、TC、IDLE 回调及队列。
    CheckWall();
    HAL_UART_ErrorCallback(&huart1);
    huart1.RxState = HAL_UART_STATE_READY;
    Radar_Update();
    Sweep(0, 1);
    CheckWall();
    {
        uint8_t noise[RADAR_TEST_FIFO_SIZE + RADAR_TEST_DMA_SIZE] = {0};
        Deliver(noise, sizeof(noise), 1);
        Flush();
        Sweep(0, 1);
        CheckWall();
    }
    // 100 ms 内先累计 2304 字节再处理，验证 10 Hz 消费不会撑满接收队列。
    Init();
    {
        uint8_t burst[2304] = {0};
        unsigned total      = (unsigned)lroundf((Width() + 10.0f) * 2.0f) + 1u;

        Deliver(burst, sizeof(burst), 1);
        for (unsigned i = 0u; i < total;)
        {
            unsigned count = total - i < 25u ? total - i : 25u;

            Emit(RADAR_TEST_START_DEG - 5.0f + i * 0.5f, count, 1, 1, 0);
            i += count;
        }
        tick += RADAR_PERIOD_MS;
        Flush();
        CheckWall();
    }
    assert(Radar_Start() == HAL_OK && Radar_Stop() == HAL_OK && command_count == 2u);
    // 少于最小点数不发布；近墙点云达到门限但展宽不足时，只返回方向投影，不能标记成墙线拟合。
    radar_test_distance_m = 0.15f;
    for (unsigned points = 1u; points <= RADAR_MIN_POINTS; ++points)
    {
        Init();
        for (float angle = RADAR_TEST_START_DEG - 1.0f; angle <= RADAR_TEST_START_DEG + Width() + 1.0f; angle += 1.0f)
        {
            float delta = Wrap(angle - Center() + 180.0f) - 180.0f;

            Emit(angle, 1u, delta >= -0.1f && delta < (float)points - 0.5f ? 0 : 2, 0, 0);
        }
        radar_result_t result = Radar_Snapshot();

        assert(radar_diag.point_count == points);
        if (points < RADAR_MIN_POINTS)
        {
            assert(!result.valid && !result.fitted && result.sequence == 0u);
            assert(radar_diag.status == RADAR_FEW_POINTS);
        }
        else
        {
            assert(result.valid && !result.fitted && result.sequence == 1u);
            assert(result.point_count == RADAR_MIN_POINTS && result.inlier_count == RADAR_MIN_POINTS);
            assert(radar_diag.status == RADAR_PROJECTED);
            assert(fabsf(result.distance_m - radar_test_distance_m / cosf(8.0f * 0.017453292519943295f)) < 0.002f);
        }
    }
    radar_test_distance_m = 1.2f;
    if (Width() >= 30.0f)
    {
        // 超过原 10° 门限的倾斜墙面，在足够角宽下仍应正确拟合并滤除离群点。
        Init();
        radar_test_tilt_deg = 20.0f;
        Sweep(0, 0);
        CheckWall();
        uint32_t previous_sequence = Radar_Snapshot().sequence;
        Sweep(1, 0);
        CheckWall();
        assert(Radar_Snapshot().sequence > previous_sequence);
        assert(Radar_Snapshot().fitted);
        radar_test_tilt_deg = 8.0f;
    }
    // 使用厂家 0.9° 角分辨率，在近距离窄角区叠加有界测距扰动。
    for (unsigned profile = 0u; profile < 2u; ++profile)
    {
        unsigned accepted = 0u, projected = 0u;
        float largest_error = 0.0f;

        radar_test_distance_m = profile == 0u ? 0.15f : 2.0f;
        radar_test_noise_m    = profile == 0u ? 0.006f : 0.04f;
        Init();
        for (unsigned round = 0u; round < 100u; ++round)
        {
            tick += RADAR_PERIOD_MS;
            for (float angle = RADAR_TEST_START_DEG - 1.8f; angle <= RADAR_TEST_START_DEG + Width() + 1.8f; angle += 0.9f)
                Emit(angle, 1u, 1, 0, 0);
            radar_result_t wall = Radar_Snapshot();

            if (wall.valid && wall.timestamp_ms == tick)
            {
                ++accepted;
                if (!wall.fitted) ++projected;
                largest_error = fmaxf(largest_error, fabsf(wall.distance_m - radar_test_distance_m));
            }
        }
        printf("Noisy %.2fm / %.1fdeg: %u/100 fresh (%u projected), max error %.4fm\n", radar_test_distance_m, Width(), accepted, projected, largest_error);
        assert(accepted >= 95u);
        if (Width() <= 10.0f) assert(projected > 0u);
        assert(largest_error < (profile == 0u ? 0.015f : 0.06f));
    }
    puts("PASS: tilted wall, 25% outliers, complete-sector-only, zero returns, checksum loss, timeout, DMA, FIFO recovery");
    return 0;
}
