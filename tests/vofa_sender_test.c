/** @file  vofa_sender_test.c  @brief 遥测串口选择、忙时缓冲保护与帧尾验证  @version  v1.0  2026-11-02 */
#include "vofa.h"
#include "usart.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart1, hlpuart1;
static UART_HandleTypeDef *test_handle;
static uint8_t *test_pending;
static uint16_t test_size;
static unsigned int test_calls;

// 模拟 DMA 持有源地址；再次发送之前，生产缓冲必须保持原帧字节。
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *handle, uint8_t *data, uint16_t size)
{
    assert(handle->gState == HAL_UART_STATE_READY);
    test_handle    = handle;
    test_pending   = data;
    test_size      = size;
    handle->gState = HAL_UART_STATE_BUSY;
    test_calls++;
    return HAL_OK;
}

// 同时验证头文件指定的句柄、64 路 JustFloat 和与其他串口无关的忙状态。
int main(void)
{
    UART_HandleTypeDef *other = VOFA_HUART == &huart1 ? &hlpuart1 : &huart1;
    uint8_t saved[260];
    const uint8_t tail[4] = {0x00u, 0x00u, 0x80u, 0x7fu};

    other->gState = HAL_UART_STATE_BUSY;
    Vofa_Send_Float(0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f,
                    8.0f, 9.0f, 10.0f, 11.0f, 12.0f, 13.0f, 14.0f, 15.0f,
                    16.0f, 17.0f, 18.0f, 19.0f, 20.0f, 21.0f, 22.0f, 23.0f,
                    24.0f, 25.0f, 26.0f, 27.0f, 28.0f, 29.0f, 30.0f, 31.0f,
                    32.0f, 33.0f, 34.0f, 35.0f, 36.0f, 37.0f, 38.0f, 39.0f,
                    40.0f, 41.0f, 42.0f, 43.0f, 44.0f, 45.0f, 46.0f, 47.0f,
                    48.0f, 49.0f, 50.0f, 51.0f, 52.0f, 53.0f, 54.0f, 55.0f,
                    56.0f, 57.0f, 58.0f, 59.0f, 60.0f, 61.0f, 62.0f, 63.0f, INFINITY);
    assert(test_calls == 1u && test_handle == VOFA_HUART && test_size == sizeof(saved));
    for (unsigned int ch = 0u; ch < 64u; ch++)
    {
        float value;

        memcpy(&value, test_pending + 4u * ch, sizeof(value));
        assert(value == (float)ch);
    }
    assert(memcmp(test_pending + 256u, tail, sizeof(tail)) == 0);
    memcpy(saved, test_pending, sizeof(saved));

    other->gState = HAL_UART_STATE_READY;
    Vofa_Send_Float(-1.0f, -2.0f, INFINITY);
    assert(test_calls == 1u && memcmp(saved, test_pending, sizeof(saved)) == 0);

    VOFA_HUART->gState = HAL_UART_STATE_READY;
    other->gState     = HAL_UART_STATE_BUSY;
    Vofa_Send_Float(NAN, 1.5f, INFINITY);
    assert(test_calls == 2u && test_size == 12u);
    assert(((float *)test_pending)[0] == 0.0f && ((float *)test_pending)[1] == 1.5f);
    assert(memcmp(test_pending + 8u, tail, sizeof(tail)) == 0);
    puts("PASS: configured VOFA UART, 64 channels, independent UART busy state, pending DMA bytes and JustFloat tail");
    return 0;
}
