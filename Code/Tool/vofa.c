/** @file  vofa.c  @brief 串口浮点遥测发送  @version  v1.8  2026-11-02 */
#include "vofa.h"
#include "usart.h"
#include <math.h>
#include <stdarg.h>
#include <string.h>

static float Vofa_tx_buffer[64 + 1] __attribute__((section(".bdma_buffer"), aligned(32))) = {0};

// 末尾传 INFINITY 终止，中间 NaN 归零；首项使用 double，满足 va_start 的默认提升约定。
void Vofa_Send_Float(double first, ...)
{

    // 上帧 DMA 未完成时丢新遥测 不改写正在发送的缓冲
    if (VOFA_HUART->gState != HAL_UART_STATE_READY)
    {
        return;
    }

    va_list args;
    va_start(args, first);

    int count = 0;
    float val = (float)first;

    const int MAX_FLOATS = sizeof(Vofa_tx_buffer) / sizeof(float) - 1;

    while (!isinf(val) && count < MAX_FLOATS)
    {
        Vofa_tx_buffer[count] = isnan(val) ? 0.0f : val;
        count++;
        if (count < MAX_FLOATS)
            val = (float)va_arg(args, double); // float 会被提升为 double
    }

    va_end(args);

    // 追加 JustFloat 帧尾标志 DMA 缓冲固定放在可访问 RAM
    memcpy(Vofa_tx_buffer + count, (uint8_t[]){0x00, 0x00, 0x80, 0x7f}, 4);

    size_t bytes_to_send = (count + 1) * sizeof(float);
    HAL_UART_Transmit_DMA(VOFA_HUART, (uint8_t *)Vofa_tx_buffer, bytes_to_send);
}
