/** @file  usart.h  @brief 遥测 DMA 串口状态主机夹具  @version  v1.0  2026-11-02 */
#ifndef TEST_VOFA_USART_H
#define TEST_VOFA_USART_H

#include <stdint.h>

#define HAL_UART_STATE_READY 0u // DMA 空闲状态
#define HAL_UART_STATE_BUSY  1u // DMA 正在发送

typedef struct
{
    uint32_t gState; // 模拟 HAL 全局发送状态
} UART_HandleTypeDef;

typedef enum
{
    HAL_OK = 0, // DMA 接纳成功
    HAL_BUSY    // 上帧 DMA 尚未完成
} HAL_StatusTypeDef;

extern UART_HandleTypeDef huart1, hlpuart1; // 两个真实工程串口的独立发送状态

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *handle, uint8_t *data, uint16_t size); // 保留 DMA 指针，不复制源缓冲

#endif
