#ifndef STUB_USART_H
#define STUB_USART_H
#include <stdint.h>
#include <stddef.h>
#define UART_WORDLENGTH_8B 0u
#define UART_STOPBITS_1 0u
#define UART_PARITY_NONE 0u
#define UART_HWCONTROL_NONE 0u
#define DMA_NORMAL 0u
#define HAL_UART_STATE_READY 0u
#define HAL_UART_STATE_BUSY_RX 1u
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY } HAL_StatusTypeDef;
typedef struct {struct {uint32_t Mode;} Init;} DMA_HandleTypeDef;
typedef struct {struct {uint32_t BaudRate,WordLength,StopBits,Parity,HwFlowCtl;} Init; DMA_HandleTypeDef *hdmarx; uint32_t RxState;} UART_HandleTypeDef;
extern UART_HandleTypeDef huart1;
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef*, uint8_t*, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef*,uint8_t*,uint16_t,uint32_t);
uint32_t HAL_GetTick(void);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef*,uint16_t);
void HAL_UART_ErrorCallback(UART_HandleTypeDef*);
#endif
