#ifndef TEST_USART_H
#define TEST_USART_H
#include <stdint.h>
typedef struct { uint32_t port; } UART_HandleTypeDef;
extern UART_HandleTypeDef huart7, huart8, huart9, huart1;
uint32_t HAL_GetTick(void);
#endif
