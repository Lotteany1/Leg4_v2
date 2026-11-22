/** @file  fdcan.h  @brief Host motor interface types  @version  v1.0  2026-10-26 */
#ifndef TEST_FDCAN_H
#define TEST_FDCAN_H
#include <stdint.h>

uint32_t HAL_GetTick(void);
typedef struct
{
    uint32_t port;
} FDCAN_HandleTypeDef;
typedef enum {
    HAL_OK,
    HAL_ERROR,
    HAL_BUSY,
    HAL_TIMEOUT
} HAL_StatusTypeDef;
extern FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;
#endif
