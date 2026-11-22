/** @file  can_recovery_test.c  @brief Abort 恢复窗口内接纳帧的确定性交错验证  @version  v1.0  2026-11-02 */
#include "can_bus.h"
#include <assert.h>
#include <stdio.h>

static uint32_t can_bus_tx_epoch;
static uint32_t accepted_epoch;
static unsigned int test_depth;

// HAL Abort 内模拟任务抢占：恢复开始后接纳的帧仍会被此次 Abort 撤销。
static void HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef *handle, uint32_t mask)
{
    (void)handle;
    (void)mask;
    assert(test_depth == 0u);
    accepted_epoch = can_bus_tx_epoch;
}
static void HAL_FDCAN_Stop(FDCAN_HandleTypeDef *handle)
{
    (void)handle;
}
static void HAL_FDCAN_Start(FDCAN_HandleTypeDef *handle)
{
    (void)handle;
}
#define taskENTER_CRITICAL() (++test_depth)
#define taskEXIT_CRITICAL() (--test_depth)

// runner 从生产文件原样提取该函数，不维护一份重写的恢复实现。
#include "can_recovery_under_test.h"

int main(void)
{
    FDCAN_HandleTypeDef handle = {0};

    Can_Bus_BusOff_Recover(&handle);
    assert(accepted_epoch != 0u);
    assert(can_bus_tx_epoch != accepted_epoch);
    assert(test_depth == 0u);
    puts("PASS: Abort recovery invalidates frames accepted during recovery");
    return 0;
}
