/** @file  execution_boundary_test.c  @brief 查询边界与异步传输切代的确定性反例  @version  v1.0  2026-11-02 */
#include "distributor.h"
#include "dm_motor.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

extern uint32_t test_bus_epoch;

// 单位映射隔离时序：旧码和新码独立选择，事件可跨过查询终点。
static execution_sample_t Test_Record(uint32_t tick, uint32_t cycle, uint32_t offset)
{
    execution_sample_t sample;

    Execution_Begin(&sample);
    sample.tick = tick;
    sample.cycle = cycle;
    sample.valid_before = EXECUTION_CHANNEL_MASK;
    sample.accepted_mask = EXECUTION_CHANNEL_MASK;
    for (unsigned int side = 0u; side < 2u; side++)
    {
        sample.map[side][0][0] = 1.0f;
        sample.map[side][1][1] = 1.0f;
    }
    for (unsigned int event = 0u; event < 5u; event++) sample.offset[event] = offset + event * 10u;
    for (unsigned int ch = 0u; ch < 4u; ch++)
    {
        sample.before[ch] = 2048;
        sample.code[ch] = 2500;
    }
    Execution_Publish(&sample);
    return sample;
}

int main(int argc, char **argv)
{
    execution_sample_t sample;
    execution_interval_t interval;
    unsigned int scenario = argc > 1 ? (unsigned int)atoi(argv[1]) : 0u;

    Execution_Init();
    Execution_Mode(0u);
    sample = Test_Record(10u, 10000u, 800u);
    switch (scenario)
    {
        case 0u: // 查询在第一发送事件前结束，未来事件不得令已知旧码区间无效。
            Execution_Read(sample.epoch, 10000u, 10500u, 10u, 1000u, &interval);
            assert(interval.complete);
            assert(fabsf(interval.total_s[0] - Dm_Motor_Torque_Decode(2048u)) < 1e-6f);
            break;
        case 1u: // 未来记录有缺拍，但查询终点之前的记录和全部事件连续可信。
            Test_Record(13u, 13000u, 100u);
            Execution_Read(sample.epoch, 10000u, 10900u, 10u, 1000u, &interval);
            assert(interval.complete);
            break;
        case 2u: // bus-off/Abort 发生后，无需等待下一控制拍才能拒绝旧代后台提交。
            test_bus_epoch++;
            Execution_Observation(0u, sample.epoch, 10u, 0u);
            {
                execution_observer_diag_t diagnostic[2];

                Execution_Observer_Diagnostic(diagnostic);
                assert(Execution_Epoch() != sample.epoch);
                assert(diagnostic[0].learned == 0u);
                Execution_Begin(&sample);
                assert(sample.valid_before == 0u);
            }
            break;
        case 3u: // 重新使能本身切断可信度，不能依赖调用者此前必定经过失能。
            Dm_Motor_Enable(&hfdcan1, 1u);
            assert(Execution_Epoch() != sample.epoch);
            Execution_Begin(&sample);
            assert(sample.valid_before == 0u);
            break;
        default: // 未定义的反例编号视为测试配置错误。
            assert(0);
            break;
    }
    puts("PASS: execution boundary scenario");
    return 0;
}
