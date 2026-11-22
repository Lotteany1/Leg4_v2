/** @file  app_spring_bench.h  @brief 右腿标定档控制与遥测快照  @version  v1.0  2026-11-22 */
#ifndef APP_SPRING_BENCH_H
#define APP_SPRING_BENCH_H

#include "app_spring_cal.h"

#if SPRING_CAL_BENCH_ENABLE
#include "dr16.h"

typedef struct
{
    uint32_t elapsed_ms, run, pair;       // 入档后经过 ms、趟次和已解析的成对关节反馈序号
    spring_cal_phase_t phase;            // 扫描阶段
    spring_cal_reason_t reason;          // 首个停止原因
    float len, rate, angle, omega, roll; // m、m/s、rad、rad/s、rad，右腿世界系姿态与横滚
    float load, command, motor, gravity; // N，收腿幅值、轴力请求、回读轴力和自重保持轴力
    float front, rear;                   // N·m，右前、右后电机回读
    float hip, hip_gravity, hip_command; // N·m，摆腿电机合成矩回读、自重保持矩与请求
    unsigned int enabled_mask;            // 本趟最近状态回读：位0右前、1右后、2左前、3左后
    uint32_t handshake, gate; // 各电机握手阶段（每台4位）和入档状态
    bool sample_ok;                      // 本对反馈适用于慢速压缩段估计
} spring_bench_frame_t;

void App_Spring_Bench_Tick(const dr16_t *rc);           // 只由控制任务调用，NULL 表示估计握手超时并失能
void App_Spring_Bench_Frame(spring_bench_frame_t *out); // 遥测读取最近一次完整控制快照
#endif
#endif
