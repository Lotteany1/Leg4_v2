/** @file  dji_motor.h  @brief DJI 电机实例与编码发送接口  @version  v3.2  2026-11-16 */
#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include "can_bus.h"
#include <stdint.h>

// 电机输出电流最大指令值
#define DJI_M2006_SET_MAX 10000
#define DJI_M3508_SET_MAX 16384

// 电机分组 用于发送
typedef enum
{
    DJI_GROUP_A = 0, // 组 A 电机 id 0x201~0x204
    DJI_GROUP_B = 1, // 组 B 电机 id 0x205~0x208
} dji_group_t;

// 大疆电机类型枚举表
typedef enum
{
    DJI_M2006 = 0, // M2006 电机
    DJI_M3508 = 1, // M3508 电机
} dji_motor_type_t;

// 大疆电机结构体
typedef struct dji_motor_t
{
    FDCAN_HandleTypeDef *handle;

    // 电机类型
    dji_motor_type_t type;
    uint16_t id;

    // 电机反馈数据
    float pos;    // rad
    float vel;    // rad/s
    float trq;    // N·m
    float circle; // 电机圈数
    float cmd_trq;
    uint32_t frame_seq;  // 已解析反馈序号，估计任务写
    uint32_t frame_tick; // ms，已解析反馈的接收时标

    // 电机状态
    volatile uint8_t parse_flag;
    volatile uint32_t rx_tick; // ms，ISR 发布的接收时标
    volatile uint64_t fb_packed __attribute__((aligned(8))); // 反馈数据快照 aligned(8) 确保8字节对齐

} dji_motor_t;

// 电机实例
extern dji_motor_t dji_wheel_lft;
extern dji_motor_t dji_wheel_rgt;

// 大疆电机API
void Dji_Motor_Init(void);
void Dji_Motor_Parse(void);
HAL_StatusTypeDef Dji_Send_Group(dji_group_t group, const float trq[4]);
void Dji_Motor_All_Stop(void);
// code 可为 NULL；否则四个槽位预先清零，返回实际编码值，接纳按 HAL 状态整组判定。
HAL_StatusTypeDef Dji_Send_Group_Encoded(dji_group_t group, const float trq[4], int16_t code[4]);

#endif
