/** @file  app_gyro.h  @brief 陀螺行为目标更新  @version  v3.2  2026-11-16 */
#ifndef APP_GYRO_H
#define APP_GYRO_H

#include "code_config.h"
#include <stdbool.h>
#include <stdint.h>

// 陀螺
#define GYRO_YAW_BIAS 10.0f                // rad/s 偏航偏置
#define GYRO_YAW_FREQ 0.5f                 // Hz 正弦频率
#define GYRO_YAW_AMP  2.5f                 // rad/s 正弦幅值
#define GYRO_YAW_DIR  (-1.0f)              // 旋转方向
#define GYRO_H_HI     0.35f                // m 高度上限
#define GYRO_H_LO     0.08f                // m 高度下限
#define GYRO_H_EXIT   (Leg_Len_Min + 0.1f) // m 退出高度
#define GYRO_RAMP_MS  1000u                 // ms 出入场斜坡

#define GYRO_FREQ_MAX 10.0f // Hz 频率限幅
#define GYRO_YAW_MAX  12.0f // rad/s 转速限幅

typedef struct
{
    float yaw_bias;   // rad/s 偏航偏置
    float yaw_freq;   // Hz 正弦频率
    float yaw_amp;    // rad/s 正弦幅值
    float yaw_dir;    // 旋转方向
    float h_lo;       // m 高度下限
    float h_hi;       // m 高度上限
    uint16_t ramp_ms; // ms 出入场斜坡
} gyro_cfg_t;

void App_Gyro_Init(void);                     // 上电一次 参数回默认 运行态清零
void App_Gyro_Reset(void);                    // 平衡入场清运行态并释放目标量 保留配置 不改机体目标
void App_Gyro_Cfg_Set(const gyro_cfg_t *cfg); // 运行时改参数 越界钳位 上下限颠倒对调
void App_Gyro_Enter(void);                    // 实际切入陀螺行为时调 相位归零包络从 0 起 高度从当前值起步
void App_Gyro_Update(bool active);            // 每拍推进 active 为 1 爬坡接管 为 0 收尾把目标量还给遥控
bool App_Gyro_Running(void);                  // 只读，含退出斜坡

#endif
