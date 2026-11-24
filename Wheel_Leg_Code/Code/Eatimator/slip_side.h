/** @file  slip_side.h  @brief 分侧速度可信度与空转确认  @version  v1.1  2026-11-16 */
#ifndef SLIP_SIDE_H
#define SLIP_SIDE_H

#include <stdbool.h>
#include <stdint.h>

#include "code_config.h"

// 分侧接地与打滑检测
#define SLIP_HALF_TRACK       LEG_HALF_TRACK // m，估计器和 MPC 共用半轮距
#define SLIP_FEEDBACK_MS      12u       // ms，分侧判断允许的最大反馈年龄
#define SLIP_CONTACT_MS       15u       // ms，分侧接触持续确认
#define SLIP_CONTACT_LOW      25.038963f // N，单侧卸载阈值
#define SLIP_CONTACT_HIGH     58.424248f // N，单侧接触阈值

// 打滑阈值为台架初值。
#define SLIP_ENTER       0.18f // m/s，进入残差及两侧速度差
#define SLIP_EXIT        0.09f // m/s，退出残差及可信侧一致性
#define SLIP_ENTER_MS    60u   // ms，新鲜配对帧连续确认
#define SLIP_EXIT_MS     80u   // ms，新鲜配对帧连续退出
#define SLIP_ACCEL       0.8f  // m/s²，强加减速禁止确认新事件

typedef enum
{
    SLIP_SIDE_NONE = 0, // 双侧一致，已持续确认无滑移
    SLIP_SIDE_LEFT,     // 左侧持续驱动空转
    SLIP_SIDE_RIGHT,    // 右侧持续驱动空转
    SLIP_SIDE_BOTH,     // 两侧观测均明显偏离预测
    SLIP_SIDE_UNKNOWN   // 样本或方向不足、候选尚未确认
} slip_side_t;

typedef struct
{
    float velocity[2];       // m/s，左右轮腿补偿速度
    float predicted;         // m/s，KF 本拍修正前速度
    float acceleration;      // m/s²，去重力纵向加速度
    float yaw_rate;          // rad/s，逆时针为正
    float intent;            // m/s，前进为正的驾驶目标
    float propulsion[2];     // N·m，发送接纳的前进方向轮矩
    uint32_t frame[2];       // 左右轮已解析的帧序号
    uint32_t now;            // ms，估计时标
    uint8_t valid_mask;      // 位 0/1，轮帧、接触及惯性估计均有效
    uint8_t action_mask;     // 位 0/1，主动动作侧；整个动作禁止新事件
    bool air;                // 整车 AIR
} slip_side_input_t;

typedef struct
{
    slip_side_t side;        // 检测侧别
    float center[2];         // m/s，换算到两髋中心
    float residual[2];       // m/s，相对本拍 KF 预测
    uint32_t duration[2];    // ms，左右候选持续时间
    uint8_t valid_mask;      // 位 0/1，残差有效
    uint8_t usable_mask;     // 位 0/1，允许用作速度观测
} slip_side_status_t;

typedef struct
{
    slip_side_status_t status; // 当前检测输出
    uint32_t frame[2];         // 最近消费的配对轮帧序号
    uint32_t pair_tick;        // ms，最近配对时标
    uint32_t clear_ms;         // ms，异常退出持续时长
    int direction;             // 上次配对样本的驱动方向
    uint8_t suspect;           // 位 0/1，待确认或滞回中的异常侧
} slip_side_detector_t;

// 估计任务每拍在 KF Predict 后、Update 前调用。左右数组按左、右顺序。
void Slip_Side_Update(slip_side_detector_t *detector, const slip_side_input_t *input);

#endif
