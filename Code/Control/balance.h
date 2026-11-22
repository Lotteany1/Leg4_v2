/** @file  balance.h  @brief 平衡出力控制接口  @version  v4.11  2026-11-19 */
#ifndef BALANCE_H
#define BALANCE_H

#include "pid.h"
#include <stdbool.h>
#include "code_config.h"
#include "distributor.h"

#define BALANCE_FORCE_EXIT_TICKS (400u / CTRL_PERIOD_MS) // 拍 离开接地后支持力退出斜坡
#define BALANCE_MPC_PERIOD_MS    4u                      // ms TinyMPC14 后台求解周期

#if BALANCE_USE_HEIGHT_FILTER
// 1 kHz 估计帧，19 Hz 陷波。速度低通的半衰期为 10 ms，alpha = 1 − 2^(-1/10)。
#define BALANCE_HEIGHT_RATE_ALPHA  0.0669670085f
#define BALANCE_HEIGHT_HOLD_MS     10u // 短缺帧按上一输入保持；超过一个低通半衰期时重新播种
#endif

// 控制模式枚举表
typedef enum
{
    LEG_CTRL_MODE_STANDING = 0,         // 地面驻车
    LEG_CTRL_MODE_SELF_RESCUE,          // 自救模式
    LEG_CTRL_MODE_AIR,                  // 空中模式
    LEG_CTRL_MODE_TEST,                 // 测试模式
    LEG_CTRL_MODE_SELF_RESCUE_SWING,    // 自救摆腿段
    LEG_CTRL_MODE_SELF_RESCUE_STAND,    // 自救立腿段
    LEG_CTRL_MODE_SELF_RESCUE_TURNOVER, // 公共翻身 腿长零出力
    LEG_CTRL_MODE_MOVING,               // 地面行驶与制动
#if STEP_ENABLE
    LEG_CTRL_MODE_STEP,                 // 单侧收腿专用出力与独立执行代次
#endif
    LEG_CTRL_MODE_NUM                   // 模式数量
} balance_ctrl_mode_t;

// 接地各模式共用支撑、分配和执行历史。
static inline bool Balance_Mode_Grounded(balance_ctrl_mode_t mode)
{
    return mode == LEG_CTRL_MODE_MOVING || mode == LEG_CTRL_MODE_STANDING;
}

typedef struct
{
    float T;  // 轮毂电机力矩 N·m
    float Tp; // 髋部力矩 N·m
    float F;  // 腿部支持力 N

    float SpringF;    // 气弹簧沿腿补偿 N，取被动力反号
    float MgF;        // 腿重力补偿力矩 N·m
    float Roll_CompF; // 向心力补偿力 N
    float Turn_DistF; // 转弯外部力矩的本腿等效力 N 仅参与模型 不下发电机

    float Comp_Len; // 本腿质心到 pitch 转轴的距离 m
    float Comp_Ang; // 本腿质心相对腿长向量的偏置角 rad

    float Coriolis_F;  // 支持力科氏力补偿 N 仅观测 未接入出力
    float Coriolis_T;  // 轮毂科氏力矩补偿 N·m 仅观测 未接入出力
    float Coriolis_Tp; // 关节科氏力矩补偿 N·m 由 BALANCE_USE_CORIOLIS_TP 选择接入

    float out[2]; // 关节力矩 motor1 motor4 N·m
} control_vector_t;

// 当前执行代次中，最近一次由两腿四个关节同时接纳的轴力命令。
typedef struct
{
    uint32_t tick; // ms 控制拍量测时标
    float sum;     // N 左右总轴力之和，含弹簧
    float diff;    // N 左总轴力减右总轴力，含弹簧
    bool valid;    // 本代次已有该类接纳记录
} balance_mpc_force_t;

// 同拍接地状态、预测请求与 CAN 接纳快照。
typedef struct
{
    uint32_t epoch;        // 执行历史模式代次
    uint32_t cycle;        // DWT 计数，状态与接纳记录的时标
    uint32_t sample_tick;  // ms 最近一次接地状态时标
    uint32_t publish_tick; // ms 最近一次提交新预测序列的时标
    uint32_t commits;     // 已提交的新预测序列累计数
    uint32_t fallbacks;   // 使用终端反馈的累计控制拍数
    uint32_t stale;       // 预测序列超龄作废累计次数
    float roll_rate;     // rad/s 接地模型横滚角速度
    float height;        // m 与本拍 x[12] 对应的绝对高度，含配置的高度滤波
    float height_rate;     // m/s，实际进入控制器的升降速度
    float lowering_reduction; // N，本拍逐腿限幅后的合力减去降高协调后的合力
    float roll_prediction; // rad/s，单节点倾覆扰动预测增量
    float roll_ref;      // rad 接地模型横滚参考
    float height_ref;    // m 转弯协调后的高度参考
    float leg_angle[2];  // rad 与 sample_tick 同拍的左右腿矢状角 x[4/6]
    float accel_y;       // g 同拍机体 y 轴比力，含重力投影
    float pitch;         // rad 同拍机体俯仰角 x[8]
    float force_error[2];  // N 同拍已建立总轴力减上拍接纳总力，分别为左、右
    uint8_t force_valid;   // 力矩反馈与接纳时标匹配的有效位，位 0 左、位 1 右
    float pending_diff_impulse; // N·s 接纳差力相对反馈差力的去偏冲量，含首节点去重项
    float plan[6];       // 轮矩、净髋矩、总轴向支持力，模型坐标
    float accepted[6];   // 同拍接纳码反算的模型输入
    uint8_t accepted_mask; // CAN API 当拍入队成功位
    bool paired;          // 同拍计划与接纳码均有效
    balance_mpc_force_t predicted_force; // 最近使用预测节点且两腿均接纳的拍
    balance_mpc_force_t terminal_force;  // 最近使用终端反馈且两腿均接纳的拍
} balance_mpc_diag_t;

// 两腿控制向量实例 力项与出力由本层写 Leg_Trq 与遥测只读
extern control_vector_t leg_control_lft, leg_control_rgt; // 左右腿出力向量
void Balance_Control_Init(void);                          // 上电初始化
void Balance_Control(balance_ctrl_mode_t ctrl_mode);      // 本拍模式由调用方指定
// 控制任务读取上一拍总支持力请求 [左,右] N，已加回弹簧，仅诊断，不代表限幅后实测力。
void Balance_Force_Diagnostic(float request[2]);
float Balance_Axial_Gravity(float len, float ang); // 悬空腿与车轮的电机保持轴力 N，输入腿长 m、世界系腿角 rad
void Balance_Mpc_Init(void);           // 求解任务上电建模一次
bool Balance_Mpc_Ready(void);          // 首个接地模型已发布
void Balance_Mpc_Stop(void);           // 退出接地时取消在途求解
void Balance_Mpc_Solve(void);          // 后台按 BALANCE_MPC_PERIOD_MS 消费状态快照
void Balance_Mpc_Diagnostic(balance_mpc_diag_t *out); // 遥测读取原子快照

#endif
