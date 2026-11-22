/** @file  fsm_running.h  @brief 平衡运行目标更新、落地恢复与倒地确认  @version  v3.7  2026-11-21 */
#ifndef FSM_RUNNING_H
#define FSM_RUNNING_H

#include "balance.h"
#include "dr16.h"
#include <stdbool.h>
#include <stdint.h>

// 驻车与落地
#define FSM_RUNNING_STOP_VEL_TH        0.10f // m/s 松杆后低于或等于此速度才锁定驻车位置
#define FSM_RUNNING_POS_LEG_ANG_TH     0.15f // rad 腿角超出时暂停位置反馈
#define FSM_RUNNING_POS_RECAPTURE      3.5f  // m 离停车点过远时重新锁点
#define FSM_RUNNING_POS_RAMP_MS        200u  // ms 腿角恢复后逐步接回位置反馈
#define FSM_RUNNING_POS_WEIGHT         0.5f  // 无量纲 驻车位置参考权重，与保护恢复斜坡相乘；速度反馈保持原值

// 落地气弹簧补偿
#define FSM_LAND_LEN_TGT          0.18f // m 确认触地时写入的腿长目标和机体高度
#define FSM_LAND_RECOVER_MS        30u  // ms 从确认触地到恢复全部气弹簧补偿的时间，0 表示立即恢复
#define FSM_LAND_SPRING_START_PCT   50u  // % 触地首拍保留的轴向气弹簧补偿，0 为全撤，100 为全保留

// 落地后的驻车判稳与再次离地
#define FSM_LAND_STABLE_SPEED  0.05f // m/s 判稳时，每条腿的伸缩速度绝对值上限
#define FSM_LAND_STABLE_ANGLE  0.15f // rad 判稳时，俯仰角和横滚角的绝对值上限
#define FSM_LAND_STABLE_OMEGA  0.30f // rad/s 判稳时，俯仰和横滚角速度的绝对值上限
#define FSM_LAND_STABLE_MS       50u  // ms 气弹簧补偿恢复后，驻车前双腿接地及姿态须连续稳定的时间

// 倒地确认
#define FSM_RUNNING_FALL_PITCH_TH 1.00f // rad 俯仰倒地阈值
#define FSM_RUNNING_FALL_ROLL_TH  1.00f // rad 横滚倒地阈值
#define FSM_RUNNING_FALL_LEG_TH   0.8f  // rad 世界系腿倾角倒地阈值
#define FSM_RUNNING_FALL_MS       200u  // ms 连续倾倒确认时长

// 平衡行为由主 FSM 译码后传入；仅平衡态调用本模块的目标更新。
typedef enum
{
    FSM_RUNNING_BEHAVIOR_GYRO = 0, // 陀螺：切入时重启陀螺包络，接管偏航与高度目标
    FSM_RUNNING_BEHAVIOR_FOLLOW,   // 跟随：陀螺退出斜坡结束后使用遥控目标
} fsm_running_behavior_t;

typedef struct
{
    float error;      // m 相对停车点位移 行驶时为零
    float weight;     // 无量纲 位置参考权重，驻车系数与保护恢复斜坡的乘积
    uint32_t capture; // 累计停车点捕获次数
    uint32_t events;  // 位 0 为位置反馈暂停 其余位为停车点作废计数
} fsm_running_position_t;

typedef struct
{
    uint32_t elapsed_ms; // ms 从本次确认落地起
    float spring_scale;  // 0～1 左右腿共用的轴向气弹簧补偿比例
    bool active;         // 触地当拍或仍在配置的恢复时间内
    bool contact_pending; // 落地后双侧接触与姿态尚在确认
} fsm_landing_t;

void Fsm_Running_Enter(void); // 平衡入场一次 锁定腿长与高度
void Fsm_Running_Exit(void);  // 作废停车点并清除本轮落地状态
balance_ctrl_mode_t Fsm_Running_Tick(fsm_running_behavior_t behavior, const dr16_t *rc); // 更新目标并选择本拍出力模式
fsm_running_position_t Fsm_Running_Position(void); // 读取本拍相对位置和掩码
fsm_landing_t Fsm_Running_Landing(void); // 同一控制拍读取补偿比例及落地进度
bool Fsm_Running_Fall_Update(void);                 // 平衡态每控制拍一次 推进倒地确认

#endif
