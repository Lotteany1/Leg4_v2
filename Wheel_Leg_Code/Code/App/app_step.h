/** @file  app_step.h  @brief 单侧自动收腿行为与遥测接口  @version  v1.5  2026-11-16 */
#ifndef APP_STEP_H
#define APP_STEP_H

#include "slip_side.h"

// 台阶行程
#define STEP_STROKE    0.10f // m，相对动作开始实测腿长的收短距离
#define STEP_MOVE_RATE 0.5f // m/s，收腿与恢复原长的峰值速度
#define STEP_HOLD_MS   50u   // ms，实测收腿到位后的保持时间

// 台阶出力与保护
// 专用出力控制器的固定标定与执行保护，不参与打滑检测。
#define STEP_SETTLE_MS       200u     // ms，轨迹结束后等待实测到位的上限
#define STEP_VEL_MAX         0.20f    // m/s，低速运动窗口
#define STEP_TRACK_ERROR     0.002f   // m，允许的轨迹跟踪误差
#define STEP_PREPARE_MS      80u      // ms，限速和交接准备
#define STEP_FAILURE_MS      60u      // ms，持续跟踪失败、限幅或发送失败

// 轴向比例增益在 2 mm 误差处给 12 N 反馈，须实测跟踪及接触后校正。
#define STEP_AXIAL_KP        6000.0f  // N/m，动作腿长度反馈
#define STEP_AXIAL_KD        10.0f    // N·s/m，动作腿伸缩速度反馈
#define STEP_HEIGHT_KP       100.0f   // N/m，支撑腿中心高度反馈
#define STEP_HEIGHT_KD       10.0f    // N·s/m，支撑腿中心高度速度反馈
#define STEP_PITCH_KP        2.0f     // N·m/rad，支撑髋俯仰反馈
#define STEP_PITCH_KD        0.1f     // N·m·s/rad，支撑髋俯仰阻尼
#define STEP_HIP_KP          2.0f     // N·m/rad，动作髋世界系腿角反馈
#define STEP_HIP_KD          0.1f     // N·m·s/rad，动作髋角速度阻尼
#define STEP_WHEEL_KP        2.0f     // N·m·s/m，支撑轮速度反馈
#define STEP_WHEEL_BRAKE     0.05f    // N·m·s/rad，动作轮空转阻尼
#define STEP_AXIAL_MAX       110.0f   // N，收缩轴向反馈限幅
#define STEP_SUPPORT_MAX     195.0f   // N，支撑总轴向力上限
#define STEP_HIP_MAX         2.0f     // N·m，髋净反馈力矩上限
#define STEP_WHEEL_MAX       1.0f     // N·m，支撑轮力矩上限
#define STEP_BRAKE_MAX       0.3f     // N·m，动作轮仅制动，力矩上限

// 单次直线收腿、保持及恢复；出力交接不另设动作阶段。
typedef enum
{
    STEP_IDLE = 0, // 等待确认单侧打滑
    STEP_RETRACT,  // 从入场实测腿长收短固定距离
    STEP_HOLD,     // 到位后保持
    STEP_RETURN,   // 恢复入场实测腿长
    STEP_WAIT_CLEAR  // 等待滑移消退，期间禁止重触发
} step_phase_t;

// 入场拒绝、动作完成及安全退出原因。
typedef enum
{
    STEP_REASON_NONE = 0, // 无拒绝或异常
    STEP_REASON_DISABLED, // 功能关闭或安全退出
    STEP_REASON_CONFIG,   // 动作配置不满足约束
    STEP_REASON_MODE,     // 行为冲突
    STEP_REASON_FEEDBACK, // 反馈或发送接纳失效
    STEP_REASON_CONTACT,  // 接触条件不足或支撑丢失
    STEP_REASON_MOTION,   // 姿态、速度或行驶方向超出窗口
    STEP_REASON_STROKE,   // 软件行程不足
    STEP_REASON_CANCEL,   // 驾驶指令回零
    STEP_REASON_AIR,      // 整车离地
    STEP_REASON_TRACK,    // 轨迹持续未跟随
    STEP_REASON_LIMIT,    // 出力持续限幅
    STEP_REASON_TIMEOUT,  // 收腿或恢复原长超时
    STEP_REASON_NO_GROUND,// 无法恢复双侧接触
    STEP_REASON_COMPLETE  // 固定行程、保持并恢复原长
} step_reason_t;

typedef struct
{
    step_phase_t phase;       // 动作当前阶段
    uint8_t side;             // 位 0 左、位 1 右，IDLE 时为 0
    step_reason_t rejected;   // 最后一次请求拒绝原因
    step_reason_t terminated; // 当前或上一动作终止原因
    uint32_t elapsed_ms;      // ms，本阶段时间
    slip_side_status_t slip;  // 同拍分侧检测状态
    uint8_t contact_valid;    // 位 0/1 分侧接触数据有效
    uint8_t contact_mask;     // 位 0/1 已确认接触
    uint8_t unloaded_mask;    // 位 0/1 已确认卸载
    uint8_t observation;      // 位 0/1 KF 修正来源，0 为纯预测
    uint8_t limit_mask;       // 位 0/1 请求超出动作出力界，位 2 分配限幅，位 3 发送未接纳
    uint8_t accepted_mask;    // 本拍六路 CAN API 接纳位
    uint8_t execution_valid;  // 六路接纳保持码有效位，包含以前已接纳的码
    float target[2], length[2]; // m，左右腿目标及实测
    float target_rate, rate[2]; // m/s，动作腿目标和两腿实测伸缩速度
    float requested[2];         // N，左右总轴向请求（含气弹簧）
    float accepted[2];          // N，左右从接纳码逆解的总轴向力
    float height, height_ref;   // m，支撑侧推得的机体中心高度及锁存目标
    float roll, roll_rate;      // rad、rad/s，横滚角与角速度
} step_status_t;

void App_Step_Init(void);                     // 上电，早于估计与控制任务
void App_Step_Status(step_status_t *status);   // 任意任务复制最近已发布状态
void App_Step_Profile(float start, float target, float initial_rate, float duration,
                      float time, float *length, float *rate); // m、m/s、s，主机可直接校验的五次轨迹

float App_Step_Blend(float time); // s，出力交接时间；返回 0～1 的平滑权重

// 控制任务在平衡运行仲裁中消费分侧检测。返回 true 表示独占六路出力。
bool App_Step_Update(bool follow, bool air, float intent, float yaw_target);
void App_Step_Exit(step_reason_t reason);     // 控制任务安全/倒地/非平衡退出
bool App_Step_Fault(void);                   // 动作失效或输入坏值，需要既有安全层锁存
bool App_Step_Owns_Output(void);             // 控制任务只读，模式仲裁使用

// Balance 在动作阶段调用；返回轴向总力和模型极性的髋、执行层极性的轮矩。
void App_Step_Output(const float gravity[2], const float hip_comp[2], float force[2], float hip[2], float wheel[2]);
void App_Step_Accepted(uint8_t limits);      // 同一控制拍发送后发布接纳量和遥测
void App_Step_Normal_Limits(uint8_t limits); // 正常接地拍报告入场前持续限幅

#endif
