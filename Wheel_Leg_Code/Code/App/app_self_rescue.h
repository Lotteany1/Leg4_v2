/** @file  app_self_rescue.h  @brief 倒地自救阶段推进  @version  v4.5  2026-11-24 */
#ifndef APP_SELF_RESCUE_H
#define APP_SELF_RESCUE_H

#include "code_config.h"
#include "balance.h"
#include "pid.h"
#include <stdbool.h>

// 自救
// 公共流程与交接判据
#define SR_COMMON_PREPARE_MS 300u              // ms 停稳等待
#define SR_COMMON_EXIT_ANG   DEG_TO_RAD(30.0f) // rad 退出姿态窗口
#define SR_COMMON_LEG_ANG    DEG_TO_RAD(40.0f) // rad 双腿退出角窗
#define SR_COMMON_HOLD_MS    10u               // ms 姿态与角速度同时进窗后连续确认时间
#define SR_ATTEMPT_TIMEOUT_MS 8000u            // ms 单次自救总时限，从准备或换向收腿开始累计
#define SR_RETRACT_TRQ_RAMP_MS 10u            // ms 收腿开始至关节力矩达到全量的时间
#define SR_RETRY_MAX          5u               // 首次尝试后允许的换向重试次数
#define SR_COMMON_PITCH_SIGN (+1.0f)           // 俯仰极性
#define SR_COMMON_ROLL_SIGN  (+1.0f)           // 横滚极性
#define SR_COMMON_LEN_ERR    0.05f             // m 交接腿长容差
#define SR_COMMON_LEN_DIFF   0.04f             // m 交接双腿长度差
#define SR_COMMON_OMG_TH     1.0f              // rad/s 交接机体角速度窗口

// 障碍卡死检测与换向收腿
#define SR_OBSTACLE_STALL_RATE_MAX_RADPS 0.26f // rad/s 有动作请求但机构角速度低于此值，视为缺少进展
#define SR_OBSTACLE_COMMAND_MIN_RADPS    1.0f  // rad/s 速度目标超过此幅值才检测卡死；到位静止不计时
#define SR_OBSTACLE_ANG_ERR_MIN_RAD      0.1f  // rad 位置目标误差超过此值才检测扫腿卡死
#define SR_OBSTACLE_RETRACT_LEN_TOL_M    0.02f // m 双腿均收至最短腿长加此容差后才启动下一路径
#define SR_OBSTACLE_SWEEP_LEAD_MAX_RAD   0.35f // rad 翻身及正转位置目标相对实测的最大超前角，防止卡住后误差累积

// 公共翻身：气弹簧伸腿，首次按 pitch 就近扫腿，超时重试时反向
#define SR_TURNOVER_EXTEND_MS  500u              // ms 腿长零出力 等待气弹簧自然伸腿
#define SR_TURNOVER_SPEED      (PI * 1.4f)             // rad/s 公共翻身扫腿速度
#define SR_TURNOVER_PITCH_TH   DEG_TO_RAD(60.0f) // rad 翻正俯仰窗口
#define SR_TURNOVER_ROLL_TH    DEG_TO_RAD(60.0f) // rad 翻正横滚窗口
#define SR_TURNOVER_HOLD_MS    100u              // ms 翻正持续确认

// 正转路径（rescue_dir = +1）：收腿 → 正向扫腿
#define SR_FORWARD_LEG_LEN_RATE_MPS    1.00f                 // m/s 腿长目标变化速率；用于正转收腿及扫腿阶段
#define SR_FORWARD_SWEEP_RATE_RADPS   (PI * 2.0f)           // rad/s 两腿同步正向扫动的角目标速率；执行时乘 CTRL_DT
#define SR_FORWARD_SWEEP_DIRECTION    (+1.0f)               // +1 增大机构角，-1 减小机构角；仅控制正转路径的扫动方向
#define SR_FORWARD_SWEEP_LEG_LEN_M    (Leg_Len_Min + 0.03f) // m 正转扫腿时的腿长目标
#define SR_FORWARD_RETRACT_LEN_TOL_M  0.03f                 // m 实测腿长低于 Leg_Len_Min 加此容差，双腿均满足才开始扫腿

// 反转路径（rescue_dir = -1）：摆腿 → 收腿 → 立腿
#define SR_REVERSE_RETRACT_LEN_RATE_MPS        0.80f                 // m/s 反转路径收腿时的腿长目标变化速率
#define SR_REVERSE_READY_PITCH_MAX_RAD         0.45f                 // rad 起立预备位允许的机体俯仰绝对值上限
#define SR_REVERSE_READY_ROLL_MAX_RAD          0.66f                 // rad 起立预备位允许的机体横滚绝对值上限；兼作摆腿方向判据
#define SR_REVERSE_READY_WORLD_LEG_ANG_MIN_RAD (-0.8f)               // rad 起立预备位的世界系腿角下界；双腿分别判断
#define SR_REVERSE_READY_WORLD_LEG_ANG_MAX_RAD 1.30f                 // rad 起立预备位的世界系腿角上界；双腿分别判断
#define SR_REVERSE_SWING_RATE_RADPS            (PI * 1.80f)          // rad/s 摆腿速度幅值；正负方向由腿角及机体姿态选择
#define SR_REVERSE_READY_HOLD_MS               10u                  // ms 双腿及机体连续处于预备窗口后，进入反转收腿
#define SR_REVERSE_STAND_LEG_LEN_M             Leg_Len_Min           // m 反转收腿和立腿阶段的腿长目标
#define SR_REVERSE_RETRACT_DONE_LEN_M          (Leg_Len_Min + 0.03f) // m 双腿实测长度均小于此值后，从收腿进入立腿
#define SR_REVERSE_STAND_RATE_RADPS            (PI * 3.0f)                // rad/s 立腿速度幅值；角度误差大于减速角窗时按此速度转向机构零位
#define SR_REVERSE_STAND_SLOWDOWN_ANG_RAD      DEG_TO_RAD(20.0f)     // rad 机构角距零位小于此角时线性减速，到零位时速度目标为零
#define SR_REVERSE_STAND_BODY_LEG_ANG_TOL_RAD  0.1f                  // rad 机构腿角相对零位的允许误差；双腿均满足才累计立腿到位时间
#define SR_REVERSE_STAND_DONE_HOLD_MS          10u                   // ms 双腿机构角连续到位后，进入公共起立稳定确认

// 公共腿长、双腿同步与翻身／正转摆角 PID
// PID 沿用 1 ms 离散标定；公共摆角环由翻身和正转路径共用。
#define SR_PID_LEG_LEN_LFT_KP  1800.0f  // 左腿腿长 PID KP 沿用 1 ms 离散标定
#define SR_PID_LEG_LEN_LFT_KI  0.0f     // 左腿腿长 PID KI 沿用 1 ms 离散标定
#define SR_PID_LEG_LEN_LFT_KD  50000.0f // 左腿腿长 PID KD 沿用 1 ms 离散标定
#define SR_PID_LEG_LEN_RGT_KP  1800.0f  // 右腿腿长 PID KP 沿用 1 ms 离散标定
#define SR_PID_LEG_LEN_RGT_KI  0.0f     // 右腿腿长 PID KI 沿用 1 ms 离散标定
#define SR_PID_LEG_LEN_RGT_KD  50000.0f // 右腿腿长 PID KD 沿用 1 ms 离散标定
#define SR_PID_LEG_SYMMETRY_KP 30.0f    // 双腿同步 PID KP 沿用 1 ms 离散标定
#define SR_PID_LEG_SYMMETRY_KI 0.0f     // 双腿同步 PID KI 沿用 1 ms 离散标定
#define SR_PID_LEG_SYMMETRY_KD 5000.0f  // 双腿同步 PID KD 沿用 1 ms 离散标定
#define SR_PID_LEG_ANG_LFT_KP  300.0f   // 左腿翻身及正转摆角 PID KP 沿用 1 ms 离散标定
#define SR_PID_LEG_ANG_LFT_KI  0.0f     // 左腿翻身及正转摆角 PID KI 沿用 1 ms 离散标定
#define SR_PID_LEG_ANG_LFT_KD  1500.0f  // 左腿翻身及正转摆角 PID KD 沿用 1 ms 离散标定
#define SR_PID_LEG_ANG_RGT_KP  300.0f   // 右腿翻身及正转摆角 PID KP 沿用 1 ms 离散标定
#define SR_PID_LEG_ANG_RGT_KI  0.0f     // 右腿翻身及正转摆角 PID KI 沿用 1 ms 离散标定
#define SR_PID_LEG_ANG_RGT_KD  1500.0f  // 右腿翻身及正转摆角 PID KD 沿用 1 ms 离散标定
#define SR_PID_LEG_ANG_OUT_MAX 40.0f    // N·m 自救髋力矩限幅
// 反转摆腿速度 PID
#define SR_REVERSE_SWING_RATE_PID_KP      9.0f  // 反转摆腿速度环比例增益
#define SR_REVERSE_SWING_RATE_PID_KI      3.0f  // 反转摆腿速度环积分增益
#define SR_REVERSE_SWING_RATE_PID_KD      0.0f  // 反转摆腿速度环微分增益
#define SR_REVERSE_SWING_RATE_PID_I_MAX   10.0f  // N·m 反转摆腿速度环积分项限幅
#define SR_REVERSE_SWING_RATE_PID_OUT_MAX 20.0f // N·m 反转摆腿速度 PID 输出限幅；尚未叠加腿重力补偿

// 反转收腿／立腿速度 PID
#define SR_REVERSE_STAND_RATE_PID_KP      3.0f  // 反转收腿／立腿速度环比例增益
#define SR_REVERSE_STAND_RATE_PID_KI      0.0f  // 反转收腿／立腿速度环积分增益
#define SR_REVERSE_STAND_RATE_PID_KD      10.0f  // 反转收腿／立腿速度环微分增益
#define SR_REVERSE_STAND_RATE_PID_OUT_MAX 10.0f // N·m 反转收腿／立腿速度 PID 输出限幅；尚未叠加腿重力补偿

// 自救阶段枚举表
typedef enum
{
    SR_PHASE_INIT = 0,         // 初始
    SR_PHASE_PREPARE,          // 等待停稳
    SR_PHASE_TURNOVER,         // 公共翻身 腿长零出力
    SR_PHASE_OBSTACLE_RETRACT, // 换向或反转失姿后先收腿
    SR_PHASE_FORWARD_RETRACT,  // 收腿
    SR_PHASE_FORWARD_SWEEP,    // 扫腿
    SR_PHASE_REVERSE_SWING,    // 摆腿
    SR_PHASE_REVERSE_RETRACT,  // 反转收腿
    SR_PHASE_REVERSE_STAND,    // 立腿
    SR_PHASE_VERIFY,           // 起立稳定确认
    SR_PHASE_HANDOFF,          // 交接平衡
    SR_PHASE_FAILED,           // 最终失败 零出力等待失能再使能
} self_rescue_phase_t;

// 自救内部进度只读诊断；遥测任务在临界区内与机体状态一起取样，不推进状态机。
typedef struct
{
    uint8_t round_count;                 // 已完成的正反路径对数
    bool second_path;                    // 本轮是否正在尝试备用路径
    uint32_t attempt_ms;                 // ms 当前尝试自准备或换向起的累计时间
    uint16_t stall_lft_ms, stall_rgt_ms; // ms 左右腿连续缺少运动进展的累计时间
    uint16_t retract_ms;                 // ms 障碍换向收腿阶段累计时间
} self_rescue_diag_t;

extern pid_t sr_pid_leg_len_lft, sr_pid_leg_len_rgt;             // 自救腿长反馈
extern pid_t sr_pid_leg_symmetry;                                // 自救防劈叉反馈
extern pid_t sr_pid_leg_ang_lft, sr_pid_leg_ang_rgt;             // 自救摆角反馈
extern pid_t sr_pid_leg_omg_swing_lft, sr_pid_leg_omg_swing_rgt; // 自救摆腿角速度反馈
extern pid_t sr_pid_leg_omg_stand_lft, sr_pid_leg_omg_stand_rgt; // 自救立腿角速度反馈

self_rescue_diag_t App_Self_Rescue_Diagnostic(void);       // 只读内部计数，与阶段和目标量一起取样
void App_Self_Rescue_Pid_Init(void);                       // 上电及自救入场复位自救 PID
void App_Self_Rescue_Pid_Update(balance_ctrl_mode_t mode); // Balance 校验输入后每控制拍调用一次 只更新 PID
void App_Self_Rescue_Enter(void);                          // 锁存 App 静态初始配置 清重试次数 重新启动自救
void App_Self_Rescue_Tick(void);                           // 阶段推进 只写目标量
bool App_Self_Rescue_Finished(void);                       // 自救交接完成标志 主 FSM 动作阶段消费 自救 Enter 时清零
self_rescue_phase_t App_Self_Rescue_Get_Phase(void);       // vofa 遥测 getter
float App_Self_Rescue_Get_Dir(void);                       // vofa 遥测 getter +1 扫腿路径 -1 摆腿路径
float App_Self_Rescue_Trq_Scale(balance_ctrl_mode_t mode); // 本拍四个腿关节的力矩比例，0..1，只读
balance_ctrl_mode_t App_Self_Rescue_Ctrl_Mode(void);

#endif
