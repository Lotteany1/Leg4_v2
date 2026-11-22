/** @file  distributor.h  @brief 分配发送、接纳历史与后台输入接口  @version  v1.10  2026-11-16 */
#ifndef DISTRIBUTOR_H
#define DISTRIBUTOR_H
#include "code_config.h"
#include "system_key.h"
#include <stdbool.h>
#include <stdint.h>

// 分配与执行记录
// 能力界可由隔离构建覆盖；标定时使用同一上下界参与分配和编码，协议量程不能代替能力依据。
#ifndef DISTRIBUTOR_LF_MIN
#define DISTRIBUTOR_LF_MIN DM_T_MIN // N·m 当前软件限幅，非已标定电机能力
#endif
#ifndef DISTRIBUTOR_LF_MAX
#define DISTRIBUTOR_LF_MAX DM_T_MAX // N·m 左前上界
#endif
#ifndef DISTRIBUTOR_LB_MIN
#define DISTRIBUTOR_LB_MIN DM_T_MIN // N·m 左后下界
#endif
#ifndef DISTRIBUTOR_LB_MAX
#define DISTRIBUTOR_LB_MAX DM_T_MAX // N·m 左后上界
#endif
#ifndef DISTRIBUTOR_RF_MIN
#define DISTRIBUTOR_RF_MIN DM_T_MIN // N·m 右前下界
#endif
#ifndef DISTRIBUTOR_RF_MAX
#define DISTRIBUTOR_RF_MAX DM_T_MAX // N·m 右前上界
#endif
#ifndef DISTRIBUTOR_RB_MIN
#define DISTRIBUTOR_RB_MIN DM_T_MIN // N·m 右后下界
#endif
#ifndef DISTRIBUTOR_RB_MAX
#define DISTRIBUTOR_RB_MAX DM_T_MAX // N·m 右后上界
#endif

#define DISTRIBUTOR_S_MIN     20.0f                               // N NORMAL 投影的软支撑下界，不抬高更低的请求
#define DISTRIBUTOR_S_MAX     150.0f                              // N NORMAL 投影的软支撑上界，不压低更高的请求
#define DISTRIBUTOR_S_RESERVE (0.5f * LEG_MASS_SUPPORT * GRAVITY) // N 直立时单腿名义重力支撑，饱和时保留至请求值，允许上游主动卸载
#define DISTRIBUTOR_S_SCALE   100.0f                              // N 支持力归一化尺度
#define DISTRIBUTOR_P_SCALE   20.0f                               // N·m 髋任务归一化尺度
#define DISTRIBUTOR_S_WEIGHT  1.0f                                // 无量纲 支持力误差权重
#define DISTRIBUTOR_P_WEIGHT  2.0f                                // 无量纲 髋误差权重
#define DISTRIBUTOR_TOL       2e-6f                               // 无量纲 归一化半平面数值容差

#ifndef EXECUTION_CAPACITY
#define EXECUTION_CAPACITY 64u // 控制拍记录容量，后台过龄另行判断
#endif
#define EXECUTION_CHANNEL_MASK   63u // LF/LB/RF/RB/WL/WR 六路有效位
#define EXECUTION_MAX_AGE_MS     30u // ms 后台允许读取的最大区间年龄

#define EXECUTION_TIME_MIN_RATIO 0.95f // 名义观测间隔下限比例
#define EXECUTION_TIME_MAX_RATIO 1.05f // 名义观测间隔上限比例

#define EXECUTION_PITCH_BUDGET   0.001f  // rad 单区间 pitch 预算

#define EXECUTION_PITCH_BOUND    {7.500249737e-07f, 7.500249737e-07f, 6.117911997e-06f, 6.117911997e-06f}

#define DISTRIBUTOR_LEG_NUM       2u // 左、右腿数量
#define DISTRIBUTOR_JOINT_NUM     4u // LF、LB、RF、RB 关节数量
#define DISTRIBUTOR_TASK_NUM      2u // 单腿 [总支持力 S,净髋力矩 p]

#define EXECUTION_QUALITY_ALIGN   1u // 首拍或模式交接，仅对齐测量
#define EXECUTION_QUALITY_HISTORY 2u // 执行记录缺段或保持码未知
#define EXECUTION_QUALITY_TIME    4u // 观测间隔不符合固定模型节点
#define EXECUTION_QUALITY_RANGE   8u // 均值误差超出允许域

// 三种策略共用输入、电机能力界与任务误差度量；PROJECT 另保留 NORMAL 支撑范围。
typedef enum
{
    DISTRIBUTOR_PROJECT = 0, // 带总支持力边界的二维加权有限候选
    DISTRIBUTOR_CLIP,        // 逐电机截断后恢复任务
    DISTRIBUTOR_SCALE        // 从零电机出力沿请求缩放
} distributor_strategy_t;

// 连续分配状态；协议码与发送接纳在驱动边界独立检查。
typedef enum
{
    DISTRIBUTOR_STATUS_INVALID = 0, // 输入非法，结果无效
    DISTRIBUTOR_STATUS_BYPASS,      // 非 NORMAL，不参与支撑分配
    DISTRIBUTOR_STATUS_NOMINAL,     // 名义请求已在界内
    DISTRIBUTOR_STATUS_PROJECTED,   // 超界请求已分配
    DISTRIBUTOR_STATUS_FALLBACK,    // 有效映射下候选失败，退回截断
    DISTRIBUTOR_STATUS_CLIPPED,     // 超界请求采用逐电机截断
    DISTRIBUTOR_STATUS_SCALED       // 超界请求采用零电机出力方向缩放
} distributor_status_t;

// 单腿同拍快照；右腿由调用方完成行交换和髋列反号，不读取全局状态。
typedef struct
{
    float map[2][2]; // A 前、后电机两行；F、Tp 两列单位 m、1
    float s_req;     // 总支持力 S = F + Fs，N，已含横滚
    float p_req;     // 净髋请求 p = Tp - cp，N·m，模型髋坐标
    float spring_f;  // 正被动弹簧 Fs，N，不沿用旧补偿字段的负号
    float hip_comp;  // 实际接入髋补偿 cp，N·m，不重复加入 bh
    float tau_lo[2]; // 最终电机坐标前、后力矩下界，N·m
    float tau_hi[2]; // 最终电机坐标前、后力矩上界，N·m
} distributor_leg_t;

// 模式合成与退出斜坡结束后冻结，分配后不得再叠加未预算前馈。
typedef struct
{
    uint32_t state_tick;             // 1 kHz 状态快照 tick
    uint32_t seq;                    // 请求序号
    uint32_t epoch;                  // 模式代次
    bool height_priority;            // true 时支撑上界不超过本腿请求，能力无交集时仅放松到最近可达值
    bool normal_mode;                // 仅 NORMAL 为 true
    distributor_strategy_t strategy; // 本次策略
    float dt;                        // s，本拍间隔，有限且大于零
    distributor_leg_t leg[2];        // 左、右腿同拍数据
} distributor_input_t;

// 连续结果不代表协议量化、总线接纳或轴端实测，不是平衡模型四输入。
typedef struct
{
    uint32_t state_tick;          // 输入状态 tick，不是发送完成
    uint32_t seq;                 // 对应请求序号
    uint32_t epoch;               // 对应模式代次
    distributor_status_t status;  // 计算状态
    bool result_valid;            // true 时 alloc 字段有效
    float tau_req[4];             // 名义电机力矩，N·m，LF/LB/RF/RB
    float s_alloc[2];             // 分配总支持力，N，左/右
    float p_alloc[2];             // 分配净髋，N·m，左/右
    float tau_alloc[4];           // 分配电机力矩，N·m，LF/LB/RF/RB
    float task_error[2][2];       // 左右 [S,p] 误差，N、N·m
    float objective[2];           // 左右归一化加权代价
    uint8_t active_mask;          // 位 0..7 为各电机下界/上界
    uint8_t request_mask;         // 位 0..7 为原请求越过各电机下界/上界，LF/LB/RF/RB 顺序
    uint8_t support_relaxed_mask; // 位 0/1 为左/右关节能力无法满足支撑范围
    float request_residual;       // 请求最大正残差，N·m
    float max_residual;           // 分配最大正残差，N·m
} distributor_result_t;

// 与请求同拍冻结的控制状态，仅供诊断；角度 rad，角速度 rad/s，长度 m。
typedef struct
{
    float pitch;      // 机体俯仰角
    float pitch_rate; // 机体俯仰角速度
    float roll;       // 机体横滚角
    float roll_rate;  // 机体横滚角速度
    float height;     // 机体几何高度
    float height_ref; // 机体目标高度
    float len[2];     // 左、右实测机构腿长
} distributor_state_t;

typedef struct
{
    float nominal[2];    // N 左右名义总力 S0，含横滚前馈
    float spring_scale;  // 0～1 左右腿共用的轴向气弹簧补偿比例
    uint32_t elapsed_ms; // ms 本轮落地已持续时间
    bool active;        // 触地当拍或气弹簧补偿仍在恢复
} distributor_landing_t;

// NORMAL 合成和退出斜坡完成后的只读帧；顺序均为左、右，不再读取后台候选。
typedef struct
{
    uint32_t tick;     // 状态快照 tick，ms
    uint32_t cycle;    // 状态快照 DWT，CPU cycle
    float phi[2][2];   // 各腿原 VMC 几何角，rad
    float force[2];    // 执行器轴向请求 F，N
    float hip[2];      // 已含实际补偿的 Tp，N·m
    float spring_f[2]; // Fs = -SpringF，被动弹簧力，N
    float hip_comp[2]; // 本拍实际叠加的 cp，N·m
    float turn[2];     // 外部转弯扰动力，N，不叠加到电机
    float wheel[2];    // 原 Balance 轮力矩请求 T，N·m，保持原发送极性
    bool height_priority; // 与已发布腿长总力同拍的高度优先标记
    bool stepping;       // 动作模式必须满足轴向上界，分配失败由控制层安全退出
    distributor_landing_t land; // 本拍保护进度，仅供诊断
    distributor_state_t state; // 与本拍请求对应的状态，不从遥测任务重新读取
} distributor_frame_t;

// 一拍五次 API 接纳事件；cycle 只在短历史窗口内按 uint32 差使用，不是永久绝对时间。
typedef struct
{
    uint32_t seq;          // 环形记录序号
    uint32_t epoch;        // 模式代次
    uint32_t tick;         // 状态快照 1 kHz tick
    uint32_t cycle;        // 状态快照 DWT 计数，可回绕
    uint32_t offset[5];    // 五次发送 API 返回相对状态快照的 CPU 周期数
    float map[2][2][2];    // 同拍最终电机坐标 A，m、1
    float spring_f[2];     // 正被动弹簧 N
    float hip_comp[2];     // 净髋恢复需扣的 cp，N·m
    float turn[2];         // 外部转弯等效力 N，不进入电机命令
    int32_t before[6];     // 本拍开始时各通道最近接纳码
    int32_t code[6];       // 本拍提交的码，失败通道仍以 before 为准
    float model_u[4];      // 末次接纳保持码的名义模型输入，N·m
    float total_s[2];      // 同拍姿态恢复的末次总支持力，N
    uint8_t valid_mask;    // 拍尾有效位，未全部有效时等效输入无效
    uint8_t valid_before;  // 本拍开始的可信码掩码
    uint8_t accepted_mask; // 本拍 API 入队成功通道，DJI 两轮成组
} execution_sample_t;

// 分配、状态与本拍接纳记录统一发布；累计统计逐控制拍更新，不受遥测降采样影响。
typedef struct
{
    distributor_input_t input;       // 本拍请求、映射和能力
    distributor_result_t result;     // 实际选择的连续结果
    distributor_result_t compare[2]; // 可选比较：另一策略（PROJECT/CLIP）、零电机出力缩放；检查 result_valid 后使用
    distributor_state_t state;       // 输入状态时标对应的机体与腿长状态
    distributor_landing_t land;      // 气弹簧补偿恢复诊断
    uint8_t land_limited_mask;       // 位 0/1 左/右保护目标受约束或接纳结果未达到
    execution_sample_t execution;    // 本次发送接纳记录；valid_mask 不表示轴端已执行
    float wheel_req[2];              // 左、右轮执行层请求，N·m，最终保护前
    uint32_t mode;                   // 最近控制模式，非 NORMAL 时 result 无效
    uint32_t normal_count;           // NORMAL 分配发送尝试累计拍数，包含输入或编码失败
    uint32_t failure_reason;         // 本拍失败原因：0 无，1 输入/映射非法，2 能力界无合法码，3 动作轴向界不可达
    uint32_t failure_count;          // 无效映射或空码区累计拍数
    uint32_t protection_count;       // 分配后末端保护改变连续命令累计通道数
    uint32_t limited_count;          // 成功编码拍中关节请求超界累计拍数
    uint32_t changed_count;          // 成功编码拍中任一关节连续力矩发生变化的累计拍数
    uint32_t fallback_count;         // 成功编码拍中候选失败退回截断的累计拍数
    uint32_t send_failure_count;     // 任一路 CAN API 未接纳的累计拍数
    uint32_t wheel_limited_count;    // 任一轮请求超出最终软件力矩界的累计拍数
    uint32_t support_relaxed_count;  // 成功编码拍中任一腿支撑范围不可达的累计拍数
    uint32_t joint_limited_count[8]; // 各关节下界/上界越界累计拍数，LF/LB/RF/RB 顺序
    uint32_t limited_run;            // 当前连续越界长度，单位控制拍；无效拍或退出 NORMAL 中断连续段
    uint32_t limited_run_max;        // 最长连续越界长度，单位控制拍，不等同于实测毫秒
    float tau_change_max;            // 成功编码拍中最大关节改变量绝对值，N·m
    float s_error_max;               // 成功编码拍中最大支持任务误差绝对值，N
    float p_error_max;               // 成功编码拍中最大净髋任务误差绝对值，N·m
    float mapping_error;             // 对原 VMC 的最大差，N·m
    float mapping_us;                // 原 VMC 和单位输入映射耗时，us
    float allocation_us;             // 所选策略耗时，us
    float send_us;                   // 五次发送到接纳记录发布耗时，us
} distributor_diag_t;

// 每个后台独立保存；末值用于速率锚，均值只用于观测传播。
typedef struct
{
    uint32_t epoch;           // 对应代次
    uint32_t seq;             // 最后扫描的记录序号
    float model_u[4];         // 区间时间加权 WL/WR/BL/BR，N·m
    float total_s[2];         // 时间加权总支持力，N
    float turn[2];            // 时间加权外部扰动，N
    float end_s[2];           // 区间末最近可信总支持力，N
    float min_u[4], max_u[4]; // 区间输入范围，N·m，均值误差预算使用
    float min_s[2], max_s[2]; // 区间总支持力加外扰范围，N
    bool complete;            // false 时不得用请求替代缺段
} execution_interval_t;

// 后台只在同代提交后发布，0 为平衡，1 为腿长；计数不因模式切换清零。
typedef struct
{
    uint32_t epoch;   // 最近已提交观测代次
    uint32_t tick;    // 最近已消费测量 tick
    uint32_t quality; // 0 为可学习，否则为 EXECUTION_QUALITY 位组合
    uint32_t skipped; // 跳过扰动学习的累计区间数
    uint32_t learned; // 完整可信且通过质量闸的累计区间数
} execution_observer_diag_t;

// 输入只读、输出独占不重叠；控制任务每拍调用，不在 ISR 使用，不发送也不推进观测器。
void Distributor_Calculate(const distributor_input_t *input, distributor_result_t *result);
// A、Fs、cp 已由分配入口验证；tau 为最终电机坐标，输出总 S 和净 p，不重复验证上游几何。
void Distributor_Recover(const distributor_leg_t leg[2], const float tau[4], float total_s[2], float hip[2]);

void Execution_Observation(uint32_t observer, uint32_t epoch, uint32_t tick, uint32_t quality); // 后台提交点调用一次，旧代不发布
void Execution_Observer_Diagnostic(execution_observer_diag_t out[2]);                           // 遥测原子复制两个后台状态
void Execution_Init(void);                                                                      // 上电清零，早于两个后台启动
void Execution_Mode(uint32_t mode);                                                             // 控制任务每拍模型调用前登记，变化时作废持有码并递增代次
void Execution_Invalidate(void);                                                                // 控制/总线恢复任务作废历史；不发送、不锁存失能
uint32_t Execution_Epoch(void);                                                                 // 短临界区同步传输切代并读取统一代次，变化时作废保持码与历史
void Execution_Begin(execution_sample_t *sample);                                               // 控制单生产者填写映射前取得 epoch、旧码和有效位
void Execution_Publish(execution_sample_t *sample);                                             // 五次发送结束后发布；不重复编码
// 读取同代过去区间，begin/end 为 DWT 计数，end_tick 为当前测量 tick；cycles_per_ms 为 CPU 频率/1000。
// 后台各自持有 result/cursor，短临界区逐条复制，积分不关中断；返回值在 result.complete 中。
void Execution_Read(uint32_t epoch, uint32_t begin, uint32_t end, uint32_t end_tick, uint32_t cycles_per_ms, execution_interval_t *result);
void Execution_Latest(execution_sample_t *sample); // 遥测只复制已生成记录

// 早于后台启动初始化历史和分配诊断；请求序号沿用原控制生命周期。
void Distributor_Init(void);
// 控制任务在模型调用前登记模式；非 NORMAL 同时使连续分配诊断失效。
void Distributor_Mode(uint32_t mode, bool normal_mode);
// NORMAL 单生产者调用；nominal 按原 VMC 行顺序返回，不修改 frame；失败交由 Balance_Stop。
bool Distributor_Send(const distributor_frame_t *frame, float nominal[2][2]);
// 复制已完成控制拍的诊断，不重新计算补偿或编码。
void Distributor_Diagnostic(distributor_diag_t *out);
uint8_t Distributor_Output_Limits(void); // 同拍关节、轮矩或支撑放宽限幅，控制任务只读
// 完整区间的观测质量判断，系数来源 tests/derive_execution_quality.py。
bool Execution_Quality_Time(uint32_t cycles, uint32_t frequency, uint32_t milliseconds);
bool Execution_Quality_Balance(const execution_interval_t *interval);
#endif
