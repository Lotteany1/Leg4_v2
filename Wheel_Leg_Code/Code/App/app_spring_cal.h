/** @file  app_spring_cal.h  @brief 悬空单腿气弹簧压缩采集与停止条件  @version  v1.0  2026-11-22 */
#ifndef APP_SPRING_CAL_H
#define APP_SPRING_CAL_H

#include <stdbool.h>
#include <stdint.h>
#include <system_key.h>

#define SPRING_CAL_SPEED_MPS         0.010f  // m/s，匀速压缩目标
#define SPRING_CAL_OVERSPEED_MPS     0.045f  // m/s，双向暂停加力速度
#define SPRING_CAL_START_SHORT_TOL_M 0.010f
#define SPRING_CAL_START_TOL_M       0.003f  // m，初始长度相对标定长端的容差
#define SPRING_CAL_STOP_MARGIN_M     0.004f  // m，短端前撤力的余量
#define SPRING_CAL_SAMPLE_MARGIN_M   0.008f  // m，长短两端各排除的采样范围

// 一次入场只允许一条压缩行程，终止后须显式复位才能再次加载。
typedef enum
{
    SPRING_CAL_IDLE = 0, // 等待首拍检查满伸就绪
    SPRING_CAL_RUNNING,  // 从设定起始力起步，跟随慢速收腿目标
    SPRING_CAL_DONE,     // 已到短端前的停止线并撤力
    SPRING_CAL_STOPPED   // 试验条件丢失，已撤力
} spring_cal_phase_t;

// 锁存停止原因，供 VOFA 区分完整行程与中止。
typedef enum
{
    SPRING_CAL_REASON_NONE = 0,   // 正常采集中
    SPRING_CAL_REASON_END,        // 到达短端停止线
    SPRING_CAL_REASON_START,      // 起点偏离长端或未静止
    SPRING_CAL_REASON_FEEDBACK,   // 反馈、时序、使能或配置无效
    SPRING_CAL_REASON_LIMIT,      // 关节出力或发送受限
    SPRING_CAL_REASON_FORCE,      // 请求或电机回读达到轴力边界
    SPRING_CAL_REASON_SPEED,      // 伸缩速度超过边界
    SPRING_CAL_REASON_STALL,      // 起始预载未克服或行程中途卡住
    SPRING_CAL_REASON_TIMEOUT,    // 达到总时间边界
    SPRING_CAL_REASON_RANGE,      // 腿长越出测量范围
    SPRING_CAL_REASON_RELEASE,    // 操作者退出本次采集
    SPRING_CAL_REASON_ANGLE       // 台架上的腿角越出允许范围
} spring_cal_reason_t;

typedef struct
{
    uint32_t tick_ms;            // ms，本次控制时标，可自然回绕
    uint32_t sample_tick;        // 同侧两关节均有新反馈才改变的样本标记，可回绕
    float length, rate;          // m、m/s，实测腿长与速度，伸腿为正
    float motor_force;           // N，两个电机回读力矩逆解的轴力，伸腿为正
    float min_length, max_length; // m，本次台架允许的短端和满伸位置
    bool valid;                  // 本侧两关节反馈新鲜、允许动作且无外部退出
    bool limited;                // 前一拍关节力矩限幅或未完整接纳
} spring_cal_input_t;

typedef struct
{
    spring_cal_phase_t phase;     // 当前动作阶段
    spring_cal_reason_t reason;   // 完成或停止原因
    double compression;            // N，限速修正后的电机收腿力幅值
    float motor_command;          // N，本拍直接给 VMC 的电机轴力，伸腿为正
    bool sample_ok;               // 当前反馈在稳速行程内，离线按样本标记去重
    bool hold_load;               // 进入短端后保持当时的加载力，等待拨杆
    float window_length;          // m，计算低速压缩进度的样窗起点
    uint32_t last_ms;   // ms，上一控制拍
    uint32_t sample_tick;         // 最近消费的成对回读标记
    uint32_t sample_ms, steady_ms; // ms，最后新回读时刻和稳速窗口起点
} spring_cal_t;

// 台架入场前清会话及锁存结果，调用者每侧独占一个状态实例。
void App_Spring_Cal_Reset(spring_cal_t *s);
// 控制任务处理同拍执行错误或操作者退出，撤力并保留首次终止原因。
void App_Spring_Cal_Stop(spring_cal_t *s, spring_cal_reason_t reason);
// 控制任务每拍调用一次，更新本拍电机轴力及采样资格，完成或失败后持续输出零。
void App_Spring_Cal_Update(spring_cal_t *s, const spring_cal_input_t *in);

#endif
