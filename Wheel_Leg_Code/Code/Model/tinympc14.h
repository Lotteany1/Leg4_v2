/**
 * @file  tinympc14.h
 * @brief 融合 MPC 单文件包对外接口：求解器 异步 输出层装配 端口
 * @version  v1.4  2026-11-22
 *
 * 数据表只有 tinympc14_data.h 一份（单点模型 + LPV 系数 + ZOH 系数），实现只有
 * tinympc14.c 一份。三笔补偿的物理量由固件侧数学库给出：MgF 查 Leg_Comp，
 * Fs 取 Leg_SpringF，Fc 用本头的 TINYMPC14_M_SUP / H_OFF / ROLL_TRACK 常量。
 */

#ifndef TINYMPC14_H
#define TINYMPC14_H

#include "tinympc14_data.h"

#include <math.h>
#include <stdint.h>

#if !defined(TINYMPC14_IFACE_VER) || (TINYMPC14_IFACE_VER != 3)
#error tinympc14 数据表口径与固件不匹配 重跑 s03
#endif

#if defined(TINYMPC14_ONLINE_IFACE_VER)
#if TINYMPC14_ONLINE_IFACE_VER != 1
#error incompatible tinympc14 online model
#endif
#elif !defined(TINYMPC14_LPV_IFACE_VER) || (TINYMPC14_LPV_IFACE_VER != 4)
#error tinympc14 LPV 表口径不匹配 重跑 s04d
#endif

#if !defined(TINYMPC14_ZOH_IFACE_VER) || (TINYMPC14_ZOH_IFACE_VER != 1)
#error tinympc14 ZOH 表口径不匹配 重跑 s07_zoh_data
#endif

#define TINYMPC14_NN            (TINYMPC14_NH - 1)
#define TINYMPC14_NW            (TINYMPC14_NX * TINYMPC14_NH)
#define TINYMPC14_NUU           (TINYMPC14_NU * TINYMPC14_NN)
/* Old data packages keep their original convergence controls. */
#ifndef TINYMPC14_MAX_ITER
#define TINYMPC14_MAX_ITER      100
#endif
#ifndef TINYMPC14_PRI_TOL
#define TINYMPC14_PRI_TOL       1e-4f
#endif
#ifndef TINYMPC14_DUA_TOL
#define TINYMPC14_DUA_TOL       1e-3f
#endif
#ifndef TINYMPC14_ROLL_TIP_GAIN
#define TINYMPC14_ROLL_TIP_GAIN 0.0f
#endif
#define TINYMPC14_ASYNC_PERIOD_MS 4u
#define TINYMPC14_NODE_TICKS    2u
#ifndef TINYMPC14_L_MIN
#define TINYMPC14_L_MIN         0.13f   /* m 调度腿长下限 */
#define TINYMPC14_L_MAX         0.32110177111662558f   /* m 调度腿长上限 */
#endif
#define TINYMPC14_L_STEP        0.005f  /* m 模型重算阈值 */
#define TINYMPC14_PSI_STEP      0.05f   /* rad 腿角重算阈值 */
#define TINYMPC14_SCHED_L       0.27f   /* m 初始调度点 */

#define TINYMPC14_OUTPUT_NU 6
#define TINYMPC14_OUTPUT_HOLD_N 5u   /* 拍 失效笔沿用上次有效值的上限，5 拍 @2 ms = 10 ms */

#define TINYMPC14_OUTPUT_ITEM_MGF 0
#define TINYMPC14_OUTPUT_ITEM_FS  1
#define TINYMPC14_OUTPUT_ITEM_FC  2
#define TINYMPC14_OUTPUT_NITEM    3

#define TINYMPC14_OUTPUT_ST_INVALID 0u   /* 无可用值 输出 0 */
#define TINYMPC14_OUTPUT_ST_FRESH   1u   /* 本拍有效 */
#define TINYMPC14_OUTPUT_ST_HELD    2u   /* 沿用上次有效值 */

typedef struct
{
    int   iters;       /* 迭代次数 */
    int   converged;   /* 1 = 收敛 */
    float pri;         /* 原始残差 */
    float dua;         /* 对偶残差 */
} tinympc14_status_t;

typedef struct
{
    unsigned nan;              /* 入口非有限拒收 */
    unsigned stale;            /* 序列超龄 */
    unsigned fallback;         /* 回退拍数 */
    unsigned commit;           /* 序列提交 */
    unsigned lpv_updates;      /* 模型重算 */
    unsigned pinv_degenerate;  /* 伪逆退化 */
    unsigned node_repeat;      /* 节点重复 */
    unsigned solve_attempt;    /* 后台已取得活动输入；每次最终落入 commit 或以下一个拒绝分支 */
    unsigned reject_start_age; /* 计算开始前输入已超预测时域 */
    unsigned reject_input;     /* 快照含非有限值或调度模型构造失败 */
    unsigned reject_convergence; /* 求解器未返回收敛序列 */
    unsigned reject_epoch;     /* 已收敛，但接地代次已取消或更换 */
    unsigned reject_finish_age; /* 同代次收敛，发布时输入已超预测时域 */
    uint32_t solve_age_ms;     /* 最近一次活动尝试返回时的样本年龄，含等待调度时间 */
} tinympc14_counters_t;

typedef struct
{
    float MgF[2];                  /* N·m 髋重力前馈 [左;右] */
    float Fs[2];                   /* N 气弹簧支持力 [左;右] */
    float Fc[2];                   /* N 向心差动 [左=−Fc;右=+Fc] */
    float ffwd[TINYMPC14_OUTPUT_NU]; /* reserved compatibility field; always zero */
    unsigned char st_MgF[2];       /* 单笔状态 TINYMPC14_OUTPUT_ST_* [左;右] */
    unsigned char st_Fs[2];        /* 单笔状态 TINYMPC14_OUTPUT_ST_* [左;右] */
    unsigned char st_Fc[2];        /* 单笔状态 TINYMPC14_OUTPUT_ST_* [左;右] */
} tinympc14_comp_t;

typedef struct
{
    float v[TINYMPC14_OUTPUT_NITEM][2];             /* 上次有效值 MgF Fs Fc [左;右] */
    unsigned char valid[TINYMPC14_OUTPUT_NITEM][2]; /* 1 = 存在上次有效值 */
    unsigned char age[TINYMPC14_OUTPUT_NITEM][2];   /* 距上次有效拍数 饱和 255 */
} tinympc14_comp_state_t;

typedef struct
{
    float u_applied[TINYMPC14_OUTPUT_NU]; /* MPC coordinates; total force includes Fc and Fs */
    float T_w[2];                    /* wheel actuator sign: -u_mpc[0:2] */
    float Tp[2];                     /* u_mpc[2:4]+MgF */
    float F_motor[2];                /* clipped(u_mpc[4:6]+Fc)-Fs */
} tinympc14_output_t;

static inline float Tinympc14_Clip(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline int Tinympc14_All_Finite(const float *v, int n)
{
    int i;

    for (i = 0; i < n; i++)
    {
        if (!isfinite(v[i]))
        {
            return 0;
        }
    }

    return 1;
}

extern volatile tinympc14_status_t   tinympc14_status;
extern volatile tinympc14_counters_t tinympc14_counters;
extern volatile float tinympc14_u[TINYMPC14_NU];   /* model inputs; Compensated consumer excludes the supplied Fc */

uint32_t Tinympc14_Port_Tick(void);
void Tinympc14_Port_Enter_Critical(void);
void Tinympc14_Port_Exit_Critical(void);

void Tinympc14_Init(void);
void Tinympc14_Seed(float u_prev[TINYMPC14_NU]);
/* Measured lengths (m) and IMU roll (rad) for the roll-inertia schedule. */
int  Tinympc14_Model_Update_Roll(float lL, float lR, float psi, float roll);
int  Tinympc14_Model_Update(float lL, float lR, float psi);
int  Tinympc14_Solve(const float x0[TINYMPC14_NX],
                        const float xref[TINYMPC14_NX],
                        const float fed[TINYMPC14_NX],
                        const float ffwd[TINYMPC14_NU],
                        const float u_prev[TINYMPC14_NU],
                        float u_tot[TINYMPC14_NU]);
const tinympc14_model_t *Tinympc14_Model(void);
const float *Tinympc14_Sequence(void);
/* 发布式仿射计算：异步求解任务取同一快照时用，控制拍不需要直接调用 */
void Tinympc14_Affine_Pub(const float xref[TINYMPC14_NX],
                             const float fed[TINYMPC14_NX],
                             const float ffwd[TINYMPC14_NU],
                             float u_ss[TINYMPC14_NU],
                             float u_ff[TINYMPC14_NU]);

/* Add the old leg-MPC tipping disturbance to fed; roll [rad], height [m]. */
void Tinympc14_Roll_Disturbance(float roll, float height, float fed[TINYMPC14_NX]);

void Tinympc14_Async_Run(void);
void Tinympc14_Async_Invalidate(void);
void Tinympc14_Async_Input(const float x0[TINYMPC14_NX],
                              const float xref[TINYMPC14_NX],
                              const float fed[TINYMPC14_NX],
                              const float ffwd[TINYMPC14_NU],
                              float lL, float lR);
/* Frozen observation timestamp in the same wrapping millisecond clock as Port_Tick.
 * Firmware callers must pass the estimator timestamp; Async_Input is for sampling now. */
void Tinympc14_Async_Input_At(const float x0[TINYMPC14_NX],
                            const float xref[TINYMPC14_NX],
                            const float fed[TINYMPC14_NX],
                            const float ffwd[TINYMPC14_NU],
                            float lL, float lR, uint32_t sample_tick);
/* 同一观测拍交接状态和执行力偏差。error = 电机反馈反算轴力 + 被动弹簧估计 − 上拍接纳总力 [N]。
 * valid 的位 0/1 表示左/右回读已通过接纳时标配对。仅首个预测区间使用该偏差。 */
void Tinympc14_Async_Input_Force_At(const float x0[TINYMPC14_NX], const float xref[TINYMPC14_NX],
                                    const float fed[TINYMPC14_NX], const float ffwd[TINYMPC14_NU],
                                    float lL, float lR, uint32_t sample_tick, const float error[2], unsigned int valid);
/* 同一估计拍中的物理力；turn 为转弯外扰，comp 为执行侧转弯补偿，均按左、右列出。 */
typedef struct
{
    float turn[2], comp[2], error[2]; // N，error 为已建立总力减接纳总力
    float height;                    // m，倾覆估计使用的绝对高度
    unsigned int valid;              // 已配对测力位，位 0 左，位 1 右
    float pending_diff_impulse;      // N·s，本拍尚待建立的左减右差力冲量，已扣除首节点动态修正
} tinympc14_ground_observation_t;
/* x0[2] 为实际连续航向减积分目标 [rad]，xref[2] 为零，xref[3] 为最终转速 [rad/s]。
 * 前台与后台按 TINYMPC14_TS 扣除目标推进量；通用 Input/Solve 接口仍由调用方提供 fed。 */
void Tinympc14_Async_Input_Ground_At(const float x0[TINYMPC14_NX], const float xref[TINYMPC14_NX],
                                     float lL, float lR, uint32_t tick, const tinympc14_ground_observation_t *observation);
float Tinympc14_Async_Roll_Increment(void); // rad/s，最近前台按其实际模型投影得到的滚速增量
void Tinympc14_Async_Solve(void);
void Tinympc14_Async_Step(void);
/* Consume a prediction node at the caller's control period (ms). */
void Tinympc14_Async_Step_Period(uint32_t control_ms);
/* force_comp: left/right axial feedforward [N]. Limit accepted total force at control_ms;
 * Call after Input, before yielding to the solve task: snapshot Fc for planner constraints.
 * tinympc14_u excludes this compensation. Pass the same Fc to Output_Apply_Period once. */
void Tinympc14_Async_Step_Compensated(uint32_t control_ms, const float force_comp[2]);
/* Ground descent: 0..1 share of relief after the attitude guard. Returns common force removed [N]. */
float Tinympc14_Async_Step_Lowering(uint32_t control_ms, const float force_comp[2], float lower_share);
/* Call after external compensation/final actuator limits, using applied MPC coordinates. */
void Tinympc14_Async_Applied(const float u_applied[TINYMPC14_NU]);

void Tinympc14_Output_Reset(tinympc14_comp_state_t *st);
void Tinympc14_Output_Calc(float len_l, float len_r, float ang_l, float ang_r,
                           float v, float omega, tinympc14_comp_t *c);
void Tinympc14_Output_Calc_Ex(float len_l, float len_r, float ang_l, float ang_r,
                              float v, float omega, tinympc14_comp_state_t *st, tinympc14_comp_t *c);
void Tinympc14_Output_Apply(const float u_mpc[TINYMPC14_OUTPUT_NU], const tinympc14_comp_t *c,
                            const float u_prev_applied[TINYMPC14_OUTPUT_NU], tinympc14_output_t *out);
/* Apply compensation using the same period and realized input anchor. */
void Tinympc14_Output_Apply_Period(const float u_mpc[TINYMPC14_OUTPUT_NU], const tinympc14_comp_t *c,
                                   const float u_prev_applied[TINYMPC14_OUTPUT_NU], uint32_t control_ms,
                                   tinympc14_output_t *out);

#endif
