/** @file  app_spring_bench.c  @brief 右腿悬空台架入档、成对关节出力与失能  @version  v1.1  2026-11-22 */
#include "app_spring_bench.h"

#if SPRING_CAL_BENCH_ENABLE
#include "app_safety.h"
#include "balance.h"
#include "body.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "fsm.h"
#include "imu.h"
#include "leg.h"
#include "Leg_Comp.h"
#include "Leg_Pos.h"
#include "Leg_Spd.h"
#include "Leg_Trq.h"
#include "wrap.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

#define BENCH_ENABLE_MS     120u  // ms，仅等待本次右侧使能后返回的反馈
#define BENCH_FRESH_MS        5u  // ms，右侧两个关节回读各自的最大年龄
#define BENCH_ROLL_MAX     0.25f  // rad，机身横滚上限
#define BENCH_ANGLE_MAX    1.57079632679f // rad，相对铅垂向下的允许摆角，含两端
#define BENCH_HIP_KP        3.0f  // N·m/rad，维持入场摆角
#define BENCH_HIP_KD        0.3f  // N·m·s/rad，摆向阻尼

static spring_cal_t bench_cal;
static spring_bench_frame_t bench_frame; // 控制任务发布，VOFA 在临界区读取
static bool bench_armed, bench_session, bench_off_sent;
static uint32_t bench_start, bench_off_tick, bench_run;
static uint32_t bench_front_cycle, bench_rear_cycle, bench_pair;
static float bench_angle;
static dm_motor_t *const bench_joints[] = {&dm_f_rgt, &dm_b_rgt, &dm_f_lft, &dm_b_lft};
static uint32_t bench_reply_after[4]; // 每台关节本次清错或使能前的反馈序号
// 每台分别推进，保留已确认的应答；等待期间只允许零力矩探测。
typedef enum
{
    BENCH_CLEAR,       // 发送清错
    BENCH_CLEAR_ACK,   // 等清错应答
    BENCH_ENABLE,      // 发送使能
    BENCH_ENABLE_ACK,  // 等本台使能应答
    BENCH_READY        // 已确认，等待全组反馈就绪
} bench_handshake_t;
static bench_handshake_t bench_step[4];
static uint32_t bench_tx_tick[4];


// 首次撤力时优先给右侧发送失能。队列满时下一拍重试，已入队仍定期重申。
static void App_Spring_Bench_Disable(uint32_t now, bool immediate)
{
    active_flag = APP_DISABLE;
    if (!immediate && bench_off_sent && now - bench_off_tick < APP_ENABLE_RETRY_MS) return;
    HAL_StatusTypeDef front = Dm_Motor_Disable(dm_f_rgt.handle, dm_f_rgt.slave_id);
    HAL_StatusTypeDef rear  = Dm_Motor_Disable(dm_b_rgt.handle, dm_b_rgt.slave_id);
    HAL_StatusTypeDef left_front = Dm_Motor_Disable(dm_f_lft.handle, dm_f_lft.slave_id);
    HAL_StatusTypeDef left_rear  = Dm_Motor_Disable(dm_b_lft.handle, dm_b_lft.slave_id);
    Dji_Motor_All_Stop();
    if (front == HAL_OK) dm_f_rgt.enabled = 0;
    if (rear == HAL_OK) dm_b_rgt.enabled = 0;
    if (left_front == HAL_OK) dm_f_lft.enabled = 0;
    if (left_rear == HAL_OK) dm_b_lft.enabled = 0;
    bench_off_tick = now;
    bench_off_sent = front == HAL_OK && rear == HAL_OK && left_front == HAL_OK && left_rear == HAL_OK;
}

// 结束本趟，不消除首个停止原因；只有双下拨杆才能重新准备入档。
static void App_Spring_Bench_End(uint32_t now, spring_cal_reason_t reason)
{
    if (bench_cal.phase != SPRING_CAL_DONE && bench_cal.phase != SPRING_CAL_STOPPED)
        App_Spring_Cal_Stop(&bench_cal, reason);
    bench_armed   = false;
    bench_session = false;
    App_Spring_Bench_Disable(now, true);
}

// 本拍新反馈和摆角均有效时才向右前后关节发力；先校验两个力矩再发送任意一帧。
static void App_Spring_Bench_Load(uint32_t now, spring_bench_frame_t *b)
{
    const bool paired = dm_f_rgt.frame_cycle != bench_front_cycle && dm_b_rgt.frame_cycle != bench_rear_cycle;
    bool fresh = true, enabled = true;
    b->enabled_mask = 0u;
    b->handshake = 0u;
    for (unsigned int j = 0; j < 4u; j++)
    {
        dm_motor_t *motor = bench_joints[j];
        const bool recent = now - motor->frame_tick <= BENCH_FRESH_MS;
        const bool reply = motor->frame_cycle != bench_reply_after[j];
        if (!recent) fresh = false;
        if (bench_cal.phase == SPRING_CAL_IDLE)
        {
            if (reply && bench_step[j] != BENCH_CLEAR && motor->err_code >= DM_ERR_OVERVOLT)
            {
                b->handshake |= (uint32_t)bench_step[j] << (4u * j);
                App_Spring_Bench_End(now, SPRING_CAL_REASON_FEEDBACK);
                return;
            }
            if (reply && bench_step[j] == BENCH_CLEAR_ACK)
                bench_step[j] = BENCH_ENABLE;
            if (reply && bench_step[j] == BENCH_ENABLE_ACK && motor->err_code == DM_ERR_ENABLE)
            {
                bench_step[j] = BENCH_READY;
                motor->enabled = 1u;
            }
            if (now - bench_tx_tick[j] >= APP_ENABLE_RETRY_MS)
            {
                if (bench_step[j] == BENCH_CLEAR_ACK) bench_step[j] = BENCH_CLEAR;
                if (bench_step[j] == BENCH_ENABLE_ACK) bench_step[j] = BENCH_ENABLE;
            }
            if (now - bench_start < BENCH_ENABLE_MS)
            {
                HAL_StatusTypeDef sent;
                if (bench_step[j] == BENCH_CLEAR || bench_step[j] == BENCH_ENABLE)
                {
                    sent = bench_step[j] == BENCH_CLEAR ? Dm_Motor_Clear_Error(motor->handle, motor->slave_id)
                                                        : Dm_Motor_Enable(motor->handle, motor->slave_id);
                    if (sent == HAL_OK)
                    {
                        bench_tx_tick[j] = now;
                        bench_reply_after[j] = motor->frame_cycle;
                        bench_step[j] = bench_step[j] == BENCH_CLEAR ? BENCH_CLEAR_ACK : BENCH_ENABLE_ACK;
                    }
                }
                else if (bench_step[j] == BENCH_READY && !recent)
                {
                    // DM 一问一答；早确认的电机需继续取得新回读，才能与最后一台共同进入出力。
                    Dm_Motor_Mit_Ctrl(motor->handle, motor->slave_id, 0, 0, 0, 0, 0);
                }
            }
        }
        b->handshake |= (uint32_t)bench_step[j] << (4u * j);
        if (motor->err_code == DM_ERR_ENABLE && bench_step[j] == BENCH_READY) b->enabled_mask |= 1u << j;
        else enabled = false;
    }
    if (bench_cal.phase == SPRING_CAL_IDLE && !(paired && fresh && enabled))
    {
        if (now - bench_start >= BENCH_ENABLE_MS) App_Spring_Bench_End(now, SPRING_CAL_REASON_FEEDBACK);
        return;
    }
    if (paired)
    {
        bench_front_cycle = dm_f_rgt.frame_cycle;
        bench_rear_cycle  = dm_b_rgt.frame_cycle;
        bench_pair++;
    }
    if (bench_cal.phase == SPRING_CAL_IDLE) bench_angle = b->angle;
    const float angle_error = Wrap_Pi(bench_angle - b->angle);
    if (!isfinite(b->angle) || !isfinite(b->roll) || !isfinite(b->omega) ||
        fabsf(b->angle) > BENCH_ANGLE_MAX ||
        fabsf(b->roll) > BENCH_ROLL_MAX)
    {
        App_Spring_Bench_End(now, SPRING_CAL_REASON_ANGLE);
        return;
    }
    const spring_cal_input_t in = {
        .tick_ms = now, .sample_tick = bench_pair, .length = b->len, .rate = b->rate,
        .motor_force = b->motor, .min_length = Leg_Len_Min, .max_length = SPRING_CAL_BENCH_START_M,
        .valid = fresh && enabled && isfinite(b->gravity) && isfinite(b->hip_gravity) && isfinite(b->hip)
    };
    App_Spring_Cal_Update(&bench_cal, &in);
    if (bench_cal.phase != SPRING_CAL_RUNNING)
    {
        App_Spring_Bench_End(now, bench_cal.reason);
        return;
    }
    const float hip = b->hip_gravity + BENCH_HIP_KP * angle_error - BENCH_HIP_KD * b->omega;
    float tau[2];
    Leg_Trq(bench_cal.motor_command, LEG_MIRROR_CHANGE * hip, leg_rgt.pitch_1, leg_rgt.pitch_4, tau);
    if (!isfinite(tau[0]) || !isfinite(tau[1]) || fabsf(tau[0]) >= DM_T_MAX || fabsf(tau[1]) >= DM_T_MAX)
    {
        App_Spring_Bench_End(now, SPRING_CAL_REASON_LIMIT);
        return;
    }
    // 左腿用零增益、零力矩帧维持反馈；任一关节发送失败，同拍撤销整趟。
    if (Dm_Motor_Mit_Ctrl(dm_f_lft.handle, dm_f_lft.slave_id, 0, 0, 0, 0, 0) != HAL_OK ||
        Dm_Motor_Mit_Ctrl(dm_b_lft.handle, dm_b_lft.slave_id, 0, 0, 0, 0, 0) != HAL_OK)
    {
        App_Spring_Bench_End(now, SPRING_CAL_REASON_LIMIT);
        return;
    }
    HAL_StatusTypeDef front = Dm_Motor_Mit_Ctrl(dm_f_rgt.handle, dm_f_rgt.slave_id, 0, 0, 0, 0, tau[1]);
    if (front != HAL_OK || Dm_Motor_Mit_Ctrl(dm_b_rgt.handle, dm_b_rgt.slave_id, 0, 0, 0, 0, tau[0]) != HAL_OK)
    {
        App_Spring_Bench_End(now, SPRING_CAL_REASON_LIMIT);
        return;
    }
    b->hip_command = hip;
    active_flag    = APP_ENABLE;
}

// 控制与估计握手保护同一拍的回读；超时路径只撤力，不读取正在变化的估计量。
void App_Spring_Bench_Tick(const dr16_t *rc)
{
    const uint32_t now = HAL_GetTick();
    const bool radio = rc != NULL && Dr16_Online();
    const bool online = radio && Imu_Online();
    const bool test = online && Fsm_Read_Request(rc).test;
    spring_bench_frame_t b = bench_frame;
    b.hip_command = 0.0f;
    if (rc != NULL)
    {
        // 常规估计在任一腿运动学无效时保留双腿旧值。台架每拍只从右侧编码器重新解算。
        float pos[2], vel[2];
        Leg_Pos(leg_rgt.pitch_1, leg_rgt.pitch_4, pos);
        Leg_Spd(leg_rgt.pitch_1, leg_rgt.pitch_4, leg_rgt.d_pitch_1, leg_rgt.d_pitch_4, vel);
        b.len = pos[0];
        b.rate = vel[0];
        b.angle = Wrap_Pi(PI_2 - pos[1] + body.ang_pitch);
        b.omega = body.omg_pitch - vel[1];
        b.roll = body.ang_roll;
        b.motor = body.F_rgt;
        b.hip = body.Tp_rgt;
        b.front = dm_f_rgt.trq;
        b.rear = dm_b_rgt.trq;
        if (isfinite(b.len) && isfinite(b.angle))
        {
            float offset, radius;
            Leg_Comp(b.len, &offset, &radius);
            // 悬空轮随腿绕髋轴转动；它的重力力臂取实测腿长，腿杆质心仍按现有表查。
            b.hip_gravity = GRAVITY * (LEG_GRAV_FF * sinf(b.angle + offset) * radius + WHEEL_MASS * b.len * sinf(b.angle));
            b.gravity = Balance_Axial_Gravity(b.len, b.angle);
        }
        else
        {
            b.hip_gravity = 0.0f;
            b.gravity = 0.0f;
        }
    }
    if (bench_session) b.elapsed_ms = now - bench_start;
    if (!test)
    {
        if (bench_session) App_Spring_Bench_End(now, online ? SPRING_CAL_REASON_RELEASE : SPRING_CAL_REASON_FEEDBACK);
        else App_Spring_Bench_Disable(now, false);
        if (!online) bench_armed = false;
        else if (rc->sw_left == DR16_SW_DOWN) bench_armed = true;
    }
    else
    {
        if (bench_armed)
        {
            App_Spring_Cal_Reset(&bench_cal);
            bench_armed       = false;
            bench_session     = true;
            bench_start       = now;

            bench_front_cycle = dm_f_rgt.frame_cycle;
            bench_rear_cycle  = dm_b_rgt.frame_cycle;
            bench_pair        = 0u;
            for (unsigned int j = 0; j < 4u; j++) bench_step[j] = BENCH_CLEAR;
            b.enabled_mask = 0u;
            bench_run++;
            b.elapsed_ms = 0u;
        }
        if (bench_session) App_Spring_Bench_Load(now, &b);
        else App_Spring_Bench_Disable(now, false);
    }
    if (rc == NULL) b.gate = 7u;
    else if (!radio) b.gate = 0u;
    else if (!online) b.gate = 1u;
    else if (bench_session) b.gate = bench_cal.phase == SPRING_CAL_RUNNING ? 5u : 4u;
    else if (bench_armed) b.gate = 3u;
    else if (test && bench_run) b.gate = 6u;
    else b.gate = 2u;
    b.run       = bench_run;
    b.pair      = bench_pair;
    b.phase     = bench_cal.phase;
    b.reason    = bench_cal.reason;
    b.load      = bench_cal.compression;
    b.command   = bench_cal.motor_command;
    // 自然满伸长于腿质心标定表。表外保留电机回读与姿态，不将端点重力估计作为拟合标签。
    b.sample_ok = bench_cal.sample_ok && b.len <= Leg_Len_Max &&
                  bench_front_cycle == dm_f_rgt.frame_cycle && bench_rear_cycle == dm_b_rgt.frame_cycle;
    taskENTER_CRITICAL();
    bench_frame = b;
    taskEXIT_CRITICAL();
}

// 拷贝完成后即退出临界区，串口打包由低优先级任务完成。
void App_Spring_Bench_Frame(spring_bench_frame_t *out)
{
    taskENTER_CRITICAL();
    *out = bench_frame;
    taskEXIT_CRITICAL();
}
#endif
