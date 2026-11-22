/** @file  dm_motor.h  @brief 达妙电机反馈与 MIT、纯力矩编码接口  @version  v3.8  2026-11-22 */
#ifndef DM_MOTOR_H
#define DM_MOTOR_H

#include "code_config.h"
#include "can_bus.h"
#include <stdint.h>
#include <stdbool.h>

#define DM_OFFLINE_MS 10u // 判离线只认"命令发出后超该时长无应答"，未发帧的静默不算
#define DM_CMD_TIMEOUT_MS 3 // 达妙反馈一问一答（收一帧回一帧，无自主流式反馈），3ms 足够等回应答

// 达妙电机 MIT 模式量程与离线判据
#define DM_P_MIN      -3.14159f
#define DM_P_MAX      3.14159f
#define DM_V_MIN      -45.0f
#define DM_V_MAX      45.0f
#define DM_KP_MIN     0.0f
#define DM_KP_MAX     500.0f
#define DM_KD_MIN     0.0f
#define DM_KD_MAX     5.0f
#define DM_T_MIN      -40.0f
#define DM_T_MAX      40.0f


// 达妙电机故障码枚举表 — 反馈帧高 4 位 ERR 字段 (D[0] >> 4)
typedef enum
{
    DM_ERR_DISABLE   = 0x0,
    DM_ERR_ENABLE    = 0x1,
    DM_ERR_OVERVOLT  = 0x8,
    DM_ERR_UNDERVOLT = 0x9,
    DM_ERR_OVERCURR  = 0xA,
    DM_ERR_MOS_OT    = 0xB,
    DM_ERR_ROTOR_OT  = 0xC,
    DM_ERR_LOST      = 0xD,
    DM_ERR_OVERLOAD  = 0xE,
} dm_err_t;

// 达妙电机数据结构
typedef struct
{
    FDCAN_HandleTypeDef *handle;
    uint16_t master_id;
    uint16_t slave_id;

    // ISR 写入的反馈快照——pos/vel/trq/err/temp_mos 打包成单个 volatile uint64_t，
    // 任务侧一次 LDRD 原子读走整帧（M4 单指令），不存在跨字段撕裂。
    // 位布局: [15:0]=pos_raw [31:16]=vel_raw [47:32]=trq_raw [55:48]=err [63:56]=temp_mos
    volatile uint64_t fb_packed __attribute__((aligned(8)));
    volatile uint8_t temp_rotor; // 仅监控用，单字节天然原子，不进快照

    // 解析结果 — 仅任务侧在 Dm_Motor_Parse() 中写入
    float pos;
    float circle;
    float vel;
    float trq;
    uint8_t err_code; // 最近一次解析的反馈状态码，无单位，取 D[0] 高 4 位，见 dm_err_t

    // 同步标志
    volatile uint8_t parse_flag;       // ISR 写 1 → Dm_Motor_Parse 消费后清 0
    volatile uint8_t feedback_arrived; // ISR 写 1 → 使能/禁用流程消费后清 0

    // 控制状态
    volatile uint8_t enabled;
    volatile uint8_t online;        // ISR 收帧置 1，故障/离线时清 0
    volatile uint32_t last_rx_tick; // 最后一帧反馈的 HAL_GetTick()，ISR 直接更新
    volatile uint32_t rx_count;     // 本电机反馈 ID 收到的完整反馈帧累计数
    volatile uint32_t rx_cycle;     // 接收回调的 DWT 计数，与关节命令接纳时标同源
    uint32_t frame_tick;           // ms，估计任务已解析反馈时标
    uint32_t frame_cycle;          // DWT，frame_tick 对应的同一力矩反馈帧
    volatile uint32_t tx_pend_tick; // 最早"已发未答"命令时刻，0=无待答；ISR 收到反馈帧清 0
    uint8_t offline_ticks;          // 连续超时计数，>=5 确认离线（消抖）
} dm_motor_t;

// 达妙电机反馈快照解包宏
#define DM_FB_POS(p)      ((uint16_t)((p) & 0xFFFFu))
#define DM_FB_VEL(p)      ((uint16_t)(((p) >> 16) & 0xFFFFu))
#define DM_FB_TRQ(p)      ((uint16_t)(((p) >> 32) & 0xFFFFu))
#define DM_FB_ERR(p)      ((uint8_t)(((p) >> 48) & 0xFFu))
#define DM_FB_TEMP_MOS(p) ((uint8_t)(((p) >> 56) & 0xFFu))

// 达妙电机全局实例
extern dm_motor_t dm_f_lft;
extern dm_motor_t dm_b_lft;
extern dm_motor_t dm_f_rgt;
extern dm_motor_t dm_b_rgt;

// 达妙电机对外接口
void Dm_Motor_Init(void);
void Dm_Motor_Parse(void);

// 达妙电机控制接口
HAL_StatusTypeDef Dm_Motor_Mit_Ctrl(FDCAN_HandleTypeDef *fdhcan, uint16_t slave_id,
                                    float pos, float vel,
                                    float kp, float kd, float trq);
HAL_StatusTypeDef Dm_Motor_Pos_Vel_Ctrl(FDCAN_HandleTypeDef *fdhcan, uint16_t slave_id,
                                        float pos, float vel);
HAL_StatusTypeDef Dm_Motor_Vel_Ctrl(FDCAN_HandleTypeDef *fdhcan, uint16_t slave_id,
                                    float vel);

// 达妙电机特殊命令（7×0xFF + 命令字节）
HAL_StatusTypeDef Dm_Motor_Enable(FDCAN_HandleTypeDef *fdhcan, uint16_t slave_id);
HAL_StatusTypeDef Dm_Motor_Disable(FDCAN_HandleTypeDef *fdhcan, uint16_t slave_id);
HAL_StatusTypeDef Dm_Motor_Set_Zero(FDCAN_HandleTypeDef *fdhcan, uint16_t slave_id);
HAL_StatusTypeDef Dm_Motor_Clear_Error(FDCAN_HandleTypeDef *fdhcan, uint16_t slave_id);

// 达妙电机批量操作与使能流程
HAL_StatusTypeDef Dm_Motor_All_Enable(void);
HAL_StatusTypeDef Dm_Motor_All_Disable(void);
void Dm_Motor_Circle_Calculate(void);
void Dm_Motor_All_Clear_Error(void);

// 失能保持时调用一次，将四台关节当前位置标为零点，返回 CAN 发送状态。
HAL_StatusTypeDef Dm_Motor_All_Set_Zero(void);
HAL_StatusTypeDef Dm_Motor_Wait_Motor_Enable(dm_motor_t *motor);

// 达妙电机在线检测
bool Dm_Motor_Online(void);

void Dm_Send_Zero_All(void);
// trq、lo、hi 为最终电机坐标 N·m；code 输出为能力内合法 12 位码，空交集返回 false。
bool Dm_Motor_Torque_Encode(float trq, float lo, float hi, uint16_t *code);
float Dm_Motor_Torque_Decode(uint16_t code);                                                              // 已验证 12 位码反解 N·m
HAL_StatusTypeDef Dm_Motor_Torque_Send(FDCAN_HandleTypeDef *fdhcan, uint16_t slave_id, uint16_t trq_raw); // 纯力矩编码直发

#endif
