/** @file  dm_motor.c  @brief 达妙电机通信  @version  v3.13  2026-11-24 */
#include "dm_motor.h"
#include "stm32h7xx.h"
#include "code_config.h"
#include "distributor.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <math.h>

// 达妙电机实例定义
dm_motor_t dm_f_lft;
dm_motor_t dm_b_lft;
dm_motor_t dm_f_rgt;
dm_motor_t dm_b_rgt;

static dm_motor_t *const dm_motors[] = {
    &dm_f_lft,
    &dm_b_lft,
    &dm_f_rgt,
    &dm_b_rgt,
};

// 电机数量计算
#define DM_MOTOR_NUM (sizeof(dm_motors) / sizeof(dm_motors[0]))

// 电机配置表结构体
static const struct
{
    FDCAN_HandleTypeDef *handle;
    dm_motor_t *motor;
    uint16_t slave_id;  // 达妙电机接收id
    uint16_t master_id; // 达妙电机发送id
} dm_motor_config[] =
    {
        // 必须用指定初始化器 本表曾因省略字段名把 handle 与 motor 写反且完全静默
        {.handle = &hfdcan1, .motor = &dm_f_lft, .slave_id = 0x01, .master_id = 0x11},
        {.handle = &hfdcan1, .motor = &dm_b_lft, .slave_id = 0x03, .master_id = 0x13},
        {.handle = &hfdcan1, .motor = &dm_f_rgt, .slave_id = 0x02, .master_id = 0x12},
        {.handle = &hfdcan1, .motor = &dm_b_rgt, .slave_id = 0x04, .master_id = 0x14},
};
#define DM_CONFIG_NUM (sizeof(dm_motor_config) / sizeof(dm_motor_config[0]))

static void Dm_Motor_Read(void *ctx, uint32_t id, const uint8_t *data, uint8_t dlc);

// 量程换算工具函数前置声明 定义在文件末尾 但被解析/控制函数先用到
static float Dm_Uint_To_Float(int x_int, float x_min, float x_max, int bits);
static int Dm_Float_To_Uint(float x, float x_min, float x_max, int bits);

// 按句柄+从机ID定位实例（Dm_Mark_Tx 打"已发待答"起点用）
static dm_motor_t *Dm_Find_Motor(FDCAN_HandleTypeDef *fdhcan, uint16_t slave_id)
{
    for (uint8_t i = 0; i < DM_MOTOR_NUM; i++)
    {
        if (dm_motors[i]->handle == fdhcan && dm_motors[i]->slave_id == slave_id)
        {
            return dm_motors[i];
        }
    }
    return NULL;
}

// 只标记发送成功的帧：TX 满被丢的帧没真正上线、不会有应答，不能算"已发待答"
static dm_motor_t *Dm_Mark_Tx(FDCAN_HandleTypeDef *fdhcan, uint16_t slave_id)
{
    dm_motor_t *motor = Dm_Find_Motor(fdhcan, slave_id);

    // 待答期间不得刷新起点，否则连续发帧时"最早未应答时刻"永远够不到超时
    if (motor != NULL && motor->tx_pend_tick == 0) motor->tx_pend_tick = HAL_GetTick();
    return motor;
}

// 达妙电机初始化函数
void Dm_Motor_Init(void)
{
    uint32_t now = HAL_GetTick();
    for (uint8_t i = 0; i < DM_CONFIG_NUM; i++)
    {
        dm_motor_t *motor = dm_motor_config[i].motor;
        motor->handle     = dm_motor_config[i].handle;
        motor->slave_id   = dm_motor_config[i].slave_id;
        motor->master_id  = dm_motor_config[i].master_id;

        // 注册失败就不置 last_rx_tick 这台电机收不到反馈 让故障在使能等反馈时暴露
        if (Can_Bus_Register(motor->handle, motor->master_id, Dm_Motor_Read, motor))
        {
            motor->last_rx_tick = now; // 避免初始零值触发离线误判
        }
    }
}

// 达妙电机接收数据函数
static void Dm_Motor_Read(void *ctx, uint32_t id, const uint8_t *data, uint8_t dlc)
{
    // 空帧检测
    if (dlc < 8) return;

    // 获取电机实例
    dm_motor_t *motor = (dm_motor_t *)ctx;

    // 将接收到的字节数据打包为 64 位整数
    uint8_t err     = data[0] >> 4;
    uint64_t packed = (uint64_t)(uint16_t)((data[1] << 8) | data[2]) | ((uint64_t)(uint16_t)((data[3] << 4) | (data[4] >> 4)) << 16) | ((uint64_t)(uint16_t)(((data[4] & 0xF) << 8) | data[5]) << 32) | ((uint64_t)err << 48) | ((uint64_t)data[6] << 56);

    motor->fb_packed  = packed;
    motor->rx_tx_stamp = motor->mit_tx_stamp;
    motor->temp_rotor = data[7];

    // 每帧单独保留故障码，避免失能或清错回包在任务读取前覆盖故障帧。
    if (err >= DM_ERR_OVERVOLT) motor->last_fault_code = err;

    // 设置解析标志和反馈到达标志
    motor->feedback_arrived = 1;
    motor->parse_flag       = 1;

    // 更新接收时间戳和在线状态；任一反馈帧到达即解除"发帧待答"预期
    motor->rx_count++;
    motor->last_rx_tick = HAL_GetTick();
    motor->rx_cycle     = DWT->CYCCNT;
    motor->online       = 1;
    motor->tx_pend_tick = 0;
}

// 达妙电机解析工具宏定义
#define DM_FB_POS(p)      ((uint16_t)((p) & 0xFFFFu))
#define DM_FB_VEL(p)      ((uint16_t)(((p) >> 16) & 0xFFFFu))
#define DM_FB_TRQ(p)      ((uint16_t)(((p) >> 32) & 0xFFFFu))
#define DM_FB_ERR(p)      ((uint8_t)(((p) >> 48) & 0xFFu))
#define DM_FB_TEMP_MOS(p) ((uint8_t)(((p) >> 56) & 0xFFu))

// 达妙电机解析函数
void Dm_Motor_Parse(void)
{
    for (uint8_t i = 0; i < DM_MOTOR_NUM; i++)
    {
        dm_motor_t *motor = dm_motors[i];

        // 检查解析标志
        if (!motor->parse_flag) continue;

        // 接地控制按 CAN 接纳时刻匹配这份回读；同时交接反馈和两个时基。
        taskENTER_CRITICAL();
        motor->frame_tick = motor->last_rx_tick;
        motor->frame_cycle = motor->rx_cycle;
        motor->frame_tx_stamp = motor->rx_tx_stamp;
        uint64_t fb       = motor->fb_packed;
        motor->parse_flag = 0;
        taskEXIT_CRITICAL();

        // 解析反馈数据
        motor->pos      = Dm_Uint_To_Float(DM_FB_POS(fb), DM_P_MIN, DM_P_MAX, 16);
        motor->vel      = Dm_Uint_To_Float(DM_FB_VEL(fb), DM_V_MIN, DM_V_MAX, 12);
        motor->trq      = Dm_Uint_To_Float(DM_FB_TRQ(fb), DM_T_MIN, DM_T_MAX, 12);
        motor->err_code = DM_FB_ERR(fb);

        // 电机自主故障标记离线 使能标志由在线检查处理
        if (motor->enabled && DM_FB_ERR(fb) >= DM_ERR_OVERVOLT)
        {
            motor->online = 0;
        }
    }
    Dm_Motor_Circle_Calculate();
}

// 已编码 MIT 字段共用打包路径，HAL_OK 仅表示进入发送 FIFO。
static HAL_StatusTypeDef Dm_Send_Mit_Raw(FDCAN_HandleTypeDef *fdhcan, uint16_t slave_id, uint16_t pos_raw, uint16_t vel_raw, uint16_t kp_raw, uint16_t kd_raw, uint16_t trq_raw)
{
    // MIT 帧: POS[15:0], VEL[11:0], KP[11:0], KD[11:0], T[11:0]
    uint8_t txdata[8] =
        {
            (uint8_t)(pos_raw >> 8),
            (uint8_t)(pos_raw),
            (uint8_t)(vel_raw >> 4),
            (uint8_t)(((vel_raw & 0xF) << 4) | (kp_raw >> 8)),
            (uint8_t)(kp_raw),
            (uint8_t)(kd_raw >> 4),
            (uint8_t)(((kd_raw & 0xF) << 4) | (trq_raw >> 8)),
            (uint8_t)(trq_raw),
        };
    // 与 RX 回调按入队顺序交接；HAL_OK 仅证明进 FIFO，协议没有命令回显序号。
    taskENTER_CRITICAL();
    HAL_StatusTypeDef status = Can_Bus_Send(fdhcan, slave_id, txdata, 8);
    if (status == HAL_OK)
    {
        dm_motor_t *motor = Dm_Mark_Tx(fdhcan, slave_id);
        if (motor != NULL) motor->mit_tx_stamp = ((uint64_t)DWT->CYCCNT << 32) | ((uint32_t)trq_raw + 1u);
    }
    taskEXIT_CRITICAL();
    return status;
}

// 将连续力矩选到能力界与协议网格交集，输出码由纯力矩接口原样发送。
bool Dm_Motor_Torque_Encode(float trq, float lo, float hi, uint16_t *code)
{
    const float step = (DM_T_MAX - DM_T_MIN) / 4095.0f;
    int first, last, nearest;

    if (code == NULL || !isfinite(trq) || !isfinite(lo) || !isfinite(hi) || lo > hi) return false;
    lo = fmaxf(lo, DM_T_MIN);
    hi = fminf(hi, DM_T_MAX);
    if (lo > hi) return false;
    first = (int)ceilf((lo - DM_T_MIN) / step);
    last  = (int)floorf((hi - DM_T_MIN) / step);
    // float 在格点附近可能舍入，反量化后的力矩是最终边界判据。
    while (first > 0 && Dm_Uint_To_Float(first - 1, DM_T_MIN, DM_T_MAX, 12) >= lo) first--;
    while (last < 4095 && Dm_Uint_To_Float(last + 1, DM_T_MIN, DM_T_MAX, 12) <= hi) last++;
    while (first <= last && Dm_Uint_To_Float(first, DM_T_MIN, DM_T_MAX, 12) < lo) first++;
    while (last >= first && Dm_Uint_To_Float(last, DM_T_MIN, DM_T_MAX, 12) > hi) last--;
    if (first > last) return false;
    trq     = fminf(hi, fmaxf(lo, trq));
    nearest = (int)floorf((trq - DM_T_MIN) / step + 0.5f);
    if (nearest < first) nearest = first;
    if (nearest > last) nearest = last;
    *code = (uint16_t)nearest;
    return true;
}

// 与反馈解析共用协议反量化公式，code 必须为已验证的 12 位力矩码。
float Dm_Motor_Torque_Decode(uint16_t code)
{
    return Dm_Uint_To_Float(code, DM_T_MIN, DM_T_MAX, 12);
}

// 零位置速度沿用原编码，增益为零；不再次量化力矩码，满即丢。
HAL_StatusTypeDef Dm_Motor_Torque_Send(FDCAN_HandleTypeDef *fdhcan, uint16_t slave_id, uint16_t trq_raw)
{
    if (trq_raw > 4095u) return HAL_ERROR;
    return Dm_Send_Mit_Raw(fdhcan, slave_id,
                           (uint16_t)Dm_Float_To_Uint(0.0f, DM_P_MIN, DM_P_MAX, 16),
                           (uint16_t)Dm_Float_To_Uint(0.0f, DM_V_MIN, DM_V_MAX, 12), 0u, 0u, trq_raw);
}

// 通用 MIT 保留原参数钳位与截断编码，其他模式调用者行为不变。
HAL_StatusTypeDef Dm_Motor_Mit_Ctrl(FDCAN_HandleTypeDef *fdhcan, uint16_t slave_id,
                                    float pos, float vel,
                                    float kp, float kd, float trq)
{
    if (kp != 0.0f || kd != 0.0f) Execution_Invalidate();

    // NaN 比较恒假会绕过 Dm_Float_To_Uint 的钳位，坏值强转 uint16 是 UB；
    // MIT 需持续收帧维持反馈，坏值按 0 发出而非 return 不发送
    if (!isfinite(pos)) pos = 0.0f;
    if (!isfinite(vel)) vel = 0.0f;
    if (!isfinite(kp)) kp = 0.0f;
    if (!isfinite(kd)) kd = 0.0f;
    if (!isfinite(trq)) trq = 0.0f;

    uint16_t pos_raw = (uint16_t)Dm_Float_To_Uint(pos, DM_P_MIN, DM_P_MAX, 16);
    uint16_t vel_raw = (uint16_t)Dm_Float_To_Uint(vel, DM_V_MIN, DM_V_MAX, 12);
    uint16_t kp_raw  = (uint16_t)Dm_Float_To_Uint(kp, DM_KP_MIN, DM_KP_MAX, 12);
    uint16_t kd_raw  = (uint16_t)Dm_Float_To_Uint(kd, DM_KD_MIN, DM_KD_MAX, 12);
    uint16_t trq_raw = (uint16_t)Dm_Float_To_Uint(trq, DM_T_MIN, DM_T_MAX, 12);

    return Dm_Send_Mit_Raw(fdhcan, slave_id, pos_raw, vel_raw, kp_raw, kd_raw, trq_raw);
}

// 位置速度模式打包发送 失败状态向上传递
HAL_StatusTypeDef Dm_Motor_Pos_Vel_Ctrl(FDCAN_HandleTypeDef *fdhcan, uint16_t slave_id,
                                        float pos, float vel)
{
    uint8_t txdata[8];

    Execution_Invalidate();
    memcpy(&txdata[0], &pos, 4);
    memcpy(&txdata[4], &vel, 4);
    HAL_StatusTypeDef status = Can_Bus_Send(fdhcan, slave_id + 0x100, txdata, 8);
    if (status == HAL_OK) Dm_Mark_Tx(fdhcan, slave_id);
    return status;
}

// 速度模式打包发送
HAL_StatusTypeDef Dm_Motor_Vel_Ctrl(FDCAN_HandleTypeDef *fdhcan, uint16_t slave_id,
                                    float vel)
{
    uint8_t txdata[8] = {0};

    Execution_Invalidate();
    memcpy(&txdata[0], &vel, 4);
    HAL_StatusTypeDef status = Can_Bus_Send(fdhcan, slave_id + 0x200, txdata, 8);
    if (status == HAL_OK) Dm_Mark_Tx(fdhcan, slave_id);
    return status;
}

// 达妙特殊命令
static HAL_StatusTypeDef Dm_Send_Cmd(FDCAN_HandleTypeDef *fdhcan, uint16_t slave_id,
                                     uint8_t cmd)
{
    uint8_t txdata[8]        = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd};
    HAL_StatusTypeDef status = Can_Bus_Send(fdhcan, slave_id, txdata, 8);
    if (status == HAL_OK) Dm_Mark_Tx(fdhcan, slave_id);
    return status;
}

// 发送单台使能命令
HAL_StatusTypeDef Dm_Motor_Enable(FDCAN_HandleTypeDef *fdhcan, uint16_t slave_id)
{
    Execution_Invalidate();
    return Dm_Send_Cmd(fdhcan, slave_id, 0xFC);
}

// 发送单台失能命令
HAL_StatusTypeDef Dm_Motor_Disable(FDCAN_HandleTypeDef *fdhcan, uint16_t slave_id)
{
    Execution_Invalidate();
    return Dm_Send_Cmd(fdhcan, slave_id, 0xFD);
}

// 发送单台零点标定命令
HAL_StatusTypeDef Dm_Motor_Set_Zero(FDCAN_HandleTypeDef *fdhcan, uint16_t slave_id)
{
    return Dm_Send_Cmd(fdhcan, slave_id, 0xFE);
}

// 发送单台清错命令
HAL_StatusTypeDef Dm_Motor_Clear_Error(FDCAN_HandleTypeDef *fdhcan, uint16_t slave_id)
{
    return Dm_Send_Cmd(fdhcan, slave_id, 0xFB);
}

// 工具函数
static float Dm_Uint_To_Float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    return (float)x_int * span / (float)((1 << bits) - 1) + x_min;
}

// 浮点量钳位后量化为协议位宽
static int Dm_Float_To_Uint(float x, float x_min, float x_max, int bits)
{
    if (x < x_min) x = x_min;
    if (x > x_max) x = x_max;
    float span = x_max - x_min;
    return (int)((x - x_min) * (float)((1 << bits) - 1) / span);
}


// 等待 Tx FIFO 有空位再发送，避免批量帧瞬间打满 FIFO 导致 Can_Bus_Send 返回 HAL_ERROR
// 使能/清错这类命令宁可等不能丢 与控制帧的"满即丢帧"是有意区分的
static HAL_StatusTypeDef Dm_Wait_Tx_Ready(FDCAN_HandleTypeDef *fdhcan)
{
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(DM_CMD_TIMEOUT_MS);
    while (!Can_Bus_Tx_Ready(fdhcan))
    {
        if (xTaskGetTickCount() >= deadline)
        {
            return HAL_ERROR;
        }
        vTaskDelay(1);
    }
    return HAL_OK;
}

// 等待 ISR 收到至少一帧反馈，消费 feedback_arrived 标志
static bool Dm_Wait_Feedback(dm_motor_t *motor)
{
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(DM_CMD_TIMEOUT_MS);
    while (xTaskGetTickCount() < deadline)
    {
        taskENTER_CRITICAL();
        uint8_t arrived = motor->feedback_arrived;
        if (arrived)
        {
            motor->feedback_arrived = 0; // 消费标志，避免残留导致下次误读
        }
        taskEXIT_CRITICAL();
        if (arrived) return true;
        vTaskDelay(1);
    }
    return false;
}

// 逐台清错使能并等待有效反馈 任一失败返回错误
HAL_StatusTypeDef Dm_Motor_All_Enable(void)
{
    HAL_StatusTypeDef status = HAL_OK;
    for (uint8_t i = 0; i < DM_MOTOR_NUM; i++)
    {
        if (Dm_Motor_Wait_Motor_Enable(dm_motors[i]) != HAL_OK)
        {
            status = HAL_ERROR;
        }
    }
    return status;
}

// 逐台发送失能 命令入队成功后更新使能标志
HAL_StatusTypeDef Dm_Motor_All_Disable(void)
{
    HAL_StatusTypeDef status = HAL_OK;
    for (uint8_t i = 0; i < DM_MOTOR_NUM; i++)
    {
        dm_motor_t *motor = dm_motors[i];

        // 清旧标志，避免读到上一轮残留
        taskENTER_CRITICAL();
        motor->feedback_arrived = 0;
        taskEXIT_CRITICAL();

        // fire-and-forget：不等反馈确认，发送成功才标记失能。
        // 0xFD 是电机最高优先级命令，极少丢帧；即使丢帧，电机自带安全看门狗也会在 ms 级停止。
        // 不阻塞的理由：串行等 4 台电机反馈 ≈ 12ms，会阻塞当前 2 ms 控制循环。
        // 发送失败保持 enabled=1：让 100ms 重申循环继续重试、Dm_Motor_Online 继续盯这台电机。
        if (Dm_Wait_Tx_Ready(motor->handle) != HAL_OK)
        {
            status = HAL_ERROR;
            continue;
        }
        if (Dm_Motor_Disable(motor->handle, motor->slave_id) != HAL_OK)
        {
            status = HAL_ERROR;
            continue;
        }
        motor->enabled = 0;
    }
    return status;
}

// 关节位置跨圈接续 首帧只立基准
void Dm_Motor_Circle_Calculate(void)
{
    static float last_pos[DM_MOTOR_NUM] = {0};
    static float circle[DM_MOTOR_NUM]   = {0};
    static uint8_t inited               = 0;

    for (uint8_t i = 0; i < DM_MOTOR_NUM; i++)
    {
        dm_motor_t *motor = dm_motors[i];

        // 首帧只记录位置——避免 last_pos=0 与实际初始位置差值 >π 导致误加减圈
        if (!(inited & (1 << i)))
        {
            last_pos[i] = motor->pos;
            inited |= (1 << i);
            motor->circle = motor->pos / TWO_PI;
            continue;
        }

        float delta = motor->pos - last_pos[i];

        if (delta > PI)
        {
            circle[i] -= 1.0f;
        }
        else if (delta < -PI)
        {
            circle[i] += 1.0f;
        }

        last_pos[i]   = motor->pos;
        motor->circle = circle[i] + motor->pos / TWO_PI;
    }
}

// 逐台等待 FIFO 空位后发送当前位置标定命令，首个失败处返回。
HAL_StatusTypeDef Dm_Motor_All_Set_Zero(void)
{
    for (uint8_t i = 0u; i < DM_MOTOR_NUM; i++)
    {
        dm_motor_t *motor = dm_motors[i];
        HAL_StatusTypeDef status = Dm_Wait_Tx_Ready(motor->handle);

        if (status != HAL_OK) return status;
        status = Dm_Motor_Set_Zero(motor->handle, motor->slave_id);
        if (status != HAL_OK) return status;
    }

    return HAL_OK;
}

// 批量清错 等待 FIFO 空位后逐台发送
void Dm_Motor_All_Clear_Error(void)
{
    for (uint8_t i = 0; i < DM_MOTOR_NUM; i++)
    {
        dm_motor_t *motor = dm_motors[i];

        // 清旧标志，避免读到上一轮残留
        taskENTER_CRITICAL();
        motor->feedback_arrived = 0;
        taskEXIT_CRITICAL();

        // 清错属关键命令 宁可等 FIFO 空位也不静默丢：丢帧会让瞬态故障锁死成永久
        if (Dm_Wait_Tx_Ready(motor->handle) != HAL_OK) continue;
        if (Dm_Motor_Clear_Error(motor->handle, motor->slave_id) != HAL_OK) continue;
    }
}

// 单台电机使能：清错 → 等清错应答 → 使能 → 等使能应答
HAL_StatusTypeDef Dm_Motor_Wait_Motor_Enable(dm_motor_t *motor)
{
    if (motor == NULL) return HAL_ERROR;
    if (motor->enabled) return HAL_OK;

    // 清旧反馈标志，确保后续反馈来自本次使能
    taskENTER_CRITICAL();
    motor->feedback_arrived = 0;
    taskEXIT_CRITICAL();

    //! 故障电机停回帧后 err 冻结，直接 return 会让清错永远发不出去、瞬态故障锁死成永久——
    //! 先发 0xFB 清错（关键命令走"宁可等"），按应答刷新后的 err 再决断，仍故障才报错
    if (DM_FB_ERR(motor->fb_packed) >= DM_ERR_OVERVOLT)
    {
        if (Dm_Wait_Tx_Ready(motor->handle) != HAL_OK) return HAL_ERROR;
        if (Dm_Motor_Clear_Error(motor->handle, motor->slave_id) != HAL_OK) return HAL_ERROR;
        if (!Dm_Wait_Feedback(motor)) return HAL_ERROR;
        if (DM_FB_ERR(motor->fb_packed) >= DM_ERR_OVERVOLT) return HAL_ERROR;
    }

    if (Dm_Wait_Tx_Ready(motor->handle) != HAL_OK) return HAL_ERROR;
    if (Dm_Motor_Clear_Error(motor->handle, motor->slave_id) != HAL_OK) return HAL_ERROR;

    //! 0xFB 与 0xFC 各有独立应答帧：发 0xFC 前必须先消费清错应答并二次清标志，
    //! 否则使能等待读到的是清错应答，0xFC 被"满即丢"丢掉也照样返回假阳性使能
    if (!Dm_Wait_Feedback(motor)) return HAL_ERROR;
    taskENTER_CRITICAL();
    motor->feedback_arrived = 0;
    taskEXIT_CRITICAL();

    if (Dm_Wait_Tx_Ready(motor->handle) != HAL_OK) return HAL_ERROR;
    if (Dm_Motor_Enable(motor->handle, motor->slave_id) != HAL_OK) return HAL_ERROR;

    // 等反馈确认电机已响应
    if (Dm_Wait_Feedback(motor) && DM_FB_ERR(motor->fb_packed) == DM_ERR_ENABLE)
    {
        motor->enabled      = 1;
        motor->online       = 1; // 刚使能首帧已到，Dm_Motor_Online 不误判
        motor->rx_count++;
    motor->last_rx_tick = HAL_GetTick();
        return HAL_OK;
    }
    return HAL_ERROR;
}

/* ============================================================================
 * 在线检测
 * ==========================================================================*/
// 已使能电机按待答超时与硬件故障判活
bool Dm_Motor_Online(void)
{
    uint32_t now   = HAL_GetTick();
    bool any_fault = false;
    for (uint8_t i = 0; i < DM_MOTOR_NUM; i++)
    {
        dm_motor_t *motor = dm_motors[i];
        if (!motor->enabled)
        {
            motor->offline_ticks = 0;
            continue;
        }

        // 硬件故障 → 失能 + 返回 false，无消抖（硬件故障不是瞬态抖动）
        if (DM_FB_ERR(motor->fb_packed) >= DM_ERR_OVERVOLT)
        {
            motor->enabled       = 0;
            motor->online        = 0;
            motor->offline_ticks = 0;
            any_fault            = true;
            continue;
        }
        // 超时消抖：连续 5 次才确认离线，避免 CAN 瞬态抖动误判
        // 一问一答协议：只有"发过帧且久久无应答"才算离线；计时从"最早未应答帧"起算——
        // 若改用 last_rx 起算，静默后第一帧会在应答到达前被追帧爆发连打 5 次（误锁复活）
        uint32_t pend = motor->tx_pend_tick; //! 单次读入：两次读之隙被 ISR 清 0 会让 now-0 恒 >10ms
        if (pend != 0 && now - pend > DM_OFFLINE_MS)
        {
            motor->offline_ticks++;
            if (motor->offline_ticks >= 5)
            {
                motor->online = 0;
            }
        }
        else
        {
            motor->offline_ticks = 0;
        }
        if (!motor->online)
        {
            any_fault = true;
        }
    }
    return !any_fault;
}

// 四台关节发送零位置增益与零力矩控制帧
void Dm_Send_Zero_All(void)
{
    for (uint8_t i = 0; i < DM_MOTOR_NUM; i++)
    {
        dm_motor_t *motor = dm_motors[i];
        Dm_Motor_Mit_Ctrl(motor->handle, motor->slave_id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }
}
