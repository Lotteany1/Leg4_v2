/** @file  dji_motor.c  @brief DJI 电机反馈解析与实际电流码发送  @version  v3.4  2026-11-19 */
#include "dji_motor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "code_config.h"
#include <math.h>

//! 大疆电机实例定义 !//
dji_motor_t dji_wheel_lft = {.handle = &hfdcan3, .type = DJI_M3508, .id = 0x201};
dji_motor_t dji_wheel_rgt = {.handle = &hfdcan3, .type = DJI_M3508, .id = 0x202};

// 大疆电机列表数组
static dji_motor_t *const dji_motor_list[] =
    {
        &dji_wheel_lft,
        &dji_wheel_rgt,
};

//! -------------- !//

// 大疆电机工具函数
#define DJI_NUM            (sizeof(dji_motor_list) / sizeof(dji_motor_list[0]))
#define DJI_ANG_DECODE(p)  ((uint16_t)((p) & 0xFFFFu))        // 位置信息解码
#define DJI_VEL_DECODE(p)  ((int16_t)(((p) >> 16) & 0xFFFFu)) // 速度信息解码
#define DJI_TRQ_DECODE(p)  ((int16_t)(((p) >> 32) & 0xFFFFu)) // 扭矩信息解码
#define DJI_TEMP_DECODE(p) ((int8_t)(((p) >> 48) & 0xFFu))    // 温度信息解码

// 电机 ID 选择发送组
static inline dji_group_t Dji_Motor_Group(uint8_t id)
{
    return (id <= 0x204) ? DJI_GROUP_A : DJI_GROUP_B;
}

// 电机 ID 转组内电流槽位
static inline uint8_t Dji_Motor_Slot(uint16_t id)
{
    return (id - 0x201) & 0x03; // 计算电机在组内的槽位号
}

// 按电机型号选择电流指令限幅
static inline int16_t Dji_Motor_Limit(dji_motor_type_t type)
{
    return (type == DJI_M3508) ? DJI_M3508_SET_MAX : DJI_M2006_SET_MAX;
}

// 大疆电机接收数据函数
static void Dji_Motor_Read(void *ctx, uint32_t id, const uint8_t *data, uint8_t dlc)
{
    if (dlc < 8) return;

    dji_motor_t *motor = (dji_motor_t *)ctx;

    // M2006 无温度传感器，填 -1 表示无效
    int8_t temp = (motor->type == DJI_M3508) ? (int8_t)data[6] : -1;

    uint64_t packed = (uint64_t)(uint16_t)((data[0] << 8) | data[1]) | ((uint64_t)(uint16_t)((data[2] << 8) | data[3]) << 16) | ((uint64_t)(uint16_t)((data[4] << 8) | data[5]) << 32) | ((uint64_t)(uint8_t)temp << 48);

    motor->fb_packed  = packed;
    motor->rx_tick    = HAL_GetTick();
    motor->parse_flag = 1;
}

// 大疆电机初始化函数
void Dji_Motor_Init(void)
{
    for (uint8_t i = 0; i < DJI_NUM; i++)
    {
        dji_motor_t *motor = dji_motor_list[i];
        if (motor->id < 0x201 || motor->id > 0x208) continue;

        // 注册失败就把句柄置空 让 Dji_Send_Group 在整组无效时返回 HAL_ERROR
        if (!Can_Bus_Register(motor->handle, motor->id, Dji_Motor_Read, motor))
        {
            motor->handle = NULL;
        }
    }
}

// 大疆电机圈数计算函数
static void Dji_Motor_Circle_Calculate(void)
{
    static float last_pos[DJI_NUM] = {0};
    static float circle[DJI_NUM]   = {0};
    static uint8_t inited          = 0;

    for (uint8_t i = 0; i < DJI_NUM; i++)
    {
        dji_motor_t *motor = dji_motor_list[i];

        // 首帧只记录位置——避免 last_pos=0 与实际初始位置差值 >π 导致误加减圈
        // inited 每一位代表一个电机是否已初始化，1 表示已初始化，0 表示未初始化
        //? 未初始化就会执行下面这个if qaq..
        if (!(inited & (1 << i)))
        {
            last_pos[i] = motor->pos;
            inited |= (1 << i);
            motor->circle = motor->pos / TWO_PI;
            continue;
        }

        // 检测跨越 ±π 的情况，进行圈数累加或减一
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

// 大疆电机解析函数
void Dji_Motor_Parse(void)
{
    for (uint8_t i = 0; i < DJI_NUM; i++)
    {
        dji_motor_t *motor = dji_motor_list[i];

        if (!motor->parse_flag) continue;

        // 临界区内交接同一 CAN 帧的反馈、接收时标和待解析标志。
        taskENTER_CRITICAL();
        const uint64_t feedback = motor->fb_packed;
        const uint32_t tick     = motor->rx_tick;
        motor->parse_flag = 0;
        taskEXIT_CRITICAL();

        motor->pos = (float)DJI_ANG_DECODE(feedback) * TWO_PI / 8192.0f - PI;
        motor->vel = (float)DJI_VEL_DECODE(feedback) * TWO_PI / 60.0f;
        motor->trq = (float)DJI_TRQ_DECODE(feedback) / M3508_TO_TORQUE;
        motor->frame_tick = tick;
        motor->frame_seq++;
    }
    Dji_Motor_Circle_Calculate();
}

// 组帧同时输出实际有符号电流码；返回状态只表示整组入队接纳。
HAL_StatusTypeDef Dji_Send_Group_Encoded(dji_group_t group, const float trq[4], int16_t code[4])
{
    FDCAN_HandleTypeDef *hfdcan = NULL;
    uint8_t data[8]             = {0};

    for (uint8_t i = 0; i < DJI_NUM; i++)
    {
        dji_motor_t *motor = dji_motor_list[i];

        if (Dji_Motor_Group(motor->id) != group) continue;
        if (hfdcan == NULL) hfdcan = motor->handle;

        // 发送槽位
        uint8_t slot  = Dji_Motor_Slot(motor->id);
        int16_t limit = Dji_Motor_Limit(motor->type);

        // 钳位到电流上限后转 int16_t，避免浮点到整型溢出
        // NaN 比较恒假会绕过钳位、(int16_t)NaN 强转 UB，坏值按 0 发
        float torque = trq[slot];
        if (!isfinite(torque)) torque = 0.0f;
        if (torque > (float)limit) torque = (float)limit;
        if (torque < -(float)limit) torque = -(float)limit;
        int16_t raw = (int16_t)torque;

        if (code != NULL) code[slot] = raw;

        // 写入数据
        data[2 * slot]     = (uint8_t)(raw >> 8);
        data[2 * slot + 1] = (uint8_t)(raw & 0xFF);

        motor->cmd_trq = trq[slot];
    }
    if (hfdcan == NULL) return HAL_ERROR;

    uint32_t can_id = (group == DJI_GROUP_A) ? 0x200 : 0x1FF;
    return Can_Bus_Send(hfdcan, can_id, data, 8);
}

// 兼容既有组发送入口，编码与扩展记录入口共用同一路径。
HAL_StatusTypeDef Dji_Send_Group(dji_group_t group, const float trq[4])
{
    return Dji_Send_Group_Encoded(group, trq, NULL);
}

// 大疆电机急停函数
void Dji_Motor_All_Stop(void)
{
    const float zero[4] = {0, 0, 0, 0};
    Dji_Send_Group(DJI_GROUP_A, zero);
    Dji_Send_Group(DJI_GROUP_B, zero);
}
