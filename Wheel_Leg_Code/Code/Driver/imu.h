/** @file  imu.h  @brief IMU 姿态接收与协议接口  @version  v3.1  2026-11-04 */
#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include "usart.h"
#include "code_config.h"

// HAL库 IMU 句柄
#define IMU_HUART (&huart5)

// IMU 接收超过 IMU_OFFLINE_MS 后 按控制任务调用拍数累计离线确认
#define IMU_OFFLINE_MS  5u // 单窗口时长 (ms) 200Hz → 5ms/帧
#define IMU_OFFLINE_CNT 5u // 拍 连续超时确认次数

// IMU 协议帧结构
#if IMU_USE_HI229 // 适配 Hi229 IMU
typedef struct __attribute__((packed))
{
    uint8_t tag;    // 包标识：0x91
    uint8_t id;     // 设备 ID
    uint8_t rev[2]; // 保留
    float prs;      // 气压 (Pa)
    uint32_t ts;    // 时间戳 (ms)
    float acc[3];   // 加速度 (G)
    float gyr[3];   // 角速度 (deg/s)
    float mag[3];   // 磁场 (uT)
    float eul[3];   // 欧拉角 (deg)，Roll/Pitch/Yaw
    float quat[4];  // 四元数，W/X/Y/Z
} imu91_t;
#else // 适配 Hi12 IMU
typedef struct __attribute__((packed))
{
    uint8_t tag;     /* 数据标签:0x91 */
    uint16_t status; /* 主状态 */
    int8_t tmp;      /* 温度   摄氏度  */
    float prs;       /* 气压   Pa */
    uint32_t ts;     /* 时间戳 ms */
    float acc[3];    /* 加速度 G   X Y Z*/
    float gyr[3];    /* 角速度 deg/s */
    float mag[3];    /* 地磁   uT*/
    float eul[3];    /* 欧拉角: Roll,Pitch,Yaw */
    float quat[4];   /* 四元数  w x y z*/
} imu91_t;
#endif

// 帧长定长：头(2) + 长度(2) + CRC(2) + 载荷(sizeof(imu91_t))
#define IMU_FRAME_LEN (6u + sizeof(imu91_t))

// IMU 数组索引枚举
// 欧拉角/角速度数组索引
typedef enum
{
    IMU_EUL_ROLL  = 0,
    IMU_EUL_PITCH = 1,
    IMU_EUL_YAW   = 2,
} eul_idx_t;

typedef enum
{
    IMU_GYR_PITCH = 0,
    IMU_GYR_ROLL  = 1,
    IMU_GYR_YAW   = 2,
} gyr_idx_t;

// 加速度体轴数组索引 — 不要用 eul_idx_t 混用，加速度是 X/Y/Z 体轴而非欧拉角
typedef enum
{
    IMU_ACC_X = 0,
    IMU_ACC_Y = 1,
    IMU_ACC_Z = 2,
} acc_idx_t;

// 四元数分量索引
typedef enum
{
    IMU_QUAT_W = 0,
    IMU_QUAT_X = 1,
    IMU_QUAT_Y = 2,
    IMU_QUAT_Z = 3,
} quat_idx_t;

// IMU 标定输出与共享状态
// IMU 标定输出 — 已套用极性+偏置的实际值
typedef struct
{
    float eul[3]; // ROLL/PITCH/YAW
    float acc[3];
    float gyr[3];
    float quat[4];
    volatile bool online;
    volatile uint32_t last_rx_tick;
} imu_t;

extern imu_t imu;

// IMU API
void Imu_Init(void);
bool Imu_Parse(void);
void Imu_Idle_Isr(UART_HandleTypeDef *huart);
bool Imu_Online(void);

#endif
