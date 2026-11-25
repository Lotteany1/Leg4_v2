/**
 * @file  polarity_config.h
 * @brief 控制极性与关节零点配置
 * @version  v1.1  2026-11-26
 */

#ifndef POLARITY_CONFIG_H
#define POLARITY_CONFIG_H

// 控制量正方向与关节零点在此配置。

/* 控制约定：Yaw 俯视逆时针为正、Pitch 低头为正、Roll 右倾为正；腿角前倾为正。 */

/* IMU 姿态角和角速度：按控制约定修正符号。 */
#define IMU_YAW_POLARITY   1.0f
#define IMU_PITCH_POLARITY -1.0f
#define IMU_ROLL_POLARITY  1.0f

/* IMU 三轴加速度极性独立配置，不复用姿态角极性，单位 g，含重力。 */
#define IMU_ACC_X_POLARITY 1.0f // 车体右向为正，左向为负
#define IMU_ACC_Y_POLARITY 1.0f // 车体前向为正，后向为负
#define IMU_ACC_Z_POLARITY 1.0f // 车体上向为正，下向为负

/* 纵向加速度控制约定：车体前向为正、后向为负。 */
#define IMU_ACC_FORWARD_AXIS     IMU_ACC_Y // 纵向加速度通道
#define IMU_ACC_FORWARD_POLARITY 1.0f      // 纵向加速度极性

/* IMU 四元数与加速度使用同一坐标系，分量极性独立配置。 */
#define IMU_QUAT_W_POLARITY 1.0f
#define IMU_QUAT_X_POLARITY 1.0f
#define IMU_QUAT_Y_POLARITY 1.0f
#define IMU_QUAT_Z_POLARITY 1.0f

// 保留的 IMU 零偏设定，当前姿态解析未使用这三项。
#define IMU_ROLL_OFFSET     0.0f
#define IMU_PITCH_OFFSET    0.0f
#define IMU_YAW_OFFSET      0.0f

/* 腿部关节：伸腿为正，原始角度乘以极性再加零点，单位 rad。 */
#define DM_LEG_LFT_F_POLARITY -1.0f
#define DM_LEG_RGT_F_POLARITY 1.0f
#define DM_LEG_LFT_B_POLARITY -1.0f
#define DM_LEG_RGT_B_POLARITY 1.0f

#define DM_LEG_MAX_LEN_ANG           1.2531f
#define DM_LEG_MAX_LEN_ANG_TO_MIN    0.7920f
#define DM_LEG_LFT_F_OFFSET    (DM_LEG_MAX_LEN_ANG - DM_LEG_MAX_LEN_ANG_TO_MIN)
#define DM_LEG_LFT_B_OFFSET    (DM_LEG_MAX_LEN_ANG + DM_LEG_MAX_LEN_ANG_TO_MIN)

#define DM_LEG_RGT_F_OFFSET   -(DM_LEG_MAX_LEN_ANG + DM_LEG_MAX_LEN_ANG_TO_MIN)
#define DM_LEG_RGT_B_OFFSET   -(DM_LEG_MAX_LEN_ANG - DM_LEG_MAX_LEN_ANG_TO_MIN)
#define LEG_MIRROR_CHANGE     (-1.0f) // 右腿镜像极性修正

// 左右轮原始反馈乘以下列极性，得到机体前进方向的轮速。
#define WHEEL_LFT_POLARITY (-1.0f)
#define WHEEL_RGT_POLARITY  1.0f

// 左轮 0x201 和右轮 0x202 的轮矩分别发到组帧的第一槽和第二槽。
// 命令极性按电机安装侧定义，沿用 V1 的左正右负。
#define WHEEL_LFT_CMD_POLARITY  (1.0f)
#define WHEEL_RGT_CMD_POLARITY (-1.0f)
#endif
