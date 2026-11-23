/**
 * @file  polarity_config.h
 * @brief 硬件极性配置 — 实际输出 = 极性 × 逻辑输出
 *
 */

#ifndef POLARITY_CONFIG_H
#define POLARITY_CONFIG_H

// 改安装方向和标定零点来这里。

/* 坐标系: 右手系 — Yaw逆时针+, Pitch低头+, Roll右倾+ (上交WBR约定: theta_b>0=低头, 腿角>0=前倾) */

/* IMU: 逆时针/抬头/右倾时原始输出方向 → 映射到右手系 */
#define IMU_YAW_POLARITY   1.0f
#define IMU_PITCH_POLARITY -1.0f
#define IMU_ROLL_POLARITY  1.0f

/* IMU 加速度体轴极性 — 体轴与欧拉角轴同向时极性一致 */
#define IMU_ACC_X_POLARITY (IMU_ROLL_POLARITY)  /* X 轴 */
#define IMU_ACC_Y_POLARITY (IMU_PITCH_POLARITY) /* Y 轴 */
#define IMU_ACC_Z_POLARITY 1.0f                 /* Z 轴 */

/* 四元数分量逐项换号：W、Z 保留，X 跟随横滚极性，Y 跟随俯仰极性。安装轴向待实机核实。 */
#define IMU_QUAT_W_POLARITY 1.0f
#define IMU_QUAT_X_POLARITY (IMU_ROLL_POLARITY)
#define IMU_QUAT_Y_POLARITY (IMU_PITCH_POLARITY)
#define IMU_QUAT_Z_POLARITY 1.0f

// 保留的 IMU 零偏设定，当前姿态解析未使用这三项。
#define IMU_ROLL_OFFSET     0.0f
#define IMU_PITCH_OFFSET    0.0f
#define IMU_YAW_OFFSET      0.0f

/* 腿部关节  伸腿为正 */
/* 原始关节角（rad）乘以极性再加零点（rad）。 */
#define DM_LEG_LFT_F_POLARITY -1.0f
#define DM_LEG_RGT_F_POLARITY 1.0f
#define DM_LEG_LFT_B_POLARITY -1.0f
#define DM_LEG_RGT_B_POLARITY 1.0f

#define DM_LEG_MAX_LEN_ANG           1.2531f
#define DM_LEG_MAX_LEN_ANG_TO_MIN    0.7920f
#define DM_LEG_LFT_F_OFFSET   (DM_LEG_MAX_LEN_ANG - DM_LEG_MAX_LEN_ANG_TO_MIN)
#define DM_LEG_LFT_B_OFFSET   (DM_LEG_MAX_LEN_ANG + DM_LEG_MAX_LEN_ANG_TO_MIN)

#define DM_LEG_RGT_F_OFFSET   -(DM_LEG_MAX_LEN_ANG + DM_LEG_MAX_LEN_ANG_TO_MIN)
#define DM_LEG_RGT_B_OFFSET   -(DM_LEG_MAX_LEN_ANG - DM_LEG_MAX_LEN_ANG_TO_MIN)
#define LEG_MIRROR_CHANGE (-1.0f)   // 右腿镜像极性修正

// 左右轮原始反馈乘以下列极性，得到机体前进方向的轮速。
#define WHEEL_LFT_POLARITY (-1.0f)
#define WHEEL_RGT_POLARITY 1.0f
// 左轮 0x201 和右轮 0x202 的轮矩分别发到组帧的第一槽和第二槽。
// 命令极性按电机安装侧定义，沿用 V1 的左正右负。
#define WHEEL_LFT_CMD_POLARITY  (1.0f)
#define WHEEL_RGT_CMD_POLARITY (-1.0f)
#endif
