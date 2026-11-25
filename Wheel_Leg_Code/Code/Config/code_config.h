/** @file  code_config.h  @brief 全局数值配置  @version  v1.4  2026-10-27 */
#ifndef CODE_CONFIG_H
#define CODE_CONFIG_H

#include "system_key.h"
#include "polarity_config.h"
#include "tinympc14_data.h"

#define PI                3.14159f // 圆周率
#define TWO_PI            6.28319f // 2*PI
#define PI_2              1.57080f // PI/2
#define GRAVITY           9.81f    // m/s² 重力加速度

#define DEG_TO_RAD(x)     ((x) * PI / 180.0f)
#define RAD_TO_DEG(x)     ((x) * 180.0f / PI)
#define RPM_TO_RAD(x)     ((x) * 2.0f * PI / 60.0f)
#define RAD_TO_RPM(x)     ((x) * 60.0f / (2.0f * PI))
#define RPM_TO_RADS       0.104719755f // 转速换算系数

#define CTRL_PERIOD_MS    1u                               // ms 控制与估计周期
#define CTRL_DT           ((float)CTRL_PERIOD_MS * 0.001f) // s 控制/估计任务周期

// 融合控制使用同一份生成数据定义估计器、分配器和 MPC 的物理量。
#if defined(TINYMPC14_URDF_MODEL)
#define LEG_MASS_TOTAL    TINYMPC14_M_TOTAL  // kg URDF 整车质量
#define LEG_MASS_WHEEL    TINYMPC14_M_WHEEL  // kg URDF 单轮质量
#define LEG_MASS_LEG      TINYMPC14_LEG_MASS // kg URDF 单腿质量，不含轮
#define WHEEL_R           TINYMPC14_WHEEL_R  // m CAD 轮胎半径
#define LEG_HALF_TRACK    TINYMPC14_HALF_TRACK // m URDF 半轮距
#define LEG_COM_H_OFF     TINYMPC14_H_OFF      // m 机身质心相对髋轴高度
#define Leg_Len_Max       TINYMPC14_L_MAX    // m URDF 膝限位对应上界
#define Leg_Len_Min       TINYMPC14_L_MIN    // m URDF 膝限位对应下界
#else
#define LEG_MASS_TOTAL    14.606061706282409f                                   // kg 整车总质量，含电池、两腿和两轮
#define LEG_MASS_WHEEL    0.5981919445961541f                                   // kg 单轮及轮毂电机质量
#define LEG_MASS_LEG      1.3212767058302579f                                  // kg 单腿质量，不含轮
#define WHEEL_R           0.060000002144914348f   // m 轮子半径
#define LEG_HALF_TRACK    0.2072000173544678f // m 半轮距
#define LEG_COM_H_OFF     -0.034041082972946314f  // m 机身质心相对髋轴的有符号高度
#define Leg_Len_Max       0.35f     // m 腿长上限
#define Leg_Len_Min       0.13f     // m 腿长下限

#define BODY_H_MAX        0.35f     // m 机身高度上限
#define BODY_H_MIN        0.13f     // m 机身高度下限
#endif
#define LEG_MASS_SUPPORT  (LEG_MASS_TOTAL - 2.0f * LEG_MASS_WHEEL)  // kg 支撑质量，整车扣除两轮
#define LEG_MASS_BODY     (LEG_MASS_SUPPORT - 2.0f * LEG_MASS_LEG)  // kg 机身质量，支撑质量扣除两腿

#define REDUCTION_RATIO   15.76470588f // M3508 轮毂减速比
#define WHEEL_MASS        LEG_MASS_WHEEL // kg 轮端质量
#define BODY_MASS         LEG_MASS_BODY  // kg 不含两腿、两轮的机身质量
#define LEG_GRAV_FF       LEG_MASS_LEG   // kg 单腿质量 支撑前馈质量项


#define LEG_LEN_CLAMP(x)  ((x) > Leg_Len_Max ? Leg_Len_Max : ((x) < Leg_Len_Min ? Leg_Len_Min : (x)))

#define M3508_OUT_MAX     16384.0f    // M3508 输出上限
#define M3508_OUT_MIN     (-16384.0f) // M3508 输出下限
#define M2006_OUT_MAX     10000.0f    // M2006 输出上限
#define M2006_OUT_MIN     (-10000.0f) // M2006 输出下限
#define ONE_MOTOR_OUT_MAX 5000.0f     // 单电机控制量限幅

#define MOTOR_TO_DEG      0.0439453125f // 电机编码器 → 角度换算

#define M3508_TO_TORQUE   3326.29f // 电流力矩换算 = ( 52438 / REDUCTION_RATIO )

#define DR16_Max          660.0f    // 通道值上限
#define DR16_Min          (-660.0f) // 通道值下限

#define BALANCE_PITCH_REF (-0.0f) // rad 机体俯仰目标角

#endif
