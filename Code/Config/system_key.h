/** @file  system_key.h  @brief 系统功能开关  @version  v1.23  2026-11-20 */
#ifndef SYSTEM_KEY_H
#define SYSTEM_KEY_H

// 选择传感器、控制算法或启停功能来这里。

// 电机出力开关：0 保持 DM 失能，每拍发零出力 MIT，DJI 仅允许零电流，1 正常控制。
#define MOTOR_OUTPUT_ENABLE             1u

// 悬空台架：从 1 N 开始加载；开关打开时仅 TEST 档可使能右腿，并发送专用 VOFA 帧。
#define SPRING_CAL_FORCE_RATE           100.0f // N/s，右腿电机收腿力最大增加速率
#define SPRING_CAL_TRANSITION_N         221.7f // N，进入慢速加载的请求力
#define SPRING_CAL_FINE_RATE            0.1f   // N/s，突变点起的最大加力速率
#define SPRING_CAL_FORCE_MAX_N       1500.0f  // N，单腿电机轴力回读及收腿加载上限
#define SPRING_CAL_BENCH_ENABLE         1u   // 1 右腿气弹簧采集；0 常规控制与遥测
#define SPRING_CAL_BENCH_START_M        0.3662f // m，台架右腿自然满伸时髋轴到轮轴的距离
#define SPRING_CAL_BENCH_ANGLE_RAD      0.0f    // rad，机身固定、轮子悬空，髋轴到轮轴竖直向下

// 选择 IMU 型号 : 0 Hi12  1 Hi229
#define IMU_USE_HI229                   0u

// 防打滑开关 : 0 轮腿速度 1 融合速度
#define BALANCE_USE_SLIP                0u 

// 持续单侧打滑时自动收腿：0 关闭，1 开启
// 动作入口还要求 BALANCE_USE_SLIP 和 BALANCE_USE_AIR_CHECK 同时开启。
#define STEP_ENABLE                    0u

// 轮腿离地检测 : 0 关闭  1 开启
#define BALANCE_USE_AIR_CHECK           0u 

// 打滑离地抑制 : 0 关闭  1 打滑时屏蔽均值离地判据，双腿持续卸载仍判 AIR
#define BALANCE_USE_SLIP_AIR_INHIBIT    0u 

// 轮腿力矩分配 : 0 截断  1 支撑约束分配
#define BALANCE_USE_DISTRIBUTOR         0u 
#define BALANCE_DISTRIBUTOR_COMPARE     0u // 1 同时计算另一种策略作诊断，实际发送仍按上一项选择 

// 轮腿转弯目标限制 : 0 关闭 1 随旋转比例缩小直行目标 
#define BALANCE_USE_TURN_LIMIT          0u 

// 轮腿S曲线限速 : 0 关闭 1 速度目标 S 曲线限速
#define BALANCE_USE_VEL_LIMIT           0u 

// 轮腿转弯降高 : 0 关闭 1 转弯时降低高度
#define LEGLEN_TURN_ENABLE             0u 

// 轮腿向心力补偿 : 0 关闭 1 转弯时前馈向心力
#define BALANCE_USE_ROLL_COMP           0u 

// 轮腿科氏力补偿 : 0 关闭 1 转弯时前馈科氏力
#define BALANCE_USE_CORIOLIS_TP         0u 

#define BALANCE_USE_HEIGHT_FILTER       0u // 接地反馈：高度/升降速度 19 Hz 陷波，升降速度另加低通
#define BODY_ROLL_USE_KALMAN            0u // 0 低通，1 按 IMU 帧时标估计横滚角速度

#define APP_USE_MUSIC                   0u // 0 关闭 1 TIM12 蜂鸣器

// 台阶或离地抑制需要轮速帧时标。
#define SLIP_AIR_SLIP_ENABLED (BALANCE_USE_AIR_CHECK && BALANCE_USE_SLIP_AIR_INHIBIT)
#define SLIP_WHEEL_STAMP_ENABLED (STEP_ENABLE || SLIP_AIR_SLIP_ENABLED)

#endif /* SYSTEM_KEY_H */
