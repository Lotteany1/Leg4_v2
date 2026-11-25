/** @file  system_key.h  @brief 系统功能开关  @version  v1.26  2026-11-24 */
#ifndef SYSTEM_KEY_H
#define SYSTEM_KEY_H
 
// 选择传感器、控制算法或启停功能来这里。

// 电机出力开关：0 保持 DM 失能，每拍发零出力 MIT，DJI 仅允许零电流，1 正常控制。
#define MOTOR_OUTPUT_ENABLE             1u

// 选择 IMU 型号 : 0 Hi12  1 Hi229
#define IMU_USE_HI229                   0u

// 防打滑开关 : 0 轮腿速度 1 融合速度
#define BALANCE_USE_SLIP                1u 

// 持续单侧打滑时自动收腿：0 关闭，1 开启
// 动作入口还要求 BALANCE_USE_SLIP 和 BALANCE_USE_AIR_CHECK 同时开启。
#define STEP_ENABLE                    0u

// 轮腿离地检测 : 0 关闭  1 开启
#define BALANCE_USE_AIR_CHECK           0u 

// 打滑离地抑制 : 0 关闭  1 采用当拍打滑信息，双腿持续卸载仍判 AIR
#define BALANCE_USE_SLIP_AIR_INHIBIT    0u 

// 落地气弹簧缓冲
#define FSM_LAND_RECOVER_MS         30u // ms 从确认触地到恢复全部气弹簧补偿的时间，0 表示立即恢复
#define FSM_LAND_SPRING_START_PCT   80u // % 触地首拍保留的轴向气弹簧补偿，0 为全撤，100 为全保留

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


#define APP_USE_MUSIC                   0u // 0 关闭 1 TIM12 蜂鸣器

// 台阶或离地抑制需要轮速帧时标。
#define SLIP_AIR_SLIP_ENABLED (BALANCE_USE_AIR_CHECK && BALANCE_USE_SLIP_AIR_INHIBIT)
#define SLIP_WHEEL_STAMP_ENABLED (STEP_ENABLE || SLIP_AIR_SLIP_ENABLED)

#endif /* SYSTEM_KEY_H */
