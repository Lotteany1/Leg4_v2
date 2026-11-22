/** @file  turn_limit.h  @brief 转弯驾驶限制、腿长协调及集中调参  @version  v3.4  2026-11-15 */
#ifndef TURN_LIMIT_H
#define TURN_LIMIT_H

#include "body.h"
#include <stdbool.h>

// 转弯限制
#define TURN_LIMIT_YAW_FULL 6.0f // rad/s 遥控旋转满量程 达到其 100% 时直行目标归零
#define TURN_LIMIT_VEL_FULL 2.85f // m/s 遥控直行满量程
#define LEGLEN_TURN_BEGIN       0.65f  // 补偿需求/名义差动力余量，开始降权
#define LEGLEN_TURN_FULL        1.00f  // 达到名义能力时使用最大降权
#define LEGLEN_TURN_HYST        0.05f  // 利用率下降支路回差
#define LEGLEN_TURN_ATTACK      4.0f   // 1/s 降权比例增加速率
#define LEGLEN_TURN_RELEASE     1.0f   // 1/s 降权比例恢复速率
#define LEGLEN_TURN_ROLL_KEEP   0.25f  // 最大降权时保留 25% 角度误差，角速度目标始终为零
#define LEGLEN_TURN_ROLL_ALLOW  0.08f  // rad 允许参考跟随的最大侧倾角，约 4.6 度
#define LEGLEN_TURN_ROLL_STOP   0.14f  // rad 达到后抑制转弯驾驶需求，非物理防倾倒保证
#define LEGLEN_TURN_DROP        0.025f // m 最大自动降高量
#define LEGLEN_TURN_FLOOR       0.20f  // m 自动降高目标下界，用户更低目标不被抬高
#define LEGLEN_TURN_RESERVE     0.015f // m 短腿需保留的压缩行程
#define LEGLEN_TURN_ACTIVE_F    1.0f   // N 向心等效力超过此值启用高度优先

// 后台单拍快照；力箱来自当前求解模型，模块不依赖 MPC 表或求解器。
typedef struct
{
    float h, h_target;         // m 实测高度与用户目标
    float roll;                // rad 实测横滚角
    float len[2];              // m 左、右腿长
    float turn_force;          // N 单侧向心等效力，正负分别表示左右方向
    float force_min, force_max; // N 单腿总支持力箱
    float dt;                  // s 策略更新周期，有限且大于零
} turn_limit_leglen_input_t;

// 调用方独占保存；首拍置零，取消求解时恢复此前副本，避免未发布策略推进。
typedef struct
{
    float relax;          // 0..1 降权状态，跨拍保留
    float ratio;          // 向心补偿需求/名义差动力余量
    float roll_keep;      // 0..1 保留角度误差比例
    float roll_ref;       // rad 放宽后的横滚参考，角速度参考仍为零
    float h_target;       // m 降高与行程上限后的目标，不覆写用户目标
    bool height_priority; // true 要求高度优先分配合力
} turn_limit_leglen_t;

// 已校验的测量快照输入；每个新测量调用一次，更新调用方状态，不读取实时全局机体状态。
void Turn_Limit_Leglen_Update(const turn_limit_leglen_input_t *input, turn_limit_leglen_t *state);

// 腿长转弯协调启用时，超允许侧倾还会缩小驾驶需求；仅改目标，不代表实际转弯立即停止。
// state 为输入输出机体目标 线速度 m/s 偏航角速度 rad/s 每拍在遥控与陀螺更新后仅调用一次
void Turn_Limit_Update(body_state_t *state);

#endif
