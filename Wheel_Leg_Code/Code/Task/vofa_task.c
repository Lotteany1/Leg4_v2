/** @file  vofa_task.c  @brief VOFA 手动通道选择与遥测发送  @version  v3.30  2026-11-24 */
// 当前启用前进加速诊断，CH0～7 沿用右腿测量布局。
// 公共项在前，专有项在后，按模块分组；关闭项用注释保留，手动注释启停。
// 单帧最多 64 路，INFINITY 仅作收尾；关闭模板中的 CH 编号沿用原帧，启用后按实际顺序更新。
#include "cmsis_os2.h"
#include "task_sync.h"
#include "FreeRTOS.h"
#include "task.h"
#include "wrap.h"
#include <math.h>
#include "vofa.h"
#include "imu.h"
#include "dm_motor.h"
#include "dr16.h"
#include "leg.h"
#include "body.h"
#include "fsm.h"
#include "app_safety.h"
#include "app_gyro.h"
#include "app_step.h"
#include "app_self_rescue.h"
#include "balance.h"
#include "distributor.h"
#include "imu_to_world.h"
#include "monitor.h"
#include "tinympc14.h"
#include "slip.h"
#include "air.h"
#include "can_bus.h"
#include "dji_motor.h"
#include "turn_limit.h"
#include "fsm_running.h"




// 加速诊断每 10 ms 采一帧。
void StartVofaTask(void *argument)
{
    (void)argument;
    while (!task_ready) osDelay(1u);
    uint32_t wake_tick    = osKernelGetTickCount();

    for (;;)
    {
        Monitor_Can_Sample();

        // 复制接地控制和执行快照，保留各自时标以便离线对齐。
        distributor_diag_t exec;
        balance_mpc_diag_t mpc;

        taskENTER_CRITICAL();
        Distributor_Diagnostic(&exec);
        Balance_Mpc_Diagnostic(&mpc);
        const uint32_t tick = osKernelGetTickCount();

        const fsm_state_t state = Fsm_State_Get();
        const fsm_running_position_t pos = Fsm_Running_Position();
        taskEXIT_CRITICAL();


        Vofa_Send_Float(
            // 上电后先显示实时状态，收到故障后持续显示最近一次故障码。
            (float)(dm_f_lft.last_fault_code ? dm_f_lft.last_fault_code : dm_f_lft.err_code), // I0 左前
            (float)(dm_b_lft.last_fault_code ? dm_b_lft.last_fault_code : dm_b_lft.err_code), // I1 左后
            (float)(dm_f_rgt.last_fault_code ? dm_f_rgt.last_fault_code : dm_f_rgt.err_code), // I2 右前
            (float)(dm_b_rgt.last_fault_code ? dm_b_rgt.last_fault_code : dm_b_rgt.err_code), // I3 右后
            (float)monitor_can[0].state, // I4 CAN1 状态码，0 正常，1 恢复中，2 总线判死
            // (float)monitor_can[0].heartbeat, // I5 控制任务心跳，每完成一拍加 1

            // dji_wheel_rgt.pos, // I6
            // dji_wheel_rgt.vel, // I7
            // dji_wheel_rgt.trq, // I8
            // 机体姿态 速度 高度
            // rad rad/s 姿态与角速度 m 机体高度 m/s 速度 g 机体系比力
            // body.ang_pitch, // rad 机体俯仰
            // body.omg_pitch,
            // body.ang_roll, // rad 机体横滚
            // body.omg_roll,
            // body.h_body, // m 机体高度
            // body.h_tgt,
            // body.ang_yaw_tgt,
            // body.ang_yaw, // rad 机体偏航
            // body.omg_yaw,
            // body.omg_yaw_filt,
            // body.body_accel_x,
            // body.body_accel_y,
            // body.body_accel_z,
            // imu_world.pitch,
            //body.vel, // m/s 两髋水平速度（未融合）
            body.vel_tgt,
            body.vel_kalman,

            // body.omg_yaw_tgt,

            // 遥控器原始通道
            // 通道与左右拨杆
            // (float)Dr16_Snapshot().ch1,
            // (float)Dr16_Snapshot().ch2,
            // (float)Dr16_Snapshot().ch3,
            // (float)Dr16_Snapshot().ch4,
            // (float)Dr16_Snapshot().ch5,
            //(float)Dr16_Snapshot().sw_left,
            //(float)Dr16_Snapshot().sw_right,
            // (float)Dr16_Snapshot().ch2, // 原始前后摇杆计数，死区 ±20

            // 关节角
            // rad 直读 DM 编码器加安装偏置
            // leg_lft.pitch_1,
            // leg_lft.pitch_4,
            // leg_rgt.pitch_1,
            // leg_rgt.pitch_4,

            // 关节角速度
            // rad/s 直读 DM 反馈角速度
            // leg_lft.d_pitch_1,
            // leg_lft.d_pitch_4,
            // leg_rgt.d_pitch_1,
            // leg_rgt.d_pitch_4,

            // 末端位姿与速度
            // m rad 五连杆极坐标 m/s rad/s 一阶导
            // leg_lft.len, // I9 m 左腿长
            // leg_lft.ang, // I10 rad 左腿机体系腿角
            // leg_lft.d_len,
            // leg_lft.d_ang,
            // leg_len_lft.tar,
            leg_rgt.len, // CH5 m 右腿长
            // leg_rgt.ang, // I12 rad 右腿机体系腿角
            // leg_rgt.d_len,
            // leg_rgt.d_ang,
            // leg_len_rgt.tar,

            // 腿端力与力矩
            // leg_control_lft.Comp_Ang,
            // leg_control_lft.Comp_Len,
            // leg_control_lft.SpringF,
            // leg_control_lft.F,
            // leg_control_lft.T,

            // dji_wheel_lft.trq,
            //leg_control_lft.Tp,
            // leg_control_lft.Roll_CompF,
            // body.leg_len_tgt_lft,
            // body.leg_len_lft,

            // leg_control_rgt.Comp_Ang,
            // leg_control_rgt.Comp_Len,
            // leg_control_rgt.SpringF,
            // leg_control_rgt.F,
            // leg_control_rgt.Tp,
            // leg_control_rgt.Roll_CompF,
            // body.leg_len_tgt_rgt,
            // body.leg_len_rgt,

            // 电机反馈
            // rad rad/s N·m 位置 速度 力矩
            // dm_f_lft.pos,
            // dm_f_lft.vel,
            // dm_f_lft.trq,
            // dji_wheel_lft.pos,
            // dji_wheel_lft.vel,
            // dji_wheel_lft.trq,
            // dji_wheel_rgt.pos,
            // dji_wheel_rgt.vel,
            // dji_wheel_rgt.trq,

            // 决策层
            // FSM 状态见 fsm_state_t；自救阶段见 self_rescue_phase_t
            // (float)Fsm_State_Get(),
            // (float)App_Self_Rescue_Get_Phase(),
            // App_Self_Rescue_Get_Dir(), // +1 现行扫腿 / -1 摆腿路径（已锁存生效值）
            // (float)leg_ctrl_mode,      // 0=NORMAL 1=SELF_RESCUE
            // (float)Fsm_Input_Get(),    // 0=DISABLE 1=RC 2=PC
            // (float)active_flag,

            // 驻车与轮髋执行诊断帧（5402）。
            /* 启用本组时，将以下快照准备移到发送调用前。
                // 驻车与轮髋执行诊断快照。请求、接纳、估计及电机反馈各自保留时标。
                distributor_diag_t exec;
                balance_mpc_diag_t mpc;

                taskENTER_CRITICAL();
                Distributor_Diagnostic(&exec);
                Balance_Mpc_Diagnostic(&mpc);
                const fsm_running_position_t park = Fsm_Running_Position();
                const uint32_t sample_tick = osKernelGetTickCount();
                const float position = body.pos;
                const float speed = body.vel;
                const float speed_ref = body.vel_tgt;
                const float hip_lft = body.Tp_lft;
                const float hip_rgt = body.Tp_rgt;
                const uint32_t estimator_tick = Estimator_Frame_Tick();
                const float hip_rate_lft = leg_lft.d_ang;
                const float hip_rate_rgt = leg_rgt.d_ang;
                const float pitch = body.ang_pitch;
                const float gyro_pitch = body.omg_pitch;
                const float gyro_yaw = body.omg_yaw;
                const uint32_t dm_tick[4] = {dm_f_lft.frame_tick, dm_b_lft.frame_tick,
                                             dm_f_rgt.frame_tick, dm_b_rgt.frame_tick};
                // 与 Execution_Values 的左右轮符号口径一致。
                const float wheel_feedback[2] = {WHEEL_LFT_POLARITY * dji_wheel_lft.trq, WHEEL_RGT_POLARITY * dji_wheel_rgt.trq};
                const uint32_t wheel_tick[2] = {dji_wheel_lft.frame_tick, dji_wheel_rgt.frame_tick};
                const float len[2] = {body.leg_len_lft, body.leg_len_rgt};
                const float roll = body.ang_roll;
                const float height = body.h_body;
                const float support[2] = {body.F_lft + exec.execution.spring_f[0],
                                          body.F_rgt + exec.execution.spring_f[1]};
                const float roll_rate = body.omg_roll;
                taskEXIT_CRITICAL();

                const float sin_roll = sinf(roll);
                const float cos_roll = cosf(roll);
                const float cos_pitch = cosf(pitch);
                const float pitch_rate = cos_roll * gyro_pitch - sin_roll * gyro_yaw;
                const float yaw_rate = fabsf(cos_pitch) > 0.1f ?
                                       (sin_roll * gyro_pitch + cos_roll * gyro_yaw) / cos_pitch : NAN;
            */
            // (float)sample_tick,                                     // CH00 ms，诊断快照时标
            // (float)mpc.sample_tick,                                 // CH01 ms，MPC 输入与计划时标
            // (float)exec.execution.tick,                             // CH02 ms，接纳记录源时标
            // (float)exec.mode,                                       // CH03 控制模式
            // position,                                               // CH04 m，两髋沿程里程
            // park.error,                                             // CH05 m，相对停车点误差
            // park.weight,                                            // CH06 位置反馈权重
            // (float)park.capture,                                    // CH07 累计停车点捕获
            // speed,                                                  // CH08 m/s，两髋速度，当前配置供 MPC 使用
            // speed_ref,                                              // CH09 m/s，速度参考
            // mpc.pitch,                                              // CH10 rad，俯仰
            // mpc.leg_angle[0],                                       // CH11 rad，左矢状腿角
            // mpc.leg_angle[1],                                       // CH12 rad，右矢状腿角
            // mpc.plan[2],                                            // CH13 N·m，左净髋请求
            // mpc.plan[3],                                            // CH14 N·m，右净髋请求
            // exec.input.leg[0].hip_comp,                             // CH15 N·m，左补偿
            // exec.input.leg[1].hip_comp,                             // CH16 N·m，右补偿
            // exec.input.leg[0].p_req + exec.input.leg[0].hip_comp,   // CH17 N·m，左合成请求
            // exec.input.leg[1].p_req + exec.input.leg[1].hip_comp,   // CH18 N·m，右合成请求
            // exec.execution.model_u[2] + exec.execution.hip_comp[0], // CH19 N·m，左接纳反算合成
            // exec.execution.model_u[3] + exec.execution.hip_comp[1], // CH20 N·m，右接纳反算合成
            // hip_lft,                                                // CH21 N·m，左电机反馈逆解
            // hip_rgt,                                                // CH22 N·m，右电机反馈逆解
            // mpc.plan[0],                                            // CH23 N·m，左轮模型请求
            // mpc.plan[1],                                            // CH24 N·m，右轮模型请求
            // exec.execution.model_u[0],                              // CH25 N·m，左轮接纳码，模型符号
            // exec.execution.model_u[1],                              // CH26 N·m，右轮接纳码，模型符号
            // (float)exec.execution.accepted_mask,                    // CH27 本拍 CAN 接纳位
            // (float)exec.execution.valid_mask,                       // CH28 接纳保持码有效位
            // (float)mpc.paired,                                      // CH29 预测与接纳已配对
            // (float)exec.limited_count,                              // CH30 关节限幅累计
            // (float)mpc.fallbacks,                                   // CH31 终端反馈累计
            // (float)exec.send_failure_count,                         // CH32 CAN 未接纳累计
            // hip_rate_lft,                                           // CH33 rad/s，左腿相对机体的矢状角速度 dψL−dθ
            // hip_rate_rgt,                                           // CH34 rad/s，右腿相对机体的矢状角速度 dψR−dθ
            // pitch_rate,                                             // CH35 rad/s，俯仰欧拉角速度
            // (float)estimator_tick,                                  // CH36 ms，估计帧时标
            // (float)dm_tick[0],                                      // CH37 ms，左前髋电机反馈帧时标
            // (float)dm_tick[1],                                      // CH38 ms，左后髋电机反馈帧时标
            // (float)dm_tick[2],                                      // CH39 ms，右前髋电机反馈帧时标
            // (float)dm_tick[3],                                      // CH40 ms，右后髋电机反馈帧时标
            // wheel_feedback[0],                                      // CH41 N·m，左轮反馈，模型符号
            // wheel_feedback[1],                                      // CH42 N·m，右轮反馈，模型符号
            // (float)wheel_tick[0],                                   // CH43 ms，左轮反馈帧时标
            // (float)wheel_tick[1],                                   // CH44 ms，右轮反馈帧时标
            // len[0],                                                 // CH45 m，左腿长
            // len[1],                                                 // CH46 m，右腿长
            // roll,                                                   // CH47 rad，横滚角
            // yaw_rate,                                               // CH48 rad/s，偏航欧拉角速度
            // height,                                                 // CH49 m，机体高度估计
            // support[0],                                             // CH50 N，左轴力反馈逆解加执行快照弹簧估算
            // support[1],                                             // CH51 N，右轴力反馈逆解加执行快照弹簧估算
            // roll_rate,                                              // CH52 rad/s，原始机体系滚转陀螺量
            // 5402.0f,                                                // CH53 布局标识

            // 高度和横滚诊断帧。差值统一左减右，请求、分配和接纳的和与差可还原左右两侧。
            /* 启用本组时，将以下快照准备移到发送调用前。
                distributor_diag_t roll_exec;
                balance_mpc_diag_t roll_solve;

                taskENTER_CRITICAL();
                Distributor_Diagnostic(&roll_exec);
                Balance_Mpc_Diagnostic(&roll_solve);
                const uint32_t sample_tick = osKernelGetTickCount();
                const tinympc14_counters_t backend = tinympc14_counters;
                const int iterations       = tinympc14_status.iters;
                const float backend_us     = monitor_win.mpc_us;
                const float roll           = body.ang_roll;
                const float roll_rate      = body.omg_roll;
                const bool gyro_running    = App_Gyro_Running();
                const float len_lft        = body.leg_len_lft;
                const float len_rgt        = body.leg_len_rgt;
                const float height         = body.h_body;
                const float height_target  = body.h_tgt;
                const float force_fb_diff  = body.F_lft - body.F_rgt;
                const float force_fb_sum   = body.F_lft + body.F_rgt;
                taskEXIT_CRITICAL();

                const uint32_t roll_flags = roll_exec.result.request_mask |
                                            ((uint32_t)roll_exec.execution.accepted_mask << 8u) |
                                            ((uint32_t)roll_exec.execution.valid_mask << 14u) |
                                            ((uint32_t)roll_exec.input.normal_mode << 20u) |
                                            ((uint32_t)roll_solve.paired << 21u) |
                                            ((uint32_t)gyro_running << 22u) |
                                            (1u << 23u); // 后台诊断帧标记，整个整数仍可由单精度无损表示
            */
            // (float)sample_tick,                                              // CH00 ms 遥测采样时标
            // (float)backend.solve_attempt,                                    // CH01 后台取得活动输入的累计尝试
            // (float)roll_solve.sample_tick,                                   // CH02 ms TinyMPC14 取样时标
            // (float)roll_solve.publish_tick,                                  // CH03 ms TinyMPC14 发布时间
            // (float)backend.lpv_updates,                                      // CH04 LPV 调度模型累计重算次数
            // roll,                                                            // CH05 rad 机体横滚角
            // roll_rate,                                                       // CH06 rad/s 原始横滚角速度
            // roll_solve.roll_rate,                                            // CH07 rad/s TinyMPC14 使用的横滚角速度
            // (float)backend.reject_start_age,                                 // CH08 输入在求解开始时已过期的累计次数
            // roll_solve.roll_ref,                                             // CH09 rad TinyMPC14 横滚参考
            // (float)backend.reject_input,                                     // CH10 输入或模型准备失败的累计次数
            // height,                                                          // CH11 m 机体高度
            // (float)backend.reject_convergence,                               // CH12 求解未达到收敛条件的累计次数
            // roll_exec.input.leg[0].s_req - roll_exec.input.leg[1].s_req,       // CH13 N 分配前总支持力请求差
            // (float)backend.reject_finish_age,                                // CH14 求解完成时序列过期的累计次数
            // roll_exec.execution.total_s[0] - roll_exec.execution.total_s[1],  // CH15 N CAN 接纳命令反算总支持力差
            // force_fb_diff,                                                   // CH16 N 电机力矩反馈逆解轴力差，不含弹簧
            // roll_exec.execution.spring_f[0] - roll_exec.execution.spring_f[1], // CH17 N 执行记录采用的弹簧力差估计
            // (float)backend.reject_epoch,                                     // CH18 输入代次切换或停止导致取消的累计次数
            // backend_us,                                                      // CH19 µs 最近一次后台任务调用耗时，含抢占
            // (float)roll_flags,                                               // CH20 位 0–7 请求超限，8–13 接纳，14–19 有效，20 接地，21 配对，22 陀螺，23 后台诊断帧
            // (float)roll_exec.mode,                                           // CH21 当前控制模式
            // (float)roll_solve.fallbacks,                                     // CH22 终端反馈回退累计拍数
            // (float)roll_solve.stale,                                         // CH23 预测序列超龄作废累计次数
            // (float)iterations,                                               // CH24 次 最近一次实际执行 ADMM 的迭代数
            // (float)Dr16_Snapshot().ch5,                                                   // CH25 遥控高度滚轮原始计数，正值超过 20 请求降低
            // height_target,                                                   // CH26 m 行为层处理后的高度目标
            // roll_solve.height_ref,                                           // CH27 m 送入 MPC 的绝对高度参考
            // roll_solve.height,                                               // CH28 m 送入 MPC 的绝对高度反馈，已按配置滤波
            // roll_solve.height_rate,                                          // CH29 m/s 送入 MPC 的升降速度
            // len_lft,                                                         // CH30 m 左腿实测长度
            // len_rgt,                                                         // CH31 m 右腿实测长度
            // roll_exec.input.leg[0].s_req + roll_exec.input.leg[1].s_req,      // CH32 N 分配前总轴力之和，与 CH13 配对
            // roll_exec.result.s_alloc[0] + roll_exec.result.s_alloc[1],        // CH33 N 分配后总轴力之和
            // roll_exec.execution.total_s[0] + roll_exec.execution.total_s[1],  // CH34 N CAN 接纳码反算总轴力之和，与 CH15 配对
            // force_fb_sum,                                                    // CH35 N 电机力矩反馈反算轴力之和，不含弹簧，与 CH16 配对
            // roll_exec.execution.spring_f[0] + roll_exec.execution.spring_f[1], // CH36 N 被动气弹簧轴力估计之和，与 CH17 配对
            // (float)backend.solve_age_ms,                                     // CH37 ms 最近活动尝试从采样至返回的年龄
            // (float)backend.commit,                                           // CH38 后台成功提交序列累计次数
            // (float)roll_exec.limited_count,                                  // CH39 关节请求超限累计控制拍数，含两次遥测之间的限幅
            // roll_solve.predicted_force.valid ? (float)roll_solve.predicted_force.tick : -1.0f, // CH40 ms 最近预测节点拍的四关节接纳时标，-1 尚无
            // roll_solve.predicted_force.sum,                                  // CH41 N 该拍 CAN 接纳总轴力之和，含弹簧
            // roll_solve.predicted_force.diff,                                 // CH42 N 该拍 CAN 接纳总轴力之差，左减右
            // roll_solve.terminal_force.valid ? (float)roll_solve.terminal_force.tick : -1.0f, // CH43 ms 最近终端反馈拍的四关节接纳时标，-1 尚无
            // roll_solve.terminal_force.sum,                                   // CH44 N 该拍 CAN 接纳总轴力之和，含弹簧
            // roll_solve.terminal_force.diff,                                  // CH45 N 该拍 CAN 接纳总轴力之差，左减右
            // roll_solve.leg_angle[0],                                         // CH46 rad CH02 拍 MPC 左腿矢状角 x[4]
            // roll_solve.leg_angle[1],                                         // CH47 rad CH02 拍 MPC 右腿矢状角 x[6]
            // roll_solve.accel_y,                                              // CH48 g CH02 拍机体 y 轴比力，含重力投影
            // roll_solve.pitch,                                                // CH49 rad CH02 拍机体实测 pitch
            // (float)roll_solve.force_valid,                                   // CH50 位0/1：本拍左/右已建立轴力估计有效
            // roll_solve.force_error[0],                                       // CH51 N 左腿已建立力减对应的接纳力
            // roll_solve.force_error[1],                                       // CH52 N 右腿已建立力减对应的接纳力
            // roll_solve.lowering_reduction,                                   // CH53 N 降高分配本拍削减的多余合力

            // (float)roll_solve.paired, (float)roll_solve.epoch, (float)roll_solve.cycle, (float)roll_solve.accepted_mask,
            // roll_solve.plan[0], roll_solve.plan[1], roll_solve.plan[2], roll_solve.plan[3], roll_solve.plan[4], roll_solve.plan[5],
            // roll_solve.accepted[0], roll_solve.accepted[1], roll_solve.accepted[2], roll_solve.accepted[3], roll_solve.accepted[4], roll_solve.accepted[5],

            // LAND 验证帧（单独启用，通道表见 tests/landing_validation.md）
            /* 启用本组时，将以下快照准备移到发送调用前。
                distributor_diag_t landing;
                balance_mpc_diag_t land_solve;
                taskENTER_CRITICAL();
                Distributor_Diagnostic(&landing);
                Balance_Mpc_Diagnostic(&land_solve);
                const uint32_t sample_tick = osKernelGetTickCount();
                const bool spring_recovering = landing.land.active;
                const fsm_running_position_t land_park = Fsm_Running_Position();
                const float land_flags = landing.input.normal_mode ? (float)(landing.result.request_mask |
                                                 ((uint32_t)landing.land_limited_mask << 8u) |
                                                 ((uint32_t)landing.result.support_relaxed_mask << 10u) |
                                                 ((uint32_t)landing.execution.accepted_mask << 12u) |
                                                 ((uint32_t)landing.execution.valid_mask << 18u)) : 0.0f;
                taskEXIT_CRITICAL();
            */
            // (float)sample_tick,
            // (float)landing.mode,
            // spring_recovering ? (float)landing.land.elapsed_ms : -1.0f,
            // spring_recovering ? 100.0f * landing.land.spring_scale : 100.0f,
            // (float)(air_status.flag | ((uint32_t)air_status.contact_mask << 1u)),
            // air_status.fn_det_lft, air_status.fn_det_rgt,
            // leg_lft.len, leg_rgt.len, leg_lft.d_len, leg_rgt.d_len,
            // landing.input.leg[0].spring_f, landing.input.leg[1].spring_f,
            // spring_recovering ? landing.land.nominal[0] : landing.input.leg[0].s_req,
            // spring_recovering ? landing.land.nominal[1] : landing.input.leg[1].s_req,
            // landing.input.leg[0].s_req, landing.input.leg[1].s_req,
            // landing.execution.total_s[0], landing.execution.total_s[1],
            // land_flags, body.ang_pitch, body.ang_roll,
            // land_park.weight, (float)land_park.capture,
            // landing.result.s_alloc[0], landing.result.s_alloc[1],
            // landing.state.roll_rate, land_solve.roll_rate, land_solve.sample_tick,
            // (float)landing.input.strategy,

            // CAN1 负载
            // 100 ms 窗口 µs/ms 占用 满总线 1000 收发帧数
            // monitor_can[0].load_us_per_ms,
            // (float)monitor_can[0].tx_frames,
            // (float)monitor_can[0].rx_frames,
            // 诊断计数 tx_full 涨发不出去 rx_loss 涨读不走
            // (float)monitor_can[0].tx_full_cnt,
            // (float)monitor_can[0].rx_loss_cnt,

            // 离地检测
            // N 含 Tp 项观测 剔除 Tp 项判定 两腿判定均值 30 N 离地 70 N 落地
            // air_status.fn_lft,
            // air_status.fn_rgt,
            // air_status.fn_det_lft, // 左腿判定用支持力
            // air_status.fn_det_rgt,
            // air_status.fn_avg,
            // (float)air_status.flag,
            // (float)air_status.slip_inhibited, // 1 本拍打滑屏蔽支持力均值离地判据

            // 打滑速度融合
            // m/s 融合速度 轮速减融合速度 离地开关电平
            // slip_status.vel_fused,
            // slip_status.diff,
            // (float)slip_status.air,

            // 单侧收腿和分侧检测，启用时读取 step 快照。
            /* 启用本组时，将以下快照准备移到发送调用前。
                step_status_t step;
                App_Step_Status(&step);
            */
            // (float)step.phase,
            // (float)step.side,
            // (float)step.rejected,
            // (float)step.terminated,
            // (float)step.elapsed_ms,
            // (float)step.slip.side,
            // step.slip.residual[0],
            // step.slip.residual[1],
            // (float)step.slip.duration[0],
            // (float)step.slip.duration[1],
            // (float)step.slip.valid_mask,
            // (float)step.slip.usable_mask,
            // (float)step.contact_valid,
            // (float)step.contact_mask,
            // (float)step.unloaded_mask,
            // step.target[0],
            // step.target[1],
            // step.length[0],
            // step.length[1],
            // step.target_rate,
            // step.rate[0],
            // step.rate[1],
            // step.requested[0],
            // step.requested[1],
            // step.accepted[0],
            // step.accepted[1],
            // (float)step.accepted_mask,
            // (float)step.execution_valid,
            // (float)step.limit_mask,
            // step.height,
            // step.height_ref,
            // step.roll,
            // step.roll_rate,
            // (float)step.observation,

            // 质心偏移判读
            // 腿角稳在 θ* 且 MgF≈0 才自洽 落到约 2θ* 说明模型缺常数重力项
            // body.leg_ang_lft,
            // body.leg_ang_rgt,
            // body.ang_pitch,
            // leg_control_lft.MgF,

            // MPC 求解与耗时
            // 迭代数与残差 converged=0 即本拍走回退 µs 本拍与窗口峰值
            // (float)tinympc14_status.iters,
            // (float)tinympc14_status.converged,
            // tinympc14_status.pri,
            // tinympc14_status.dua,
            // monitor_win.mpc_us,
            // monitor_win.mpc_us_max,
            // monitor_win.tick_us,
            // monitor_win.tick_us_avg,
            // monitor_win.tick_us_max,
            // monitor_win.period_us_max,
            // (float)monitor_win.overrun_cnt,

            // MPC 计量计数
            // 累计计数看增长量 commit 线性涨 stale async_fb 与 overrun 常态为 0
            // (float)tinympc14_counters.commit,
            // (float)tinympc14_counters.stale,
            // (float)tinympc14_counters.fallback,
            // (float)tinympc14_counters.nan,
            // 腿长侧 fallback 常态不涨 stale 涨即求解停摆
            // 两路常态不涨 一涨说明 F 与 Tp 混合后撞 DM_T_MAX
            // (float)monitor_total.leg_clamp_lft_cnt,
            // (float)monitor_total.leg_clamp_rgt_cnt,

            // 自救测试：时间与状态（CH0～9）
            // (float)osKernelGetTickCount(),         // CH00 ms 任务取样时刻
            // (float)Fsm_State_Get(),                // CH01 外层 FSM
            // (float)App_Self_Rescue_Get_Phase(),    // CH02 自救阶段
            // App_Self_Rescue_Get_Dir(),             // CH03 +1 正转扫腿，-1 反转摆腿
            // (float)rescue_diag.round_count,        // CH04 已结束轮数，从 0 开始
            // rescue_diag.second_path ? 1.0f : 0.0f, // CH05 0 初始路径，1 本轮备用路径
            // (float)rescue_diag.attempt_ms,         // CH06 ms 当前路径累计时间
            // (float)rescue_diag.stall_lft_ms,       // CH07 ms 左腿卡死累计时间
            // (float)rescue_diag.stall_rgt_ms,       // CH08 ms 右腿卡死累计时间
            // (float)rescue_diag.retract_ms,         // CH09 ms 换向收腿累计时间

            // 自救测试：机体姿态（CH10～13）
            // Wrap_Pi(imu_world.pitch), // CH10 rad 还原后的 pitch，折回 (-π, π]
            // body.ang_roll,            // CH11 rad 机体 roll
            // body.omg_pitch,           // CH12 rad/s 机体俯仰角速度
            // body.omg_roll,            // CH13 rad/s 机体横滚角速度

            // 自救测试：腿长目标与实测（CH14～19）
            // leg_lft.len,          // CH14 m 左腿实测长度
            // leg_rgt.len,          // CH15 m 右腿实测长度
            // body.leg_len_tgt_lft, // CH16 m 左腿目标长度
            // body.leg_len_tgt_rgt, // CH17 m 右腿目标长度
            // leg_lft.d_len,        // CH18 m/s 左腿实测伸缩速度
            // leg_rgt.d_len,        // CH19 m/s 右腿实测伸缩速度

            // 自救测试：腿角目标与实测（CH20～25）
            // leg_lft.ang,                       // CH20 rad 左腿连续机构角
            // leg_rgt.ang,                       // CH21 rad 右腿连续机构角
            // body.leg_ang_tgt_lft,              // CH22 rad 左腿位置环目标机构角
            // body.leg_ang_tgt_rgt,              // CH23 rad 右腿位置环目标机构角
            // Imu_To_World_Leg_Ang(leg_lft.ang), // CH24 rad 左腿世界系摆角，折回 (-π, π]
            // Imu_To_World_Leg_Ang(leg_rgt.ang), // CH25 rad 右腿世界系摆角，折回 (-π, π]

            // 自救测试：角速度目标与实测（CH26～31）
            // leg_lft.d_ang,        // CH26 rad/s 左腿机构角速度；卡死检测输入
            // leg_rgt.d_ang,        // CH27 rad/s 右腿机构角速度；卡死检测输入
            // body.leg_omg_lft,     // CH28 rad/s 左腿世界系角速度；速度 PID 反馈
            // body.leg_omg_rgt,     // CH29 rad/s 右腿世界系角速度；速度 PID 反馈
            // body.leg_omg_tgt_lft, // CH30 rad/s 左腿速度环目标
            // body.leg_omg_tgt_rgt, // CH31 rad/s 右腿速度环目标

            // 自救测试：PID 输出（CH32～39，仅对应阶段有效）
            // sr_pid_leg_len_lft.out,       // CH32 N 左腿长度 PID 输出
            // sr_pid_leg_len_rgt.out,       // CH33 N 右腿长度 PID 输出
            // sr_pid_leg_ang_lft.out,       // CH34 N·m 左腿位置 PID 输出
            // sr_pid_leg_ang_rgt.out,       // CH35 N·m 右腿位置 PID 输出
            // sr_pid_leg_omg_swing_lft.out, // CH36 N·m 左腿反转摆腿速度 PID 输出
            // sr_pid_leg_omg_swing_rgt.out, // CH37 N·m 右腿反转摆腿速度 PID 输出
            // sr_pid_leg_omg_stand_lft.out, // CH38 N·m 左腿反转收腿／立腿速度 PID 输出
            // sr_pid_leg_omg_stand_rgt.out, // CH39 N·m 右腿反转收腿／立腿速度 PID 输出

            // 自救测试：合成请求与反馈（CH40～47，请求为关节限幅前）
            // leg_control_lft.F,  // CH40 N 左腿合成轴向力请求；映射和关节限幅前
            // leg_control_rgt.F,  // CH41 N 右腿合成轴向力请求；映射和关节限幅前
            // leg_control_lft.Tp, // CH42 N·m 左腿合成髋力矩请求；映射和关节限幅前
            // leg_control_rgt.Tp, // CH43 N·m 右腿合成髋力矩请求；映射和关节限幅前
            // body.F_lft,         // CH44 N 左腿电机反馈逆解轴向力
            // body.F_rgt,         // CH45 N 右腿电机反馈逆解轴向力
            // body.Tp_lft,        // CH46 N·m 左腿电机反馈逆解髋力矩
            // body.Tp_rgt,        // CH47 N·m 右腿电机反馈逆解髋力矩

            // 自救测试：关节限幅与安全状态（CH48～51）
            // (float)monitor_total.leg_clamp_lft_cnt, // CH48 左腿关节限幅累计次数，看增量
            // (float)monitor_total.leg_clamp_rgt_cnt, // CH49 右腿关节限幅累计次数，看增量
            // App_Safety_Latched() ? 1.0f : 0.0f,     // CH50 0 正常，1 安全锁存
            // (float)active_flag,                     // CH51 0 失能，1 使能请求

            // 静止轮震荡：时间与状态（CH0～2）
            // 顺序读取，不保证同一控制拍；时间戳用于识别实际采样间隔与丢帧。
            // (float)osKernelGetTickCount(), // CH00 ms 任务取样时刻
            // (float)Fsm_State_Get(),        // CH01 外层 FSM
            // (float)active_flag,           // CH02 使能请求

            // 静止轮震荡：机体姿态（CH3～10）
            // body.ang_pitch,     // CH03 rad 俯仰角
            // body.omg_pitch,     // CH04 rad/s 原始俯仰角速度，平衡反馈使用
            // body.omg_pitch_filt, // CH05 rad/s 滤波俯仰角速度
            // body.ang_roll,      // CH06 rad 横滚角
            // body.omg_roll,      // CH07 rad/s 横滚角速度
            // body.ang_yaw,       // CH08 rad 偏航角
            // body.omg_yaw_filt,  // CH09 rad/s 滤波偏航角速度
            // body.omg_yaw_tgt,   // CH10 rad/s 目标偏航角速度

            // 静止轮震荡：平移与腿状态（CH11～22）
            // body.pos,         // CH11 m 位移反馈
            // body.vel,         // CH12 m/s 运动学速度
            // body.vel_kalman,  // CH13 m/s 融合速度
            // body.vel_tgt,     // CH14 m/s 目标速度
            // body.leg_len_lft, // CH15 m 左腿长
            // body.leg_len_rgt, // CH16 m 右腿长
            // body.leg_ang_lft, // CH17 rad 左腿世界系摆角
            // body.leg_ang_rgt, // CH18 rad 右腿世界系摆角
            // body.leg_omg_lft, // CH19 rad/s 左腿世界系角速度
            // body.leg_omg_rgt, // CH20 rad/s 右腿世界系角速度
            // body.h_body,      // CH21 m 机体高度
            // body.h_tgt,       // CH22 m 目标高度

            // 静止轮震荡：轮电机请求与反馈（CH23～32）
            // cmd_trq 为协议计数，trq 为电流换算 N·m；请求不代表总线已接纳，右轮反馈比较请求时取反。
            // leg_control_lft.T,     // CH23 N·m 左轮控制请求
            // leg_control_rgt.T,     // CH24 N·m 右轮控制请求
            // dji_wheel_lft.cmd_trq, // CH25 原始计数 左轮电流请求
            // dji_wheel_rgt.cmd_trq, // CH26 原始计数 右轮电流请求
            // dji_wheel_lft.trq,     // CH27 N·m 左轮反馈力矩
            // dji_wheel_rgt.trq,     // CH28 N·m 右轮反馈力矩，电机原生符号
            // dji_wheel_lft.vel,     // CH29 rad/s 左轮电机反馈速度
            // dji_wheel_rgt.vel,     // CH30 rad/s 右轮电机反馈速度
            // body.whl_vel_lft,      // CH31 rad/s 左轮地相对角速度
            // body.whl_vel_rgt,      // CH32 rad/s 右轮地相对角速度

            // 静止轮震荡：髋腿出力与扰动（CH33～39）
            // leg_control_lft.Tp, // CH33 N·m 左髋合成请求，关节限幅前
            // leg_control_rgt.Tp, // CH34 N·m 右髋合成请求，关节限幅前
            // body.Tp_lft,        // CH35 N·m 左髋反馈逆解
            // body.Tp_rgt,        // CH36 N·m 右髋反馈逆解
            // leg_control_lft.F,  // CH37 N 左腿合成轴向力，关节限幅前
            // leg_control_rgt.F,  // CH38 N 右腿合成轴向力，关节限幅前

            // 静止轮震荡：求解与时序（CH40～48）
            // (float)tinympc14_status.converged,        // CH40 最近一次 TinyMPC14 求解收敛标志
            // tinympc14_status.pri,                     // CH41 原始残差
            // tinympc14_status.dua,                     // CH42 对偶残差
            // (float)tinympc14_counters.stale,     // CH44 累计解停更次数，看增量
            // (float)tinympc14_counters.fallback,  // CH45 累计异步回退次数，看增量
            // (float)monitor_win.overrun_cnt,        // CH46 窗口控制掉队次数，窗口清零
            // monitor_win.period_us_max,             // CH47 us 窗口控制周期峰值
            // (float)tinympc14_counters.commit,    // CH48 累计 TinyMPC14 序列提交次数

            // 静止轮震荡：机体系比力（CH49～51）
            // body.body_accel_x, // CH49 g 机体 X 比力
            // body.body_accel_y, // CH50 g 机体 Y 比力
            // body.body_accel_z, // CH51 g 机体 Z 比力

            // 执行器分配诊断：原 64 路帧，开启时关闭其他发送项，避免超出 64 路。
            /* 启用本组时，将以下快照准备移到发送调用前。
                distributor_diag_t distributor;
                execution_observer_diag_t observer[2];
                const uint32_t config_flags = BALANCE_DISTRIBUTOR_COMPARE ? 1u : 0u;
                taskENTER_CRITICAL();
                Distributor_Diagnostic(&distributor);
                Execution_Observer_Diagnostic(observer);
                taskEXIT_CRITICAL();
            */
            // 命令、状态和接纳记录来自同一控制拍；配置位及比较结果有效位见原采集说明。
            // 3.0f,
            // (float)distributor.input.state_tick,
            // (float)distributor.input.seq,
            // (float)distributor.input.epoch,
            // (float)distributor.mode,
            // (float)distributor.input.strategy,
            // (float)distributor.result.status,
            // (float)distributor.result.result_valid,
            // distributor.input.leg[0].s_req,
            // distributor.input.leg[1].s_req,
            // distributor.input.leg[0].p_req,
            // distributor.input.leg[1].p_req,
            // distributor.result.s_alloc[0],
            // distributor.result.s_alloc[1],
            // distributor.result.p_alloc[0],
            // distributor.result.p_alloc[1],
            // distributor.result.tau_alloc[0],
            // distributor.result.tau_alloc[1],
            // distributor.result.tau_alloc[2],
            // distributor.result.tau_alloc[3],
            // (float)distributor.execution.code[0],
            // (float)distributor.execution.code[1],
            // (float)distributor.execution.code[2],
            // (float)distributor.execution.code[3],
            // (float)distributor.execution.code[4],
            // (float)distributor.execution.code[5],
            // (float)distributor.execution.accepted_mask,
            // (float)distributor.execution.valid_mask,
            // (float)((uint32_t)distributor.result.active_mask | ((uint32_t)distributor.result.request_mask << 8)),
            // (float)distributor.result.support_relaxed_mask,
            // (float)distributor.normal_count,
            // (float)distributor.limited_count,
            // (float)distributor.changed_count,
            // (float)distributor.failure_count,
            // (float)distributor.protection_count,
            // (float)distributor.fallback_count,
            // (float)distributor.support_relaxed_count,
            // (float)distributor.send_failure_count,
            // (float)distributor.wheel_limited_count,
            // distributor.tau_change_max,
            // distributor.s_error_max,
            // distributor.p_error_max,
            // distributor.state.pitch,
            // distributor.state.pitch_rate,
            // distributor.state.roll,
            // distributor.state.roll_rate,
            // distributor.state.height,
            // distributor.state.height_ref,
            // distributor.state.len[0],
            // distributor.state.len[1],
            // distributor.wheel_req[0],
            // distributor.wheel_req[1],
            // distributor.execution.model_u[0],
            // distributor.execution.model_u[1],
            // distributor.execution.model_u[2],
            // distributor.execution.model_u[3],
            // distributor.execution.total_s[0],
            // distributor.execution.total_s[1],
            // (float)observer[0].quality,
            // (float)observer[0].skipped,
            // (float)distributor.limited_run_max,
            // (float)(config_flags | (distributor.compare[0].result_valid ? 2u : 0u) | (distributor.compare[1].result_valid ? 4u : 0u)),

            // 试验台气弹簧补偿与关节执行
            // (float)osKernelGetTickCount(),                    // CH11 ms 遥测采样时标
            // (float)Fsm_State_Get(),                   // CH12 FSM 状态，5 为 TEST
            // (float)active_flag,                     // CH13 电机使能
            // leg_lft.d_len,                          // CH14 m/s 左腿伸缩速度
            // leg_rgt.d_len,                          // CH15 m/s 右腿伸缩速度
            // leg_control_lft.SpringF,                // CH16 N 左侧气弹簧抵消力
            // leg_control_rgt.SpringF,                // CH17 N 右侧气弹簧抵消力
            // leg_control_lft.F,                      // CH18 N 左侧电机轴向请求，伸腿为正
            // leg_control_rgt.F,                      // CH19 N 右侧电机轴向请求，伸腿为正
            // leg_control_lft.Tp,                     // CH20 N·m 左侧髋力矩请求
            // leg_control_rgt.Tp,                     // CH21 N·m 右侧髋力矩请求
            // leg_lft.pitch_1,                        // CH22 rad 左侧 VMC 输入关节角
            // leg_lft.pitch_4,                        // CH23 rad 左侧 VMC 输入关节角
            // leg_rgt.pitch_1,                        // CH24 rad 右侧 VMC 输入关节角
            // leg_rgt.pitch_4,                        // CH25 rad 右侧 VMC 输入关节角
            // leg_control_lft.out[0],                 // CH26 N·m 左前关节限幅前请求
            // leg_control_lft.out[1],                 // CH27 N·m 左后关节限幅前请求
            // leg_control_rgt.out[1],                 // CH28 N·m 右前关节限幅前请求
            // leg_control_rgt.out[0],                 // CH29 N·m 右后关节限幅前请求
            // dm_f_lft.trq,                           // CH30 N·m 左前力矩反馈
            // dm_b_lft.trq,                           // CH31 N·m 左后力矩反馈
            // dm_f_rgt.trq,                           // CH32 N·m 右前力矩反馈
            // dm_b_rgt.trq,                           // CH33 N·m 右后力矩反馈
            // (float)monitor_total.leg_clamp_lft_cnt,  // CH34 左侧关节限幅累计次数
            // (float)monitor_total.leg_clamp_rgt_cnt,  // CH35 右侧关节限幅累计次数
            // (float)monitor_can[0].tx_full_cnt,       // CH36 CAN 1 发送失败累计次数
            // (float)monitor_can[1].tx_full_cnt,       // CH37 CAN 2 发送失败累计次数
            // (float)monitor_can[2].tx_full_cnt,       // CH38 CAN 3 发送失败累计次数
            // leg_control_lft.MgF,                    // CH39 N·m 左侧重力髋力矩补偿
            // leg_control_rgt.MgF,                    // CH40 N·m 右侧重力髋力矩补偿
            // leg_lft.d_ang,                          // CH41 rad/s 左腿机体系角速度
            // leg_rgt.d_ang,                          // CH42 rad/s 右腿机体系角速度
            // (float)monitor_can[0].bus_off_cnt,       // CH43 CAN 1 bus-off 累计次数
            // (float)monitor_can[1].bus_off_cnt,       // CH44 CAN 2 bus-off 累计次数
            // (float)monitor_can[2].bus_off_cnt,       // CH45 CAN 3 bus-off 累计次数
            // body.F_lft,                                          // CH46 N 左腿反馈轴力，不含气弹簧
            // body.F_rgt,                                          // CH47 N 右腿反馈轴力，不含气弹簧

            // 驻车模式与位置反馈
            // (float)osKernelGetTickCount(),                                  // CH11 ms 遥测采样时标
            // body.vel_tgt,                                        // CH14 m/s 前后速度目标
            // body.vel_kalman,                                     // CH15 m/s 融合速度
            // body.omg_yaw_tgt,                                    // CH16 rad/s 转向速度目标
            // body.ang_yaw - body.ang_yaw_tgt,                     // CH17 rad 偏航角误差
            // body.omg_yaw_filt,                                   // CH18 rad/s 滤波偏航角速度
            // body.omg_pitch,                                      // CH19 rad/s 机体俯仰角速度
            // body.leg_ang_lft,                                    // CH20 rad 左腿世界系倾角
            // body.leg_ang_rgt,                                    // CH21 rad 右腿世界系倾角
            // parking.error,                                       // CH22 m 相对驻车点位置，目标为 0
            // odometry,                                            // CH23 m 两髋水平累计里程
            // parking.weight,                                      // CH24 位置参考权重，含驻车系数与保护恢复斜坡
            // (float)(parking.events >> 1),                        // CH25 驻车点作废累计次数
            // (float)parking.capture,                              // CH26 驻车点捕获累计次数

            // POS/YAW 扰动观测和直接补偿·
            // 执行与支撑
            // (float)tinympc14_counters.fallback,               // CH41 控制回退累计次数
            // body.h_tgt,                                          // CH45 m 机体高度目标
            // 备用驻车诊断项
            // body.leg_len_lft,                                 // m 左腿长
            // body.leg_len_rgt,                                 // m 右腿长
            // body.ang_pitch,                                   // rad 俯仰
            // (float)tinympc14_status.converged,                  // 最近求解收敛
            // monitor_win.mpc_us,                               // µs 最近平衡求解耗时
            // monitor_win.mpc_us_max,                           // µs 平衡求解耗时窗口峰值
            // body.omg_yaw,                                     // rad/s 偏航角速度

            // CAN1 故障记录保留到下一次 bus-off，短于遥测周期的故障也能看到。
            // (float)monitor_can[0].bus_off_cnt, // I13 累计进入 bus-off 次数
            // (float)monitor_can[0].bo_recover_cnt, // I14 累计确认控制器恢复次数
            // (float)monitor_can[0].bus_off_psr, // I15 进入 bus-off 时的协议状态
            // (float)monitor_can[0].bus_off_ecr, // I16 同次故障的收发错误计数寄存器
            // (float)monitor_can[0].rx_frames, // I17 最近 100 ms 窗口的总线收帧数
            // (float)dm_f_lft.rx_count, // I18 左前累计反馈
            // (float)dm_b_lft.rx_count, // I19 左后累计反馈
            // (float)dm_f_rgt.rx_count, // I20 右前累计反馈
            // (float)dm_b_rgt.rx_count, // I21 右后累计反馈
            // (float)tick,                // CH9  ms 遥测时标
            // (float)state,               // CH10 FSM 状态
            // (float)mpc.sample_tick,     // CH11 ms 接地控制状态时标
            // (float)exec.execution.tick, // CH12 ms 执行状态时标
            // exec.state.height_ref,      // CH13 m 原始高度目标
            // mpc.height_ref,             // CH14 m 协调后的控制目标
            // mpc.height,                 // CH15 m 控制器使用的高度
            // mpc.height_rate,            // CH16 m/s 控制器使用的升降速度
            // exec.state.height,          // CH17 m 原始估计高度
            // mpc.roll_ref,               // CH18 rad 横滚目标
            // exec.state.roll,            // CH19 rad 横滚角
            // mpc.roll_rate,              // CH20 rad/s 模型横滚角速度
            // exec.state.len[0],          // CH21 m 执行拍左腿长
            // exec.state.len[1],          // CH22 m 执行拍右腿长
            // mpc.plan[4],                // CH23 N 模型请求左总轴力
            // mpc.plan[5],                // CH24 N 模型请求右总轴力
            // mpc.accepted[4],            // CH25 N 入队命令折算左总轴力
            // mpc.accepted[5],            // CH26 N 入队命令折算右总轴力
            // (float)mpc.paired,          // CH27 请求与接纳记录同拍
            // (float)mpc.accepted_mask,   // CH28 本拍入队成功位
            // mpc.force_error[0],         // CH29 N 左侧反馈总轴力减上拍接纳值
            // mpc.force_error[1],         // CH30 N 右侧反馈总轴力减上拍接纳值
            // (float)mpc.force_valid,     // CH31 反馈有效位，位 0 左、位 1 右
            // mpc.pitch,                  // CH32 rad 俯仰角
            // mpc.leg_angle[0],           // CH33 rad 左腿矢状角
            // mpc.leg_angle[1],           // CH34 rad 右腿矢状角
            // (float)mpc.fallbacks,       // CH35 使用终端反馈的累计拍数
            // (float)exec.limited_count,  // CH36 关节请求超界累计拍数
            // mpc.lowering_reduction,           // CH37 N 降高协调削减的合力
            // (float)mpc.predicted_force.valid, // CH38 本代次预测节点接纳记录有效
            // (float)mpc.predicted_force.tick,  // CH39 ms 最近预测节点接纳时标
            // mpc.predicted_force.sum,          // CH40 N 预测节点接纳的左右总轴力之和
            // mpc.predicted_force.diff,         // CH41 N 预测节点接纳的左减右轴力
            // (float)mpc.terminal_force.valid,  // CH42 本代次终端反馈接纳记录有效
            // (float)mpc.terminal_force.tick,   // CH43 ms 最近终端反馈接纳时标
            // mpc.terminal_force.sum,           // CH44 N 终端反馈接纳的左右总轴力之和
            // mpc.terminal_force.diff,          // CH45 N 终端反馈接纳的左减右轴力
            // exec.execution.spring_f[0],       // CH46 N 执行快照左腿被动气弹簧估计力
            // exec.execution.spring_f[1],       // CH47 N 执行快照右腿被动气弹簧估计力
            dm_b_rgt.pos, // CH6 rad right rear encoder
            dm_f_rgt.pos, // CH7 rad right front encoder
            (float)tick, // CH8 ms 遥测时标
            (float)mpc.sample_tick, // CH9 ms MPC 量测时标
            (float)Dr16_Snapshot().ch2, // CH10 前后摇杆
            body.vel_tgt, // CH11 m/s 最终目标速度
            body.vel, // CH12 m/s 运动学速度
            body.vel_kalman, // CH13 m/s 融合速度
            body.pos, // CH14 m 累计位置
            pos.error, // CH15 m 停车点误差
            pos.weight, // CH16 位置反馈权重
            (float)pos.capture, // CH17 停车点捕获次数
            (float)exec.mode, // CH18 控制模式
            (float)state, // CH19 FSM 状态
            slip_status.a_lon, // CH20 m/s² 纵向加速度
            body.ang_pitch, // CH21 rad 俯仰
            body.omg_pitch, // CH22 rad/s 俯仰角速度
            mpc.plan[0], // CH23 左轮请求
            mpc.plan[1], // CH24 右轮请求
            mpc.accepted[0], // CH25 左轮接纳
            mpc.accepted[1], // CH26 右轮接纳
            (float)mpc.paired, // CH27 请求和接纳同拍
            (float)mpc.fallbacks, // CH28 回退累计
            (float)exec.wheel_limited_count, // CH29 轮矩限幅累计
            (float)Slip_Observation(), // CH30 速度观测来源位
            (float)exec.send_failure_count, // CH31 发送失败累计
            INFINITY);



        uint32_t now = osKernelGetTickCount();
        wake_tick += 10u;
        if ((int32_t)(wake_tick - now) <= 0) wake_tick = now + 10u;
        osDelayUntil(wake_tick);
    }
}
