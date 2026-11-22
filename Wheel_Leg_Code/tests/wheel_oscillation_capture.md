# 静止轮震荡采集（2026-10-30）

固件：当前工作区配置，平衡 MPC、融合速度、腿长 MPC、俯仰 LESO 开启。未调整 Q/R 或控制参数。烧录后启用此帧。

VOFA+：JustFloat，UART8 对应串口，460800 baud，8N1，无流控；52 通道，编号从 CH0 开始。任务间隔 10 ms，名义约 100 Hz，实际间隔以 CH0 为准。每帧 212 字节，名义 21.2 kB/s。

录制从使能前开始，覆盖静止站立震荡约 20～30 秒；保持摇杆回中，不推机体、不改变腿长。一次记录保留为单独文件。导出全部通道原始 CSV，保留表头及时间列，不只发截图。提供所烧固件版本、腿高、地面情况，以及轮电机是前后同步抖动还是左右反向抖动。

优先查看 CH3/4 俯仰、CH12/13 运动学与融合速度、CH23/24 轮力矩请求、CH25～30 电流请求/反馈和电机速度、CH39 俯仰扰动，以及 CH40～48 求解与时序。

角度 rad，角速度 rad/s。轮电机 CH25～28 是电流协议原始计数，不是 N·m；左右安装方向不同，分析时按控制映射统一符号。通道按参数顺序读取，不保证同一控制拍；求解状态可能在两个遥测样本间变化。100 Hz 遥测不能可靠判别接近或超过 50 Hz 的震荡，若波形显示高频特征再做精简高速采集。

CH46 是窗口掉队次数，会随监控窗口清零；CH43～45、CH48 是累计计数，观察增量。LQR 模式下 MPC 诊断不作为有效控制依据。发送器将 NaN 转零，遇到 INFINITY 会提前结束本帧，导出时需检查通道数。

| 通道 | 表达式 | 含义与单位 |
| --- | --- | --- |
| CH00 | `(float)osKernelGetTickCount()` | ms 任务取样时刻 |
| CH01 | `(float)Fsm_State_Get()` | 外层 FSM |
| CH02 | `(float)active_flag` | 使能请求 |
| CH03 | `body.ang_pitch` | rad 俯仰角 |
| CH04 | `body.omg_pitch` | rad/s 原始俯仰角速度，平衡反馈使用 |
| CH05 | `body.omg_pitch_filt` | rad/s 滤波俯仰角速度 |
| CH06 | `body.ang_roll` | rad 横滚角 |
| CH07 | `body.omg_roll` | rad/s 横滚角速度 |
| CH08 | `body.ang_yaw` | rad 偏航角 |
| CH09 | `body.omg_yaw_filt` | rad/s 滤波偏航角速度 |
| CH10 | `body.omg_yaw_tgt` | rad/s 目标偏航角速度 |
| CH11 | `body.pos` | m 位移反馈 |
| CH12 | `body.vel` | m/s 运动学速度 |
| CH13 | `body.vel_kalman` | m/s 融合速度 |
| CH14 | `body.vel_tgt` | m/s 目标速度 |
| CH15 | `body.leg_len_lft` | m 左腿长 |
| CH16 | `body.leg_len_rgt` | m 右腿长 |
| CH17 | `body.leg_ang_lft` | rad 左腿世界系摆角 |
| CH18 | `body.leg_ang_rgt` | rad 右腿世界系摆角 |
| CH19 | `body.leg_omg_lft` | rad/s 左腿世界系角速度 |
| CH20 | `body.leg_omg_rgt` | rad/s 右腿世界系角速度 |
| CH21 | `body.h_body` | m 机体高度 |
| CH22 | `body.h_tgt` | m 目标高度 |
| CH23 | `leg_control_lft.T` | N·m 左轮控制请求 |
| CH24 | `leg_control_rgt.T` | N·m 右轮控制请求 |
| CH25 | `dji_wheel_lft.cmd_trq` | 原始计数 左轮电流请求 |
| CH26 | `dji_wheel_rgt.cmd_trq` | 原始计数 右轮电流请求 |
| CH27 | `dji_wheel_lft.trq` | 原始计数 左轮反馈电流 |
| CH28 | `dji_wheel_rgt.trq` | 原始计数 右轮反馈电流 |
| CH29 | `dji_wheel_lft.vel` | rad/s 左轮电机反馈速度 |
| CH30 | `dji_wheel_rgt.vel` | rad/s 右轮电机反馈速度 |
| CH31 | `body.whl_vel_lft` | rad/s 左轮地相对角速度 |
| CH32 | `body.whl_vel_rgt` | rad/s 右轮地相对角速度 |
| CH33 | `leg_control_lft.Tp` | N·m 左髋合成请求，关节限幅前 |
| CH34 | `leg_control_rgt.Tp` | N·m 右髋合成请求，关节限幅前 |
| CH35 | `body.Tp_lft` | N·m 左髋反馈逆解 |
| CH36 | `body.Tp_rgt` | N·m 右髋反馈逆解 |
| CH37 | `leg_control_lft.F` | N 左腿合成轴向力，关节限幅前 |
| CH38 | `leg_control_rgt.F` | N 右腿合成轴向力，关节限幅前 |
| CH39 | `Model_Pitch_Dist()` | N·m/髋 俯仰 LESO 扰动估计 |
| CH40 | `(float)tinympc_status.converged` | 最近一次平衡求解收敛标志 |
| CH41 | `tinympc_status.pri` | 原始残差 |
| CH42 | `tinympc_status.dua` | 对偶残差 |
| CH43 | `(float)monitor_total.mpc_fallback_cnt` | 累计回退次数，看增量 |
| CH44 | `(float)monitor_total.mpc_stale_cnt` | 累计解停更次数，看增量 |
| CH45 | `(float)monitor_total.mpc_async_fb_cnt` | 累计异步回退次数，看增量 |
| CH46 | `(float)monitor_win.overrun_cnt` | 窗口控制掉队次数，窗口清零 |
| CH47 | `monitor_win.period_us_max` | us 窗口控制周期峰值 |
| CH48 | `(float)monitor_total.mpc_commit_cnt` | 累计平衡解提交次数 |
| CH49 | `body.body_accel_x` | g 机体 X 比力 |
| CH50 | `body.body_accel_y` | g 机体 Y 比力 |
| CH51 | `body.body_accel_z` | g 机体 Z 比力 |
