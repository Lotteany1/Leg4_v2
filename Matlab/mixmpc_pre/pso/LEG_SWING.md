# VOFA 摆腿峰值与 Q/R 复验

最新录波有 664 帧有效的 5402 数据，覆盖 11.324 s。按左右腿对世界系矢状角中较大的绝对值统计，速度指令幅值至少 0.25 m/s 时峰值为 28.1°。速度参考归零而系统仍处于行驶模式时，峰值为 41.3°。两次主要峰值出现在 5.469 s 和 9.315 s，分别为 41.3° 和 37.8°。两次左右轮接纳力矩均为 4.93 N·m。轮矩反馈时标距采样为 0–1 ms。录波最大缺口 0.489 s，峰值以实际收到的帧为准。

以当前 C 工程内的 brake01 作为基线。隔离仿真覆盖 17–26.5 cm、最高 1.58 m/s 指令、正反向切换和松杆。被控对象包含独立的负载、俯仰与横滚扰动、执行延迟和髋阻尼。场景源于录波的行驶范围，未逐点重放实车轨迹。

优先筛除发散，随后要求最差腿角至少下降 10%，单个场景的腿角不得增加超过 0.004 rad。有限基线工况限制速度误差 RMS 增量 0.035 m/s、停稳时间增量 0.25 s，同时检查俯仰、横滚和求解回退。仿真结果需用独立实机试验复核。

[实车录波统计](build/leg_swing/capture_metrics.json)

## 选定测试参数

测试了 24 组宽域、9 组折中和 3 组补充组合。选定的 swing14 在最后 13 个工况中全部跑完。brake01 有 3 个反向切换工况触发发散判据。其中最急的一次在 34.6° 触发判据，swing14 峰值为 28.3°。在 17 cm 高速倒车松杆工况，松杆后的腿角峰值从 23.2° 降到 15.7°，停稳由 0.746 s 延至 1.286 s。

这组权重把两侧腿角代价从 699.15 增至 3143.72。速度误差 RMS 的最大增加量为 0.058 m/s，停稳时间最大增加量为 0.546 s，均超过最初设定的兼顾跟踪门槛。因本轮优先抑制摆腿，选它作为独立实车对照参数。

[完整 Q/R](build/leg_swing/candidate.json) · [逐工况结果](build/leg_swing/stress/swing14.json) · [压力复验对比](build/leg_swing/stress_results.json)

隔离测试固件已通过 fast MATLAB/C 对拍、主机闭环和 ARM 编译链接，终端 P 离网格相对误差为 0.163%。HEX 校验和通过，已核对固件构建使用的 Q/R 与候选一致。

[swing14 测试固件](build/leg_swing/packages/swing14/arm/Wheel_Leg_Code.hex) · [构建记录](build/leg_swing/verification.json)

## 复跑

从仓库根目录执行以下命令。运行读取 C 工程中的 brake01 Q/R 作为基线，现有同方案输出可续跑。

    python -B Matlab/mixmpc_pre/pso/search_leg_swing.py --workers 3
    python -B Matlab/mixmpc_pre/pso/search_leg_swing.py --workers 3 --refine
    python -B Matlab/mixmpc_pre/pso/search_leg_swing.py --workers 3 --stress
