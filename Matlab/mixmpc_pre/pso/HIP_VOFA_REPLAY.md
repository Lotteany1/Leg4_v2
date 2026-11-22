# 新轮矩时标录波与低腿驻车验证

本次保留当前 Q/R。六组候选均至少剩一个低腿直行工况无法在 3.2 s 非线性仿真内满足停稳条件。

## 实车数据与辨识

录波覆盖 20.762 s，有 747 帧 5402 诊断数据。I43/I44 全部有效，两路 CAN 轮矩反馈距 I0 采样 0–1 ms。相邻采样中位数为 13 ms，有 40 处间隔超过 80 ms，最大缺口 1.39 s。实测腿长中位数 0.17 m，5% 和 95% 分位为 0.15 m 和 0.20 m。这份录波没有中腿对照。285 帧驻车权重已达到 0.45，估计速度 RMS 为 0.668 m/s。在行驶模式且速度参考为零的 186 帧中，估计速度 RMS 为 0.838 m/s。

将轮矩、髋矩与估计帧对齐，以 100/140 ms 无重叠短窗积分动力学，用录波前后半段互相留出。使用轮矩反馈直接拟合，同时复算投影消去轮矩的模型。按输入矩阵验证了髋矩和腿相对机身角速度的正方向，仿真中的被动阻尼矩取与相对角速度反向。

| 拟合方式 | 两折与时序扰动范围（N·m·s/rad） |
|---|---:|
| 直接纳入实测轮矩，黏性项 | 2.07–5.23 |
| 投影消去轮矩，黏性项 | 0.51–2.08 |
| 同拟黏性项和库仑摩擦 | 0–7.44 |

两个黏性项版本都比各自的零阻尼留出模型误差小。加入库仑摩擦后部分折落到零边界。本轮用 0–5.5 的阻尼范围做筛选，并附加 7.5 的非线性压力工况。物理阻尼标定仍需独立录波。PPO 中 4–24 ms 的执行延迟是仿真扰动范围。

## PPO 与闭环复核

15 组倍率覆盖所有 20 个 Q/R 项，左右同类项共同缩放。宽域倍率范围为 1/2.25–2.25，局部倍率范围为 1/1.22–1.22。两个范围都用固定种子 19、59、117，另有基线对照，共 1,344 次采样。线性留出工况包含低腿起停、驻车扰动和中腿回归。宽域没有候选同时通过两组中腿门限并改善留出代价，局部有 10 组通过两组中腿门限。

基线和宽域、局部各三组候选完成 70 项非线性闭环。每组包含六个低腿和四个中腿工况，采用逐工作点 Riccati、约束 MPC、2 ms 预测节点和 4 ms 后台求解周期。模拟了 0.10 m/s 驻车锁点门槛、腿角门限和位置权重爬升。

| 指标 | 基线 | 宽域 s59_trial01 | 局部 local_s59_trial05 |
|---|---:|---:|---:|
| 六个低腿工况的末段速度 RMS 均值，m/s | 0.0532 | 0.0271 | 0.0450 |
| 低腿最差末段速度 RMS，m/s | 0.1476 | 0.0616 | 0.1259 |
| 低腿最差末段位置跨度，mm | 48.2 | 14.9 | 31.4 |
| 低腿直行距零速指令约 1.68 s 时仍未停稳 | 2/4 | 1/4 | 2/4 |
| 中腿直行两工况速度 RMS，m/s | 0.206 / 0.267 | 0.203 / 0.257 | 0.204 / 0.262 |
| 十个工况最大姿态误差，° | 17.90 | 14.29 | 17.12 |

宽域候选在 0.19 m、阻尼 2.1、倒车参考 −0.71 m/s 的工况中，未在时窗内持续低于 0.06 m/s。该候选也没有通过线性留出的中腿回归门限。局部候选守住线性中腿门限，仍有两个低腿起停工况未停稳。偏离停车点后的回位时间也有触及观察窗上限的工况。本轮候选未进入 fast、完整构建或实机 A/B。

[辨识记录](build/hip_vofa_replay/identification.json) · [宽域 PPO](build/hip_vofa_replay/ppo_results.json) · [局部 PPO](build/hip_vofa_replay/ppo_local_results.json) · [非线性基线](build/hip_vofa_replay/mpc/baseline.json) · [最接近候选与完整 Q/R](build/hip_vofa_replay/mpc/s59_trial01.json) · [局部候选](build/hip_vofa_replay/mpc/local_s59_trial05.json)

## 复跑

从仓库根目录执行。原始录波已归档到 build/hip_vofa_replay/low_capture.csv，配置和模型源在同目录的 src 快照。

    $scripts = 'Matlab/mixmpc_pre/pso'
    $output = "$scripts/build/hip_vofa_replay"
    $env:OPENBLAS_NUM_THREADS = '1'
    $env:OMP_NUM_THREADS = '1'
    python -B "$scripts/verify_hip_vofa_ppo.py" --export-model
    python -B "$scripts/fit_hip_vofa.py" "$output/low_capture.csv"
    python -B "$scripts/run_hip_vofa_ppo.py" --steps 256
    python -B "$scripts/run_hip_vofa_ppo.py" --tag local --radius 1.22 --steps 192
    python -B "$scripts/verify_hip_vofa_ppo.py"
    python -B "$scripts/verify_hip_vofa_ppo.py" --tag local

MATLAB 复核会复用已完成的同名 JSON。更换输入或重新搜索后，将原 mpc 目录移到另一个名字再复核。
