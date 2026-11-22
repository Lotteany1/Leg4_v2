# TinyMPC14 Q/R 自动调参

在仓库根目录用 PowerShell 运行。需要 Python 3.10+、NumPy 2.x、仓库现有的 MATLAB / Control System Toolbox / Coder / Optimization Toolbox，以及原主机测试使用的 MSVC、Clang 和固件 AC5 工具链。默认走仓库已有工具路径，`--matlab` 和 `--armcc` 可覆盖 MATLAB 和 AC5 路径。

```powershell
python Matlab/mixmpc_pre/pso/run_pso.py
```

本次归档位于 `build/experiment`。重新搜索使用默认的新运行目录。恢复原位搜索保持相同粒子数和轮数。搜索阶段用按粒子落盘的检查点重建 PSO 状态。验证阶段按排名重新执行各门限。

```powershell
python Matlab/mixmpc_pre/pso/run_pso.py --run Matlab/mixmpc_pre/pso/build/experiment --phase search --particles 8 --rounds 4 --archive-rounds 3
python Matlab/mixmpc_pre/pso/run_pso.py --run Matlab/mixmpc_pre/pso/build/experiment --phase validate --finalists 8
python Matlab/mixmpc_pre/pso/render_report.py Matlab/mixmpc_pre/pso/build/experiment
```

`--phase baseline` 单独运行基线与 VOFA 分析。可用 `--capture-dir` 指向四份同命名录波的副本。记录按 `baseline_build.json`、`vofa.json`、`progress*.json`、`candidates/`、`ranking.json`、`validation.json`、`logs/`、`packages/` 查阅。最终判定见 [报告](REPORT.md)。

每次运行从当前工作树复制源码和需要的模型缓存。只有隔离副本的配置会被写入，且只替换 Q/R 块。缓存校验走原入口，过期按完整构建重建。MATLAB 搜索辅助源码以 `.m.txt` 保存，运行时才写出到 `pso/build/matlab`。验证入口的 MATLAB 进程不加载搜索辅助函数。固件交叉编译使用隔离副本，构建产物落到 `packages/<id>/arm`。

## 搜索范围与排序

倍率作用于 `pitch dpitch roll droll H dH R_FL=R_FR`。保持前八个 Q 和前四个 R。各维先取自然对数再更新粒子，边界截断，单步速度不得超过区间宽度 35%。惯性 0.55，自身与群体系数均为 1.45，随机数算法 MT19937。

| 粒子群 | 初始设计 | 七维倍率下界 | 上界 |
|---|---|---|---|
| 基线附近，种子 20261119 | 基线、3 个手选探针和随机粒子 | 0.5, 0.25, 0.5, 0.25, 0.5, 0.25, 0.2 | 2, 4, 2, 4, 2, 4, 20 |
| 独立重启，种子 20262881 | 各维分层打乱采样 | 同上 | 同上 |
| 历史权重扩展，种子 20264643 | 分层采样并植入 4 个历史方向探针 | 0.1, 0.001, 0.5, 0.25, 0.1, 0.25, 0.2 | 2, 4, 2, 100, 2, 4, 20 |

历史方向参照 `Wheel_Leg_Code/build/height_roll_structure_20261119` 的编译快照。保持当前轮矩等未搜索权重。两路轴力 R 搜索区间每侧 0.01–1.0。多起点和分层采样扩大覆盖，仍以实际评估过的位置为结论范围。

搜索集 8 个工况，留出集 4 个工况。两组都覆盖定高直行、升降、横滚扰动和转弯升降，具体取值保存在每个候选的 `conditions` 数组里。顺序依次为起点高度 m、目标行程 m、执行传输 ms、一阶响应 ms、横滚惯量 kg·m²、等效升降附加载荷 kg、初始滚速冲击 rad/s、目标车速 m/s、目标偏航率 rad/s、仿真求解间隔 ms。

- 搜索每个工况 2.4 s，参考以 0.25 m/s 斜坡升降。扰动在 0.60 s 施加。控制与 plant 步长 2 ms。档案中的板端控制为 1 ms，高度滤波在每个 2 ms 区间用保持输入推进两次原系数。
- 两轮平地 plant 在各工况起点线性化，闭环重建腿长，调度控制模型在每个更新点解 DARE。模型极点必须稳定。使用仓库的约束 ADMM、输出限幅和回退，搜索期间以精确终端 P 代替部署插值。完整链路使用生成的终端表。
- 腿长调度域 0.16–0.34 m，初始高度 0.18–0.30 m。高度范围参考原模型、录波及测试夹具，初始近短腿场景为模型压力工况。载荷基准来自装车称重和 CAD 等效质量，增加 0–0.35 kg 是轻微装备或载荷误差的敏感性假设。横滚惯量取 0.232–1.371 kg·m²，覆盖仓库闭链估计的两种约束。两腿支持力为每侧 20–150 N、变化率 3 N/ms。
- 执行分两至六毫秒传输和六至十二毫秒一阶响应，归一化静态增益采用 1。滚速观测按约 1.9 ms 一阶滤波近似。反馈冲量观测采用仓库 28 ms 常数的离散近似。转弯用速度与偏航率估算横向载荷转移。轮髋电机动态、CAN 丢包、轮胎滑移和背景线程墙钟时延在搜索模型中省略，独立 C fixture 加入实际固件的电机编码、执行记录和控制输出。主机测试不会复现真实板端抢占。
- 综合分用 55% 工况均值与 45% 最差工况。工况分对姿态和横滚峰值给最高权重，并计入横滚恢复、高度 RMS / 稳态误差 / 超调 / 10% 行程延迟 / 稳定时间、速度误差、限位和限速占比、回退率。完整公式在 MATLAB 模板。筛选还要求搜索与留出分别优于基线综合分，每个工况姿态与横滚峰值不超过基线的 105% 加 0.003 rad，回退不比基线多 5 个百分点。
- 候选筛选保留两个分数集。留出不反馈给三个粒子群，最终用于部署候选资格。默认验证八个排名靠前的候选，保留一个历史区间候选，依次 fast、仓库独立固件 C 闭环、完整构建、AC5 交叉编译链接。排名较高的合格候选通过后结束验证。完整/fast 的六个文件若哈希有差异，按完整包再跑 C 闭环。未过原门限的候选记录出错阶段与原始日志。

## 录波对照

基线对 11 月 19 日四次录波做板上时钟对齐。最新结构修复前录波用于识别差力路径，另一高位窗验证相位预测。02:06 和 02:24 的高位窗用于 CAN 输入重放。重放用台架标称参数积分 120 ms，不可辨认的轮矩和扰动置零。诊断结果中保留原始版本差异、两位小数导出限制和短窗误差。实际当前版本的 A/B 时使用 [通道与口径](VOFA.md)。

## 交叉复核

[交叉复核报告](RECHECK.md) 记录补点、新非线性工况、扩展固件 C 网格及实际发布结果。下列命令使用原实验的隔离基线作为参照，证据写入 `build/recheck`。

```powershell
python Matlab/mixmpc_pre/pso/recheck_pso.py
python Matlab/mixmpc_pre/pso/recheck_firmware.py baseline g04_p02
python Matlab/mixmpc_pre/pso/select_rechecked.py
python Matlab/mixmpc_pre/pso/render_recheck.py
```

选择入口以实际执行结果作为门限，并将候选放进隔离构建。用 `--apply` 会把选定参数写入实际 config，执行完整、fast 构建，并把生成 C/H 同步至 `Code/Model`。需保留最初的实际基线配置才可重复应用这个归档的结果。独立完整构建日志与输入快照在 `build/recheck` 归档。

## 全参数搜索

在仓库根目录运行 `python Matlab/mixmpc_pre/pso/run_full20.py`。它以当时实际配置为基线，独立搜索 14 个 Q 和 6 个 R，验证结果写入 `build/full20`，过程留有检查点。选择 `--apply` 会在候选完成现有构建验证后写入实际 config，同步 C/H 并重新链接固件。需要保留新的或单独的运行目录时可用 `--run`。本轮方法与逐工况结果见 [全参数报告](FULL20.md)。
