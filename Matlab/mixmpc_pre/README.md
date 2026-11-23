# TinyMPC14 融合控制器

当前恢复为 v1 的理想五连杆计算与原有构建流程。质量、质心、惯量和轮胎尺寸采用 001 CAD 的当前参数；`leg_config.m` 与 `tinympc14_config.m` 保留直接赋值和参数表。

实车气弹簧前馈继续使用当前 18 点经验表。升降、横滚和偏航采用原流程的名义参数近似，不使用新增的 CAD 动态惯量表或高度参考制动器。完整 CAD 资料与恢复前源码保存在 `research/v1_format_restore_20261122/execution_before`。

日常 14 态模型采用装车质量标定。升降等效惯量和静态轴力由完整 URDF 在双轮接地工作点的动能与重力势能投影得到，再按装车分组质量折算。横滚惯量仍采用现有等效值。核算假设、适用腿长及复算命令见[高度和横滚工作点核算](modeling/MODELING_HANDOFF.md)。

MATLAB 运行时接收估计层给出的 14 个状态。实车的高度陷波、升降速度低通及横滚角速度滤波位于固件控制和估计层，开关在 `Wheel_Leg_Code/Code/Config/system_key.h`。这些滤波参数可通过重新编译固件生效。

双轮平地支撑时，虚腿长度满足 `Lᵢ=(H±b sinφ)/(cosφ cosψᵢ)`。高度、横滚的广义输入使用同一腿长雅可比转置；等效升降质量和横滚惯量是在这两个坐标下使用的降阶参数。计算倾覆力矩时单独处理静载预压，调度模型、静态配平与非线性校核沿用同一约定。[闭链输入核算](modeling/MODELING_HANDOFF.md)

接地控制区分每腿上拍 CAN 接纳的总轴力与本拍电机反馈加气弹簧估算得到的总轴力。前者是力变化率约束的起点，后者为预测提供首个 2 ms 区间的跟踪误差。模型重算后，用该模型的输入矩阵重新投影同拍的转弯载荷和倾覆力矩。前台使用的预测节点和终端回退都能响应新反馈。[接地控制入口](../../Wheel_Leg_Code/Code/Control/balance.c)

降高时先计算按幅值与速率限制所能达到的左右腿力。横滚差力挤占卸载所需的合力时，在当拍可行范围内分配两腿，保留可提供的静态回正差力。随当前或预测横滚接近姿态保护边界，逐步恢复原控制的差力余量。[输出分配](deploy/solver/tinympc14.c)

横滚换向有两段可观测的滞后。已接纳的左右差力需要时间才能按限速撤回。已经发出、仍在电机侧建立的差力，可用命令与反馈的动态差值估计其冲量。观测器在固件按 1 ms 更新，去除静态偏置，任一侧反馈失效或采样中断后重新播种。后台同时发布已算出的横滚角速度轨迹，前台按观测时刻插值，用已有终端反馈增益修正左右差力。[实车观测](../../Wheel_Leg_Code/Code/Control/balance.c) · [预测序列消费](deploy/solver/tinympc14.c)

修改前先读 [构建报错与维护约束](docs/BUILD_FAILURES.md)；其中记录 S8/G19 根因、倾覆扰动合同和完整/fast 回归步骤。

## 顶层五个 MATLAB 入口

| 文件 | 用途 |
|---|---|
| [mixmpc_paths.m](mixmpc_paths.m) | 项目路径、生成物目录和共享 `Matlab/Tool` 路径。 |
| [tinympc14_config.m](tinympc14_config.m) | 物理参数、Q/R、周期、约束和默认输出位置。 |
| [verify_mixmpc_pre.m](verify_mixmpc_pre.m) | 默认验证普通模型；可显式选择 URDF 或闭链动力学对照。 |
| [build_mixmpc_pre.m](build_mixmpc_pre.m) | 完整生成、验证并输出普通模型包。 |
| [build_mixmpc_pre_fast.m](build_mixmpc_pre_fast.m) | 仅改 Q/R 后复用模型缓存，重算增益，按需运行 C 对拍。 |

从本目录运行：

```matlab
verify_mixmpc_pre
build_mixmpc_pre('E:/TEMP/mixmpc_pre_output')
% 只改 tinympc14_config.m 的 Q/R-BLOCK 后：
build_mixmpc_pre_fast('E:/TEMP/mixmpc_pre_fast')
```

默认验证检查模型不变量、线性化、物理量和 VL-WIP 动量/功率；完整构建另运行生成表、QP、调度与闭环验证。快速构建默认跳过横滚合同和主机 C 对拍。`build_mixmpc_pre(output_dir, false)` 仅校验并发布已有表；`build_mixmpc_pre_fast(output_dir, true)` 启用横滚合同和主机 C 对拍。两种快速模式均保留缓存和数值检查。

修改物理参数或调度几何后，先完整构建并重新生成模型缓存。后续只改 Q/R 时可用快速构建。

## 子目录

- `functions/`：模型、求解与补偿辅助函数，含 URDF 参数提取和 VL-WIP 参考模型。
- `deploy/`：生成和发布实现；`solver/` 保留单个 `tinympc14.c/.h` 求解器。
- `verify/`：细分验证和主机 C 对拍。
- `modeling/`：完整 URDF、闭链动力学、接触与物理对照。
- `build/`：生成数据、缓存和验证输出，保留历史产物。
- `archive/`：历史实现、说明和自动保存文件，不加入运行路径。
- `docs/`：推导和设计记录。

`mixmpc_paths` 仅加入现役目录，不递归加入历史产物。输出为 4 个 C 编译单元和 2 个头文件，详见嵌入式拆分包。

## URDF 与历史在线包

```matlab
verify_mixmpc_pre('urdf')  % 完整/训练 URDF、几何和参数对照
verify_mixmpc_pre('p2')    % 闭链动力学、接触与过渡对照
```

这两个模式沿用 [建模验证](modeling/run_modeling_validation.m) 和 [P2 验证](modeling/run_p2_validation.m) 的可选路径参数与输出位置，需要 Robotics System Toolbox 等对应工具箱。详细适用范围见 [建模交接](modeling/MODELING_HANDOFF.md)。

已有在线固件包的复现入口保留在 [deploy/build_mixmpc_online.m](deploy/build_mixmpc_online.m)，先运行 `mixmpc_paths` 即可调用。它与日常多项式完整/快速构建是不同链路；此次仅整理文件，不切换或重新发布固件。旧包参数及历史说明保存在 [整理前说明](archive/README_before_entry_cleanup_20261117.md)。在线包发布仍需显式传入目标目录。

完整生成需要 Control System Toolbox、Optimization Toolbox 和 MATLAB Coder；C 对拍需要 MSVC。模型验证与主机对拍不能替代 STM32H723 的实测截止时间验证。

## Q/R 修改与固件生效

默认完整和 fast 构建均将验证后的拆分包自动同步至 `Wheel_Leg_Code/Code/Model`。
先在 `build/polynomial_package` 组装，来源与数值检查通过后才发布；发布核对全部六个文件的 SHA256。
显式传入输出目录仍可生成独立诊断包，不改默认目标。日常修改 Q/R 后运行：

```matlab
build_mixmpc_pre_fast
```

首次或模型源码变化后运行 `build_mixmpc_pre`。随后在 EIDE 重新编译固件并烧录；MATLAB 构建不会烧录。

fast 复用模型，但 Q/R 会改变整个 11,913 点终端表；DARE、压缩和 C 生成仍需运行。
现共用一次生成阶段的已核验模型缓存，并在多个压缩秩候选之间复用同一组离网格参考解；
不会复用上轮权重的终端增益。日志分别记录标称生成与终端/代码生成耗时。

## 嵌入式拆分包

完整/fast 将以下拆分文件直接发布到输出目录，构建副本保留在 `build/polynomial_package/embedded/`：

| 文件 | 职责 |
|---|---|
| `tinympc14.c` | ADMM 求解器 |
| `tinympc14_data.c` | 标称模型与 Q/R、约束、ZOH 数据 |
| `tinympc14_schedule.c` | 调度模型和终端插值计算 |
| `tinympc14_cost.c` | 终端压缩表与调度器使用的 R 数据 |

四个 `.c` 和两个 `.h` 必须一起加入工程，并以此包替换旧四文件包，避免重复符号。
不能只替换一份 Q/R 数组，终端表也依赖权重。导出保持未变文件的内容和修改时间，
并去掉会导致无效重编译的生成时间与头文件内缓存摘要；完整性由包内 SHA256 清单保留。
同一压缩秩下只改 Q/R，调度计算代码无需随大数组一起重编译。
EIDE 的 `srcDirs` 已明确包含 `Code/Model`，自动编译全部四个 C 单元；不要在该目录保留另一套包或嵌套的 `embedded/` 副本。

终端网格现用分页矩阵运算批量求解 Riccati 方程。每个点仍检查 DARE 残差，离网格
验证仍用独立 `dlqr` 作为参考，精度/稳定性门限不变。
