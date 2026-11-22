# mixmpc_pre — 融合 LPV MPC

14 态、6 输入的融合 MPC，将平衡和高度/横滚统一求解，重力、弹簧和向心补偿在输出层合成。

默认 MATLAB 参数由 `Tool/leg_config.m` 提供 17.5 kg 整车质量和 16.2 kg 轮以上支撑质量。当前固件包使用训练 URDF 的约 13.153 kg 整车参数。在线包在 14 态、6 输入模型中保留高度、横滚对腿长和平衡预测的影响。

运行 `verify_mixmpc_vlwip` 可验证直线对称滚动的 VL-WIP 参考模型。它保留伸缩与俯仰的速度耦合，用来对照加速、刹车和快速伸腿的物理口径。14 态模型包含腿长几何耦合，VL-WIP 速度项仍在独立参考模型中。

运行 `verify_mixmpc_terminal_grid` 可验证新的调度方案。它在线计算动力学矩阵和五阶 ZOH，用定点压缩网格插值终端代价的 Cholesky 因子，解码后恢复正定矩阵。验证覆盖腿长 0.16–0.34 m、平均腿角 ±0.30 rad 内的随机点和边界。

`build_mixmpc_online('E:/TEMP/mixmpc_online')` 生成 TinyMPC14 在线调度包，需要 MATLAB Coder 和 MSVC。网格、闭环及 C 对拍通过后，`solver/` 下的 `tinympc14.c/.h` 与 `tinympc14_data.c/.h` 可发布到固件。清单 `tinympc14_online.sha256` 记录四个文件的哈希，`tinympc14_package_config.mat` 保存本次生成的参数。查看原始 Coder 输出可用 `build_mixmpc_online_model`。

在线包使用 2 ms 预测节点、4 ms 后台求解和 1 ms 控制拍。接地六路出力由 TinyMPC14 提供，分配器以 CAN 接纳保持码回算下一拍输入。求解前及序列失效时使用同一模型的终端反馈，输入箱内轴向总力请求每拍最多变化 3 N。落地确认期间若双侧接触未就绪，先按测得轴力执行落地保护。

```matlab
urdf = 'E:/轮腿资料/URDF/chuanliantui_new_1/chuanliantui_new_1/urdf/chuanliantui_train.urdf';
model_dir = 'E:/DevelopLotteany1/0_Code/Leg4_v1_ultimate/Wheel_Leg_Code/Code/Model';
addpath('config');
mixmpc_paths();
p = tinympc14_urdf_config(urdf, 0.060);
p.URDF_USE_SPRING = true;
p.R_DIAG(1:2) = 8100; % 此次固件迁移采用原部署轮矩权重
build_mixmpc_online('E:/TEMP/mixmpc_urdf', model_dir, p);
```

生成包提供训练 URDF 的质量、惯量、质心、腿长调度范围、左右轮力矩限值与腿重力补偿表。轮半径 0.060 m 取自轮胎 STL。固件继续使用现有五连杆硬件 VMC、电机换算和气弹簧标定。接地力滞回按新整车质量相对原 17.5 kg 标定缩放。

## 用 URDF 运行单 MPC

在 MATLAB 中运行以下命令。示例采用训练版 STL 的约 60 mm 轮半径，实车应传入实测滚动半径。

```matlab
cd('E:/DevelopLotteany1/0_Code/Leg4_v1_ultimate/Matlab/mixmpc_pre')
urdf = 'E:/轮腿资料/URDF/chuanliantui_new_1/chuanliantui_new_1/urdf/chuanliantui_train.urdf';
verify_mixmpc_urdf(urdf, 0.060);
```

输入是两侧轮矩、虚拟髋矩和轴向腿力。每次求解在一个预测模型和一套约束内计算六路输入，同时处理纵向运动、偏航、两腿摆角、俯仰、横滚和高度。URDF 路径从关节限位、质量、质心和惯量生成腿长参数表，并分别使用左右腿角线性化，在线计算 ZOH 和 Riccati 系数。轮矩箱取自 URDF，腿力和虚拟髋矩箱沿用现有 MPC 参数。该配置将偏航角权重由 7000 提高到 70000，以减小组合跟踪场景中的偏航滞后。需要 Robotics System Toolbox 和 Control System Toolbox。

接入已有 MATLAB 仿真时，先运行 `addpath('config'); mixmpc_paths();`，用 `p = tinympc14_urdf_config(urdf, 0.060)` 替代 `tinympc14_config()`，再使用 `tinympc14_runtime_init` 和 `tinympc14_runtime_step`。验证入口检查标称平衡、异侧腿角线性化、组合跟踪、腿长工作区、输入箱和腿力变化率。

训练版 URDF 将副连杆固定在大腿上，聚合惯量取左右平均，横滚惯量用标称整机质心惯量近似。完整 URDF 的闭合铰点候选、接触约束和离线顶限用例见 [建模交接](modeling/MODELING_HANDOFF.md)。

此验证入口用于 MATLAB 仿真。固件在线包使用上面的 `build_mixmpc_online` 入口重建。旧 `build_mixmpc_pre` 入口仍用于旧多项式模型。

## 1. 目录与使用

| 位置 | 内容 |
|---|---|
| `build_mixmpc_online.m` | TinyMPC14 在线包构建、对拍和发布入口。 |
| `build_mixmpc_pre.m` | 多项式对照构建入口，结束或报错时恢复路径和工作目录。 |
| `modeling/` | 完整 URDF 闭链校核、浮动基参考动力学、双/单支撑接触和长腿基线。 |
| `config/` | 模型/调参和项目路径定义；配置函数只加入现役源码目录。 |
| `functions/` | 模型计算、参考求解器和辅助函数。 |
| `deploy/` | 数据生成、代码后处理；solver 子目录为配套 C 求解器和 port。 |
| `verify/` | 验证脚本、MEX 或主机测试源码。 |
| `build/` | 生成表、缓存、MEX、测试向量、日志和历史产物。 |
| `archive/` | 原说明、迁移记录和自动保存/差异版本，不进入执行路径。 |
| `docs/` | 推导、报告及设计背景资料。 |

从本项目根目录在 MATLAB 中执行：

```matlab
build_mixmpc_pre('E:\TEMP\mixmpc_pre_output') % 生成到临时目录并验证
build_mixmpc_pre                              % 生成到 build/polynomial_package 并验证
```

`build_mixmpc_pre(output_dir, false)` 校验并复制已有数据表，不重新运行完整仿真；现有表与源码的哈希必须一致。完整 MATLAB 验证也可通过 `addpath('config'); mixmpc_paths(); tinympc14_build_and_verify` 单独运行，不复制到固件目录。

生成器按现有逻辑写入输出目录，验证失败也可能已经产生新表；建议先使用临时目录检查。这里只生成源码/数据，不负责固件工程重编或烧录。

## Q/R 快速构建

首次或改动物理/模型设置时先运行 `build_mixmpc_pre`。仅改 Q/R 时编辑 [config/tinympc14_config.m](config/tinympc14_config.m) 里 `Q/R-BLOCK` 标记块内的 `Q_DIAG`/`R_DIAG`，再点击运行 [build_mixmpc_pre_fast.m](build_mixmpc_pre_fast.m)：

```matlab
build_mixmpc_pre_fast                          % 当前默认输出目录，包含必要 C 对拍
build_mixmpc_pre_fast('E:\TEMP\mixmpc_pre_fast') % 临时输出目录
build_mixmpc_pre_fast('E:\TEMP\mixmpc_pre_fast', false) % 跳过 C 对拍，保留缓存及数值检查
```

复用标称/LPV 网格的平衡点、离散模型、拟合基和 ZOH 系数；重算增益与拟合表，刷新来源哈希及本次权重的固定/随机/补偿向量并运行主机 C 对拍，不执行长闭环。

完整与 fast 共用模型、求解和导出代码。fast 总是检查权重、模型缓存、有限值以及适用的 DARE/正定/闭环条件；新代码在 `build/` 内生成并完成所选检查后，才写入输出目录。`false` 不会关闭缓存或基础数值检查。输出位置以当前 config 为准，fast 不修改配置文件，也不负责固件重编或烧录。

`build/qr_model_cache.mat` 保存 Q/R 无关模型，旁边的 `.sha256` 校验其完整性。签名包含非 Q/R 配置、源码、共享 Tool、物理参数、Ts、rho、约束和网格；`Q/R-BLOCK` 标记块与输出目录不参与失效判断。缓存缺失、损坏或不符时，会指出原因并提示先完整构建，不会悄悄重算模型。

完整构建在模型阶段通过基础检查后建立缓存，后续完整验收仍会正常报告失败。fast PASS 仅表示必要检查通过，不等同完整验收，也不修复已知腿长 V2、平衡 V6 问题。首次模型缓存需要由新增缓存功能后的完整入口生成，整理前的旧缓存不能直接替代。

`build/fast/output/` 是非混合项目的待检查 C/H；混合项目沿用 `build/staging/`。`build/fast/report.mat` 记录耗时、缓存命中、C 对拍是否执行和 `full_validation=false`。C 代码生成与编译仍需时间，小模型的速度提升可能主要来自省略长仿真。

### 快速通道及共用实现

| 文件 | 作用 |
|---|---|
| [build_mixmpc_pre_fast.m](build_mixmpc_pre_fast.m) | Q/R-only 构建、必要检查和部署入口；不会退回完整重建。 |
| [config/tinympc14_config.m](config/tinympc14_config.m) 的 `Q/R-BLOCK` 块 | Q_DIAG/R_DIAG 唯一编辑位置；块内改动不进模型缓存签名，仍走 fast 通道。 |
| [deploy/tinympc14_cache_generate.m](deploy/tinympc14_cache_generate.m) | 完整/快速模式共用的标称点模型缓存、DARE 和单点表导出。 |
| [deploy/tinympc14_lpv_generate.m](deploy/tinympc14_lpv_generate.m) | 完整/快速模式共用的 LPV 网格、增益和拟合；fast 不重解非线性平衡及离散化。 |
| [deploy/tinympc14_zoh_generate.m](deploy/tinympc14_zoh_generate.m) | 完整模式计算并缓存 ZOH 系数；fast 复用系数，仍通过同一导出代码生成新表。 |
| [functions/tinympc14_solve_reference.m](functions/tinympc14_solve_reference.m) | 从原 S8 提取的参考求解/输出合成步骤，共用给完整运行检查和 C 对拍向量生成。 |
| [verify/tinympc14_generate_vectors.m](verify/tinympc14_generate_vectors.m) | 独立生成本次权重对应的固定、1000 例随机及补偿 C 对拍向量，无需先跑长闭环。 |

## 2. 各文件作用

### 配置与定义

| 文件 | 作用 |
|---|---|
| [config/tinympc14_config.m](config/tinympc14_config.m) | 唯一配置入口：实测硬件、Q/R 权重（`Q/R-BLOCK` 标记内可在 fast 通道改动）、模型、约束、采样周期、调度参数和默认部署目标；MPC 外层补偿约定也在这里统一。 |
| [config/mixmpc_paths.m](config/mixmpc_paths.m) | 定位 functions/config/deploy/verify、共享 Tool 和 build 下的缓存、staging、主机产物目录。 |

### 模型与辅助函数

| 文件 | 作用 |
|---|---|
| [functions/tinympc14_admm.m](functions/tinympc14_admm.m) | 混合 MPC 的 MATLAB ADMM 参考求解器。 |
| [functions/tinympc14_audit_regression.m](functions/tinympc14_audit_regression.m) | 检查独立物理和求解器不变量，防止历史实现中的问题被回归测试共同沿用。 |
| [functions/tinympc14_bal_block.m](functions/tinympc14_bal_block.m) | 按左右腿各自参数装配 10 态平衡块的线性模型和仿射项。 |
| [functions/tinympc14_check_refs.m](functions/tinympc14_check_refs.m) | 提供独立自检用的测试点、历史基准和容差。 |
| [functions/tinympc14_output.m](functions/tinympc14_output.m) | 输出层三笔补偿：查 Tool 的 Leg_Comp 得 MgF、取 Leg_SpringF 得 Fs、按 cfg 常量算 Fc，再按笔沿用失效值并给出状态码。 |
| [functions/tinympc14_ctrb_rank.m](functions/tinympc14_ctrb_rank.m) | 计算可控性矩阵的秩。 |
| [functions/tinympc14_dare_gain.m](functions/tinympc14_dare_gain.m) | 计算 rho 增强 DARE 的 K/P 及求解器所需预处理矩阵。 |
| [functions/tinympc14_data_manifest.m](functions/tinympc14_data_manifest.m) | 记录生成表与真实源码的 SHA256；当前 v3 清单覆盖共享 Tool 来源。 |
| [functions/tinympc14_manifest_sources.m](functions/tinympc14_manifest_sources.m) | 集中列出每组数据表必须跟踪的当前配置、模型、生成、验证及 C 求解器源码。 |
| [functions/tinympc14_dtab.m](functions/tinympc14_dtab.m) | 对参数表某一列求腿长方向的数值导数。 |
| [functions/tinympc14_equilibrium.m](functions/tinympc14_equilibrium.m) | 提供混合模型的静止平衡状态和六路平衡输入。 |
| [functions/tinympc14_f_extra.m](functions/tinympc14_f_extra.m) | 计算 psi 相关前馈仿射增量。 |
| [functions/tinympc14_fused_ab.m](functions/tinympc14_fused_ab.m) | 对统一非线性动力学核进行数值线性化，得到 14 态 A/B/f。 |
| [functions/tinympc14_fused_ab_legacy.m](functions/tinympc14_fused_ab_legacy.m) | 保留的旧分块装配实现，用于历史对照。 |
| [functions/tinympc14_fused_dyn.m](functions/tinympc14_fused_dyn.m) | 14 态非线性动力学核，计算腿力和轮矩的角度投影。 |
| [functions/tinympc14_fused_eq.m](functions/tinympc14_fused_eq.m) | 通过牛顿迭代求非线性模型的平衡状态与输入。 |
| [functions/tinympc14_fused_op.m](functions/tinympc14_fused_op.m) | 打包当前工作点的平衡块与腿长块参数，供动力学和线性化复用。 |
| [functions/tinympc14_k_eff.m](functions/tinympc14_k_eff.m) | 按配置返回固定弹簧等效刚度或由弹簧力数值求导；当前部署采用固定零刚度。 |
| [functions/tinympc14_lpv_eval.m](functions/tinympc14_lpv_eval.m) | 读取当前 LPV 拟合缓存，在腿长和 psi 调度点计算模型及增益表。 |
| [functions/tinympc14_m_tot.m](functions/tinympc14_m_tot.m) | 计算整机站立质量。 |
| [functions/tinympc14_mv_pick.m](functions/tinympc14_mv_pick.m) | 按所选口径确定高度通道的竖直等效惯量。 |
| [functions/tinympc14_plant_init.m](functions/tinympc14_plant_init.m) | 初始化闭环验证的真实模型和平衡点。 |
| [functions/tinympc14_project_input.m](functions/tinympc14_project_input.m) | 对力箱与跨节点速率约束做欧氏投影。 |
| [functions/tinympc14_qp_check.m](functions/tinympc14_qp_check.m) | 将参考问题凝聚为独立 QP，检查求解器、参考项和速率约束。 |
| [functions/tinympc14_riccati_cache.m](functions/tinympc14_riccati_cache.m) | 计算有限时域的增广 Riccati 因子。 |
| [functions/tinympc14_runtime_init.m](functions/tinympc14_runtime_init.m) | 初始化逐拍运行状态、调度点、总力种子和异步计数。 |
| [functions/tinympc14_runtime_step.m](functions/tinympc14_runtime_step.m) | 执行逐拍 LPV 更新、前馈、异步结果消费、总力限制及输出合成。 |
| [functions/tinympc14_scenarios.m](functions/tinympc14_scenarios.m) | 定义闭环验证共用的场景。 |
| [functions/tinympc14_sched_pt.m](functions/tinympc14_sched_pt.m) | 将高度和横滚等状态转换为左右腿调度工作点。 |
| [functions/tinympc14_sha256.m](functions/tinympc14_sha256.m) | 计算文件 SHA256，供来源校验和部署逐文件核对。 |
| [functions/tinympc14_section.m](functions/tinympc14_section.m) | 在合并数据文件里替换或插入带标记的段（单点模型 / LPV / ZOH 三段共用一个文件）。 |
| [共享 Leg_SpringF](../Tool/model/Leg_SpringF.m) | 气弹簧沿腿力模型，离线分析用双精度输入。 |
| [functions/tinympc14_table_col.m](functions/tinympc14_table_col.m) | 查询参数表指定列的插值值。 |
| [functions/tinympc14_table_row.m](functions/tinympc14_table_row.m) | 对整行参数做线性插值，越界取端点。 |
| [functions/tinympc14_zoh_ab.m](functions/tinympc14_zoh_ab.m) | 通过增广矩阵指数将连续 A/B/f 做 ZOH 离散化。 |
| [functions/tinympc14_zoh_apply.m](functions/tinympc14_zoh_apply.m) | 将额外仿射力经 ZOH 系数映射到离散状态增量。 |

### 生成与求解器

| 文件 | 作用 |
|---|---|
| [deploy/tinympc14_deploy.m](deploy/tinympc14_deploy.m) | 可选完整重建验证，再把求解器与数据各一对 .c/.h（共 4 个文件）复制到目标目录并核对哈希。 |
| [deploy/s03_tinympc14_cache.m](deploy/s03_tinympc14_cache.m) | 构建当前标称工作点的离散模型、增益，并写出数据文件的基表。 |
| [deploy/s04b_tinympc14_lpv.m](deploy/s04b_tinympc14_lpv.m) | 保留的二维腿长 LPV 拟合入口，供历史方案对照；不是当前部署主链。 |
| [deploy/s04d_tinympc14_lpv_psi.m](deploy/s04d_tinympc14_lpv_psi.m) | 当前主链的腿长/psi LPV 拟合、检查和表生成入口，段写入数据文件。 |
| [deploy/s07_tinympc14_zoh_data.m](deploy/s07_tinympc14_zoh_data.m) | 生成额外仿射项的 ZOH 系数段与参考缓存，并刷新数据文件的来源清单。 |
| [deploy/solver/tinympc14.c](deploy/solver/tinympc14.c) | 单文件求解器：核心、ADMM、异步与输出层装配（三笔补偿由 Tool 原语组装）。 |
| [deploy/solver/tinympc14.h](deploy/solver/tinympc14.h) | 单头文件：公共 API、数据类型、输出装配接口与 port 声明。 |

### 验证及接口

| 文件 | 作用 |
|---|---|
| [verify/tinympc14_build_and_verify.m](verify/tinympc14_build_and_verify.m) | 当前完整生成与 MATLAB 验证流水线，重建数据表、来源清单和 C 对拍向量。 |
| [verify/s01_tinympc14_sym7_model.m](verify/s01_tinympc14_sym7_model.m) | 7 自由度符号推导、模型和气弹簧交叉自检，属于扩展分析入口。 |
| [verify/s02_tinympc14_one_mpc.m](verify/s02_tinympc14_one_mpc.m) | 单一 14 态 MPC 模型预览和独立自检；预览表写在 build/staging/preview。 |
| [verify/s03_tinympc14_wholebody.m](verify/s03_tinympc14_wholebody.m) | 保留的七坐标全身模型推导与分块模型对照。 |
| [verify/s04_tinympc14_mvp_scan.m](verify/s04_tinympc14_mvp_scan.m) | 单点模型的稳定性、稳态误差和时域扫描。 |
| [verify/s04c_tinympc14_psi_ff.m](verify/s04c_tinympc14_psi_ff.m) | 保留的 psi 仿射前馈方案检查。 |
| [verify/s05a_tinympc14_admm_replica.m](verify/s05a_tinympc14_admm_replica.m) | 通过独立凝聚 QP 检查参考求解器的代价与约束。 |
| [verify/s05b_tinympc14_admm_fused.m](verify/s05b_tinympc14_admm_fused.m) | 检查当前 14 态、6 输入的受约束 QP。 |
| [verify/s06_tinympc14_closed_loop.m](verify/s06_tinympc14_closed_loop.m) | 历史单点调度和前馈闭环方案的仿真对照。 |
| [verify/s07_tinympc14_static_check.m](verify/s07_tinympc14_static_check.m) | 核对生成表的源码哈希、覆盖范围、物理参数、接口版本和缓存数值。 |
| [verify/s08_tinympc14_runtime_check.m](verify/s08_tinympc14_runtime_check.m) | 验证 2 ms 控制/4 ms 异步求解运行过程，并生成主机 C 验证向量。 |
| [verify/s09_tinympc14_fused_verify.m](verify/s09_tinympc14_fused_verify.m) | 检查局部动力学、实测硬件一致性、离网 LPV 和独立 QP。 |
| [verify/s10_tinympc14_nonlinear_check.m](verify/s10_tinympc14_nonlinear_check.m) | 用 RK4 非线性投影模型验证三组闭环工况。 |
| [verify/host/tinympc14_solver_host_test.c](verify/host/tinympc14_solver_host_test.c) | 固定/随机向量、补偿、异步和故障注入的主机 C 测试。 |
| [verify/host/tinympc14_port_host.c](verify/host/tinympc14_port_host.c) | 主机测试使用的 port 接口实现。 |
| [verify/host/tinympc14_port_host.h](verify/host/tinympc14_port_host.h) | 主机 port 接口声明。 |
| [verify/host/audit_arm_main.c](verify/host/audit_arm_main.c) | Cortex-M7 链接与大小分析入口，不负责烧录。 |
| [verify/host/run_host_msvc.cmd](verify/host/run_host_msvc.cmd) | 使用本机 VS2022 编译并运行主机测试；支持 asan 参数，产物在 build/host。 |
| [verify/host/Makefile](verify/host/Makefile) | 其他 C99 编译器使用的 run/asan/clean 目标，产物同样集中到 build/host。 |

## 3. 共享 Tool 与本项目边界

原内嵌 `tool/` 已撤出当前源码布局：VMC/补偿函数和腿长表统一用 `../Tool/functions/`、`../Tool/config/`；本项目独有的实测硬件、Q/R 权重与输出层三笔补偿常量都在 `config/tinympc14_config.m`，补偿的物理量由 `tinympc14_output.m` / 求解器内的 output 段组装（查 `Leg_Comp`、取 `Leg_SpringF`、用生成表头的常量），Tool 只提供原语。

原 `Leg_RollComp`、`Leg_CoriolisComp` 使用实测全轮距和不同力偶换算，不能当作 Tool 的同义副本，源码保存在 `archive/tool_variants/*.m.orig`。对应旧生成脚本也归档；旧 Coder/CMSIS 产物位于 `build/tool_history/`。输出层三笔补偿由本项目的 `tinympc14_output.m` 与求解器 output 段组装（查 `Leg_Comp`、取 `Leg_SpringF`、用生成表头常量），不能再叠加一遍旧向心补偿。

项目应与 `Matlab/Tool` 保持同级。不要用 `addpath(genpath(...))` 将历史快照、旧 MEX 或差异模型加入路径；入口和路径配置会添加所需的现役目录。

## 4. 生成物、缓存与附件

| 位置 | 用途 |
|---|---|
| `build/staging/` | 当前数据对 `tinympc14_data.c/.h`（单点模型 + LPV 系数 + ZOH 系数三段）；preview 为可选模型预览。 |
| `build/tinympc14_lpv_psi_cache.mat、tinympc14_zoh_cache.mat` | 当前 LPV 系数和 ZOH 系数参考缓存。 |
| `build/tinympc14_data.sha256` | 数据对及实际源码的 v3 来源哈希清单，包含共享 Tool 文件。 |
| `build/host/` | 生成的三组 C 对拍向量、主机可执行文件、对象和调试信息。 |
| `build/audit_full_validation.log` | 当前完整 MATLAB 验证流水线日志。 |
| `build/history_before_reorganization/、tool_history/` | 整理前当前产物与原工具生成缓存。 |
| `build 内其他 audit、migration、wip 目录` | 旧审计、迁移和实验快照，保留溯源，不能作为当前部署验收结果。 |

`archive/README_previous.md` 保存整理前说明；其中旧路径、独立快照等描述以当前 README 为准。

## 5. 修改后的操作

修改模型、硬件、Q/R 或采样周期后，必须完整重建数据表及来源清单，再生成对拍向量并运行 C 主机验收。完整主链需要 Control System Toolbox 和 Optimization Toolbox，扩展符号推导另需 Symbolic Math Toolbox。

从项目根目录运行主机验收：

```bat
verify\host\run_host_msvc.cmd
verify\host\run_host_msvc.cmd asan
```

其他 C99 编译器可在 verify/host 运行 `make run`。部署复制求解器与数据各一对 .c/.h，共 4 个文件并附哈希清单；port 适配和实机截止时间仍需在固件侧验证。详细接入见 [求解器说明](deploy/solver/README.md)。

## 6. 本次整理验证（2026-10-25）

完整 MATLAB 生成/模型/QP/闭环验证通过；MSVC 主机 C 对拍通过，最大绝对差 1.526e-5，失败项为 0（含 13 段补偿向量与外部装配用例）。三笔补偿的物理量改由求解器输出层自己组装（查 `Leg_Comp`、取 `Leg_SpringF`、用生成表头常量），Tool 只保留原语；求解器与数据各合并成一对文件，部署包从 14 个收成 4 个。未进行实机烧录、ARM 工具链编译或截止时间验证。
