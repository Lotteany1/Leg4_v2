# tinympc14 C 求解器与外层补偿

一个求解器 + 一个数据：`tinympc14.c/.h` 是求解器（核心 + ADMM + 异步 + 输出层装配 + 端口），`tinympc14_data.c/.h` 包含单点模型、生成的 LPV 调度器、压缩终端表和 ZOH 系数。所有导出矩阵按行存储；部署表版本为 data=3、LPV=4、ZOH=1。

LPV=4 用 MATLAB Coder 生成与 MATLAB 共用的解析动力学和 ZOH 运算，不再对动力学逐元素做全局多项式拟合。终端表在相对局部平衡腿角的 asinh 坐标上加密，用三次插值重建 log-Cholesky 因子；`Pinf-rho*I=L*L'`。回退增益单独插值，避免从近似终端矩阵反求增益时放大误差。表压缩阶数必须通过原有 3% 精度、正定性和稳定性门槛。

完整构建和 fast 构建均需 Control System Toolbox 和 MATLAB Coder。fast 从带源码指纹的缓存读取网格动力学，只重算当前 Q/R 对应的 DARE、压缩表和 C 代码。首次升级或修改非 Q/R 源码后必须先完整构建。两个入口均运行全部七个矩阵的 C/MATLAB 对拍及原有求解器对拍，完整构建另外运行原有闭环验证。

三笔补偿的物理量不在本包内实现：MgF 查固件数学库的 `Leg_Comp`，Fs 取 `Leg_SpringF`，Fc 用本包表头的 `TINYMPC14_M_SUP` / `H_OFF` / `ROLL_TRACK` / `ROLL_CLAMP` 常量，逐笔失效与沿用由 `Tinympc14_Output_*` 负责。

## 文件与编译

编入：`tinympc14.c`、`tinympc14_data.c`，固件 port 实现，以及 Tool 部署在 `Code/Math` 的 `Leg_Comp.c` 与 `Leg_SpringF.c`。包含目录为本目录、build/staging 与 Code/Math。

`verify/tinympc14_build_and_verify.m` 重建表并校验源文件哈希。不要将旧表与新求解器混用。MPC 侧硬件与模型参数都在 `tinympc14_config.m`；输出层装配读的是生成头文件里的对应宏，s07 会逐个核对宏与 cfg 是否一致。

部署运行 `deploy/tinympc14_deploy.m`。默认位置只改 `tinympc14_config.m` 的 `cfg.OUT_DIR = '绝对路径';`，当前为 Leg4 固件 `Wheel_Leg_Code\Code\Model`。部署入口先生成/验证，再复制本目录 2 个 .c/.h 和 build/staging 的 2 个数据表文件，逐一核对哈希并写入 `tinympc14_deployment.sha256`。临时目标可传 `tinympc14_deploy('E:\TEMP\mixmpc')`；固件 port 实现仍需按下文接入。

## 实车 1 ms 控制拍与 2 ms 预测节点

实车每 1 ms 在 `Balance_Ground_Update` 中冻结本拍状态、参考、接纳命令和各侧力矩反馈。后台任务按 4 ms 周期尝试求解，每个预测节点覆盖 2 ms。降高分配在前台运行。完整调用顺序及 CAN 反馈配对见[接地控制入口](../../../../Wheel_Leg_Code/Code/Control/balance.c)。

独立的 2 ms 调用方也可在上电时用 `Tinympc14_Init()`、`Tinympc14_Seed(prev_applied)` 初始化，后台调用 `Tinympc14_Async_Solve()`。以下为通用 2 ms 接口的最小示例。

```c
/* fed and ffwd are zero in this deployment; physical gravity is in model.fd. */
Tinympc14_Async_Input(x, xref, zero14, zero6, len_l, len_r);
Tinympc14_Async_Step();
for (int i=0; i<6; ++i) u_mpc[i] = tinympc14_u[i];

Tinympc14_Output_Calc_Ex(len_l, len_r, world_ang_l, world_ang_r, v, yaw_rate, &st, &comp);
Tinympc14_Output_Apply(u_mpc, &comp, prev_applied, &out);
/* out.T_w: wheel actuator sign already applied.
 * out.Tp: TB + MgF.
 * out.F_motor: clipped(Fmpc + Fc) - Fs.
 * Feed out.F_motor / out.Tp to Leg_Trq, preserving the right-leg mirror sign. */

Tinympc14_Async_Applied(out.u_applied);
memcpy(prev_applied, out.u_applied, sizeof(prev_applied));
```

`prev_applied` 在第一次调用前必须初始化，可用 `Tinympc14_Output_Apply` 对上电种子进行同口径装配。`tinympc14_u` 是 MPC 原始输出；`out.u_applied` 是补偿和最终总力限制后的模型输入，后两路含气弹簧的总作用力。MgF 不再作为附加力矩喂给已经按重力补偿口径建模的仿真 plant。

进模式时先 `Tinympc14_Output_Reset(&st)`。逐笔失效只作废依赖它的那一笔：`st_*==0` 表示输出 0 只是无可用值，调用方须按无效处理；`st_*==2` 表示沿用上次有效采样，上限 5 拍（10 ms）。

实车在关节限幅和 CAN 接纳后反算总轴力及髋矩，分别使用 `Ftotal=Fmotor+Fs`、`TB=Tp-MgF`，将这一对命令作为 `Async_Applied` 的速率锚点。另由电机力矩反馈估计本拍已建立的总轴力，和同代接纳命令配对，作为 `Async_Input_Ground_At` 的测力误差输入。两路数据的时标、单位与有效位见[遥测通道说明](../../../../Wheel_Leg_Code/Code/Config/README.md)。

C 的 `comp.ffwd` 兼容字段恒为零；不要将三笔补偿再传入 `Tinympc14_Solve` 的 ffwd。该求解接口仍支持通用输入偏置，留给独立测试/其他应用；当前部署全部传零。

## 接口约定

状态：`[s ds yaw dyaw psiL dpsiL psiR dpsiR pitch dpitch roll droll H-H_REF dH]`。psi 是世界腿角，已含机身 pitch。调度腿长应为各腿实测值。

MPC 输入：`[Twl Twr TBl TBr FmpcL FmpcR]`，前四路 N·m，后两路 N。`NH=8` 是 8 个状态节点、7 个控制节点，实际预测时域 14 ms。力速率为每 2 ms 6 N。

直接求解：`Model_Update(lL,lR,psi)` 后 `Solve(x0,xref,fed,ffwd,u_prev,u)`。它包含内部工作区，是单求解任务接口，不可并发调用多个 Solve；控制拍通过异步接口消费结果。

异步模型更新/求解任务优先级低于控制任务。port 的临界区必须支持嵌套；发布模型、仿射量和回退增益保持同一快照。`Tinympc14_Port_Tick()` 必须是以毫秒计数、按 uint32_t 自然回绕的单调时钟，例如固件的 `HAL_GetTick()`。只有内核节拍为 1 kHz 时才可直接用 `osKernelGetTickCount()`。

`Async_Invalidate()` 是显式停用；`Async_Run()` 恢复允许求解。普通未收敛/非法输入只作废序列，下一份有效输入会自动重试。取消代次阻止在途旧求解重新提交。序列绑定采样时刻和求解时的输入偏置；超过 7 个控制节点或观测超龄后不再使用旧序列。

## 数值方法与资源

物理阶段代价为 Q/R，rho 只作为 ADMM 罚参数。物理终端权为 `Pinf-rho*I`。插值后的终端表不严格满足 DARE，模型更新时额外缓存 7 步有限时域 Riccati 递推。箱与速率约束用真正的欧氏投影；内部双精度工作区降低参考/仿射项相消误差。

资源需要按生成版本重新测量：LPV=3 的旧报告约 52 KB 静态 RAM、5 KB 最大已知调用栈和 175 KB Flash，不适用于 LPV=4。构建输出当前压缩表的字节数；此外还需计入生成的动力学代码和工作区。生成器禁用动态内存分配，调度器输出缓冲为静态存储，只能由一个模型更新任务使用。烧录前须核对目标链接容量、任务栈和最坏执行时间；主机数值对拍不能代替这些目标测量。

主机测试含 8 个固定例、1000 个连续热启动例、外层补偿装配、NaN、异步恢复、取消和过期输入。比较函数显式拒绝非有限输出，收敛标记不一致也失败；不能用“接近残差门限”豁免超差。

独立 QP 和 C 对拍验证的是数值与接口正确性。真实目标上的最坏执行时间、关节耦合力矩限制和力学标定仍需台架验证。
