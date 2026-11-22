# Leg4 融合 MPC

当前实现为 14 态、6 输入的连续 LPV MPC。对 Leg3 两条 MPC 的逐层审计、修复依据和验证结果见 [详细报告](docs/混合MPC审计与修复_2026-10-23.md)。旧 build/leg4_mpc_report_S*.md 描述历史版本，不能作为当前部署验收依据。

硬件参数的唯一来源是 `tool/leg4_hardware.m`：轮半径 **0.0525 m**、单腿质量 **1.855 kg**、半轮距 **0.212755 m**。`mpc/leg4_sym7_config.m` 管理权重、约束和模型设置。

按用户确定的 Leg3 结构，三笔补偿都在 MPC 外调用函数：

```text
观测 → LPV MPC → u_mpc
                   ↓
        Tp = TB + MgF
        Ftotal = 限幅及速率限制(Fmpc + Fc)
        Fmotor = Ftotal - Fs
                   ↓
                 VMC → 电机
                   ↓
           实际施加量反馈至下一拍
```

模型保留重力动力学。气弹簧刚度固定为零，MgF、Fs、Fc 均不烘入模型，不送入 MPC 的 ffwd。直接将内部腿力加到水平加速度的 `F_pitch` 耦合因违反水平动量关系，已从部署配置关闭。

从本目录在 MATLAB 执行：

```matlab
addpath('mpc');
leg4_build_and_verify
```

该入口重建 staging 单点表、LPV 表和 ZOH 表，检查生成物及模型来源哈希，再运行独立 QP、离网点、运行时和非线性投影闭环验证，更新 C 对拍向量。日志：`build/audit_full_validation.log`。需要 Control System Toolbox、Optimization Toolbox；旧 s01 的符号分析另需 Symbolic Math Toolbox。

部署目标集中在 `mpc/leg4_sym7_config.m`，与 Leg3 示例相同，直接替换一行：

```matlab
cfg.OUT_DIR = 'E:\DevelopLotteany1\0_Code\Leg4_v1_ultimate\Wheel_Leg_Code\Code\Control';
```

运行 `leg4_deploy` 会完整生成/验证，再将 8 个求解器文件及 6 个数据表文件写入该目标，并逐文件校验 SHA256。`leg4_deploy('E:\TEMP\mixmpc')` 可临时替换目标；末尾加 `false` 可只校验并复制现有产物。内部 `cfg.STAGING_DIR` 随项目位置移动，用于生成和验证，不随部署目标改变。`leg4_build_and_verify` 本身只在项目内构建验证。

C 主机验收：

```bat
solver\test\run_host_msvc.cmd
solver\test\run_host_msvc.cmd asan
```

Windows 脚本使用本机 VS2022；其他 C99 编译器可在 `solver/test` 使用 `make run`。向量重生成后必须重跑 C 验证。

VMC 函数生成：

```matlab
addpath('tool');
leg_vmc_codegen
```

输出 `tool/cmsis/*.c/.h`。`Leg_RollComp` / `Leg_CoriolisComp` 共用实测全轮距 0.42551 m，力偶换算分母为全轮距。不要再单独添加旧版向心补偿，否则会重复。

部署文件、调用次序、实际施加量反馈及内存开销见 [solver/README.md](solver/README.md)。此处生成的代码尚未烧录或验证实机 4 ms 截止时间。

`s02` 为模型预览；WRITE 打开时只写 `staging/preview`，不会覆盖部署表。`s04b`、`s04c`、`s06` 和 `s03_wholebody` 保留为历史推导/对照入口；当前部署采用上述完整流水线中的 `s04d`、`s08`、`s09`、`s10`。
