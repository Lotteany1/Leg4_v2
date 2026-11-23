# Tool — VMC 腿模型与 C 代码生成

当前恢复为 v1 的理想五连杆计算与原有构建流程。质量、质心、惯量和轮胎尺寸采用 001 CAD 的当前参数；`leg_config.m` 与 `tinympc14_config.m` 保留直接赋值和参数表。

实车气弹簧前馈继续使用当前 18 点经验表。升降、横滚和偏航采用原流程的名义参数近似，不使用新增的 CAD 动态惯量表或高度参考制动器。完整 CAD 资料与恢复前源码保存在 `research/v1_format_restore_20261122/execution_before`。

五连杆腿的正/逆运动学、雅可比、关节力矩映射，以及气弹簧、转弯向心、旋转耦合三笔补偿。
生成单精度 C 供固件直接编译，部署到 `Wheel_Leg_Code/Code/Math`（16 个 `Leg_*.c/.h`）。

骨架、fast 通道、部署目标、环境要求见 [上级 README](../README.md)。

## 入口

| 文件 | 说明 |
|---|---|
| `leg_config.m` | 参数与物理量：18×4 腿标定表（腿长 0.15:0.01:0.32 m） + 全部标定常量，分区定义（带 `%#codegen`） |
| `build_leg_tool.m` | 完整构建：Coder 生成 → CMSIS 后处理 → 部署到 `Code/Math` |
| `verify_leg_tool.m` | 验证：四组检查，见下 |

本模块**没有 fast 入口**：VMC 代码生成与 Q/R 无关，不存在「只改权重」的场景。

```matlab
build_leg_tool                                  % 生成到默认固件目录并验证
build_leg_tool('E:\TEMP\tool_output')           % 临时输出目录
build_leg_tool('E:\TEMP\tool_output', false)    % 只生成
verify_leg_tool                                 % 单独复跑验证
```

模型整车总质量修改 `leg_config.m` 的 `m_total`；单轮、单腿质量分别为 `m_wheel`、`m_leg`。
支撑质量 `M_SUP = m_total - 2*m_wheel`，机身质量 `m_body = M_SUP - 2*m_leg`，均自动计算。
固件质量宏统一在 `Code/Config/code_config.h` 维护，与上述模型参数同步修改；`verify_leg_tool` 核对两者一致性。
`build_leg_tool` 生成补偿函数。
质量变化后还须运行完整的 `build_leglen_mpc` 重建腿长模型与 LESO 表，再重编固件；fast 不适用于质量变更。
前进平衡 MPC 与 LQR 的数据表使用各自的独立生成入口。

## 为什么 model/ 里的函数必须各自成文件

`codegen` 按**函数名**逐一生成 C：`codegen('-config', cfg, 'Leg_Pos', '-args', ...)`。
把 8 个入口合并或降为 local function 都会让生成链找不到入口。同理，
`leg_config.m` 带 `%#codegen` 且被 4 个模型函数调用，里面**不能**出现
`fileparts(mfilename(...))` 这类 codegen 不支持的构造 —— 所以路径单独放在
`shared/leg_paths.m`。

## 验证覆盖

`verify_leg_tool.m` 依次跑四组，任一组不合格都抛错：

首先核对整车、腿轮、机身与支撑质量的守恒关系，以及固件 `code_config.h` 与模型配置的一致性。

1. **模型级独立断言**：绕开 C，用独立几何/数值方法校验 `Leg_*` 的语义 ——
   `Leg_Pos` 与圆-圆求交闭式解对比、`Leg_Spd` 与 `Leg_Pos` 中心差分雅可比对比、
   `Leg_Trq` 与雅可比转置对比、`Leg_InvTrq∘Leg_Trq` 往返与 `λ/σ_min²` 对比、
   奇异线行为、固件 ABI（标量调用、形参不含数组）。
2. **Leg_Comp 对拍**：后处理 C vs codegen 原始 C vs .m 模型三方对比，
   外加表结构与 `leg_config()` 的一致性、钳位不变量。
3. **气弹簧对拍**：`Leg_SpringF` 覆盖钳位端点，校验整体摆角不改变沿腿力，并用销距势能差分复核虚功。运行 `python Wheel_Leg_Code/tests/run_distributor.py --spring-mount Wheel_Leg_Code/Code/Math/Leg_SpringF.c` 核对水平姿态下的控制与反馈。
4. **VMC 四函数对拍**：`Leg_Pos/Spd/Trq/InvTrq` 两份 C 逐位相同（证明后处理的文本
   变换没改数值），再与 .m 模型比，误差落在单精度量级；最后核对部署目录与固件目录同步。

PC 上把 `arm_*` 接回 libm 做影子对拍，因此覆盖不到两处真实差异：`arm_sqrt_f32` 对负入参
返回 0 而 `sqrtf` 返回 NaN；`arm_sin/cos_f32` 是 512 点查表插值（绝对误差 1.89e-5）。
后处理的详细说明在 `build_leg_tool.m` 的 `cmsis_postprocess` 段注释里。

## 生成链的硬约束

- `build/codegen_current/lib/Leg_InvTrq/` 会连带生成一份 `Leg_Trq.c/.h`，其 `F/Tp` 被
  推成 `double`（`Leg_InvTrq.m` 里传的是 `1.0` 这类 double 字面量），与 `Leg_Trq/` 目录下
  的同名函数签名不同。混着编会被「引号包含就近目录」选中错误的头文件、链到另一份实现，
  ABI 对不上且**静默返回 0**。后处理只取 `lib/<函数名>/<函数名>.c`，所以部署目录是干净的一套。
- 后处理与 codegen 在同一个入口里原子绑定：手工漏跑会让部署目录停留在上一次的表。
- 标定常量改后必须重建 C 并重跑验证。
