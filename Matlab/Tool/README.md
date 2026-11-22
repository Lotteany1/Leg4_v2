# Tool — VMC 腿模型与 C 代码生成

按 `001/001/urdf/001.urdf` 的实际双闭环传动计算正运动学、解析雅可比与虚功力矩映射，并生成重力、实车轴向前馈、转弯与旋转补偿。
生成单精度 C 供固件直接编译，部署到 `Wheel_Leg_Code/Code/Math`（16 个 `Leg_*.c/.h`）。

骨架、fast 通道、部署目标、环境要求见 [上级 README](../README.md)。

## 入口

| 文件 | 说明 |
|---|---|
| `leg_config.m` | 统一参数接口；CAD 数据来自 `leg_urdf_data.m`，经验增益单独注明 |
| `leg_urdf_data.m`、`leg_mechanism_data.m` | `calibrate_001_urdf` 从 URDF、STL 显式重算的惯性参数表与闭环几何，不手改 |
| `build_leg_tool.m` | 完整构建：Coder 生成 → CMSIS 后处理 → 部署到 `Code/Math` |
| `verify_leg_tool.m` | 验证：四组检查，见下 |

本模块**没有 fast 入口**：VMC 代码生成与 Q/R 无关，不存在「只改权重」的场景。

```matlab
% 首次或 CAD 变化后：
cd('../mixmpc_pre'); mixmpc_paths;
calibrate_001_urdf
cd('../Tool');
build_leg_tool                                  % 生成到默认固件目录并验证
build_leg_tool('E:\TEMP\tool_output')           % 临时输出目录
build_leg_tool('E:\TEMP\tool_output', false)    % 只生成
verify_leg_tool                                 % 单独复跑验证
```

质量、质心和惯量应修改 CAD 来源后运行 `calibrate_001_urdf`，它同时生成固件 `Code/Config/physical_config.h`。轮距按 STL 轮胎中面计算，膝限位给出约 321.102 mm 上界。机身惯量注明绕质心或髋轴；横滚有效惯量由双轮无侧滑切向动能计算，保留与升降的交叉项。

Leg_SpringF 使用手感反馈试调的 18 点轴向补偿表，左右共用。170～340 mm 每 10 mm 一个节点，短端 75 N、长端 185 N，区间内线性插值。表在 model/Leg_SpringF.m 中维护，修改后运行本构建入口同步 C 并验证。原始实测表保存在 Wheel_Leg_Code/build/spring_id_fit，当前运行值属于手感试调参数。

完成 Tool 后还须从 `mixmpc_pre` 运行 `build_mixmpc_pre`，同步 MPC 六文件，再重编固件；物理参数变化不能用 fast 代替完整构建。

## 为什么 model/ 里的函数必须各自成文件

`codegen` 按**函数名**逐一生成 C：`codegen('-config', cfg, 'Leg_Pos', '-args', ...)`。
把 8 个入口合并或降为 local function 都会让生成链找不到入口。同理，
`leg_config.m` 带 `%#codegen` 且被 4 个模型函数调用，里面**不能**出现
`fileparts(mfilename(...))` 这类 codegen 不支持的构造 —— 所以路径单独放在
`shared/leg_paths.m`。

## 验证覆盖

`verify_leg_tool.m` 依次跑四组，任一组不合格都抛错：

首先核对整车、腿轮、机身与支撑质量的守恒关系，以及固件 `code_config.h` 与模型配置的一致性。

1. **实际 CAD 闭环**：425 个有效构型核对完整 URDF 正解、解析速度、虚功和正逆力矩。参考对象包含后电机实际传动，使用固定零点的理想五连杆不能替代。
2. **原始与固件 C**：四个 VMC 函数与重力、前馈、转弯补偿进行原始生成 C、后处理 C、MATLAB 三方对拍。
3. **实车查表**：验证 18 点试调补偿查表、端点保持与 MATLAB/C 数值一致。
4. **独立物理检查**：`verify_001_physics` 核对 STL 孔位、惯量合法性、动能、重力虚功、双轮无侧滑约束和整机静态质心。结果保存在 `research/urdf_migration_20261122`。

PC 上把 `arm_*` 接回 libm 做影子对拍，因此覆盖不到两处真实差异：`arm_sqrt_f32` 对负入参
返回 0 而 `sqrtf` 返回 NaN；`arm_sin/cos_f32` 是 512 点查表插值（绝对误差 1.89e-5）。
后处理的详细说明在 `build_leg_tool.m` 的 `cmsis_postprocess` 段注释里。

## 生成链的硬约束

- `leg_chain` 和几何/查表辅助函数内联到各入口，固件保留统一 float 标量 ABI。每个入口只发布自己的 `.c/.h`，验证先摊平公共入口再编译，避免跨入口的私有变体混入。
- 后处理与 codegen 在同一个入口里原子绑定：手工漏跑会让部署目录停留在上一次的表。
- 标定常量改后必须重建 C 并重跑验证。
