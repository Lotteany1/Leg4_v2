# Tool — 五连杆腿模型（MATLAB → 固件 C 生成链）

同轴双曲柄五连杆的腿模型：`l1 = 0.21 m` 大腿/曲柄、`l2 = 0.25 m` 小腿、
两髋同轴（`l5 = 0`）。本目录既是模型源，也是固件 `Wheel_Leg/Code/Math/Leg_*.c/.h`
的唯一生成源。

## 文件职责

| 文件 | 职责 | 来源 |
|---|---|---|
| `leg_vmc_generate.mlx` | 符号推导：正运动学 → 速度映射 → 虚功力矩映射 | 手改（Live Script） |
| `Leg_Pos.m` / `Leg_Spd.m` / `Leg_Trq.m` | 上一步 `matlabFunction` 的产物 | **生成，禁止手改** |
| `Leg_InvTrq.m` | VMC 逆映射（关节力矩 → 末端 F/Tp），对 `Leg_Trq` 抽样建 J + Tikhonov 求逆 | 手写 |
| `Leg_Comp.m` | 腿长 → 质心偏置角/重力力臂，对 `leg_data` 做分段线性插值（表外钳位） | 手写 |
| `Leg_SpringF.m` | 气弹簧等效力 Fs(L0)（三角几何 + 虚功，恒预压力模型） | 手写 |
| `Leg_RollComp.m` | 转弯向心力前馈：(L0, ω, v) → 左右腿推力补偿 ±Fc（对称等幅反号，±F_MAX 限幅） | 手写 |
| `Leg_CoriolisComp.m` | 旋转耦合补偿：ω² 离心负刚度（pitch 按 G_HIP 分轮/髋两路、roll 出腿力差）+ 可关断的 2ω×v 科氏交叉项 | 手写 |
| `leg_data.m` | 19 行腿长参数表 `[l_leg, theta0, l_d, I_leg]`（**唯一数据源**） | 手写 |
| `leg_vmc_codegen.m` | `codegen` 八个函数 → `codegen/lib/`，末尾自动调用后处理 | 手写 |
| `leg_cmsis_postprocess.m` | `codegen/lib/` → `cmsis/`（cosf/sinf/sqrtf → `arm_*`，去 Coder 运行时） | 手写 |
| `cmsis/*.{c,h}` | **拷给固件的成品**（`Wheel_Leg/Code/Math/`） | 生成 |
| `codegen/lib/` | Coder 原始产物（含 `_initialize/_terminate` 等，不带 CMSIS 替换） | 生成 |
| `verify/` | PC 上的对拍与独立断言（见下） | 手写 |

## 执行顺序

1. **改模型**：编辑 `leg_vmc_generate.mlx`，在 Live Editor 里重新运行，
   覆盖 `Leg_Pos/Spd/Trq.m`；或直接改 `Leg_InvTrq/Leg_Comp/Leg_SpringF/leg_data`。
2. **生成 C**：`matlab -batch "cd('.../Matlab/tool'); leg_vmc_codegen"`
   —— 它跑完 8 个 `codegen` 后会自动调用 `leg_cmsis_postprocess()`，
   把结果落到 `codegen/lib/` 与 `cmsis/`（后处理与 codegen 原子绑定：漏跑后处理
   曾让固件表掉队一个模型版本）。
3. **同步固件**：把 `cmsis/Leg_{Pos,Spd,Trq,InvTrq,Comp,SpringF,RollComp,CoriolisComp}.{c,h}` 共 16 个
   文件原样覆盖到 `Wheel_Leg/Code/Math/`（不要手改固件侧内容），然后 EIDE 重编。
   这一步由下一步的对拍逐字节核对（固件目录不可达时跳过）。
4. **对拍**：`matlab -batch "cd('.../Matlab/Tool/verify'); leg_vmc_verify"`（`verify/` 在 Leg3_v2 的 Tool 链，本文件夹未随附）
   —— 必须全部 PASS 才算改完；它自带 mex 编译（MSVC）与 6 组汇总。

## 改 `leg_data.m` 必须重跑整条链

`leg_data.m` 是腿参数的唯一数据源，两份消费方都要跟着走：

- `Leg_Comp.m` 在 codegen 时把表常量**折叠进生成的 C**（`static const float`），
  所以改表后不重跑 codegen + 后处理 + 固件同步，固件里仍是旧表；
- `Lqr_K/s03_compute_lqr_fast.m` 直接 `leg_data()`，表值进几何缓存指纹，
  改表会自动触发几何重算并改变 LQR 的 `K` 面（`Lqr_K/README.md`）。

顺序：改表 → `leg_vmc_codegen` → 拷 `cmsis/` 到固件 → `leg_vmc_verify` →
`s03` 之后的 Lqr 链。

## 验证覆盖（`verify/`）

| 入口 | 覆盖 |
|---|---|
| `leg_vmc_verify.m` | 总入口：4 函数 C-vs-C 逐位 + C-vs-.m 阈值；末尾调用下面三个并汇总，再加一条可选的固件同步检查（`Wheel_Leg/Code/Math/` 可达时逐字节比 16 个生成文件），任一项失败 `error()` |
| `verify/mex_leg_vmc.c` | `Leg_Pos/Spd/Trq/InvTrq` 的 N×6 接口 |
| `verify/mex_leg_comp.c` | `Leg_Comp` 的 N×1 → (θ0, l_d) 接口 |
| `leg_comp_verify.m` | `Leg_Comp`：C-vs-C 逐位、C-vs-.m 阈值、**表结构 vs `leg_data()`**（行数/起点/步长/钳位常数/逐位表值）——陈旧表的回归防线 |
| `leg_model_verify.m` | 模型级独立断言：`Leg_Pos` 与圆-圆求交（独立闭式）、`Leg_Spd` 与中心差分雅可比、`Leg_Trq` 与雅可比转置、`Leg_InvTrq` 往返 ≈ λ/σ_min²、工作带与全域覆盖、标量 ABI |
| `leg_springf_verify.m` | `Leg_SpringF`：C-vs-C 逐位、C-vs-.m 阈值、钳位端点不变量 |
| `verify/shim/arm_math.h` | PC 影子 `arm_cos/sin/sqrt_f32`（接回 libm）——**真机不是这个实现**，见限制 2 |

## 已知限制

1. **奇异线 `phi1 ≡ phi4 (mod 2π)`**：该位形下两曲柄平行、两连杆共线，机构退化。
   `Leg_Spd` 输出 NaN、`Leg_Trq` 输出 ±Inf，`Leg_Pos` 返回一个有限但无几何意义的
   退化值（且与邻域不连续）。该线有一段（`L0 = |l1·u1 + l2|` 落在 0.16~0.34 m）
   就在工作带内，固件应避免停在 `phi1 = phi4`。
2. **CMSIS 三角函数不是 libm**：后处理把 `cosf/sinf` 换成 512 点表插值的
   `arm_sin_f32/arm_cos_f32`（绝对误差 ~1.9e-5），`arm_sqrt_f32` 对负入参返回 0。
   对 `Leg_SpringF` 的 Fs 影响 ≤1.4e-3 N（工作带内 8.7e-4 N）。对 `Leg_Pos/Spd/Trq`：
   标称对称位形族（φ1=δ/2、φ4=−δ/2，L0 扫 0.16~0.34 m）实测 ≤5.2e-6 m / 2.5e-5 rad /
   5.2e-5 N·m，但这是位形相关量 —— 工作带内**任意**构型实测可达 8.9e-5 m / 3.3e-4 rad /
   1.4e-1 N·m，最坏点落在 L0≈0.32 m 的中段（离奇异线 1.78 rad），不是死点折叠：
   个别 φ 组合处相位条件数尖峰把 1.9e-5 的表误差放大到 N·m 量级（φ=(0.1788,−1.6013)
   处 τ1 偏 3.8 N·m，邻域挪 1e-4 rad 即回落）。
   `verify/` 的影子实现把 `arm_*` 接回 libm，所以这部分差异对拍不出来。
3. **标量接口**：`Leg_Pos/Spd/Trq/InvTrq` 按标量调用（codegen/固件 ABI 都是标量）。
   `.m` 传行向量恰好逐元素一致，但别依赖——`verify/` 一律逐点调用。
4. **`I_leg` 口径未裁定**：`leg_data.m` 第 4 列与平行轴定理冲突（19/19 行
   `I_leg < m_leg·l_d²`，`m_leg = 1.855 kg`），待作者确认；裁定前不要改表值。
5. **表外不外推**：`Leg_Comp` 与 `Leg_SpringF` 都是钳位到端点；`Leg_SpringF` 的
   钳位域是 `[0.05, 0.45] m`（几何可达 0.04~0.46，两端各留 0.01 m 余量），
   `±Inf` 折到对应端点、`NaN` 落到下端点 0.05（`fminf/fmaxf` 的 C99 语义，实测）。
6. `codegen/lib/Leg_InvTrq/` 里会连带生成一份 `Leg_Trq.c/.h`，其 `F/Tp` 是 double
   变体、与 `codegen/lib/Leg_Trq/` 同名不同签名；`verify/` 用摊平目录的方式避开，
   手工编译时也务必只取 `codegen/lib/<Name>/<Name>.c`。
7. **`conj` / `*0.0F` 是复数降维残留**：`Leg_Pos/Spd/Trq.m` 里 `conj(...)` 与 `...i`
   参与的运算在实数输入下恒等，Coder 把复数量降成实/虚两路后，C 里就留下 `x*0.0F`
   这类恒零项。**别据 `conj` 推断符号推导需要复数通道**（不需要），也别手改这些残留
   （下次 `matlabFunction` 整段覆盖）。
8. **生成的 `.m` 不带 `%#codegen`**：`Leg_Pos/Spd/Trq.m` 是 `matlabFunction` 产物，
   生成时无从加该标记（手写文件都带）；要补只能后处理改 `.m`，属新工作。生成物一律禁手改。

## 附：五连杆闭合式（交叉核对旁证）

`Leg_Pos/Spd/Trq` 的 codegen 展开式不便阅读，下面是同一套几何的闭合式。19999 个随机点对照
生成函数：`L0` 相对误差 ≤ 1.1e-12、`dL0/dphi1` 绝对误差 ≤ 2.4e-14、`dphi0/dphi1 - 0.5` ≤ 9.6e-9，
可用来独立核对生成产物或估算工作区裕度（`l1 = 0.21 m`、`l2 = 0.25 m`）：

```text
D  = wrap(phi1 - phi4)              % 奇异线 D ≡ 0 (mod 2π)
s  = sin(D/2); c = cos(D/2)
q  = sqrt(0.0625 - 0.0441*s^2)      % = sqrt(l2^2 - (l1*s)^2)
L0 = 0.21*c + q        (D > 0)      % 拉直分支: 0.46 -> 0.136 -> 0.04
L0 = abs(0.21*c - q)   (D < 0)      % 折拢分支: 0.04 -> 0.136 -> 0.46
phi0 = (phi1 + phi4)/2 (+π 分支)     % 严格成立 (D ≠ 0)
dL0/dphi1 = -dL0/dphi4 = -0.105*s - 0.02205*s*c/q        (D > 0)
                       = sign(0.21*c - q)*(-0.105*s + 0.02205*s*c/q)   (D < 0)
detJ = dL0/dphi1                     % 两条分支都成立; detJ = 0 即限制 1 的奇异线
```

两条结构性质值得记住：`L0` 只依赖 `D = wrap(phi1-phi4)`；**分支按 `sign(D)` 选**，因此
`L0(phi1,phi4) ≠ L0(phi4,phi1)` —— 同一对角度交换输入顺序会落到另一分支，`L0` 可差 0.42 m
（与限制 1 的奇异线是同一个机构退化现象的两个侧面）。
