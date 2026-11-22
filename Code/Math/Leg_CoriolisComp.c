/*
 * File: Leg_CoriolisComp.c
 *
 * 旋转耦合补偿：入参 L0 [m]、omega [rad/s]、pitch/roll [rad]、dpitch/droll [rad/s]; 出参 dT_w/dT_p pitch 通道轮/髋扭矩补偿 [N·m]、dF_lft/dF_rgt roll 通道腿推力补偿 [N] —— 由 MATLAB Symbolic Math Toolbox 生成。
 *
 * 原始生成日期: 2026-11-22 05:14:22
 */

/* Include Files */
#include "Leg_CoriolisComp.h"
#include <math.h>

/* Function Definitions */
/*
 * LEG_CORIOLISCOMP  旋转耦合补偿: yaw 角速度下被线性化丢掉的 ω 项  (codegen 版)
 *
 *    [dT_w, dT_p, dF_lft, dF_rgt] = Leg_CoriolisComp(L0, omega, pitch, roll,
 * dpitch, droll)
 *
 *    模型在 v=0/ω=0 处线性化, 含 ω 的速度二次项全部消失 (lqr/s02 里的
 *    -l·sinθ·dθ² 等线性化后归零)。本函数按计算力矩思路把两项 dominant 补回,
 *    不改 LQR 模型与 K——净效果是让旋转中的真实机器人表现得像 ω=0 的模型:
 *
 *    ① 离心负刚度 (原地自转精确, 主项): 车身偏 θ 角 → 质心偏离 yaw 轴 r=h·θ,
 *       离心力 m·ω²·h·θ 作用在高度 h 处, 产生使倾角继续增大的失稳力矩
 *           τ = m·ω²·h²·θ        (pitch/roll 两通道同构, 与重力失稳项 m·g·h·θ
 * 同型) ω=5 rad/s 时系数 ≈74 N·m/rad, 已接近 m·g·h ≈83, 失稳刚度近翻倍。
 *       补偿取反: τ_c = -G_CEN·m·ω²·h²·θ。
 *       //! 该项含状态 θ, 是随 ω 调度的状态反馈而非纯前馈: 符号错了就是正反馈,
 *       上车前必须架空验证方向 (补偿力矩应把车身往竖直方向拉)。
 *
 *    ② 科氏交叉耦合 (次项, G_COR 可置 0 关断): x 前/y 左/z 上、ω>0=左转 约定下,
 *       pitch 角速度经 2ω×v 耦进 roll, roll 角速度耦进 pitch:
 *           τ_roll_dist  = -2·m·h²·ω·θ̇     τ_pitch_dist = +2·m·h²·ω·φ̇
 *       补偿取反。量级 ~6·ω·θ̇ N·m, 仅急转+急俯仰时显著。
 *
 *    pitch 通道分两路输出 (LQR 本来就是轮/髋两路共同控制 pitch):
 *        dT_w = (1-G_HIP)·t_pitch   → 轮扭矩 (经地面摩擦作用, 间接拉车身)
 *        dT_p =    G_HIP ·t_pitch   → 单腿髋扭矩 (直接作用于腿, 车身受反作用;
 *               两腿同号叠加后车身实际感受约 -2·dT_p, 故 G_HIP 别大,
 * 符号架空验证) G_HIP = 0 时退化为单轮通道 (旧行为)。
 *
 *    输出挂载 (与 Leg_RollComp 同方式, 叠在 LQR/腿长环输出之后、电机限幅之前):
 *        pitch → 轮扭矩:  T_wl -= dT_w;  T_wr -= dT_w;   //! 取负:
 * 补偿走电机反作用 扭矩路径 (同 MgF), 与 LQR
 * 轮通道的"基座加速度"路径机制不同、符号相反; 实测已确认同号会明显变差 pitch →
 * 髋扭矩:  Tp_lft -= dT_p;  Tp_rgt -= dT_p; roll  → 腿力差:  F_lft -= dF_lft;
 * F_rgt -= dF_rgt;   //! 取负: roll>0(右倾)的 恢复对是"右腿加/左腿减"(与固件
 * body_roll PID 同向), 函数输出相反 (roll 力矩经力偶 = Fc·TRACK 转换, 与
 * Leg_RollComp 同一杠杆)
 *
 *    输入 (single/float 标量):
 *        L0    - 两腿平均等效腿长 [m]
 *        omega - yaw 角速度 [rad/s], >0 = 左转
 *        pitch / roll   - 机体俯仰/横滚角 [rad]
 *        dpitch / droll - 机体俯仰/横滚角速度 [rad/s]
 *
 *    输出 (固定 single，对应固件 float):
 *        dT_w   - pitch 通道轮扭矩补偿 [N·m], 两轮同号叠加
 *        dT_p   - pitch 通道髋扭矩补偿 [N·m], 两腿 Tp 同号叠加 (见上文 2
 * 倍反作用) dF_lft / dF_rgt - roll 通道腿推力补偿 [N], 等幅反号
 *
 *    保护:
 *      1) L0 钳位 [0.17, 0.34] m (与固件 LEG_LEN_CLAMP 一致)
 *      2) |ω| < 0.3 rad/s 死区: 防静止时陀螺噪声经 ω²/ω 泄漏成静差
 *      3) dT_w/dT_p 限幅 ±10 N·m、dF 限幅 ±30 N: ω 大时 ① 项无界, 必须钳
 *
 *    See also: Leg_RollComp, Leg_SpringF
 */
void Leg_CoriolisComp(float L0, float omega, float pitch, float roll,
                      float dpitch, float droll, float *dT_w, float *dT_p,
                      float *dF_lft, float *dF_rgt)
{
  float Mh2;
  float b_t_pitch_tmp;
  float t_pitch_tmp;
  /*  ==================== 标定参数 (唯一来源: config/leg_config.m 的 par)
   * ==================== */
  /* LEG_CONFIG  腿长参数表与全部物理常量 (唯一数据源,
   * 改表或改标定只改这一个文件) */
  /*  */
  /*    data = leg_config()           N×4 参数表 (单出口调用, 老调用方不受影响)
   */
  /*    [data, par] = leg_config()    额外给出各模型的标定物理量 par
   * (字段含单位标签) */
  /*  */
  /*    极性: theta_leg0 与动力学模型 theta_l0/theta_r0 同号, 直接代入 — */
  /*          正 = 质心偏向 theta 正方向一侧 (后摆方向, 从左看逆时针), */
  /*          重力项为 m*g*l_leg_d*sin(theta + theta_leg0), g = -9.81。 */
  /*          详见 lqr/推导-v2.md §1.2/§8。 */
  /*  */
  /*    消费方: */
  /*        Tool/model/Leg_Comp.m              重力补偿用 theta0/l_d 插值
   * (同目录, 直接调用) */
  /*        Tool/model/Leg_SpringF.m           气弹簧标定常量 */
  /*        Tool/model/Leg_RollComp.m          转弯向心标定常量 */
  /*        Tool/model/Leg_CoriolisComp.m      旋转耦合标定常量 */
  /*        固件 tinympc14 输出层装配      向心/重力前馈的口径与这里的
   * RollComp/Len 常量对应 */
  /*        lqr/legacy/s03_compute_lqr_fast.m  Leg_data = leg_config(); */
  /*  */
  /*    注意: */
  /*      - 表值在 s03 几何缓存指纹 (cache_key.leg_data) 内, 改表会自动触发 */
  /*        几何重算 —— 这是正确行为, 无需手动清缓存。 */
  /*      - 腿长步长必须保持均匀 (当前 0.16:0.01:0.34), Leg_Comp 的 O(1) 定位 */
  /*        依赖这一前提。 */
  /*      - par 只放标定物理量, 每个消费方都按 single(...) 取用, 保持固件 float
   * 口径; */
  /*        改任一常量后必须重建 C 代码并重跑验证。 */
  /*  */
  /* ! I_leg 口径冲突: 按上表口径 (绕 pitch 轴) 平行轴定理要求 I_leg ≥
   * m_leg·l_d², */
  /* ! 但 m_leg=1.855 kg 时 19/19 行违反 (I_leg − m_leg·l_d² ∈ [−4.66e-2,
   * −9.0e-3] */
  /* ! kg·m², 反推等效质量仅 0.55~1.19 kg); m_leg / l_d / I_leg
   * 三者至少一项口径有误, */
  /* ! 待作者裁定, 在裁定前不要改表值 (一改会自动触发 s03 几何缓存重算)。 */
  /*  */
  /*    See also: Leg_Comp, Leg_SpringF, Leg_RollComp, Leg_CoriolisComp, */
  /*              lqr_config */
  /*  */
  /*    data = leg_config() 返回 N×4 矩阵 (行数随表, 每行: */
  /*          [l_leg, theta_leg0, l_leg_d, I_leg] */
  /*  */
  /*          l_leg      - 腿长 (pitch 转轴到轮轴)                  [m] */
  /*          theta_leg0 - 腿质心相对腿长向量的偏置角               [rad] */
  /*          l_leg_d    - 腿质心到 pitch 转轴的距离 (等效重力力臂) [m] */
  /*          I_leg      - 腿绕 pitch 转轴的转动惯量                [kg·m²] */
  /*  ==================== 整机与硬件 ==================== */
  /*  修改整车质量时只改
   * m_total；分项质量沿用现有标定，支撑质量和机身质量自动派生。 */
  /*  修改后重跑 build_leg_tool 和
   * build_leglen_mpc；平衡模型的数据表有独立生成入口。 */
  /*  先计算局部标量，再组装 par，避免 MATLAB Coder 在结构体读后新增字段。 */
  /*  整车含电池、两腿和两轮的实测总质量 [kg] */
  /*  单腿质量（轮以上部分） [kg] */
  /*  单轮与轮毂电机质量 [kg] */
  /*  轮以上支撑质量 [kg] */
  /*  不含两腿和两轮的机身质量 [kg] */
  /*  重力加速度                              [m/s²] */
  /*  整车总质量                              [kg] */
  /*  单腿质量 (轮以上部分)                    [kg] */
  /*  机身质量，不含腿轮                       [kg] */
  /*  单轮 + 轮毂电机质量                      [kg] */
  /*  半轮距, 两轮中心线到机体对称面            [m] */
  /*  支撑质心在 pitch 轴上方的偏置             [m] */
  /*  轮以上支撑质量 = m_total - 2*m_wheel     [kg] */
  /*  ==================== 腿长工作区 (Leg_RollComp / Leg_CoriolisComp 共用)
   * ==================== */
  /*  标称腿长下限, 各补偿函数按此钳位         [m] */
  /*  标称腿长上限                             [m] */
  /*  ==================== 气弹簧 (Leg_SpringF) ==================== */
  /*  大腿轴心距 [m] */
  /*  小腿轴心距 [m] */
  /*  小腿下销到膝轴 [m] */
  /*  大腿上销到膝轴 [m] */
  /*  最长腿长时的装车销距 [m] */
  /*  最短销距处的轴向推力 [N]，与当前固件标定一致 */
  /*  规格图全伸销距 [m] */
  /*  规格图全伸销距减 70 mm 行程 [m] */
  /*  伸腿时销距增加的装配分支 [rad] */
  /*  腿长计算域下限 [m] */
  /*  腿长计算域上限 [m] */
  /*  ==================== 转弯向心 (Leg_RollComp) ==================== */
  /*  两轮接地点间距 (全轮距)                  [m] */
  /*  全轮距 = 2*par.half_track 2026-10-26 用户确认 */
  /*  单侧补偿力限幅, 防内侧腿被卸载到离地      [N] */
  /*  ==================== 旋转耦合 (Leg_CoriolisComp) ==================== */
  /*  离心负刚度补偿增益 (0~1.5 可标定) */
  /*  科氏交叉项增益 (置 0 = 关断) */
  /*  pitch 补偿分到髋扭矩通道的比例 (0 = 只走轮通道) */
  /*  ω 死区                            [rad/s] */
  /*  pitch 通道单路补偿力矩限幅         [N·m] */
  /*  roll 通道单侧腿力限幅              [N] */
  /*  轮以上支撑质量 = 整车总质量减两轮质量 [kg] */
  /*  支撑质心在 pitch 轴上方的偏置, h = L0 + H_OFF [m] */
  /*  两轮接地点间距 (全轮距) [m] 用户确认值 */
  /*  ① 离心负刚度补偿增益 (0~1.5 可标定) */
  /*  ② 科氏交叉项增益 (置 0 = 关断) */
  /*  pitch 补偿分到髋扭矩通道的比例 (0 = 只走轮通道) */
  /*  ==================== 安全边界 ==================== */
  /*  ω 死区 [rad/s] */
  /*  pitch 通道单路补偿力矩限幅 [N·m] */
  /*  roll 通道单侧腿力限幅 [N] */
  /*  ==================== 输入处理 ==================== */
  if (fabsf(omega) < 0.3F) {
    omega = 0.0F;
  }
  Mh2 = fminf(fmaxf(L0, 0.17F), 0.34F) - 0.0383856297F;
  Mh2 = 10.1979866F * (Mh2 * Mh2);
  /*  m·h² [kg·m²], 两项的公共系数 */
  /*  ==================== ① 离心负刚度 (pitch + roll) ==================== */
  /*  ==================== ② 科氏交叉 (补偿 = 扰动取反) ==================== */
  t_pitch_tmp = -Mh2 * (omega * omega);
  b_t_pitch_tmp = 2.0F * Mh2 * omega;
  Mh2 = t_pitch_tmp * pitch - b_t_pitch_tmp * droll;
  /*  ==================== 输出转换与限幅 ==================== */
  *dT_w = fminf(fmaxf(0.7F * Mh2, -10.0F), 10.0F);
  *dT_p = fminf(fmaxf(0.3F * Mh2, -10.0F), 10.0F);
  /*  roll 力矩 → 单腿力差 [N] */
  *dF_rgt = fminf(
      fmaxf((t_pitch_tmp * roll + b_t_pitch_tmp * dpitch) / 0.4144F, -30.0F),
      30.0F);
  *dF_lft = -*dF_rgt;
}

/*
 * File trailer for Leg_CoriolisComp.c
 *
 * [EOF]
 */
