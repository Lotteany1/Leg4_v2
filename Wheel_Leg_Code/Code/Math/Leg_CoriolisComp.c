/*
 * File: Leg_CoriolisComp.c
 *
 * 旋转耦合补偿：入参 L0 [m]、omega [rad/s]、pitch/roll [rad]、dpitch/droll [rad/s]; 出参 dT_w/dT_p pitch 通道轮/髋扭矩补偿 [N·m]、dF_lft/dF_rgt roll 通道腿推力补偿 [N] —— 由 MATLAB Coder 从 001 模型生成。
 *
 * 原始生成日期: 2026-11-22 10:57:36
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
 *      1) L0 按 001 工作区钳位 (与固件 LEG_LEN_CLAMP 一致)
 *      2) |ω| < 0.3 rad/s 死区: 防静止时陀螺噪声经 ω²/ω 泄漏成静差
 *      3) dT_w/dT_p 限幅 ±10 N·m、dF 限幅 ±30 N: ω 大时 ① 项无界, 必须钳
 *
 *    See also: Leg_RollComp, Leg_SpringF
 */
void Leg_CoriolisComp(float L0, float omega, float pitch, float roll,
                      float dpitch, float droll, float *dT_w, float *dT_p,
                      float *dF_lft, float *dF_rgt)
{
  static const float T[40] = {
      0.15F,          0.159005359F,   0.168010712F,   0.177016065F,
      0.186021432F,   0.195026785F,   0.204032138F,   0.213037491F,
      0.222042844F,   0.231048211F,   0.240053564F,   0.249058917F,
      0.25806427F,    0.267069638F,   0.276074976F,   0.285080343F,
      0.294085711F,   0.303091049F,   0.312096417F,   0.321101785F,
      -0.0307118148F, -0.0315166637F, -0.0322900899F, -0.0330369361F,
      -0.0337610841F, -0.0344657041F, -0.0351534F,    -0.0358263329F,
      -0.0364863127F, -0.0371348597F, -0.0377732739F, -0.0384026654F,
      -0.0390239842F, -0.0396380536F, -0.0402455926F, -0.0408472233F,
      -0.0414435044F, -0.0420349129F, -0.0426218845F, -0.043204803F};
  float a_tmp;
  float b_a_tmp;
  float b_value;
  int b_index;
  /*  ==================== 标定参数 (唯一来源: config/leg_config.m 的 par)
   * ==================== */
  /*  轮以上支撑质量 = 整车总质量减两轮质量 [kg] */
  /*  固定大小物理参数表的分段线性插值，表外保持端点。 */
  b_value = fminf(fmaxf(L0, 0.15F), 0.321101785F);
  b_index = 1;
  while ((b_index < 19) && (b_value >= T[b_index])) {
    b_index++;
  }
  /*  支撑质心在 pitch 轴上方的偏置, h = L0 + H_OFF + 轮半径 [m] */
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
  a_tmp = T[b_index - 1];
  b_a_tmp = T[b_index + 19];
  b_value = (fminf(fmaxf(L0, 0.17F), 0.321101785F) +
             (b_a_tmp + (b_value - a_tmp) / (T[b_index] - a_tmp) *
                            (T[b_index + 20] - b_a_tmp))) +
            0.0600000024F;
  b_value = 13.4096775F * (b_value * b_value);
  /*  集中质量近似 m·h² [kg·m²], 两项的公共系数 */
  /*  ==================== ① 离心负刚度 (pitch + roll) ==================== */
  /*  ==================== ② 科氏交叉 (补偿 = 扰动取反) ==================== */
  b_a_tmp = -b_value * (omega * omega);
  a_tmp = 2.0F * b_value * omega;
  b_value = b_a_tmp * pitch - a_tmp * droll;
  /*  ==================== 输出转换与限幅 ==================== */
  *dT_w = fminf(fmaxf(0.7F * b_value, -10.0F), 10.0F);
  *dT_p = fminf(fmaxf(0.3F * b_value, -10.0F), 10.0F);
  /*  roll 力矩 → 单腿力差 [N] */
  *dF_rgt = fminf(
      fmaxf((b_a_tmp * roll + a_tmp * dpitch) / 0.414400041F, -30.0F), 30.0F);
  *dF_lft = -*dF_rgt;
}

/*
 * File trailer for Leg_CoriolisComp.c
 *
 * [EOF]
 */
