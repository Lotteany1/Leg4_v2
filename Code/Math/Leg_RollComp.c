/*
 * File: Leg_RollComp.c
 *
 * 转弯向心力前馈：入参 L0 等效腿长 [m]、omega yaw 角速度 [rad/s]、v 前进速度 [m/s]; 出参 dF_lft/dF_rgt 为限幅后补偿 [N]，Fc_raw 为限幅前等效力 [N]，模型外部扰动为 [Fc_raw,-Fc_raw] —— 由 MATLAB Symbolic Math Toolbox 生成。
 *
 * 原始生成日期: 2026-11-22 05:14:20
 */

/* Include Files */
#include "Leg_RollComp.h"
#include <math.h>

/* Function Definitions */
/*
 * LEG_ROLLCOMP  转弯向心力前馈: 腿长/yaw 角速度/车速 → 左右腿推力补偿  (codegen
 * 版)
 *
 *    [dF_lft, dF_rgt, Fc_raw] = Leg_RollComp(L0, omega, v)
 *
 *    模型 (前馈, 与侧倾角 PID 反馈叠加使用, 参考 Leg_SpringF 的用法):
 *    转弯向心加速度 a_c = v*omega 作用在质心高度 h 处, 产生绕纵轴的滚转
 *    惯性力矩 tau = M_SUP*a_c*h, 由左右腿竖直推力差平衡:
 *        (dF_rgt - dF_lft) * (TRACK/2) = tau   (对称分配: 一腿 +Fc, 另一腿 -Fc)
 *        =>  Fc = M_SUP * v * omega * h / TRACK                          [N]
 *    质心高度用腿长近似: h = L0 + H_OFF (机身支撑质心在 pitch 轴上方的偏置,
 *    腿/机身质量的质心差折进 H_OFF 一并标定)。
 *
 *    输出用法 (与固件 balance.c 腿长环输出叠加):
 *        leg_control[lft].F = PID_out + ... + dF_lft;
 *        leg_control[rgt].F = PID_out + ... + dF_rgt;
 *
 *    极性: omega > 0 = 左转 (俯视逆时针), v > 0 = 前进。前进左转时车身向外(右)
 *    倾 → 右腿(外侧)加载 dF_rgt = +Fc, 左腿(内侧)卸载 dF_lft = -Fc; 倒车/右转
 *    时 v*omega 变号, 补偿方向自动翻转, 无需显式判内外侧。
 *
 *    输入 (single/float 标量):
 *        L0    - 等效腿长 [m], 传两腿平均值即可 (左右腿不等长对 h
 * 的影响是二阶小量) omega - yaw 角速度 [rad/s] v     - 车身前进速度 [m/s]
 *
 *    输出 (固定 single，对应固件 float):
 *        dF_lft - 左腿推力补偿量 [N], 直接加进左腿竖直推力指令 F
 *        dF_rgt - 右腿推力补偿量 [N]
 *        Fc_raw - 限幅前的单侧等效力 [N]；模型外部扰动为 [Fc_raw;
 * -Fc_raw]，不下发电机
 *
 *    保护:
 *      1) L0 钳位 [0.17, 0.34] m: 腿长标称工作区间 (与固件 LEG_LEN_CLAMP 一致)
 *      2) Fc 限幅 ±F_MAX: 内侧腿卸载过度会把腿顶离地面, 参考实现缺这道限幅
 *         (其 TURN_RATE 限幅是死代码), 高速转弯时 Fc 无界 —— 本实现补上
 *
 *    See also: Leg_SpringF, Leg_Comp, Leg_Trq
 */
void Leg_RollComp(float L0, float omega, float v, float *dF_lft, float *dF_rgt,
                  float *Fc_raw)
{
  /*  ==================== 标定参数 (唯一来源: config/leg_config.m 的 par)
   * ==================== */
  /*  单位: 质量 kg, 长度 m, 力 N。固定 single，对应固件 float。 */
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
  /*  (轮与轮毂电机在地面高度, 不产生滚转力矩, 不计入) */
  /*  支撑质心在 pitch 轴上方的偏置高度 h = L0 + H_OFF [m] */
  /*  //! TODO: 占位值, 须按整车质心实测标定 */
  /*  两轮接地点间距 (全轮距) [m] */
  /*  用户确认全轮距 0.42551 m，半轮距为 0.212755 m */
  /*  单侧补偿力限幅 [N], 防内侧腿被卸载到离地 */
  /*  ==================== 输入钳位 (标称区间内是恒等映射) ====================
   */
  /*  ==================== 向心力前馈 ==================== */
  /*  支撑质心离地高度 [m] */
  *Fc_raw = 10.1979866F * v * omega *
            (fminf(fmaxf(L0, 0.17F), 0.34F) - 0.0383856297F) / 0.4144F;
  /*  转弯力矩的单侧等效力 [N]，保留饱和后的残余扰动 */
  *dF_rgt = fminf(fmaxf(*Fc_raw, -150.0F), 150.0F);
  /*  外侧(转弯外沿)加载 */
  *dF_lft = -*dF_rgt;
  /*  内侧卸载 */
}

/*
 * File trailer for Leg_RollComp.c
 *
 * [EOF]
 */
