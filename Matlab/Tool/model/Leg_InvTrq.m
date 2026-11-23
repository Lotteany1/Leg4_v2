%#codegen
function [F, Tp] = Leg_InvTrq(tau1, tau4, phi1, phi4)
%LEG_INVTRQ  VMC 逆映射: 关节力矩 → 末端力/虚拟力矩  (codegen 版)
%
%   [F, Tp] = Leg_InvTrq(tau1, tau4, phi1, phi4)
%
%   与 Leg_Trq(F, Tp, phi1, phi4) = [tau1; tau4] 互为逆运算。
%   通过调用 Leg_Trq 抽样构建 Jacobian，保证正逆数值一致。
%
%   输入 (类型随输入, codegen 用 single 标量):
%       tau1  - 关节1 (大腿) 力矩  [N·m]
%       tau4  - 关节4 (小腿) 力矩  [N·m]
%       phi1  - 关节1 角度         [rad]
%       phi4  - 关节4 角度         [rad]
%
%   输出 (类型随输入):
%       F     - 等效末端力     [N]
%       Tp    - 等效虚拟力矩   [N·m]
%
%   See also: Leg_Trq, Leg_Pos, Leg_Comp

% ==================== Jacobian 抽样 ====================
%   Leg_Trq 对 (F, Tp) 线性且过原点:
%       tau = J * [F; Tp]
%   本文件的 J 是 Leg_Trq 的 2x2 响应矩阵 (列 = 单位 (F,Tp) 的 tau 响应),
%   等于运动学雅可比 d pos/d q 的转置 — 虚功对偶 tau = (d pos/d q)' * [F; Tp]。
%   抽样 F=1,Tp=0 得 J 第一列，F=0,Tp=1 得第二列
%   直接调用 Leg_Trq 保证与正映射数值一致，避免手动推导错误。

eps_val = cast(1.0, 'like', phi1);  % 抽样幅值，1 N / 1 N·m 合理 (线性映射，与幅值无关)

T_fwd = Leg_Trq(eps_val, cast(0, 'like', phi1), phi1, phi4);
J11 = T_fwd(1) / eps_val;
J21 = T_fwd(2) / eps_val;

T_fwd = Leg_Trq(cast(0, 'like', phi1), eps_val, phi1, phi4);
J12 = T_fwd(1) / eps_val;
J22 = T_fwd(2) / eps_val;

% ==================== Tikhonov 正则化求逆 ====================
%   J⁺ = (JᵀJ + λI)⁻¹ Jᵀ
%   JᵀJ 的对角元是平方和，无灾难相消；λ 抑制近零奇异值。
%   λ=1e-6 的往返偏置 ≈ λ/σ_min(J)²: 标称腿长带 0.16~0.34 m 内 σ_min²∈[0.0295,
%   0.0434] (cond J ∈ [3.39, 4.12]), 实测往返相对误差中位 2.46e-5、最大 3.39e-5,
%   与预测 λ/σ_min² 之比中位 0.99998; >1% 偏置需 cond J ≳ 100 (L0 ≳ 0.46 m, 带外)。

lambda = 1e-6;

A11 = J11*J11 + J21*J21 + lambda;
A12 = J11*J12 + J21*J22;
A22 = J12*J12 + J22*J22 + lambda;

det_A = A11*A22 - A12*A12;  % λ > 0 保证正定

iA11 =  A22 / det_A;
iA12 = -A12 / det_A;
iA22 =  A11 / det_A;

F  = (iA11*J11 + iA12*J12)*tau1 + (iA11*J21 + iA12*J22)*tau4;
Tp = (iA12*J11 + iA22*J12)*tau1 + (iA12*J21 + iA22*J22)*tau4;

end
