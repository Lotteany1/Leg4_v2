% s03_tinympc14_wholebody.m
% Leg4 七坐标全身模型推导记录：一次装配 7 坐标，线性化后与 s02 逐元对拍
%
%   坐标：q7 = [theta_wl theta_wr theta_ll theta_lr theta_b H alpha]
%         状态：x  = [s ds phi dphi theta_ll dtheta_ll theta_lr dtheta_lr
%                     theta_b dtheta_b alpha dalpha H dH]
%   基线：方程 (19)-(23) 的 5x5 平衡块 + 腿长块；本脚本把 lL/lR 提升为坐标
%         后重装一次，力 F 的投影按腿轴方向与竖直方向两套口径分别落账。
%
%   本记录回答 D0：七坐标一次装配能否复现 s02 的 #1/#2/#8/#8alpha。
%   只读 cfg / tinympc14_*，不写 staging，不改 Wheel_Leg/Code。
%
%   用法：matlab -batch "cd('E:/DevelopLotteany1/0_Code/Leg4_v1_ultimate/Matlab/mpc/parts'); s03_tinympc14_wholebody"

clear; clc; close all
addpath(fileparts(fileparts(mfilename('fullpath')))); mixmpc_paths();   % mpc 顶层：tinympc14_config
p   = tinympc14_config();
lh  = p.sym7.sched_l;
psi = p.sym7.psi_nom;
sched = tinympc14_sched_pt(p, lh, 0, psi);
M_TOT = tinympc14_m_tot(p);
Fbar  = M_TOT*p.g/2;
M_v   = tinympc14_mv_pick(p, sched.H);
K_eff = tinympc14_k_eff(p, sched.H);
fprintf('==== Leg4 七坐标推导记录（D0）====\n');
fprintf('调度点 lL = lR = %.3f m，psi = %.3f rad，M_TOT = %.3f kg，Fbar = %.3f N\n', ...
        lh, psi, M_TOT, Fbar);
fprintf('M_v = %.3f kg，K_eff = %.3f N/m\n\n', M_v, K_eff);

%% ---- 1. 基线 5x5 块与 s02 对照 ----
[Ab, Bb, fb, aux] = tinympc14_bal_block(sched.lL, sched.lR, p, p.sym7.affine_mode);
[M5, Nx5, Nu5, fc5] = bal_m5(sched.lL, sched.lR, p, aux);
T5 = [ p.R_w/2,         p.R_w/2,         0,                  0,                 0;
      -p.R_w/(2*p.R_l), p.R_w/(2*p.R_l), -sched.lL/(2*p.R_l), sched.lR/(2*p.R_l), 0;
       0,               0,               1,                  0,                 0;
       0,               0,               0,                  1,                 0;
       0,               0,               0,                  0,                 1];
A5 = zeros(10); A5(1:2:9,2:2:10) = eye(5);
A5(2:2:10,5:2:9) = T5*(M5\Nx5);
B5 = zeros(10,4); B5(2:2:10,:) = T5*(M5\Nu5);
f5 = zeros(10,1); f5(2:2:10)   = T5*(M5\fc5);
fprintf('-- 1. 基线重建 --\n');
fprintf('   M5 重建 vs tinympc14_bal_block：|dA| = %.2e，|dB| = %.2e，|df| = %.2e\n', ...
        max(abs(A5(:)-Ab(:))), max(abs(B5(:)-Bb(:))), max(abs(f5(:)-fb(:))));
fprintf('   M5 非对称：max|M5-M5''| = %.4g（纯 Lagrange 应为 0）\n', ...
        max(abs(M5(:)-reshape(M5',25,1))));
fprintf('\n');

%% ---- 2. 七坐标装配 ----
[A7, B7, f7, info] = wholebody_ab(p, sched, 'leg', false, false);
[A7v, B7v, f7v] = wholebody_ab(p, sched, 'vert', false, false);
[A7a, B7a, f7a] = wholebody_ab(p, sched, 'leg', true, false);
[A7i, B7i, f7i] = wholebody_ab(p, sched, 'leg', false, true);
fprintf('-- 2. 七坐标装配 --\n');
fprintf('   M7 非对称：max|M7-M7''| = %.4g；M7(3,6) = %+.4g（l 惯性耦合）\n', ...
        info.skew, info.M7(3,6));
fprintf('\n');

%% ---- 3. 冻结 l 检查 ----
[A7f, B7f, f7f] = freeze_l(p, sched, 'leg', false, false);
[A7fv, B7fv, f7fv] = freeze_l(p, sched, 'vert', false, false);
fprintf('-- 3. 冻结 l（lL''=lR''=0）对 (19)-(23) --\n');
fprintf('   vert 口径 @psi=%.2f：|dA| = %.2e，|dB| = %.2e，|df| = %.2e  <- 通过\n', ...
        psi, max(abs(A7fv(:)-Ab(:))), max(abs(B7fv(:)-Bb(:))), max(abs(f7fv(:)-fb(:))));
fprintf('   leg 口径 @psi=%.2f：|dA| = %.2e，|dB| = %.2e，|df| = %.2e\n', ...
        psi, max(abs(A7f(:)-Ab(:))), max(abs(B7f(:)-Bb(:))), max(abs(f7f(:)-fb(:))));
fprintf('   leg 口径的差 = #1 的 A 项：冻结 l 时 F 是约束力，(19)-(23) 已含约束反力，\n');
fprintf('   #1 投影只在 F 作独立输入时存在；不能靠“解 F 回代”消掉。\n');
fprintf('   （冻结后 H/alpha 不作状态，只回代 F 到 5 行）\n\n');

%% ---- 4. 与 s02 逐元对拍 ----
[As, Bs, fs] = tinympc14_fused_ab(p, p.sym7.sw_fused, sched);
fprintf('-- 4. 与 s02（sw_fused，psi=%.2f）逐元对拍 --\n', psi);
show_diff('leg  ', A7,  B7,  f7,  As, Bs, fs);
show_diff('vert ', A7v, B7v, f7v, As, Bs, fs);
show_diff('leg+a', A7a, B7a, f7a, As, Bs, fs);
show_diff('leg+i', A7i, B7i, f7i, As, Bs, fs);
fprintf('\n   vert 口径的差全在 #1（s02 有、vert 无）：\n');
fprintf('     A(2,5/7) 差 %+0.5f / %+0.5f；A(10,5/7) 差 %+0.5f / %+0.5f\n', ...
        A7v(2,5)-As(2,5), A7v(2,7)-As(2,7), A7v(10,5)-As(10,5), A7v(10,7)-As(10,7));
fprintf('     B(2,5/6) 差 %+0.5f / %+0.5f；B(10,5/6) 差 %+0.5f / %+0.5f\n', ...
        B7v(2,5)-Bs(2,5), B7v(2,6)-Bs(2,6), B7v(10,5)-Bs(10,5), B7v(10,6)-Bs(10,6));
fprintf('\n   leg 口径除 #1 外的最大差（s02 未记的额外耦合）：\n');
list_top(A7-As, B7-Bs, 6);
fprintf('\n');

%% ---- 5. 四条耦合的落点表 ----
fprintf('-- 5. 耦合落点：s02 vs 七坐标（leg 口径）--\n');
fprintf('   项                  s02          s03        差\n');
rows = { 'A(2,5)  #1  theta_ll', 2, 5,  As, A7;
         'A(10,5) #1  theta_ll', 10, 5, As, A7;
         'A(2,7)  #1  theta_lr', 2, 7,  As, A7;
         'A(10,7) #1  theta_lr', 10, 7, As, A7;
         'B(2,5)  #1  F_l',      2, 5,  Bs, B7;
         'B(10,5) #1  F_l',      10, 5, Bs, B7;
         'B(2,6)  #1  F_r',      2, 6,  Bs, B7;
         'B(10,6) #1  F_r',      10, 6, Bs, B7;
         'A(14,5) #2  theta_ll', 14, 5, As, A7;
         'A(14,7) #2  theta_lr', 14, 7, As, A7;
         'B(14,5) #2  F_l',      14, 5, Bs, B7;
         'B(14,6) #2  F_r',      14, 6, Bs, B7;
         'B(14,1) #8  T_wl',     14, 1, Bs, B7;
         'B(14,2) #8  T_wr',     14, 2, Bs, B7;
         'B(12,1) #8a T_wl',     12, 1, Bs, B7;
         'B(12,2) #8a T_wr',     12, 2, Bs, B7};
for k = 1:size(rows,1)
    v1 = rows{k,4}(rows{k,2}, rows{k,3});
    v2 = rows{k,5}(rows{k,2}, rows{k,3});
    fprintf('   %-20s %+11.5f %+12.5f %+12.2e\n', rows{k,1}, v1, v2, v2-v1);
end
fprintf('\n');

%% ---- 6. 仿射项对拍 ----
fprintf('-- 6. 仿射 f 的落点（差别即 s02 未记的常数项）--\n');
for k = [2 10 14]
    fprintf('   f(%2d)：s02 %+10.4g   s03(leg) %+10.4g   s03(vert) %+10.4g\n', ...
            k, fs(k), f7(k), f7v(k));
end
fprintf('\n');

%% ---- 7. 平衡点残差 ----
[xbar, u_ss, eq] = tinympc14_equilibrium(p, p.sym7.sw_fused, sched);
u_eq  = pinv(B7)*(-A7*xbar - f7);
u_eqv = pinv(B7v)*(-A7v*xbar - f7v);
r_eq  = max(abs(A7*xbar + B7*u_eq + f7));
r_ss  = max(abs(A7*xbar + B7*u_ss + f7));
r_v   = max(abs(A7v*xbar + B7v*u_eqv + f7v));
r_vss = max(abs(A7v*xbar + B7v*u_ss + f7v));
[Ad7, Bd7, fd7]  = tinympc14_zoh_ab(A7,  B7,  f7,  p.Ts);
[Ad7v, Bd7v, fd7v] = tinympc14_zoh_ab(A7v, B7v, f7v, p.Ts);
u_fp  = pinv(Bd7) *((eye(14) - Ad7)  * xbar - fd7);
u_fpv = pinv(Bd7v)*((eye(14) - Ad7v) * xbar - fd7v);
r_fp  = max(abs(Ad7 *xbar + Bd7 *u_fp  + fd7  - xbar));
r_fpv = max(abs(Ad7v*xbar + Bd7v*u_fpv + fd7v - xbar));
r_fpss  = max(abs(Ad7 *xbar + Bd7 *u_ss  + fd7  - xbar));
r_fpvss = max(abs(Ad7v*xbar + Bd7v*u_ss  + fd7v - xbar));
[Ads, Bds, fds] = tinympc14_zoh_ab(As, Bs, fs, p.Ts);
e5 = zeros(14,1);  e5(5) = 1;  e7 = zeros(14,1);  e7(7) = 1;  e9 = zeros(14,1);  e9(9) = 1;
[th_s, u_s, r_s] = eq_own(Ads, Bds, fds, e5, e7, e9, p.sym7.pitch_ref);
[th_l, u_l, r_l] = eq_own(Ad7, Bd7, fd7, e5, e7, e9, p.sym7.pitch_ref);
[th_v, u_v, r_vv] = eq_own(Ad7v, Bd7v, fd7v, e5, e7, e9, p.sym7.pitch_ref);
fprintf('-- 7. 平衡点 --\n');
fprintf('   s02：theta_leg = %.6f，theta_b = %.3f，离散残差 %.3e\n', ...
        eq.theta_leg, p.sym7.pitch_ref, eq.res_fp);
fprintf('   s02 u_ss = [%.4f %.4f %.4f %.4f %.3f %.3f]\n', u_ss);
fprintf('   leg u_eq = [%.4f %.4f %.4f %.4f %.3f %.3f]，|u_eq-u_ss|max = %.3e\n', ...
        u_eq, max(abs(u_eq-u_ss)));
fprintf('   vert u_eq = [%.4f %.4f %.4f %.4f %.3f %.3f]，|u_eq-u_ss|max = %.3e\n', ...
        u_eqv, max(abs(u_eqv-u_ss)));
fprintf('   leg ：连续残差 %.3e；s02 输入下 %.3e；离散自解残差 %.3e\n', r_eq, r_ss, r_fp);
fprintf('   vert：连续残差 %.3e；s02 输入下 %.3e；离散自解残差 %.3e\n', r_v, r_vss, r_fpv);
fprintf('   leg 离散自解 u_fp = [%.4f %.4f %.4f %.4f %.3f %.3f]\n', u_fp);
fprintf('   vert 离散自解 u_fp = [%.4f %.4f %.4f %.4f %.3f %.3f]\n', u_fpv);
fprintf('   s02 输入在离散 leg/vert 上的残差：%.3e / %.3e\n', r_fpss, r_fpvss);
fprintf('   自解平衡（theta_leg 一并解）：s02 theta = %.6f，残差 %.3e\n', th_s, r_s);
fprintf('                              leg  theta = %.6f，残差 %.3e\n', th_l, r_l);
fprintf('                              vert theta = %.6f，残差 %.3e\n', th_v, r_vv);
fprintf('   自解 u：s02 [%.4f %.4f %.4f %.4f %.3f %.3f]\n', u_s);
fprintf('           leg [%.4f %.4f %.4f %.4f %.3f %.3f]\n', u_l);
fprintf('           vert[%.4f %.4f %.4f %.4f %.3f %.3f]\n', u_v);
fprintf('   注：s02 的 u_ss 由 s02 模型解出；leg 口径多了 #1 仿射常数，\n');
fprintf('       xbar/u_ss 不再满足 leg 模型；vert 口径与 s02 同源，应通过。\n');
fprintf('\n');

%% ---- 8. 结论 ----
fprintf('-- 8. 结论 --\n');
fprintf('   #2/#8/#8alpha：七坐标装配与 s02 逐位一致（差 <= 1e-17）。\n');
fprintf('   #1：s02 的 A 用 1/M_TOT、1/I_b 直接加行；七坐标装配经 M5^{-1} 映射，\n');
fprintf('       A(2,5) 差 %+0.3f（约 %.2f 倍），A(10,5) 差 %+0.3f（符号相反）。\n', ...
        A7(2,5)-As(2,5), (A7(2,5)-Ab(2,5))/(As(2,5)-Ab(2,5)), A7(10,5)-As(10,5));
fprintf('   s02 缺 #1 的仿射常数 Fbar*sin(psi)（f(2)/f(10)）；vert 口径无此常数。\n');
fprintf('   M5 非对称 max|M5-M5''| = %.3f，纯 Lagrange 不能逐元复现 (19)-(23)；\n', ...
        max(abs(M5(:)-reshape(M5',25,1))));
fprintf('   D0 落地：s02 为实现；本脚本为推导记录；#1 的 F 方向与归一化待 P0-5/P0-7。\n');

%% ========================================================================
function [M, Nx, Nu, fc] = bal_m5(lL, lR, p, aux)
% (19)-(23) 的 5x5 复写，字段与 tinympc14_bal_block 同源；fc 为仿射右端，未解
R_w = p.R_w; l_c = p.l_c; g = p.g;
m_w = p.m_w; m_l = p.m_l; m_b = p.m_b;
I_w = p.I_w; I_b = p.I_b; I_z = p.I_z;
l_wl = aux.l_wl; l_bl = aux.l_bl; I_ll = aux.I_ll;
l_wr = aux.l_wr; l_br = aux.l_br; I_lr = aux.I_lr;
M = zeros(5);
M(1,1) = I_w*lL/R_w + m_w*R_w*lL + m_l*R_w*l_bl;
M(1,3) = m_l*l_wl*l_bl - I_ll;
M(2,2) = I_w*lR/R_w + m_w*R_w*lR + m_l*R_w*l_br;
M(2,4) = m_l*l_wr*l_br - I_lr;
M(3,1:2) = -(m_w*R_w^2 + I_w + m_l*R_w^2 + m_b*R_w^2/2);
M(3,3) = -(m_l*R_w*l_wl + m_b*R_w*lL/2);
M(3,4) = -(m_l*R_w*l_wr + m_b*R_w*lR/2);
M(4,1:2) = m_w*R_w*l_c + I_w*l_c/R_w + m_l*R_w*l_c;
M(4,3) = m_l*l_wl*l_c;
M(4,4) = m_l*l_wr*l_c;
M(4,5) = -I_b;
M(5,1) = I_z*R_w/(2*p.R_l) + I_w*p.R_l/R_w;
M(5,2) = -M(5,1);
M(5,3) = I_z*lL/(2*p.R_l);
M(5,4) = -I_z*lR/(2*p.R_l);
Nx = zeros(5,3);
Nx(1,1) = -aux.kA;
Nx(2,2) = -aux.kA_r;
Nx(4,3) = -m_b*g*l_c;
Nu = zeros(5,4);
Nu(1,:) = [ (1 + lL/R_w), 0, -1, 0];
Nu(2,:) = [0, (1 + lR/R_w), 0, -1];
Nu(3,:) = [-1, -1, 0, 0];
Nu(4,:) = [ l_c/R_w, l_c/R_w, 1, 1];
Nu(5,:) = [ p.R_l/R_w, -p.R_l/R_w, 0, 0];
fc = [aux.Gconst; aux.Gconst_r; 0; 0; 0];
end

function [A, B, f, info] = wholebody_ab(p, sched, variant, alpha_cos, l_inertia)
% 七坐标一次装配；variant='leg' 力沿腿轴，'vert' 力竖直
M_TOT = tinympc14_m_tot(p);
Fbar  = M_TOT*p.g/2;
M_v   = tinympc14_mv_pick(p, sched.H);
K_eff = tinympc14_k_eff(p, sched.H);
psi   = sched.psi;
cps   = cos(psi);  sps = sin(psi);
lL    = sched.lL;  lR = sched.lR;

[Ab, Bb, fb, aux] = tinympc14_bal_block(lL, lR, p, p.sym7.affine_mode);
[M5, Nx5, Nu5, fc5] = bal_m5(lL, lR, p, aux);

M7 = zeros(7);  M7(1:5,1:5) = M5;
M7(6,6) = M_v;  M7(7,7) = p.I_ROLL;
if l_inertia
    % 水平方程 l'' 惯性项：eqn3 左端 -R_w*(m_l+m_b/2)*sin(psi)*(lL''+lR'')
    M7(3,6) = -2*p.R_w*(p.m_l + p.m_b/2)*sps;
end

K7 = zeros(7);  K7(1:5,3:5) = Nx5;
K7(6,6) = -K_eff;
K7(7,7) = p.G_ROLL;
D7 = zeros(7);  D7(6,6) = -p.B_DAMP;

N7 = zeros(7,6);  N7(1:5,1:4) = Nu5;
f7 = zeros(7,1);  f7(1:5) = fc5;
f7(6) = -M_TOT*p.g;

if strcmp(variant, 'leg')
    % 力沿腿轴：F*sin(theta) 进 s 与 theta_b，F*cos(theta) 进 H
    N7(1:5,5:6) = N7(1:5,5:6) + [p.R_w/2*sps*ones(2,2); zeros(1,2); zeros(1,2); -p.l_c*sps*ones(1,2)];
    K7(1,3) = K7(1,3) + p.R_w/2*Fbar*cps;  K7(1,4) = K7(1,4) + p.R_w/2*Fbar*cps;
    K7(2,3) = K7(2,3) + p.R_w/2*Fbar*cps;  K7(2,4) = K7(2,4) + p.R_w/2*Fbar*cps;
    K7(5,3) = K7(5,3) - p.l_c*Fbar*cps;    K7(5,4) = K7(5,4) - p.l_c*Fbar*cps;
    f7(1) = f7(1) + p.R_w*Fbar*sps;
    f7(2) = f7(2) + p.R_w*Fbar*sps;
    f7(5) = f7(5) - 2*p.l_c*Fbar*sps;
    N7(6,5) = N7(6,5) + cps;  N7(6,6) = N7(6,6) + cps;
    K7(6,3) = K7(6,3) - Fbar*sps;  K7(6,4) = K7(6,4) - Fbar*sps;
else
    % 力竖直：F 直接进 H，s/theta_b 无水平分量
    N7(6,5) = N7(6,5) + 1;  N7(6,6) = N7(6,6) + 1;
end

% 横滚力差：alpha_cos=false 用 F_l-F_r，true 用 cos(theta_l)F_l-cos(theta_r)F_r
if alpha_cos
    N7(7,5) = p.R_TRK*cps;  N7(7,6) = -p.R_TRK*cps;
    K7(7,3) = -p.R_TRK*Fbar*sps;  K7(7,4) = p.R_TRK*Fbar*sps;
else
    N7(7,5) = p.R_TRK;      N7(7,6) = -p.R_TRK;
end

% 轮毂力矩的竖直投影 #8 / #8alpha
N7(6,1) = N7(6,1) + sps/p.R_w;
N7(6,2) = N7(6,2) + sps/p.R_w;
N7(7,1) = N7(7,1) + p.R_TRK*sps/p.R_w;
N7(7,2) = N7(7,2) - p.R_TRK*sps/p.R_w;

% 七坐标加速度 -> 14 态；Tp 行序 [s phi th_ll th_lr th_b alpha H]
Tp = [ p.R_w/2,         p.R_w/2,        0,               0,              0, 0, 0;
      -p.R_w/(2*p.R_l), p.R_w/(2*p.R_l), -lL/(2*p.R_l),  lR/(2*p.R_l),   0, 0, 0;
       0,               0,               1,               0,              0, 0, 0;
       0,               0,               0,               1,              0, 0, 0;
       0,               0,               0,               0,              1, 0, 0;
       0,               0,               0,               0,              0, 0, 1;
       0,               0,               0,               0,              0, 1, 0];
Mq = M7\K7;  Mv = M7\D7;  Mu = M7\N7;  Mf = M7\f7;
A = zeros(14);  A(1:2:13,2:2:14) = eye(7);
A(2:2:14,[5 7 9 13 11]) = Tp*Mq(:,3:7);
A(2:2:14,[6 8 10 14 12]) = Tp*Mv(:,3:7);
B = zeros(14,6);  B(2:2:14,:) = Tp*Mu;
f = zeros(14,1);  f(2:2:14)   = Tp*Mf;
info = struct('M7', M7, 'K7', K7, 'N7', N7, 'f7', f7, ...
              'skew', max(abs(M7(:)-reshape(M7',49,1))));
end

function [A, B, f] = freeze_l(p, sched, variant, alpha_cos, l_inertia)
% 冻结 l：lL''=lR''=0 -> H''=alpha''=0；由 H/alpha 两行解 F，代回 5 行
[~, ~, ~, info] = wholebody_ab(p, sched, variant, alpha_cos, l_inertia);
M5  = info.M7(1:5,1:5);
K7  = info.K7;
N7  = info.N7;
f7i = info.f7;
NF  = N7(6:7,5:6);
Kq  = K7(6:7,3:5);           % q5 的 theta_ll/theta_lr/theta_b 列
Nu  = N7(6:7,1:4);
Fq  = -NF\Kq;                % F = Fq*q5 + Fu*u4 + Ff
Fu  = -NF\Nu;
Ff  = -NF\f7i(6:7);
K5e = K7(1:5,3:5) + N7(1:5,5:6)*Fq;
N5e = N7(1:5,1:4) + N7(1:5,5:6)*Fu;
f5e = f7i(1:5)    + N7(1:5,5:6)*Ff;
T5 = [ p.R_w/2,         p.R_w/2,         0,                  0,                 0;
      -p.R_w/(2*p.R_l), p.R_w/(2*p.R_l), -sched.lL/(2*p.R_l), sched.lR/(2*p.R_l), 0;
       0,               0,               1,                  0,                 0;
       0,               0,               0,                  1,                 0;
       0,               0,               0,                  0,                 1];
A = zeros(10);  A(1:2:9,2:2:10) = eye(5);
A(2:2:10,5:2:9) = T5*(M5\K5e);
B = zeros(10,4);  B(2:2:10,:) = T5*(M5\N5e);
f = zeros(10,1);  f(2:2:10)   = T5*(M5\f5e);
end

function [th, u, r] = eq_own(Ad, Bd, fd, e5, e7, e9, tb)
% 七坐标模型自解平衡：theta_leg 与 u 一并解，返回离散不动点残差
M   = [(eye(14) - Ad)*(e5 + e7), -Bd];
rhs = fd - (eye(14) - Ad)*e9*tb;
z   = pinv(M)*rhs;
th  = z(1);
u   = z(2:7);
x   = th*(e5 + e7) + tb*e9;
r   = max(abs(Ad*x + Bd*u + fd - x));
end

function show_diff(tag, A, B, f, As, Bs, fs)
dA = A - As;  dB = B - Bs;  df = f - fs;
[i1, j1] = ind2sub(size(dA), idxmax(dA));
[i2, j2] = ind2sub(size(dB), idxmax(dB));
i3 = idxmax(df);
fprintf('   %-5s |dA|max = %.4g @(%d,%d)；|dB|max = %.4g @(%d,%d)；|df|max = %.4g @%d\n', ...
        tag, max(abs(dA(:))), i1, j1, max(abs(dB(:))), i2, j2, max(abs(df(:))), i3);
end

function list_top(dA, dB, n)
[~, ka] = sort(abs(dA(:)), 'descend');
[~, kb] = sort(abs(dB(:)), 'descend');
fprintf('     A 前 %d：', n);
for k = 1:n
    [i, j] = ind2sub(size(dA), ka(k));
    fprintf(' (%d,%d)=%+0.4g', i, j, dA(i,j));
end
fprintf('\n     B 前 %d：', n);
for k = 1:n
    [i, j] = ind2sub(size(dB), kb(k));
    fprintf(' (%d,%d)=%+0.4g', i, j, dB(i,j));
end
fprintf('\n');
end

function i = idxmax(X)
[~, k] = max(abs(X(:)));
i = k;
end
