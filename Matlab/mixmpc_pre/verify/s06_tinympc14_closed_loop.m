% s06_tinympc14_closed_loop.m
% S6：14/6 融合 MPC 闭环仿真（真模型 vs 单点调度模型 + psi-A 前馈）
% 对照：同模型 C11 LQR（部署链非 MPC 路径的等价物）
% 工况：H 阶跃、psi 阶跃、横滚角速度冲激、持续力扰动、非对称腿长、已知 ffwd 等价
%       alpha 箱应力例只记录不入门禁
% 判据：G18 不发散/NaN、G19 H 跟踪、G20 横滚峰值、G21 箱/速率/力矩、G22 回退率、G23 迭代
%
% 版本：v0.3  日期：2026-10-20
% v0.2：三笔补偿由 tinympc14_output 插件接在模型外——MgF 进 plant 髋力矩、Fs 折输出电机力、
%       向心 ffwd 并进 MPC 输入平移量；场景 6 用常值前馈压箱平移与速率门。
% 用法：matlab -batch "cd('E:/DevelopLotteany1/0_Code/Leg4_v1_ultimate/Matlab/mpc/parts'); s06_tinympc14_closed_loop"

clear; clc
here = fileparts(mfilename('fullpath'));
addpath(here);
addpath(fileparts(here)); mixmpc_paths();   % mpc 顶层：tinympc14_config

p = tinympc14_config();
NX = p.NX;  NU = p.NU;  NH = p.NH;  NN = NH - 1;
Qm = diag(p.Q_DIAG);  Rm = diag(p.R_DIAG);

%% ---- 控制器单点模型（l=0.27、psi=0）----
sp0 = tinympc14_sched_pt(p, 0.27, 0, 0);
[A0c, B0c, f0c] = tinympc14_fused_ab(p, p.sym7.sw_fused, sp0);
[Ad0, Bd0, fd0] = tinympc14_zoh_ab(A0c, B0c, f0c, p.Ts);
[xb0, uss0, ~] = tinympc14_equilibrium(p, p.sym7.sw_fused, sp0);
[Kinf, Pinf, eig_cl] = tinympc14_dare_gain(Ad0, Bd0, Qm, Rm, p.rho, NX, NU);
QuuInv = inv(Rm + p.rho*eye(NU) + Bd0'*Pinf*Bd0);
AmBKt = (Ad0 - Bd0*Kinf)';
mdl = struct('Ad', Ad0, 'Bd', Bd0, 'Kinf', Kinf, 'Pinf', Pinf, 'QuuInv', QuuInv, ...
             'AmBKt', AmBKt, 'q_diag', p.Q_DIAG(:), 'r_diag', p.R_DIAG(:), ...
             'u_min', p.u_min(:), 'u_max', p.u_max(:), ...
             'x_min', p.x_min(:), 'x_max', p.x_max(:), 'Bd_pinv', pinv(Bd0));
rate6 = [Inf(NU-2,1); p.DF_RATE; p.DF_RATE];
Fz = zeros(NX);
for j = 1:NX
    ej = zeros(NX, 1);  ej(j) = 1;
    [~, ~, col] = tinympc14_zoh_ab(A0c, [], ej, p.Ts);
    Fz(:, j) = col;
end

fprintf('==== S6 闭环（Ts=%.1f ms，NH=%d，rho=%g，闭环谱半径 %.6f）====\n', ...
        p.Ts*1e3, NH, p.rho, eig_cl);

% 插件基线自检（MgF / Fs / 向心三笔与 balance.c 口径对照）
c0 = tinympc14_output(p, 0.27, 0.27, 0, 0, 0, 0);
c1 = tinympc14_output(p, 0.27, 0.27, 0, 0, 1.0, 1.0);
cn = tinympc14_output(p, NaN, 0.27, 0, 0, 0, 0);
assert(abs(c1.Fs(1) - Leg_SpringF(double(0.27))) < 1e-4);
assert(all(c1.ffwd==0)); % physical compensation is outside the optimizer
assert(all(cn.ffwd == 0));
assert(cn.Fs(1) == 0 && cn.MgF(1) == 0 && cn.st(2,1) == 0);
assert(abs(cn.Fs(2) - Leg_SpringF(double(0.27))) < 1e-4 && cn.st(2,2) == 1);
fprintf('插件自检：MgF(0) = %.4f N·m，Fs(0.27) = %.2f N，Fc(v=w=1) = [%.2f %.2f] N（限幅 %.0f）\n', ...
        c0.MgF(1), c0.Fs(1), c1.Fc(1), c1.Fc(2), p.COMP.ROLL_CLAMP);
fprintf('   向心按全轮距全值；现役 Leg_RollComp.c 分母 0.8 只给一半（报告 #6b）\n');

%% ---- 工况定义 ----
T_END = 3.0;
t = (0:round(T_END/p.Ts)-1)' * p.Ts;   % 本拍开始时刻 s
Nt = numel(t);

S = tinympc14_scenarios(p);

%% ---- 闭环 ----
R = struct('name', {}, 'ctrl', {}, 'x', {}, 'u_tot', {}, 'u_bare', {}, ...
           'iters', {}, 'conv', {}, 'eH', {}, 'metrics', {});
nS = numel(S);
nC = 2;   % 1 = C11 LQR 基线，2 = ADMM MPC
wall = tic;

for si = 1:nS
    for ci = 1:nC
        sc = S(si);
        pl = sc.pl1;
        x = pl.xb;
        if ~isempty(sc.x0_alpha)
            x(11) = sc.x0_alpha;
        end
        u_prev = pl.uss;
        F_app_prev = pl.uss(5:6);
        T_w_prev = pl.uss(1:2);
        ws = struct('v', zeros(NX,NH), 'g', zeros(NX,NH), ...
                    'z', zeros(NU,NN), 'y', zeros(NU,NN));

        log_x = zeros(NX, Nt);
        log_u_tot = zeros(NU, Nt);
        log_u_motor = zeros(NU, Nt);
        log_u_bare = zeros(NU, Nt);
        log_it = zeros(1, Nt);
        log_cv = zeros(1, Nt);
        log_eH = zeros(1, Nt);
        log_eB = zeros(1, Nt);
        log_vmax = zeros(1, Nt);

        for k = 1:Nt
            tk = t(k);
            if isfinite(sc.t_sw) && tk >= sc.t_sw
                pl = sc.pl2;
            end
            if strcmp(sc.xref_mode, 'true')
                xref_k = pl.xb;
            else
                xref_k = xb0;
            end
            xref_k(13) = xref_k(13) + sc.H_step * (tk >= sc.t_h0 && tk < sc.t_h1);

            if isfinite(sc.t_sw) && tk >= sc.t_sw
                psi_e = sc.psi2;
            else
                psi_e = sc.psi1;
            end

            Hx = p.H_REF+x(13);
            Lmid = Hx / cos(x(11));
            c = tinympc14_output(p, Lmid + p.R_TRK*x(11), Lmid - p.R_TRK*x(11), ...
                             x(5), x(7), x(2), x(4));
            ffwd6 = [zeros(NU-2,1); sc.ffwd(:)];

            fe = tinympc14_f_extra(p, F_app_prev, T_w_prev, psi_e, psi_e, 0);
            fed = Fz * fe;
            u_ff = mdl.Bd_pinv * (xref_k - Ad0*xref_k - fd0 - fed);
            u_shift = u_ff + ffwd6;
            f_eff = fd0 + fed + Bd0 * u_shift;

            u_prev_clip = min(max(u_prev, p.u_min(:)), p.u_max(:));
            u_seed = min(max(u_prev_clip - u_shift, p.u_min(:) - u_shift), ...
                         p.u_max(:) - u_shift);
            tot_lo = max(p.u_min(:), u_prev_clip - rate6);
            tot_hi = min(p.u_max(:), u_prev_clip + rate6);

            if ci == 1
                rhs_l = xref_k - Ad0*xref_k - f_eff;
                u_raw = mdl.Bd_pinv * rhs_l - Kinf * (x - xref_k);
                u_bare = min(max(u_raw, max(p.u_min(:) - u_shift, u_seed - rate6)), ...
                                   min(p.u_max(:) - u_shift, u_seed + rate6));
                u_tot = min(max(u_bare + u_shift, tot_lo), tot_hi);
                u_bare = u_tot - u_shift;
                it_k = 0;  cv_k = 1;
            else
                mdl_s = mdl;
                mdl_s.u_min = p.u_min(:) - u_shift;
                mdl_s.u_max = p.u_max(:) - u_shift;
                Pf = Pinf * f_eff;
                opts = struct('fd', f_eff, 'APf', AmBKt*Pf, 'BPf', Bd0'*Pf, ...
                              'df_rate', rate6, 'u_prev', u_seed, ...
                              'fallback', 'uss', 'fb_ffwd', ffwd6, 'return_proj', true);
                [u_bare, info] = tinympc14_admm(mdl_s, x, repmat(xref_k,1,NH), zeros(NU,NN), p, ws, opts);
                u_tot = min(max(u_bare + u_shift, tot_lo), tot_hi);
                u_bare = u_tot - u_shift;
                ws = struct('v', info.v, 'g', info.g, 'z', info.z, 'y', info.y);
                it_k = info.iters;  cv_k = info.converged;
                log_vmax(k) = max(info.v(11, 2:NH));
            end

            b6 = [zeros(NU-2,1); sc.b_force(:) * (tk >= sc.t_dist)];
            u_tot(5:6)=min(max(u_tot(5:6)+c.Fc,tot_lo(5:6)),tot_hi(5:6));
            u_plant = u_tot;
            u_motor = [u_tot(1:2); u_tot(3:4)+c.MgF; u_tot(5:6) - c.Fs];
            x = pl.Adp * x + pl.Bdp * (u_plant + b6) + pl.fdp;
            if abs(tk - sc.t_roll) < p.Ts/2
                x(12) = x(12) + sc.roll_imp;
            end

            log_x(:,k) = x;
            log_u_tot(:,k) = u_tot;
            log_u_motor(:,k) = u_motor;
            log_u_bare(:,k) = u_bare;
            log_it(k) = it_k;
            log_cv(k) = cv_k;
            log_eH(k) = x(13) - xref_k(13);
            log_eB(k) = max(abs([x(5)-xref_k(5), x(7)-xref_k(7), x(9)-xref_k(9), x(11)-xref_k(11)]));

            u_prev = u_tot;
            F_app_prev = u_tot(5:6);
            T_w_prev = u_tot(1:2);
        end

        m = metrics_of(log_x, log_u_tot, log_u_motor, log_it, log_cv, log_eH, log_eB, p, rate6);
        R(end+1).name = sc.name;
        R(end).ctrl = ci;
        R(end).x = log_x;
        R(end).u_tot = log_u_tot;
        R(end).u_motor = log_u_motor;
        R(end).u_bare = log_u_bare;
        R(end).iters = log_it;
        R(end).conv = log_cv;
        R(end).eH = log_eH;
        R(end).eB = log_eB;
        R(end).vmax = log_vmax;
        R(end).metrics = m;
        R(end).ffwd = sc.ffwd;
        R(end).scen = si;
    end
end
wall_s = toc(wall);

%% ---- 打印 ----
fprintf('\n%-18s %-4s %9s %10s %9s %9s %10s %10s %8s %8s\n', ...
        '工况', 'ctrl', 'max|eH|', 'rms(eH)', 'max|eB|', 'rms(eB)', 'max|a|', 'maxF', '回退%', '均迭代');
for i = 1:numel(R)
    m = R(i).metrics;
    fprintf('%-18s %-4s %9.4f %10.5f %9.4f %9.5f %10.4f %10.2f %7.1f%% %8.1f\n', ...
            R(i).name, ctrl_txt(R(i).ctrl), m.max_eH, m.rms_eH, m.max_eB, m.rms_eB, ...
            m.max_alpha, m.max_F, 100*m.fallback, m.it_mean);
end

fprintf('\n-- 场景 1 H 轨迹抽样（t: H_LQR H_MPC F_LQR F_MPC）--\n');
a1 = find([R.scen] == 1 & [R.ctrl] == 1, 1);
b1 = find([R.scen] == 1 & [R.ctrl] == 2, 1);
for tt = [0.4 0.5 0.55 0.6 0.8 1.0 1.5 1.9 2.0 2.05 2.2 2.5 2.9]
    k = find(t >= tt - 1e-12, 1);
    fprintf('   %6.2f s  %8.4f %8.4f  %8.3f %8.3f\n', t(k), ...
            R(a1).x(13,k), R(b1).x(13,k), R(a1).u_tot(5,k), R(b1).u_tot(5,k));
end

fprintf('\n-- G18 不发散/NaN --\n');
g18 = true;
for i = 1:numel(R)
    if R(i).scen == 7, continue; end
    m = R(i).metrics;
    ok = m.finite && m.max_absH < 1.00 && m.max_alpha < 1.0;
    g18 = g18 && ok;
    fprintf('   %-18s %-4s 有限 %s，max|H|=%.3f m，max|a|=%.3f rad %s\n', ...
            R(i).name, ctrl_txt(R(i).ctrl), pass_txt(m.finite), m.max_absH, m.max_alpha, pass_txt(ok));
end

fprintf('\n-- G19 H 跟踪（MPC <= LQR + 20%%）--\n');
g19 = true;
for si = 1:nS
    if si == 7, continue; end
    a = find([R.scen] == si & [R.ctrl] == 1, 1);
    b = find([R.scen] == si & [R.ctrl] == 2, 1);
    if si == 5
        ea = R(a).metrics.max_eB;  eb = R(b).metrics.max_eB;
        ra = R(a).metrics.rms_eB;  rb = R(b).metrics.rms_eB;
        lab = '平衡态';
    else
        ea = R(a).metrics.max_eH;  eb = R(b).metrics.max_eH;
        ra = R(a).metrics.rms_eH;  rb = R(b).metrics.rms_eH;
        lab = 'H';
    end
    ok = eb <= 1.2*ea + 1e-9 && rb <= 1.2*ra + 1e-9;
    g19 = g19 && ok;
    fprintf('   %-18s %s max %.4f / %.4f，rms %.5f / %.5f（LQR/MPC）%s\n', ...
            R(a).name, lab, ea, eb, ra, rb, pass_txt(ok));
end

fprintf('\n-- G20 横滚峰值（MPC <= LQR + 20%%）--\n');
a = find([R.scen] == 3 & [R.ctrl] == 1, 1);
b = find([R.scen] == 3 & [R.ctrl] == 2, 1);
g20 = R(b).metrics.max_alpha <= 1.2*R(a).metrics.max_alpha + 1e-9;
fprintf('   横滚冲激 max|a| %.4f / %.4f rad（LQR/MPC）%s\n', ...
        R(a).metrics.max_alpha, R(b).metrics.max_alpha, pass_txt(g20));

fprintf('\n-- G21 力箱/速率/力矩钳位 --\n');
g21 = true;
for i = 1:numel(R)
    m = R(i).metrics;
    ok = m.vios_box == 0 && m.vios_rate == 0 && m.max_tau <= max(p.u_max(1:4)) + 1e-9;
    g21 = g21 && ok;
    fprintf('   %-18s %-4s 箱越界 %d，速率越界 %d，max|T_mot|=%.3f N·m %s\n', ...
            R(i).name, ctrl_txt(R(i).ctrl), m.vios_box, m.vios_rate, m.max_tau, pass_txt(ok));
end

fprintf('\n-- alpha 箱应力例（记录，不入门禁）--\n');
a7 = find([R.scen] == 7 & [R.ctrl] == 2, 1);
vmax7 = max(R(a7).vmax);
nact7 = sum(R(a7).vmax >= p.x_max(11) - 1e-6);
m7 = R(a7).metrics;
fprintf('   v(11) 峰 %.6f rad，投影夹紧拍数 %d，实际 max|a| %.3f rad\n', ...
        vmax7, nact7, m7.max_alpha);
fprintf('   回退 %.1f%%，均迭代 %.1f，峰 %d\n', ...
        100*m7.fallback, m7.it_mean, m7.it_max);
if p.A_MAX >= 1e5, fprintf('   当前预测状态箱已关闭。\n'); end

fprintf('\n-- G22 回退率（MPC <= 5%%）--\n');
g22 = true;
for i = 1:numel(R)
    if R(i).ctrl ~= 2 || R(i).scen == 7, continue; end
    ok = R(i).metrics.fallback <= 0.05;
    g22 = g22 && ok;
    fprintf('   %-18s 回退 %.2f%% %s\n', R(i).name, 100*R(i).metrics.fallback, pass_txt(ok));
end

fprintf('\n-- G23 迭代预算 --\n');
g23 = true;
for i = 1:numel(R)
    if R(i).ctrl ~= 2 || R(i).scen == 7, continue; end
    m = R(i).metrics;
    ok = m.it_mean <= 50 && m.it_max <= p.max_iter;
    g23 = g23 && ok;
    fprintf('   %-18s 均 %.1f，p95 %d，峰 %d %s\n', R(i).name, m.it_mean, m.it_p95, m.it_max, pass_txt(ok));
end
fprintf('   总墙钟 %.1f s（%d 次 MPC 求解）\n', wall_s, nS*Nt);

fprintf('\n-- 常值前馈 INFO（场景 6 vs 场景 1）--\n');
a1 = find([R.scen] == 1 & [R.ctrl] == 1, 1);
a2 = find([R.scen] == 1 & [R.ctrl] == 2, 1);
b1 = find([R.scen] == 6 & [R.ctrl] == 1, 1);
b2 = find([R.scen] == 6 & [R.ctrl] == 2, 1);
d_lqr = max(max(abs(R(b1).x - R(a1).x)));
d_mpc = max(max(abs(R(b2).x - R(a2).x)));
d_tot = max(max(abs(R(b2).u_tot - R(a2).u_tot)));
f6 = [zeros(4,1); S(6).ffwd(:)];
d_shift = max(max(abs(R(b2).u_bare - R(a2).u_bare + repmat(f6, 1, Nt))));
fprintf('   LQR max|dx|=%.2e（仿射律不变）；MPC max|dx|=%.2e，max|du_tot|=%.2e\n', ...
        d_lqr, d_mpc, d_tot);
fprintf('   MPC 变量位移 max|du_bare+ffwd|=%.2e N（R 代价罚前馈前变量，参考点随 ffwd 移动）\n', d_shift);

%% ---- 汇总 ----
fprintf('\n==== S6 汇总（核心场景 1-6）====\n');
fprintf('   G18 %s | G19 %s | G20 %s | G21 %s | G22 %s | G23 %s\n', ...
        pass_txt(g18), pass_txt(g19), pass_txt(g20), pass_txt(g21), ...
        pass_txt(g22), pass_txt(g23));
fprintf('   alpha 箱应力例：OPEN（状态约束不可行/收敛差，见报告）\n');

assert(g18, 'S6 G18 未通过');
assert(g19, 'S6 G19 未通过');
assert(g20, 'S6 G20 未通过');
assert(g21, 'S6 G21 未通过');
assert(g22, 'S6 G22 未通过');
assert(g23, 'S6 G23 未通过');
fprintf('S6：核心六场景闭环通过；alpha 箱应力例为已知开口，S7 前需裁定；Wheel_Leg 零改动\n');

%% ========================================================================
function m = metrics_of(x, u_tot, u_motor, it, cv, eH, eB, p, rate6)
m.finite = all(isfinite(x(:))) && all(isfinite(u_tot(:)));
m.max_absH = max(abs(x(13,:)));
m.max_alpha = max(abs(x(11,:)));
m.max_eH = max(abs(eH));
m.rms_eH = sqrt(mean(eH.^2));
m.max_eB = max(abs(eB));
m.rms_eB = sqrt(mean(eB.^2));
m.max_F = max(u_tot(5:6,:), [], 'all');
m.min_F = min(u_tot(5:6,:), [], 'all');
m.max_tau = max(max(abs(u_motor(1:4,:))));
m.fallback = mean(~cv);
m.it_mean = mean(it);
m.it_p95 = prctile_local(it, 95);
m.it_max = max(it);
d = abs(diff(u_tot, 1, 2));
m.vios_rate = sum(any(d > rate6 + 1e-9, 1));
m.vios_box = sum(any(u_tot < p.u_min(:) - 1e-9 | u_tot > p.u_max(:) + 1e-9, 1));
end

function v = prctile_local(x, q)
y = sort(x(:));
if isempty(y)
    v = 0;
    return;
end
i = max(1, min(numel(y), ceil(q/100*numel(y))));
v = y(i);
end

function s = ctrl_txt(ci)
if ci == 1
    s = 'LQR';
else
    s = 'MPC';
end
end

function s = pass_txt(ok)
if ok
    s = 'PASS';
else
    s = 'FAIL';
end
end
