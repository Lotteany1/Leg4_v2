% s08_tinympc14_runtime_check.m
% S8：运行时参考（LPV + 插件 + psi-A 前馈 + 4 ms 异步）闭环
% 判据：G18~G23 沿用 S6；G24 异步节点 / G25 LPV / G26 输出层 / G27 回退 ffwd
% 导出：solver/test/tinympc14_vectors.h、solver/test/tinympc14_rand_vectors.h、
%       solver/test/tinympc14_comp_vectors.h、build/tinympc14_solver_vectors.mat
% 版本：v0.1  日期：2026-10-20
% 用法：matlab -batch "cd('E:/DevelopLotteany1/0_Code/Leg4_v1_ultimate/Matlab/mpc/parts'); s08_tinympc14_runtime_check"

clear; clc
here = fileparts(mfilename('fullpath'));
addpath(here);
addpath(fileparts(here)); mixmpc_paths();
p = tinympc14_config();
NX = p.NX;  NU = p.NU;  NH = p.NH;  NN = NH - 1;
Qm = diag(p.Q_DIAG);  Rm = diag(p.R_DIAG);
umin = p.u_min(:);  umax = p.u_max(:);
rate6 = [Inf(NU-2,1); p.DF_RATE; p.DF_RATE];

%% ---- 单点 LQR 基线模型（与 S6 同口径）----
sp0 = tinympc14_sched_pt(p, 0.27, 0, 0);
[A0c, B0c, f0c] = tinympc14_fused_ab(p, p.sym7.sw_fused, sp0);
[Ad0, Bd0, fd0] = tinympc14_zoh_ab(A0c, B0c, f0c, p.Ts);
[xb0, uss0, ~] = tinympc14_equilibrium(p, p.sym7.sw_fused, sp0);
[Kinf0, ~, eig_cl] = tinympc14_dare_gain(Ad0, Bd0, Qm, Rm, p.rho, NX, NU);
Bd0_pinv = pinv(Bd0);

fprintf('==== S8 运行时闭环（Ts=%.1f ms，NH=%d，异步 4 ms 求解 / 2 ms 消费）====\n', ...
        p.Ts*1e3, NH);
fprintf('单点基线闭环谱半径 %.6f；LPV 调度点 0.27 m，阈值 0.005 m\n', eig_cl);

%% ---- 工况 ----
T_END = 3.0;
t = (0:round(T_END/p.Ts)-1)' * p.Ts;
Nt = numel(t);
S = tinympc14_scenarios(p);
nS = numel(S);

%% ---- 闭环 ----
R = struct('name', {}, 'ctrl', {}, 'x', {}, 'u_tot', {}, 'u_motor', {}, ...
           'u_bare', {}, 'iters', {}, 'fb', {}, 'eH', {}, 'eB', {}, ...
           'metrics', {}, 'lpv_up', {}, 'stale', {}, 'tp_err', {}, 'fm_err', {});
wall = tic;

for si = 1:nS
    for ci = 1:2
        sc = S(si);
        pl = sc.pl1;
        x = pl.xb;
        if ~isempty(sc.x0_alpha)
            x(11) = sc.x0_alpha;
        end
        u_prev = pl.uss;
        F_app_prev = pl.uss(5:6);
        T_w_prev = pl.uss(1:2);
        rt = [];
        if ci == 2
            rt = tinympc14_runtime_init(p);
        end

        log_x = zeros(NX, Nt);
        log_u_tot = zeros(NU, Nt);
        log_u_motor = zeros(NU, Nt);
        log_u_bare = zeros(NU, Nt);
        log_it = zeros(1, Nt);
        log_fb = zeros(1, Nt);
        log_eH = zeros(1, Nt);
        log_eB = zeros(1, Nt);
        log_tp = zeros(1, Nt);
        log_fm = zeros(1, Nt);

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

            % Reconstruct each leg from its measured sagittal angle on level ground.
            H = p.H_REF + x(13);
            hip_offset = p.R_TRK*sin(x(11));
            lL = (H+hip_offset)/(cos(x(11))*cos(x(5)));
            lR = (H-hip_offset)/(cos(x(11))*cos(x(7)));
            ffwd_ext = [zeros(NU-2,1); sc.ffwd(:)];

            if ci == 1
                c = tinympc14_output(p, lL, lR, x(5), x(7), x(2), x(4));
                ffwd6 = ffwd_ext;
                % Both controllers see the same known model-external tipping disturbance.
                fed = Bd0(:,5:6)*tinympc14_roll_force(p,p.H_REF+x(13),x(11),sp0.alpha,sp0.psi);
                u_ff = Bd0_pinv * (xref_k - Ad0*xref_k - fd0 - fed);
                u_shift = u_ff + ffwd6;
                f_eff = fd0 + fed + Bd0 * u_shift;
                u_prev_clip = min(max(u_prev, umin), umax);
                u_seed = min(max(u_prev_clip - u_shift, umin - u_shift), umax - u_shift);
                tot_lo = max(umin, u_prev_clip - rate6);
                tot_hi = min(umax, u_prev_clip + rate6);
                rhs_l = xref_k - Ad0*xref_k - f_eff;
                u_raw = Bd0_pinv * rhs_l - Kinf0 * (x - xref_k);
                u_bare = min(max(u_raw, max(umin - u_shift, u_seed - rate6)), ...
                                     min(umax - u_shift, u_seed + rate6));
                u_tot = min(max(u_bare + u_shift, tot_lo), tot_hi);
                u_bare = u_tot - u_shift;
                u_tot(5:6)=min(max(u_tot(5:6)+c.Fc,tot_lo(5:6)),tot_hi(5:6));
                u_plant = u_tot;
                u_motor = [u_tot(1:2); u_tot(3:4)+c.MgF; u_tot(5:6) - c.Fs];
                it_k = 0;
                fb_k = 0;
                tp_err = 0;
                fm_err = 0;
            else
                [rt, y] = tinympc14_runtime_step(rt, 2*(k-1), x, xref_k, lL, lR, ...
                                            x(5), x(7), x(2), x(4), ffwd_ext);
                u_tot = y.u_tot;
                u_bare = y.u_bare;
                u_motor = y.u_motor;
                u_plant = y.u_plant;
                it_k = y.iters;
                fb_k = y.fallback;
                tp_err = max(abs(y.Tp - (y.MgF + u_tot(3:4))));
                fm_err = max(abs(y.F_motor - (u_tot(5:6) - y.Fs)));
            end

            b6 = [zeros(NU-2,1); sc.b_force(:) * (tk >= sc.t_dist)];
            x = tinympc14_plant_step(p, pl, x, u_plant + b6);
            if abs(tk - sc.t_roll) < p.Ts/2
                x(12) = x(12) + sc.roll_imp;
            end

            log_x(:,k) = x;
            log_u_tot(:,k) = u_tot;
            log_u_motor(:,k) = u_motor;
            log_u_bare(:,k) = u_bare;
            log_it(k) = it_k;
            log_fb(k) = fb_k;
            log_eH(k) = x(13) - xref_k(13);
            log_eB(k) = max(abs([x(5)-xref_k(5), x(7)-xref_k(7), x(9)-xref_k(9), x(11)-xref_k(11)]));
            log_tp(k) = tp_err;
            log_fm(k) = fm_err;

            u_prev = u_tot;
            F_app_prev = u_tot(5:6);
            T_w_prev = u_tot(1:2);
        end

        itv = log_it(log_it > 0);
        m = metrics_of(log_x, log_u_tot, log_u_motor, itv, log_fb, log_eH, log_eB, p, rate6);
        R(end+1).name = sc.name;
        R(end).ctrl = ci;
        R(end).x = log_x;
        R(end).u_tot = log_u_tot;
        R(end).u_motor = log_u_motor;
        R(end).u_bare = log_u_bare;
        R(end).iters = itv;
        R(end).fb = log_fb;
        R(end).eH = log_eH;
        R(end).eB = log_eB;
        R(end).metrics = m;
        R(end).scen = si;
        R(end).tp_err = max(log_tp);
        R(end).fm_err = max(log_fm);
        if ci == 2
            R(end).lpv_up = rt.lpv_updates;
            R(end).stale = rt.stale;
        else
            R(end).lpv_up = 0;
            R(end).stale = 0;
        end
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
    fprintf('   %-18s 均 %.1f，p95 %d，峰 %d %s\n', ...
            R(i).name, m.it_mean, m.it_p95, m.it_max, pass_txt(ok));
end

fprintf('\n-- LPV / 异步计数（MPC）--\n');
for i = 1:numel(R)
    if R(i).ctrl ~= 2, continue; end
    fprintf('   %-18s LPV 更新 %d 次，stale %d 次\n', R(i).name, R(i).lpv_up, R(i).stale);
end

fprintf('\n-- alpha 箱应力例（记录，不入门禁）--\n');
a7 = find([R.scen] == 7 & [R.ctrl] == 2, 1);
m7 = R(a7).metrics;
fprintf('   实际 max|a| %.3f rad，回退 %.1f%%，均迭代 %.1f，峰 %d\n', ...
        m7.max_alpha, 100*m7.fallback, m7.it_mean, m7.it_max);

%% ---- G24 异步节点 ----
fprintf('\n-- G24 异步节点 / 超龄回退 --\n');
rt2 = tinympc14_runtime_init(p);
rt2.solve_period_ms = 1000;
x2 = xb0;
nodes = -ones(1, 15);
stale_tick = -1;
u_prev2 = rt2.u_prev;
u_fb_expect = zeros(NU,1);
u_fb_got = zeros(NU,1);
for k = 0:14
    [rt2, y2] = tinympc14_runtime_step(rt2, 2*k, x2, xb0, 0.27, 0.27, 0, 0, 0, 0);
    nodes(k+1) = y2.node;
    if y2.stale
        stale_tick = 2*k;
        u_fb_expect = min(max(y2.u_ss + y2.ffwd - rt2.model.Kinf*(x2 - xb0), ...
                              max(umin, min(max(u_prev2, umin), umax) - rate6)), ...
                          min(umax, min(max(u_prev2, umin), umax) + rate6));
        u_fb_got = y2.u_tot;
    end
    u_prev2 = y2.u_tot;
end
nodes_ok = all(nodes(1:7) == 0:6) && nodes(8) == -1 && stale_tick == 14;
g24 = nodes_ok && rt2.stale == 1 && rt2.node_repeat == 0;
fprintf('   节点序列 [%s]，首超龄拍 %d，stale 计数 %d，node_repeat %d %s\n', ...
        num2str(nodes), stale_tick, rt2.stale, rt2.node_repeat, pass_txt(g24));

%% ---- G25 LPV 更新 ----
fprintf('\n-- G25 LPV 阈值 / 钳位 / 清热启动 --\n');
rt3 = tinympc14_runtime_init(p);
warm_before = rt3.warm;
[rt3, ~] = tinympc14_runtime_step(rt3, 0, xb0, xb0, 0.27, 0.27, 0, 0, 0, 0);
n0 = rt3.lpv_updates;
[rt3, ~] = tinympc14_runtime_step(rt3, 2, xb0, xb0, 0.30, 0.30, 0, 0, 0, 0);
n1 = rt3.lpv_updates;
warm_after = rt3.warm;
[rt3, ~] = tinympc14_runtime_step(rt3, 4, xb0, xb0, 0.301, 0.301, 0, 0, 0, 0);
n2 = rt3.lpv_updates;
[rt3, ~] = tinympc14_runtime_step(rt3, 6, xb0, xb0, 0.10, 0.10, 0, 0, 0, 0);
n3 = rt3.lpv_updates;
g25 = n0 == 0 && n1 == 1 && n2 == 1 && n3 == 2 && isempty(warm_before) && ...
      isempty(warm_after) && abs(rt3.model.lL - double(single(p.LEG_RANGE(1)))) < 1e-12;
fprintf('   更新计数 %d/%d/%d/%d，更新后 warm 空 %s，钳位 lL=%.2f %s\n', ...
        n0, n1, n2, n3, pass_txt(isempty(warm_after)), rt3.model.lL, pass_txt(g25));

%% ---- G26 输出层恒等 ----
fprintf('\n-- G26 输出层恒等 --\n');
g26 = true;
tp_max = 0;  fm_max = 0;
for i = 1:numel(R)
    if R(i).ctrl ~= 2, continue; end
    tp_max = max(tp_max, R(i).tp_err);
    fm_max = max(fm_max, R(i).fm_err);
end
g26 = tp_max <= 1e-12 && fm_max <= 1e-12;
fprintf('   max|Tp−(MgF+u_tot)|=%.1e，max|F_motor−(u_tot−Fs)|=%.1e %s\n', ...
        tp_max, fm_max, pass_txt(g26));

%% ---- G27 回退 ffwd 合同 ----
fprintf('\n-- G27 回退 ffwd 合同 --\n');
d27 = max(abs(u_fb_got - u_fb_expect));
g27 = stale_tick >= 0 && d27 <= 1e-9;
fprintf('   stale 拍回退与 u_ss+ffwd−Kinf·(x−xref) 对拍 max|du|=%.1e（<=1e-9）%s\n', ...
        d27, pass_txt(g27));

%% ---- 同步链一致性（运行时 vs 直接链）----
fprintf('\n-- 同步链一致性（每拍求解，LPV 固定 0.27）--\n');
rt4 = tinympc14_runtime_init(p);
rt4.solve_period_ms = p.Ts*1e3;
ws4 = [];
u_prev4 = rt4.u_prev;
x4 = xb0;
d_sync = 0;
for k = 1:20
    c4 = tinympc14_output(p, 0.27, 0.27, 0, 0, 0, 0);
    fed4 = zeros(NX, 1);
    [u_ref4, ~, ws4] = tinympc14_solve_reference(p, 0.27, 0.27, x4, xb0, fed4, c4.ffwd, u_prev4, ws4);
    [rt4, y4] = tinympc14_runtime_step(rt4, 2*(k-1), x4, xb0, 0.27, 0.27, 0, 0, 0, 0);
    d_sync = max(d_sync, max(abs(y4.u_tot - u_ref4)));
    u_prev4 = y4.u_tot;
end
sync_ok = d_sync <= 1e-9;
fprintf('   两条路径 max|Δu_tot|=%.1e（<=1e-9）%s\n', d_sync, pass_txt(sync_ok));

%% ---- 汇总 ----
fprintf('\n==== S8 汇总（核心场景 1-6）====\n');
fprintf('   G18 %s | G19 %s | G20 %s | G21 %s | G22 %s | G23 %s\n', ...
        pass_txt(g18), pass_txt(g19), pass_txt(g20), pass_txt(g21), ...
        pass_txt(g22), pass_txt(g23));
fprintf('   G24 %s | G25 %s | G26 %s | G27 %s | 同步链 %s\n', ...
        pass_txt(g24), pass_txt(g25), pass_txt(g26), pass_txt(g27), pass_txt(sync_ok));
fprintf('   总墙钟 %.1f s（%d 场景 × 2 控制器）\n', wall_s, nS);

tinympc14_roll_contract_check(p);

assert(g18, 'S8 G18 未通过');
assert(g19, 'S8 G19 未通过');
assert(g20, 'S8 G20 未通过');
assert(g21, 'S8 G21 未通过');
assert(g22, 'S8 G22 未通过');
assert(g23, 'S8 G23 未通过');
assert(g24, 'S8 G24 未通过');
assert(g25, 'S8 G25 未通过');
assert(g26, 'S8 G26 未通过');
assert(g27, 'S8 G27 未通过');
assert(sync_ok, 'S8 同步链一致性未通过');

tinympc14_generate_vectors(xb0, uss0);
fprintf('S8 full runtime/closed-loop checks: PASS\n');

%% ========================================================================
function m = metrics_of(x, u_tot, u_motor, it, fb, eH, eB, p, rate6)
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
m.fallback = mean(fb);
if isempty(it)
    m.it_mean = 0;  m.it_p95 = 0;  m.it_max = 0;
else
    m.it_mean = mean(it);
    m.it_p95 = prctile_local(it, 95);
    m.it_max = max(it);
end
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
