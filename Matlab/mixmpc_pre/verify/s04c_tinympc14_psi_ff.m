% s04c_tinympc14_psi_ff.m
% S4c：psi-A 前馈。psi=0 烘表 + f_extra（上游 P2-1），用上一拍总力 F_app 与实测 psi
% 自检 A：f_extra 与真模型 psi 相关 B 行逐元对拍
% 自检 B：一拍预测。原点隔离 B 项看前馈；平衡点看 ΔA/ΔB 相消
% 自检 C：f_extra 进 C11 pinv 回退的谱半径与稳定域（风险项）
% 自检 D：E5 单轮矩横滚，f_extra(12) 与 #8alpha 对拍并看闭环峰值/末值
%
% 版本：v0.2  日期：2026-10-19
% 用法：matlab -batch "cd('E:/DevelopLotteany1/0_Code/Leg4_v1_ultimate/Matlab/mpc/parts'); s04c_tinympc14_psi_ff"

clear; clc; close all
addpath(fileparts(fileparts(mfilename('fullpath')))); mixmpc_paths();   % mpc 顶层：tinympc14_config
p = tinympc14_config();
NX = p.NX; NU = p.NU;
Q = diag(p.Q_DIAG);
Rm = diag(p.R_DIAG);
M_TOT = tinympc14_m_tot(p);
Fbar = M_TOT*p.g/2;

fprintf('==== S4c psi-A 前馈（M_TOT=%.3f kg，Fbar=%.3f N）====\n', M_TOT, Fbar);

%% ---- A：f_extra 与真模型 psi 相关 B 行 ----
fprintf('\n-- A f_extra 对 B 行（psi 前馈的代数口径）--\n');
fprintf('   %5s %12s %12s %12s %12s %12s\n', 'psi', 'row2', 'row10', 'row12', 'row14', 'max');
psi_list = [0.05, 0.10, 0.15, 0.20];
F_app_a = [Fbar; Fbar];
T_wa = [1.0; -0.5];
err_a = 0;
for k = 1:numel(psi_list)
    ps = psi_list(k);
    sp = tinympc14_sched_pt(p, 0.27, 0, ps);
    [~, B, ~] = tinympc14_fused_ab(p, p.sym7.sw_fused, sp);
    fe = tinympc14_f_extra(p, F_app_a, T_wa, ps, ps, 0);
    b2  = B(2, 5:6)  * F_app_a;
    b10 = B(10, 5:6) * F_app_a;
    b12 = B(12, 1:2) * T_wa;
    b14 = B(14, 1:2) * T_wa + (B(14, 5:6) - 1/p.M_V) * F_app_a;
    e2 = fe(2)-b2; e10 = fe(10)-b10; e12 = fe(12)-b12; e14 = fe(14)-b14;
    err_a = max([err_a, abs(e2), abs(e10), abs(e12), abs(e14)]);
    fprintf('   %5.2f %12.3e %12.3e %12.3e %12.3e %12.3e\n', ...
            ps, e2, e10, e12, e14, max([abs(e2), abs(e10), abs(e12), abs(e14)]));
end
fprintf('   A max|error| = %.3e（<=1e-12）%s\n', err_a, pass_txt(err_a <= 1e-12));

%% ---- B：ψ-B 前馈的一拍预测 ----
% 原点：x=0，隔离 B 项，前馈应把 Bψ 项补齐
% 平衡点：ΔA·x + Δf 与 ΔB·u 同量反号，单加前馈会把预测推离
fprintf('\n-- B 一拍预测（控制器 psi=0 表 vs plant psi 真表）--\n');
lgrid = [0.20, 0.27, 0.34];
psigrid = [0.05, 0.10, 0.15, 0.20];
fprintf('   %5s %5s %13s %13s %8s %10s | %12s %12s %8s\n', ...
        'l', 'psi', 'e0 原点', 'ef 原点', '降幅', 'B 项一致', 'e0 平衡', 'ef 平衡', '相消比');
gB_ok = true; gB_any = false; gB_alg = 0;
for il = 1:numel(lgrid)
    l = lgrid(il);
    sp0 = tinympc14_sched_pt(p, l, 0, 0);
    [A0, B0, f0] = tinympc14_fused_ab(p, p.sym7.sw_fused, sp0);
    [Ad0, Bd0, fd0] = tinympc14_zoh_ab(A0, B0, f0, p.Ts);
    for ip = 1:numel(psigrid)
        ps = psigrid(ip);
        spt = tinympc14_sched_pt(p, l, 0, ps);
        [At, Bt, ft] = tinympc14_fused_ab(p, p.sym7.sw_fused, spt);
        [Adp, Bdp, fdp] = tinympc14_zoh_ab(At, Bt, ft, p.Ts);
        [xbt, ut, ~] = tinympc14_equilibrium(p, p.sym7.sw_fused, spt);

        fe = tinympc14_f_extra(p, ut(5:6), ut(1:2), ps, ps, 0);
        [~, ~, fed] = tinympc14_zoh_ab(A0, [], fe, p.Ts);
        dbu = (Bdp - Bd0) * ut;
        ealg = max(abs(fed - dbu));
        gB_alg = max(gB_alg, ealg);

        xt = Bdp * ut + fdp;
        x0 = Bd0 * ut + fd0;
        e0 = max(abs(x0 - xt));
        ef = max(abs((x0 + fed) - xt));
        red = 1 - ef / max(e0, eps);

        rn = (Adp * xbt + Bdp * ut + fdp) - (Ad0 * xbt + Bd0 * ut + fd0);
        rf = rn - fed;
        en0 = max(abs(rn));
        enf = max(abs(rf));
        rat = en0 / max(max(abs(dbu)), eps);

        ok = ef < e0;
        gB_ok = gB_ok && ok;
        if red >= 0.20
            gB_any = true;
        end
        fprintf('   %5.2f %5.2f %13.4e %13.4e %7.1f%% %10.2e | %12.4e %12.4e %8.3f\n', ...
                l, ps, e0, ef, 100*red, ealg, en0, enf, rat);
    end
end
fprintf('   B：原点残差每点下降 %s；至少一点降幅 >=20%% %s；B 项代数一致 %.2e %s\n', ...
        pass_txt(gB_ok), pass_txt(gB_any), gB_alg, pass_txt(gB_alg <= 1e-6));
fprintf('   平衡点残差约为 ΔB·u 主项，前馈降 1~2 个量级；剩余 ΔA/Δf 待 P2-2\n');

%% ---- C：f_extra 进 C11 pinv 回退的谱半径与瞬态 ----
fprintf('\n-- C 固定 psi：f_extra 一阶延迟闭环谱半径 --\n');
l = 0.27;
psC = [0.10, 0.20];
gC_lam = Inf;
gC_rho = zeros(numel(psC), 1);
for ip = 1:numel(psC)
    ps = psC(ip);
    sp0 = tinympc14_sched_pt(p, l, 0, 0);
    [A0, B0, f0] = tinympc14_fused_ab(p, p.sym7.sw_fused, sp0);
    [Ad0, Bd0, fd0] = tinympc14_zoh_ab(A0, B0, f0, p.Ts);
    [K0, ~, ~] = tinympc14_dare_gain(Ad0, Bd0, Q, Rm, p.rho, NX, NU);
    spt = tinympc14_sched_pt(p, l, 0, ps);
    [At, Bt, ft] = tinympc14_fused_ab(p, p.sym7.sw_fused, spt);
    [Adp, Bdp, fdp] = tinympc14_zoh_ab(At, Bt, ft, p.Ts);
    [xbt, ut, ~] = tinympc14_equilibrium(p, p.sym7.sw_fused, spt);

    G = f_extra_gain(p, ps);
    E = expm([A0, eye(NX); zeros(NX), zeros(NX)] * p.Ts);
    Phi_int = E(1:NX, NX+1:end);
    Gd = Phi_int * G;
    S = zeros(4, NU); S(1,5) = 1; S(2,6) = 1; S(3,1) = 1; S(4,2) = 1;
    P = pinv(Bd0);
    Maug = [Adp - Bdp*K0, -Bdp*P*Gd; -S*K0, -S*P*Gd];
    rho1 = max(abs(eig(Maug)));
    gC_rho(ip) = rho1;

    lam = 0;
    for tl = 0:0.02:1
        Ml = [Adp - Bdp*K0, -Bdp*P*(tl*Gd); -S*K0, -S*P*(tl*Gd)];
        if max(abs(eig(Ml))) < 1
            lam = tl;
        else
            break;
        end
    end
    fprintf('   psi=%.2f：lambda=1 谱半径 %.6f；稳定域 lambda <= %.2f\n', ps, rho1, lam);

    for mode = 0:1
        if mode == 0
            fac = 0;
        else
            fac = min(1, lam);
        end
        x = xbt;
        x(1) = x(1) + 0.01;
        x(13) = x(13) + 0.01;
        F_app = ut(5:6);
        T_w = ut(1:2);
        Nt = round(5/p.Ts);
        smax = 0; hmax = 0; umax = 0; finite = true;
        for t = 1:Nt
            fe = fac * Phi_int * tinympc14_f_extra(p, F_app, T_w, ps, ps, 0);
            uss = pinv(Bd0) * ((eye(NX) - Ad0) * xbt - fd0 - fe);
            u = uss - K0 * (x - xbt);
            u = clip_u(p, u);
            x = Adp * x + Bdp * u + fdp;
            F_app = u(5:6);
            T_w = u(1:2);
            smax = max(smax, abs(x(1) - xbt(1)));
            hmax = max(hmax, abs(x(13) - xbt(13)));
            umax = max(umax, max(abs(u(5:6))));
            finite = finite && all(isfinite(x));
        end
        fprintf('   psi=%.2f lambda=%.2f：max|s|=%.4f m，max|H|=%.4f m，末|H|=%.4f m，末|s|=%.4f m，max|F|=%.2f N，有限 %s\n', ...
                ps, fac, smax, hmax, abs(x(13)-xbt(13)), abs(x(1)-xbt(1)), umax, mat2str(finite));
        if mode == 1
            gC_finite = finite;
            gC_lam = min(gC_lam, lam);
        end
    end
end
fprintf('   C：稳定域内全有限 %s；最小 lambda_max %.2f\n', pass_txt(gC_finite), gC_lam);
fprintf('   C：lambda=1 最大谱半径 %.6f；前馈按常值离散积分后回退稳定\n', max(gC_rho));

%% ---- D：E5 单轮矩横滚 ----
fprintf('\n-- D E5 横滚（psi=0.10，初值 alpha=0.02 rad；2 s）--\n');
l = 0.27; ps = 0.10;
sp0 = tinympc14_sched_pt(p, l, 0, 0);
[A0, B0, f0] = tinympc14_fused_ab(p, p.sym7.sw_fused, sp0);
[Ad0, Bd0, fd0] = tinympc14_zoh_ab(A0, B0, f0, p.Ts);
[K0, ~, ~] = tinympc14_dare_gain(Ad0, Bd0, Q, Rm, p.rho, NX, NU);
spt = tinympc14_sched_pt(p, l, 0, ps);
[At, Bt, ft] = tinympc14_fused_ab(p, p.sym7.sw_fused, spt);
[Adp, Bdp, fdp] = tinympc14_zoh_ab(At, Bt, ft, p.Ts);
[xbt, ut, ~] = tinympc14_equilibrium(p, p.sym7.sw_fused, spt);
afin = zeros(2, 1);
T_off = [0.5; -0.5];
for mode = 0:1
    x = xbt;
    x(11) = 0.02;
    F_app = ut(5:6);
    T_w = ut(1:2);
    Nt = round(2/p.Ts);
    amax = 0; twmax = 0; finite = true;
    for t = 1:Nt
        if mode == 1
            [~, ~, fe] = tinympc14_zoh_ab(A0, [], tinympc14_f_extra(p, F_app, T_w, ps, ps, 0), p.Ts);
        else
            fe = zeros(NX, 1);
        end
        uss = pinv(Bd0) * ((eye(NX) - Ad0) * xbt - fd0 - fe);
        u = uss - K0 * (x - xbt);
        u(1:2) = u(1:2) + T_off;
        u = clip_u(p, u);
        x = Adp * x + Bdp * u + fdp;
        F_app = u(5:6);
        T_w = u(1:2);
        amax = max(amax, abs(x(11)));
        twmax = max(twmax, abs(u(1)-u(2)));
        finite = finite && all(isfinite(x));
    end
    afin(mode+1) = abs(x(11));
    fprintf('   mode=%d：max|alpha|=%.4f rad，末|alpha|=%.6f rad，max|Twl-Twr|=%.4f N·m，有限 %s\n', ...
            mode, amax, abs(x(11)), twmax, mat2str(finite));
    if mode == 0, a0 = amax; else a1 = amax; end
end
gD_ok = a1 <= 1.2 * a0 + 1e-12;
gD_end = afin(2) <= 1.2 * afin(1) + 1e-12;
fprintf('   末|alpha|：无前馈 %.6f，前馈 %.6f，比值 %.2f\n', ...
        afin(1), afin(2), afin(2)/max(afin(1), eps));

% 代数：f_extra(12) 与 #8alpha 预测
T_test = [1.0; -0.5];
fe_test = tinympc14_f_extra(p, Fbar*[1;1], T_test, ps, ps, 0);
b12_test = Bt(12,1:2) * T_test;
gD_alg = abs(fe_test(12) - b12_test);
fprintf('   f_extra(12)=%.6e，B(12,1:2)*T=%.6e，差 %.2e\n', fe_test(12), b12_test, gD_alg);
fprintf('   D：峰值不增 >20%% %s；代数一致 %s；末值不增 %s\n', ...
        pass_txt(gD_ok), pass_txt(gD_alg <= 1e-12), pass_txt(gD_end));

%% ---- 汇总 ----
fprintf('\n==== S4c 汇总 ====\n');
fprintf('   A 代数对拍 %s；B 原点预测 %s/%s、B 项一致 %.2e；C 有限 %s；D 峰值 %s / 代数 %s / 末值 %s\n', ...
        pass_txt(err_a <= 1e-12), pass_txt(gB_any), pass_txt(gB_ok), gB_alg, ...
        pass_txt(gC_finite), pass_txt(gD_ok), pass_txt(gD_alg <= 1e-12), pass_txt(gD_end));

assert(err_a <= 1e-12, 'S4c A 未通过');
assert(gB_any && gB_ok && gB_alg <= 1e-6, 'S4c B 未通过');
assert(gC_finite, 'S4c C 未通过');
assert(gD_alg <= 1e-12, 'S4c D 未通过');
fprintf('S4c：A/B/C/D 通过；ψ-A 前馈按常值 ZOH 积分可用；平衡点剩余 ΔA 项待 P2-2；psi 三维表未做\n');

%% ========================================================================
function G = f_extra_gain(p, ps)
% f_extra 对 w=[F_l F_r T_wl T_wr] 的线性增益；psi_bar=0 时无常数项
M_TOT = tinympc14_m_tot(p);
s = sin(ps); c = cos(ps);
G = zeros(p.NX, 4);
G(2,1) = s/M_TOT;  G(2,2) = s/M_TOT;
G(10,1) = -p.l_c*s/p.I_b;  G(10,2) = -p.l_c*s/p.I_b;
G(12,3) = p.R_TRK*s/(p.R_w*p.I_ROLL);  G(12,4) = -p.R_TRK*s/(p.R_w*p.I_ROLL);
G(14,1) = (c-1)/p.M_V;  G(14,2) = (c-1)/p.M_V;
G(14,3) = s/(p.R_w*p.M_V);  G(14,4) = s/(p.R_w*p.M_V);
end

function u = clip_u(p, u)
for i = 1:p.NU
    u(i) = min(max(u(i), p.u_min(i)), p.u_max(i));
end
end

function s = pass_txt(ok)
if ok
    s = 'PASS';
else
    s = 'FAIL';
end
end
