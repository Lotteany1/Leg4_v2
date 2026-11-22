% s04_tinympc14_mvp_scan.m
% S4a 单点 MVP 固定增益：稳定性扫描 + 稳态误差 + 时域抽检
% 用 S3 缓存（lL=lR=0.27 / psi=0）的 Kinf，扫 H in [0.20,0.34]、psi in [0,0.20]
% 不调 Q/R；误差只记录，作为 S4b/S4c 的输入
%
% 版本：v0.1  日期：2026-10-19
% 用法：matlab -batch "cd('E:/DevelopLotteany1/0_Code/Leg4_v1_ultimate/Matlab/mpc/parts'); s04_tinympc14_mvp_scan"

clear; clc; close all
addpath(fileparts(fileparts(mfilename('fullpath')))); mixmpc_paths();   % mpc 顶层：tinympc14_config
p = tinympc14_config();
NX = p.NX; NU = p.NU;
Hn = p.sym7.sched_l;
psin = p.sym7.sched_psi;

%% ---- 名义缓存（与 S3 同口径）----
sn = tinympc14_sched_pt(p, Hn, 0, psin);
[An, Bn, fn] = tinympc14_fused_ab(p, p.sym7.sw_fused, sn);
[Adn, Bdn, fdn] = tinympc14_zoh_ab(An, Bn, fn, p.Ts);
[xbn, ussn, eqn] = tinympc14_equilibrium(p, p.sym7.sw_fused, sn);
Q = diag(p.Q_DIAG);
Rm = diag(p.R_DIAG);
[Kinf, Pinf, ecl] = tinympc14_dare_gain(Adn, Bdn, Q, Rm, p.rho, NX, NU);

fprintf('==== S4a 单点 MVP 固定增益（Hn=%.2f m, psin=%.2f）====\n', Hn, psin);
fprintf('名义：|eig(Ad-Bd*Kinf)|max = %.9f，|Kinf|max = %.3f\n\n', ecl, max(abs(Kinf(:))));

%% ---- 稳定性网格 ----
Hgrid = 0.20:0.01:0.34;
Pgrid = 0.00:0.02:0.20;
nH = numel(Hgrid); nP = numel(Pgrid);
rho_scan = zeros(nH, nP);
for i = 1:nH
    for j = 1:nP
        sp = tinympc14_sched_pt(p, Hgrid(i), 0, Pgrid(j));
        [Ap, Bp, fp] = tinympc14_fused_ab(p, p.sym7.sw_fused, sp);
        [Adp, Bdp, ~] = tinympc14_zoh_ab(Ap, Bp, fp, p.Ts);
        rho_scan(i, j) = max(abs(eig(Adp - Bdp*Kinf)));
    end
end
[mx, imx] = max(rho_scan(:));
[iH, iP] = ind2sub(size(rho_scan), imx);
margin = 1 - mx;
fprintf('-- 稳定性网格（固定 Kinf）--\n');
fprintf('   max |eig| = %.9f @ H = %.2f, psi = %.2f；裕度 1-|eig| = %.3e\n', ...
        mx, Hgrid(iH), Pgrid(iP), margin);
fprintf('   全部 < 1：%s；名义点 %.9f\n\n', mat2str(all(rho_scan(:) < 1)), rho_scan(1 + round((Hn - Hgrid(1))/0.01), 1 + round((psin - Pgrid(1))/0.02)));

%% ---- 稳态误差（名义控制器 + 坐标平移）----
% 控制器坐标以 Hn 为零点；在 Hp 处参考 H 为 Hp-Hn，其余状态取该点平衡
errH = zeros(nH, nP);
errS = zeros(nH, nP);
errTh = zeros(nH, nP);
uF = zeros(nH, nP);
for i = 1:nH
    for j = 1:nP
        sp = tinympc14_sched_pt(p, Hgrid(i), 0, Pgrid(j));
        [Ap, Bp, fp] = tinympc14_fused_ab(p, p.sym7.sw_fused, sp);
        [Adp, Bdp, fdp] = tinympc14_zoh_ab(Ap, Bp, fp, p.Ts);
        [xbp, ussp, ~] = tinympc14_equilibrium(p, p.sym7.sw_fused, sp);
        xref = xbp;
        xref(13) = Hgrid(i) - Hn;
        uc = pinv(Bdn) * ((eye(NX) - Adn) * xref - fdn);
        Mcl = eye(NX) - Adp + Bdp*Kinf;
        xcl = Mcl \ (Bdp*uc + Bdp*Kinf*xref + fdp);
        e = xcl - xbp;
        errH(i, j) = e(13);
        errS(i, j) = e(1);
        errTh(i, j) = e(5);
        uF(i, j) = (uc(5) + uc(6))/2;
    end
end
[emx, iemx] = max(abs(errH(:)));
[iEH, iEP] = ind2sub(size(errH), iemx);
fprintf('-- 稳态误差（无 psi 前馈，控制器参考取该点平衡）--\n');
fprintf('   最大 |dH| = %.4f m @ H = %.2f, psi = %.2f；dS = %+.4f m，dtheta = %+.4f rad\n', ...
        errH(iEH, iEP), Hgrid(iEH), Pgrid(iEP), errS(iEH, iEP), errTh(iEH, iEP));
fprintf('   H 边缘（psi=0）：H=0.20 dH=%+.4f；H=0.34 dH=%+.4f\n', ...
        errH(1, 1), errH(end, 1));
fprintf('   psi 边缘（H=0.27）：psi=0.10 dH=%+.4f；psi=0.20 dH=%+.4f\n', ...
        errH(1 + round((Hn - Hgrid(1))/0.01), 1 + round(0.10/0.02)), ...
        errH(1 + round((Hn - Hgrid(1))/0.01), 1 + round(0.20/0.02)));
fprintf('   力指令范围 [%.2f, %.2f] N，箱 [%.0f, %.0f] N，全在箱内 %s\n\n', ...
        min(uF(:)), max(uF(:)), p.F_MIN, p.F_MAX, ...
        mat2str(all(uF(:) >= p.F_MIN & uF(:) <= p.F_MAX)));

%% ---- 角落表 ----
corners = [0.20 0.00; 0.20 0.20; 0.34 0.00; 0.34 0.20; 0.27 0.20];
fprintf('-- 角落 / psi 边缘 --\n');
fprintf('   %6s %6s %12s %12s %12s %12s\n', 'H', 'psi', '|eig|max', 'dH [m]', 'dS [m]', 'F [N]');
for k = 1:size(corners, 1)
    Hp = corners(k, 1); pp = corners(k, 2);
    ii = 1 + round((Hp - Hgrid(1))/0.01);
    jj = 1 + round((pp - Pgrid(1))/0.02);
    fprintf('   %6.2f %6.2f %12.9f %+12.4f %+12.4f %12.2f\n', ...
            Hp, pp, rho_scan(ii, jj), errH(ii, jj), errS(ii, jj), uF(ii, jj));
end
fprintf('\n');

%% ---- 时域抽检：名义点 + 最坏稳态误差点，初值加 1 cm 扰动 ----
fprintf('-- 时域抽检（无约束，10 s，初值在平衡上加 ds=dH=1 cm）--\n');
sim_pts = [Hn psin; Hgrid(iEH) Pgrid(iEP)];
for k = 1:size(sim_pts, 1)
    Hp = sim_pts(k, 1); pp = sim_pts(k, 2);
    sp = tinympc14_sched_pt(p, Hp, 0, pp);
    [Ap, Bp, fp] = tinympc14_fused_ab(p, p.sym7.sw_fused, sp);
    [Adp, Bdp, fdp] = tinympc14_zoh_ab(Ap, Bp, fp, p.Ts);
    [xbp, ~, ~] = tinympc14_equilibrium(p, p.sym7.sw_fused, sp);
    xref = xbp; xref(13) = Hp - Hn;
    uc = pinv(Bdn) * ((eye(NX) - Adn) * xref - fdn);
    x = xbp; x(1) = x(1) + 0.01; x(13) = x(13) + 0.01;
    Nt = round(10/p.Ts);
    xmax = 0;
    for t = 1:Nt
        u = uc - Kinf*(x - xref);
        x = Adp*x + Bdp*u + fdp;
        xmax = max(xmax, max(abs(x - xbp)));
    end
    fprintf('   H=%.2f psi=%.2f：max|dx| = %.4f，末拍 dH = %+.4f，dS = %+.4f，有限 %s\n', ...
            Hp, pp, xmax, x(13) - xbp(13), x(1) - xbp(1), mat2str(all(isfinite(x))));
end
fprintf('\n');

assert(all(rho_scan(:) < 1), 'S4a 稳定性网格未通过');
assert(ecl < 1, 'S4a 名义点不稳定');
fprintf('S4a 判据通过：固定增益全网格 |eig| < 1；误差已记录\n');
