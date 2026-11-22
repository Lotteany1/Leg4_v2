% s04b_tinympc14_lpv.m
% S4b：Leg4 单 MPC 的 (lL,lR) 20x20 LPV 表
% 每点算 Ad/Bd/fd/Kinf/Pinf/QuuInv/AmBKt，七张表逐元做 poly33 最小二乘
% 自检 G9 拟合残差、G10 拟合表闭环稳定、G11 网格连续性、G12 内存/计算量
% G13 静态闭环定点对比（单点 MVP vs LPV）；动态场景留给 S6
%
% 版本：v0.1  日期：2026-10-19
% 用法：matlab -batch "cd('E:/DevelopLotteany1/0_Code/Leg4_v1_ultimate/Matlab/mpc/parts'); s04b_tinympc14_lpv"

clear; clc; close all
addpath(fileparts(fileparts(mfilename('fullpath')))); mixmpc_paths();   % mpc 顶层：tinympc14_config
p = tinympc14_config();
NX = p.NX; NU = p.NU;
lgrid = linspace(0.16, 0.34, 20);   % 运行时钳位域；0.15 行被 Leg_comp 端点钳住，不进拟合
n = numel(lgrid);
psi = p.sym7.sched_psi;
Q = diag(p.Q_DIAG);
Rm = diag(p.R_DIAG);
rho = p.rho;

fprintf('==== S4b Leg4 LPV（%d x %d，l = %.2f:%.2f:%.2f m，psi = %.2f）====\n', ...
        n, n, lgrid(1), lgrid(2)-lgrid(1), lgrid(end), psi);

%% ---- 网格真值 ----
Ad_num     = zeros(NX, NX, n, n);
Bd_num     = zeros(NX, NU, n, n);
fd_num     = zeros(NX, 1, n, n);
Kinf_num   = zeros(NU, NX, n, n);
Pinf_num   = zeros(NX, NX, n, n);
QuuInv_num = zeros(NU, NU, n, n);
AmBKt_num  = zeros(NX, NX, n, n);
for il = 1:n
    for ir = 1:n
        lL = lgrid(il); lR = lgrid(ir);
        alpha = atan((lL-lR)*cos(psi)/(2*p.R_TRK));
        H = (lL+lR)*cos(psi)*cos(alpha)/2;
        sched = struct('H', H, 'alpha', alpha, 'lL', lL, 'lR', lR, 'psi', psi);
        [A, B, f] = tinympc14_fused_ab(p, p.sym7.sw_fused, sched);
        [Ad, Bd, fd] = tinympc14_zoh_ab(A, B, f, p.Ts);
        try
            [Kinf, Pinf] = tinympc14_dare_gain(Ad, Bd, Q, Rm, rho, NX, NU);
        catch ME
            error('DARE 失败 @ (%.2f, %.2f)：%s', lL, lR, ME.message);
        end
        QuuInv = inv(Rm + rho*eye(NU) + Bd'*Pinf*Bd);
        AmBKt = (Ad - Bd*Kinf)';
        Ad_num(:,:,il,ir) = Ad;
        Bd_num(:,:,il,ir) = Bd;
        fd_num(:,:,il,ir) = fd;
        Kinf_num(:,:,il,ir) = Kinf;
        Pinf_num(:,:,il,ir) = Pinf;
        QuuInv_num(:,:,il,ir) = QuuInv;
        AmBKt_num(:,:,il,ir) = AmBKt;
    end
end

%% ---- poly33 拟合 ----
[LL, LR] = ndgrid(lgrid, lgrid);
X = [ones(n*n,1), LL(:), LR(:), LL(:).^2, LL(:).*LR(:), LR(:).^2, ...
     LL(:).^3, LL(:).^2.*LR(:), LL(:).*LR(:).^2, LR(:).^3];
tw = { 'ad',     'Ad',     NX, NX, Ad_num; ...
       'bd',     'Bd',     NX, NU, Bd_num; ...
       'fd',     'fd',     NX, 1,  fd_num; ...
       'kinf',   'Kinf',   NU, NX, Kinf_num; ...
       'pinf',   'Pinf',   NX, NX, Pinf_num; ...
       'quuinv', 'QuuInv', NU, NU, QuuInv_num; ...
       'ambkt',  'AmBKt',  NX, NX, AmBKt_num };
coef = struct();
res = struct();
for t = 1:size(tw,1)
    key = tw{t,1};
    [coef.(key), res.(key)] = poly33_fit(X, tw{t,3}, tw{t,4}, tw{t,5});
end

%% ---- G9 拟合残差（相对量，对齐平衡链同口径）----
% 平衡链 poly33 六表相对残差（resmax / max|M|，2026-10-18 缓存）：
ref_rel = struct('ad', 3.5361e-03, 'bd', 2.1757e-03, 'kinf', 8.6222e-04, ...
                 'pinf', 1.1238e-03, 'quuinv', 3.9477e-04, 'ambkt', 4.5860e-03);
fprintf('\n-- G9 拟合残差（max|fit-truth| / max|truth|）--\n');
fprintf('   %-8s %12s %12s %12s %12s\n', '表', '绝对 max', '相对 max', '平衡链相对', '倍数');
g9_pass = true;
for t = 1:size(tw,1)
    key = tw{t,1};
    if isfield(ref_rel, key)
        ratio = res.(key).rel / ref_rel.(key);
        ok = ratio <= 2.0;
        g9_pass = g9_pass && ok;
        fprintf('   %-8s %12.4e %12.4e %12.4e %12.3f %s\n', ...
                tw{t,2}, res.(key).max, res.(key).rel, ref_rel.(key), ratio, pass_txt(ok));
    else
        fprintf('   %-8s %12.4e %12.4e %12s %12s\n', ...
                tw{t,2}, res.(key).max, res.(key).rel, '—', '—');
    end
end
[~, kf] = max(res.fd.elem_max);
fprintf('   fd 绝对 max %.4e，相对 max|fd| %.4e（最差元素 #%d）；无平衡链参考，按绝对量记录\n', ...
        res.fd.max, res.fd.rel, kf);
fprintf('   G9 六表 2x 门限：%s\n', pass_txt(g9_pass));

here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
build_dir = fullfile(here, '..', 'build');
if ~exist(build_dir, 'dir'), mkdir(build_dir); end
save(fullfile(build_dir, 'tinympc14_lpv_cache.mat'), 'coef', 'res', 'lgrid', 'ref_rel', '-v7');
fprintf('   已存 build/tinympc14_lpv_cache.mat（coef/res/lgrid）\n');

%% ---- G10 拟合表闭环稳定 + 正定 + 自洽 ----
eig_fit = zeros(n, n);
pd_p = Inf; pd_q = Inf;
sc1 = 0; sc2 = 0; sc3 = 0; sc4 = 0;
for q = 1:n*n
    Adq = eval_row(coef.ad,     X(q,:), NX, NX);
    Bdq = eval_row(coef.bd,     X(q,:), NX, NU);
    Ki  = eval_row(coef.kinf,   X(q,:), NU, NX);
    Pi  = eval_row(coef.pinf,   X(q,:), NX, NX);
    Qu  = eval_row(coef.quuinv, X(q,:), NU, NU);
    Am  = eval_row(coef.ambkt,  X(q,:), NX, NX);
    eig_fit(q) = max(abs(eig(Adq - Bdq*Ki)));
    pd_p = min(pd_p, min(eig((Pi + Pi')/2)) / max(max(abs(Pi(:))), eps));
    pd_q = min(pd_q, min(eig((Qu + Qu')/2)) / max(max(abs(Qu(:))), eps));
    sc1 = max(sc1, max(abs(Ki - Qu*Bdq'*Pi*Adq), [], 'all') / max(max(abs(Ki(:))), eps));
    sc2 = max(sc2, max(abs(Qu*(Rm + rho*eye(NU) + Bdq'*Pi*Bdq) - eye(NU)), [], 'all'));
    sc3 = max(sc3, max(abs(Am - (Adq - Bdq*Ki)'), [], 'all') / max(max(abs(Am(:))), eps));
    sc4 = max(sc4, max(abs(Pi - (Q + rho*eye(NX) + Adq'*Pi*Adq ...
              - Adq'*Pi*Bdq*Qu*Bdq'*Pi*Adq)), [], 'all') / max(max(abs(Pi(:))), eps));
end
g10_pass = all(eig_fit(:) < 1) && pd_p > 0 && pd_q > 0;
fprintf('\n-- G10 拟合表稳定性 / 正定 --\n');
fprintf('   max|eig(Ad_fit-Bd_fit*Kinf_fit)| = %.9f @ (%.2f, %.2f)\n', ...
        max(eig_fit(:)), lgrid(1+mod(find(eig_fit == max(eig_fit(:)),1)-1,n)), ...
        lgrid(1+floor((find(eig_fit == max(eig_fit(:)),1)-1)/n)));
fprintf('   全部 |eig| < 1：%s\n', pass_txt(all(eig_fit(:) < 1)));
fprintf('   最小相对特征值：Pinf %.3e，QuuInv %.3e（>0）%s\n', pd_p, pd_q, pass_txt(pd_p > 0 && pd_q > 0));
fprintf('   自洽 INFO：Kinf 定义式 %.3e；QuuInv 恒等式 %.3e；AmBKt 恒等式 %.3e；DARE 残差 %.3e\n', ...
        sc1, sc2, sc3, sc4);
fprintf('   G10：%s\n', pass_txt(g10_pass));

%% ---- G11 网格连续性 ----
% 相邻网格点的表面跳变：拟合面不得超过真值跳变的 2 倍 + 2 倍拟合残差
fprintf('\n-- G11 网格连续性（相邻点 max|Δ|）--\n');
fprintf('   %-8s %12s %12s %12s %s\n', '表', '真值 max', '拟合 max', '判据', '结论');
g11_pass = true;
for t = 1:size(tw,1)
    key = tw{t,1}; r = tw{t,3}; c = tw{t,4}; Mnum = tw{t,5};
    Ft = zeros(n*n, r*c);
    Ff = zeros(n*n, r*c);
    for q = 1:n*n
        Ft(q,:) = vec_row(Mnum, q, r, c, n);
        Ff(q,:) = vec_row_fit(coef.(key), X(q,:), r, c);
    end
    jt = 0; jf = 0;
    for ir = 1:n
        for il = 1:n
            q = il + (ir-1)*n;
            if il < n
                jt = max(jt, max(abs(Ft(q+1,:) - Ft(q,:))));
                jf = max(jf, max(abs(Ff(q+1,:) - Ff(q,:))));
            end
            if ir < n
                jt = max(jt, max(abs(Ft(q+n,:) - Ft(q,:))));
                jf = max(jf, max(abs(Ff(q+n,:) - Ff(q,:))));
            end
        end
    end
    lim = 2*jt + 2*res.(key).max;
    ok = jf <= lim;
    g11_pass = g11_pass && ok;
    fprintf('   %-8s %12.4e %12.4e %12.4e %s\n', tw{t,2}, jt, jf, lim, pass_txt(ok));
end
fprintf('   G11：%s\n', pass_txt(g11_pass));

%% ---- G12 内存 / 计算量 ----
elems = NX*NX + NX*NU + NX + NU*NX + NX*NX + NU*NU + NX*NX;
bytes = elems * 10 * 4;
flops = elems * 19;
fprintf('\n-- G12 内存 / 计算量（结构预算）--\n');
fprintf('   七表元素 %d（平衡链六表 792，+%d 个 fd）；系数 %.2f KB（10 系数/元）\n', ...
        elems, elems - 792, bytes/1024);
fprintf('   模型工作区 %d float = %.1f B；单次更新约 %d FLOP（10 乘 + 9 加/元）\n', ...
        elems, elems*4, flops);
fprintf('   结论：结构预算内；固件更新耗时与 1 kHz 隔离策略待 S7 实测\n');

%% ---- G13 静态闭环定点：单点 MVP vs LPV ----
fprintf('\n-- G13 静态闭环定点（psi=0，plant=网格真值，控制器=表值）--\n');
sn = tinympc14_sched_pt(p, 0.27, 0, psi);
[An, Bn, fn] = tinympc14_fused_ab(p, p.sym7.sw_fused, sn);
[Adn, Bdn, fdn] = tinympc14_zoh_ab(An, Bn, fn, p.Ts);
[xbn, ~, ~] = tinympc14_equilibrium(p, p.sym7.sw_fused, sn);
[Ksn, ~, ~] = tinympc14_dare_gain(Adn, Bdn, Q, Rm, rho, NX, NU);
pts = [0.20, 0.20; 0.27, 0.27; 0.34, 0.34];
fprintf('   %5s %5s %14s %14s %10s %8s\n', 'lL', 'lR', 'dH 单点', 'dH LPV', '改善', '结论');
g13_ok = false; g13_worse = false;
for k = 1:size(pts,1)
    lL = pts(k,1); lR = pts(k,2);
    alpha = atan((lL-lR)*cos(psi)/(2*p.R_TRK));
    H = (lL+lR)*cos(psi)*cos(alpha)/2;
    sp = struct('H', H, 'alpha', alpha, 'lL', lL, 'lR', lR, 'psi', psi);
    [Ap, Bp, fp] = tinympc14_fused_ab(p, p.sym7.sw_fused, sp);
    [Adp, Bdp, fdp] = tinympc14_zoh_ab(Ap, Bp, fp, p.Ts);
    [xbp, ~, ~] = tinympc14_equilibrium(p, p.sym7.sw_fused, sp);

    % 单点：控制器坐标以 0.27 为零点
    xref_s = xbp; xref_s(13) = H - 0.27;
    us_s = pinv(Bdn) * ((eye(NX) - Adn) * xref_s - fdn);
    Ms = eye(NX) - Adp + Bdp*Ksn;
    xs = Ms \ (Bdp*us_s + Bdp*Ksn*xref_s + fdp);
    dHs = xs(13) - xbp(13);

    % LPV：控制器坐标以本点 H 为零点；表值取本点
    Xq = [1, lL, lR, lL^2, lL*lR, lR^2, lL^3, lL^2*lR, lL*lR^2, lR^3];
    Adf = eval_row(coef.ad, Xq, NX, NX);
    Bdf = eval_row(coef.bd, Xq, NX, NU);
    fdf = eval_row(coef.fd, Xq, NX, 1);
    Kif = eval_row(coef.kinf, Xq, NU, NX);
    us_l = pinv(Bdf) * ((eye(NX) - Adf) * xbp - fdf);
    Ml = eye(NX) - Adp + Bdp*Kif;
    xl = Ml \ (Bdp*us_l + Bdp*Kif*xbp + fdp);
    dHl = xl(13) - xbp(13);

    if abs(dHs) > 1e-6
        imp = 1 - abs(dHl)/abs(dHs);
    else
        imp = NaN;
    end
    if abs(dHl) < 0.8*abs(dHs) || abs(dHs) < 1e-6 && abs(dHl) < 1e-6
        g13_ok = true;
    end
    if abs(dHl) > 1.1*abs(dHs) && abs(dHs) > 1e-6
        g13_worse = true;
    end
    fprintf('   %5.2f %5.2f %+14.4e %+14.4e %9.1f%% %8s\n', ...
            lL, lR, dHs, dHl, 100*imp, pass_txt(abs(dHs) < 1e-4 || abs(dHl) <= 0.8*abs(dHs)));
end
g13_pass = g13_ok && ~g13_worse;
fprintf('   G13：%s（至少一个场景改善 >=20%%，且无场景变差 >10%%）\n', pass_txt(g13_pass));

%% ---- 写 staging ----
write_lpv_data(p.STAGING_DIR, coef, res, tw, p, lgrid, NX, NU);
fprintf('\n已写 tinympc14_lpv_data.h/.c 到 %s\n', p.STAGING_DIR);

assert(g9_pass, 'G9 未通过');
assert(g10_pass, 'G10 未通过');
assert(g11_pass, 'G11 未通过');
assert(g13_pass, 'G13 未通过');
fprintf('S4b 判据 G9/G10/G11/G13 通过；G12 结构预算已记录\n');

%% ========================================================================
function [C, res] = poly33_fit(X, r, c, Mnum)
% 逐元 poly33 最小二乘；列序 = 行优先 k = i*c+j（与固件 M[i*cols+j] 同序）
N = size(X,1);
E = r*c;
Y = zeros(N, E);
for i = 1:r
    for j = 1:c
        k = (i-1)*c + j;
        Y(:,k) = reshape(Mnum(i,j,:,:), N, 1);
    end
end
C = X \ Y;
R = X*C - Y;
res.max = max(abs(R(:)));
res.rel = res.max / max(max(abs(Y(:))), eps);
res.elem_max = max(abs(R), [], 1);
res.elem_rel = max(abs(R), [], 1) ./ max(max(abs(Y), [], 1), eps);
end

function M = eval_row(C, xrow, r, c)
% 单点求值：C 为 10 x (r*c)，xrow 为 1 x 10；返回 r x c
M = reshape(xrow * C, [c, r])';
end

function v = vec_row(Mnum, q, r, c, n)
% 网格第 q 点的行优先展平；q = il + (ir-1)*n
il = 1 + mod(q-1, n);
ir = 1 + floor((q-1)/n);
v = zeros(1, r*c);
for i = 1:r
    for j = 1:c
        v((i-1)*c + j) = Mnum(i,j,il,ir);
    end
end
end

function v = vec_row_fit(C, xrow, r, c)
% 单点拟合值的行优先展平
M = eval_row(C, xrow, r, c);
v = zeros(1, r*c);
for i = 1:r
    for j = 1:c
        v((i-1)*c + j) = M(i,j);
    end
end
end

function s = pass_txt(ok)
if ok
    s = 'PASS';
else
    s = 'FAIL';
end
end

%% ========================================================================
function write_lpv_data(out_dir, coef, res, tw, p, lgrid, NX, NU)
if ~exist(out_dir, 'dir'), mkdir(out_dir); end
stamp = datestr(now, 'yyyy-mm-dd HH:MM:SS');
n = numel(lgrid);
elems = struct('ad', NX*NX, 'bd', NX*NU, 'fd', NX, 'kinf', NU*NX, ...
               'pinf', NX*NX, 'quuinv', NU*NU, 'ambkt', NX*NX);

fh = fopen(fullfile(out_dir, 'tinympc14_lpv_data.h'), 'w');
fprintf(fh, '// Leg4 单 MPC LPV 数据头文件（%dx%d，lL/lR，poly33，psi=0 烘表）\n', n, n);
fprintf(fh, '// 由 Matlab/mpc/parts/s04b_tinympc14_lpv.m 生成于 %s，勿手改\n', stamp);
fprintf(fh, '#ifndef TINYMPC14_LPV_DATA_H\n#define TINYMPC14_LPV_DATA_H\n\n');
fprintf(fh, '#include "tinympc14_data.h"\n\n');
fprintf(fh, '#define TINYMPC14_LPV_IFACE_VER 1\n');
fprintf(fh, '#define TINYMPC14_LPV_NGRID %d\n', n);
fprintf(fh, '#define TINYMPC14_LPV_L_MIN %.3ff /* m 网格下限 */\n', lgrid(1));
fprintf(fh, '#define TINYMPC14_LPV_L_MAX %.3ff /* m 网格上限；运行时进表前钳到 0.16~0.34 */\n', lgrid(end));
fprintf(fh, '#define TINYMPC14_LPV_NCOEF 10  /* poly33：1 lL lR lL^2 lL*lR lR^2 lL^3 lL^2*lR lL*lR^2 lR^3 */\n');
fprintf(fh, '#define TINYMPC14_LPV_ELEMS_AD %d\n', elems.ad);
fprintf(fh, '#define TINYMPC14_LPV_ELEMS_BD %d\n', elems.bd);
fprintf(fh, '#define TINYMPC14_LPV_ELEMS_FD %d\n', elems.fd);
fprintf(fh, '#define TINYMPC14_LPV_ELEMS_KINF %d\n', elems.kinf);
fprintf(fh, '#define TINYMPC14_LPV_ELEMS_PINF %d\n', elems.pinf);
fprintf(fh, '#define TINYMPC14_LPV_ELEMS_QUUINV %d\n', elems.quuinv);
fprintf(fh, '#define TINYMPC14_LPV_ELEMS_AMBKT %d\n', elems.ambkt);
fprintf(fh, '/* 七张表：Ad/Bd/fd/Kinf/Pinf/QuuInv/AmBKt；元素行优先 k=i*cols+j */\n');
fprintf(fh, '/* fd 是物理仿射的离散增益；六矩阵口径沿用平衡链，fd 为 Leg4 新增第七表 */\n\n');
fprintf(fh, 'void Tinympc14_Lpv_Data_Update(float lL, float lR, tinympc14_model_t *mdl);\n\n');
fprintf(fh, '#endif\n');
fclose(fh);

fc = fopen(fullfile(out_dir, 'tinympc14_lpv_data.c'), 'w');
fprintf(fc, '// Leg4 单 MPC LPV 数据（%dx%d，poly33，psi=0）\n', n, n);
fprintf(fc, '// 由 Matlab/mpc/parts/s04b_tinympc14_lpv.m 生成于 %s，勿手改\n', stamp);
fprintf(fc, '#include "tinympc14_lpv_data.h"\n\n');
for t = 1:size(tw,1)
    key = tw{t,1};
    E = elems.(key);
    fprintf(fc, 'static const float tinympc14_lpv_%s_coef[%d][TINYMPC14_LPV_NCOEF] =\n{\n', key, E);
    C = coef.(key);   % 10 x E
    for k = 1:E
        fprintf(fc, '    {');
        for c = 1:10
            fprintf(fc, ' %.9e%s', C(c,k), sep_txt(c, 10));
        end
        fprintf(fc, ' },\n');
    end
    fprintf(fc, '};\n\n');
end
fprintf(fc, '// 调度量只做 poly33 求值；固件侧进表前必须钳到 0.16~0.34 m\n');
fprintf(fc, 'void Tinympc14_Lpv_Data_Update(float lL, float lR, tinympc14_model_t *mdl)\n{\n');
fprintf(fc, '    const float t2  = lL * lL;\n');
fprintf(fc, '    const float t3  = lR * lR;\n');
fprintf(fc, '    const float tLR = lL * lR;\n');
fprintf(fc, '    const float t4  = t2 * lL;\n');
fprintf(fc, '    const float t5  = t2 * lR;\n');
fprintf(fc, '    const float t6  = tLR * lR;\n');
fprintf(fc, '    const float t7  = t3 * lR;\n\n');
    emit_loop(fc, 'ad', 'Ad', elems.ad);
    emit_loop(fc, 'bd', 'Bd', elems.bd);
    emit_loop(fc, 'fd', 'fd', elems.fd);
    emit_loop(fc, 'kinf', 'Kinf', elems.kinf);
    emit_loop(fc, 'pinf', 'Pinf', elems.pinf);
    emit_loop(fc, 'quuinv', 'QuuInv', elems.quuinv);
    emit_loop(fc, 'ambkt', 'AmBKt', elems.ambkt);
fprintf(fc, '}\n');
fclose(fc);
end

function s = sep_txt(c, n)
if c == n
    s = '';
else
    s = ',';
end
end

function emit_loop(fid, key, field, E)
fprintf(fid, '    for (int c = 0; c < %d; c++)\n', E);
fprintf(fid, '    {\n');
fprintf(fid, '        const float *p = tinympc14_lpv_%s_coef[c];\n', key);
fprintf(fid, '        mdl->%s[c] = p[0] + p[1]*lL + p[2]*lR + p[3]*t2 + p[4]*tLR + p[5]*t3 + p[6]*t4 + p[7]*t5 + p[8]*t6 + p[9]*t7;\n', field);
fprintf(fid, '    }\n\n');
end
