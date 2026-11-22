% s02_tinympc14_one_mpc.m
% Leg4 单 MPC 融合：14 态 / 6 输入，一个 A/B/f，一套求解口径
%
%   与 v0.2（块对角基线）的差别：
%     ① 不再 blkdiag + 默认关的耦合开关：跨链耦合 #1/#2/#8/#8α 作为 A/B 的
%        矩阵元素常驻，模型天生耦合，不是"两条链拼起来再补丁"；
%     ② 平衡块按实测 (lL,lR) 重建（腿长参数化 #3），即单一 LPV 模型族；
%     ③ 整机走一组 Q/R/rho、一次 DARE、一个 max_iter —— 单求解口径。
%
%   版本：v0.5  日期：2026-10-20。v0.5：生产口径切 B 族线性重力 + K_EFF=0，三笔补偿
%     交 tinympc14_output.m 装配；WRITE 默认关，避免三字段表覆盖 S3 的七字段缓存。
%   v0.4（S2）：保留物理仿射 f_raw，删除 xbar−A·xbar 重锚；
%     xbar 改由 tinympc14_equilibrium 解出（theta_b = pitch_ref、两腿同角）；新增 u_ss 与离散定点校验；
%     RHO 生成改为合法浮点字面量；生成物落 staging。
%   v0.3 相对 v0.2 的差别（模型核共享化）：
%     ① 常量与模型函数改由 tinympc14_config + mpc/parts/tinympc14_*.m 提供，删除本文件自带的
%        one_cfg / one_model / two_block / bal_block / table_row / zoh_ab / ctrb_rank_
%     ② 调度点、psi 扫描与耦合开关取 cfg.sym7.sched_l / sched_psi / psi_scan / sw_fused
%     ③ 模型核与 s01 同一份：两腿各按自身腿长取表、#2 含腿角列、#8α 常驻、K_EFF 按 spring
%
%   物理来源见 《Tinympc14_单MPC融合_设计与推导.md》§3。
%   独立性：常量与模型函数来自 mpc/tinympc14_config.m 与 mpc/parts/tinympc14_*.m，不引用 Leg3_v2 任何路径；
%           预览只写 cfg.STAGING_DIR/preview；正式部署目标见 cfg.OUT_DIR，由 tinympc14_deploy 导出。
%   口径：模型走 B 族线性重力列、无仿射常数；MgF / Fs / 向心三笔补偿由 tinympc14_comp.m
%         插件在模型外接（输出层两笔 + 输入层 ffwd 一笔），模型矩阵不烘补偿项。
%
%   用法：matlab -batch "cd('E:/DevelopLotteany1/0_Code/Leg4_v1_ultimate/Matlab/mpc/parts'); s02_tinympc14_one_mpc"

clear; clc; close all
addpath(fileparts(fileparts(mfilename('fullpath')))); mixmpc_paths();   % mpc 顶层：tinympc14_config
p = tinympc14_config();
NX = p.NX; NU = p.NU;

lh  = p.sym7.sched_l;        % m 调度点腿长
psi = p.sym7.sched_psi;      % rad 调度点世界腿角（动态倾角；静止直立取 0）
sched = tinympc14_sched_pt(p, lh, 0, psi);

[A, B, f, aux] = tinympc14_fused_ab(p, p.sym7.sw_fused, sched);
[xbar, u_ss, eq] = tinympc14_equilibrium(p, p.sym7.sw_fused, sched);
r_grav = max(abs(A*xbar + f));              % 零输入残差 = 重力与耦合仿射项，由 u_ss 补足

fprintf('==== Leg4 单 MPC（%d 态 / %d 输入）====\n', NX, NU);
fprintf('调度点：lL = lR = %.3f m，psi = %.3f rad，theta* = %.6f rad\n', lh, psi, aux.theta_star_l);
fprintf('平衡解：theta_leg = %.6f rad，theta_b = %.3f rad，rank(E) = %d，cond(E) = %.3e\n\n', ...
        eq.theta_leg, p.sym7.pitch_ref, eq.rank_E, eq.cond_E);

%% ---- ① 天生耦合：证明不是两条链 ----
% 烘表点 psi=0 的 sin(psi) 跨块元为 0；耦合演示取 psi_nom 点，随 psi 的扫描见 ④
sched_demo = tinympc14_sched_pt(p, lh, 0, p.sym7.psi_nom);
[A_demo, B_demo] = tinympc14_fused_ab(p, p.sym7.sw_fused, sched_demo);
isbal = false(NX,NX); isbal(1:10,1:10) = true; isbal(11:14,11:14) = true;
isblk = false(NX,NU); isblk(1:10,1:4) = true;  isblk(11:14,5:6) = true;
offA = A_demo(~isbal); offB = B_demo(~isblk);
fprintf('-- 跨块耦合（演示点 psi = %.2f；v0.2 为恒 0）--\n', p.sym7.psi_nom);
fprintf('   A 的跨块元：%2d 个，max|.| = %.4g\n', nnz(offA), max(abs(offA)));
fprintf('   B 的跨块元：%2d 个，max|.| = %.4g\n', nnz(offB), max(abs(offB)));
fprintf('   逐行非零数 = %s\n', mat2str(sum(A_demo~=0,2)'));
fprintf('   关键耦合落点：\n');
fprintf('     #1  A(2,5/7)=%+.4f  A(10,5/7)=%+.4f  B(2,5:6)=%+.3e  B(10,5:6)=%+.3e\n', ...
        A_demo(2,5), A_demo(10,5), B_demo(2,5), B_demo(10,5));
fprintf('     #2  A(14,5/7)=%+.4f  B(14,5:6)=%+.3e\n', A_demo(14,5), B_demo(14,5));
fprintf('     #8  B(14,1:2)=%+.4f  #8α B(12,1:2)=%+.4f\n', B_demo(14,1), B_demo(12,1));
fprintf('\n');

%% ---- ② 单一求解口径：一组 Q/R/rho，一次 DARE ----
Q = diag(p.Q_DIAG); Rm = diag(p.R_DIAG); rho = p.rho;
[Ad, Bd, fd] = tinympc14_zoh_ab(A, B, f, p.Ts);
[Kinf, Pinf, ecl] = tinympc14_dare_gain(Ad, Bd, Q, Rm, rho, NX, NU);
QuuInv = inv(Rm + rho*eye(NU) + Bd'*Pinf*Bd);
r_fp = eq.res_fp;
fprintf('-- 单一求解口径（一个 rho = %g，一套 Q/R）--\n', rho);
fprintf('   开环：max Re(eig) = %+.4f，不稳模态 %d 个；连续可控秩 %d/%d\n', ...
        max(real(eig(A))), sum(real(eig(A)) > 0), tinympc14_ctrb_rank(A, B), NX);
fprintf('   DARE：|eig(Ad-Bd*Kinf)|max = %.6f，|Kinf|max = %.2f，cond(QuuInv) = %.3e\n', ...
        ecl, max(abs(Kinf(:))), cond(QuuInv));
fprintf('   零输入残差 max|A*xbar+f| = %.2e（重力与耦合仿射项，由 u_ss 补足）\n', r_grav);
fprintf('   平衡方程：rank(Bd) = %d，cond(Bd) = %.3e，连续残差 %.2e\n', ...
        rank(Bd), cond(Bd), eq.res_E);
fprintf('   u_ss = [%+.2f %+.2f %+.2f %+.2f %+.2f %+.2f]\n', u_ss);
fprintf('   定点残差 max|Ad*xbar+Bd*u_ss+fd-xbar| = %.2e\n', r_fp);
fprintf('   u_ss 在箱内：%s（前 4 路 N·m，后 2 路 N 总支持力名义箱）\n', ...
        mat2str(all(u_ss >= p.u_min' & u_ss <= p.u_max')));
fprintf('\n');

%% ---- ③ 与"两条链"（块对角）对照：耦合带来的极点移动 ----
[A2, B2, f2] = tinympc14_fused_ab(p, p.sym7.sw_none, sched);
e_off = eig(A2); e_on = eig(A);
fprintf('-- 块对角（两条链） vs 单 MPC：开环极点对照 --\n');
fprintf('   块对角 max Re = %+.4f；单 MPC max Re = %+.4f\n', max(real(e_off)), max(real(e_on)));
[Ad2, Bd2] = tinympc14_zoh_ab(A2, B2, f2, p.Ts);
[K2, ~, ~] = tinympc14_dare_gain(Ad2, Bd2, Q, Rm, rho, NX, NU);
fprintf('   闭环 |λ|max：块对角 %.6f；单 MPC %.6f；Δ|Kinf|max = %.3f\n', ...
        max(abs(eig(Ad2-Bd2*K2))), ecl, max(abs(Kinf(:))) - max(abs(K2(:))));
fprintf('\n');

%% ---- ④ 耦合增益的调度范围（LPV）：psi 扫描 ----
fprintf('-- 耦合随世界腿角 psi 的调度范围 --\n');
fprintf('     psi    A(2,5/7)   A(14,5/7)   B(14,1)    B(12,1)\n');
for ps = p.sym7.psi_scan
    s3 = tinympc14_sched_pt(p, lh, 0, ps);
    [A3, B3, ~] = tinympc14_fused_ab(p, p.sym7.sw_fused, s3);
    fprintf('   %.3f   %+8.4f   %+8.4f   %+8.4f   %+8.4f\n', ps, A3(2,5), A3(14,5), B3(14,1), B3(12,1));
end
fprintf('\n');

%% ---- ⑤ 算力预算（单求解器一次算 14/6）----
mac = @(nx, nu, nh) (3*nx*nh + 2*nu*(nh-1) + nx^2 + nx) + ...
      ((nx*nu + nu^2 + nx^2 + nu*nx + nx + nu)*(nh-1)) + ...
      ((nu*nx + nx^2 + nx*nu + nx + nu)*(nh-1)) + 6*(nu*(nh-1) + nx*nh);
m_old = mac(10,4,p.NH) + mac(4,2,p.NH);
m_new = mac(NX,NU,p.NH);
fprintf('-- 算力：两条链合计 %d MAC/轮 → 单 MPC %d MAC/轮（%.2fx）--\n', m_old, m_new, m_new/m_old);
fprintf('   单 MPC 一次求解覆盖全部 14 态；不再需要两个任务各自求解\n');

%% ---- ⑥ 预览（可选）：写单模型表到 STAGING_DIR/preview ----
if p.WRITE
    preview=fullfile(p.STAGING_DIR,'preview');
    write_fused_data(preview, A, B, f, p);
    fprintf('   Preview-only three-field model written to %s; deployment uses s03\n',preview);
else
    fprintf('   （项目内部预览根目录 STAGING_DIR = %s；WRITE=false 暂不落盘）\n', p.STAGING_DIR);
end

%% ========================================================================
function write_fused_data(out_dir, A, B, f, p)
% 单模型表落盘：单点口径（LPV 网格版待 O1）；文件名 tinympc14_data.*，不覆盖平衡链 tinympc_data.*
NX = p.NX; NU = p.NU;
if ~exist(out_dir, 'dir'), mkdir(out_dir); end
[Ad, Bd, fd] = tinympc14_zoh_ab(A, B, f, p.Ts);
fh = fopen(fullfile(out_dir, 'tinympc14_data.h'), 'w');
fprintf(fh, '// Leg4 单 MPC 模型数据头文件\n');
fprintf(fh, '// 由 Leg4_v1_ultimate/Matlab/mpc/parts/s02_tinympc14_one_mpc.m 生成，勿手改；重新部署重跑该脚本\n');
fprintf(fh, '#ifndef TINYMPC14_DATA_H\n#define TINYMPC14_DATA_H\n\n');
fprintf(fh, '#define TINYMPC14_NX %d\n#define TINYMPC14_NU %d\n#define TINYMPC14_NH %d\n', NX, NU, p.NH);
fprintf(fh, '#define TINYMPC14_TS %.3ff   /* s，求解周期 */\n', p.Ts);
rhotxt = sprintf('%.9g', p.rho);
if ~any(rhotxt == '.') && ~any(rhotxt == 'e') && ~any(rhotxt == 'E')
    rhotxt = [rhotxt '.0'];
end
fprintf(fh, '#define TINYMPC14_RHO %sf  /* ADMM 罚参数，已烘焙进矩阵 */\n\n', rhotxt);
fprintf(fh, '/* 行优先展平 M[i*cols+j]；状态序 [s ds phi dphi th_ll dth_ll th_lr dth_lr th_b dth_b a da H dH] */\n');
fprintf(fh, '/* 输入序 [T_wl T_wr T_bl T_br F_l F_r]；跨链耦合已常驻 A/B，无需开关 */\n');
fprintf(fh, 'typedef struct\n{\n');
fprintf(fh, '    float Ad[TINYMPC14_NX * TINYMPC14_NX];\n');
fprintf(fh, '    float Bd[TINYMPC14_NX * TINYMPC14_NU];\n');
fprintf(fh, '    float fd[TINYMPC14_NX];\n');
fprintf(fh, '} tinympc14_model_t;\n\n');
fprintf(fh, 'extern tinympc14_model_t tinympc14_model;\n');
fprintf(fh, '\n#endif\n');
fclose(fh);
fc = fopen(fullfile(out_dir, 'tinympc14_data.c'), 'w');
fprintf(fc, '// Leg4 单 MPC 模型数据（由 s02_tinympc14_one_mpc.m 生成，勿手改）\n');
fprintf(fc, '#include "tinympc14_data.h"\n\n');
fprintf(fc, 'tinympc14_model_t tinympc14_model = {\n');
fprintf(fc, '  .Ad = {%s},\n', num2str(Ad(:)', '%.9gf '));
fprintf(fc, '  .Bd = {%s},\n', num2str(Bd(:)', '%.9gf '));
fprintf(fc, '  .fd = {%s},\n', num2str(fd(:)', '%.9gf '));
fprintf(fc, '};\n');
fclose(fc);
end
