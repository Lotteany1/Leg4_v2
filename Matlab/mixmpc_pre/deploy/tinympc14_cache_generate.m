function tinympc14_cache_generate(fast_mode, staging_dir, p, cache)
if nargin < 1, fast_mode = false; end
% s03_tinympc14_cache.m
% Leg4 单 MPC 完整离线缓存：单点 lL=lR=0.27 / psi=0 / theta_b=0.05
% 生成 staging/tinympc14_data.h/.c：Ad/Bd/fd/Kinf/Pinf/QuuInv/AmBKt + q/r/u/x/du_rate
% 自检 G1~G8 与单精度敏感性；不写 Wheel_Leg
%
% 版本：v0.2  日期：2026-10-20。v0.2：模型口径切 B 族重力 + K_EFF=0（补偿插件在模型外），
%   IFACE_VER 升 2；生成物带插件三个常数（MgF / 向心）。
% 用法：matlab -batch "cd('E:/DevelopLotteany1/0_Code/Leg4_v1_ultimate/Matlab/mixmpc_pre/deploy'); s03_tinympc14_cache"


addpath(fileparts(fileparts(mfilename('fullpath')))); mixmpc_paths();   % 模型函数在 parts/
if nargin < 3, p = tinympc14_config(); end
if nargin >= 2, p.STAGING_DIR = staging_dir; end
qr_build_check(zeros(p.NX), zeros(p.NX,p.NU), diag(p.Q_DIAG), diag(p.R_DIAG));
NX = p.NX; NU = p.NU;

if fast_mode
    if nargin<4, cache = qr_build_cache('read', fileparts(fileparts(mfilename('fullpath')))); end
    model = cache.nominal;
    A = model.A;
    B = model.B;
    f = model.f;
    aux = model.aux;
    Ad = model.Ad;
    Bd = model.Bd;
    fd = model.fd;
    xbar = model.xbar;
    u_ss = model.u_ss;
    eq = model.eq;
    lh = model.lh;
    psi = model.psi;
else
lh = p.sym7.sched_l;
psi = p.sym7.sched_psi;
sched = tinympc14_sched_pt(p, lh, 0, psi);
[xbar, u_ss, eq] = tinympc14_equilibrium(p, p.sym7.sw_fused, sched);
sched.eq = eq.eq;                 % 仿射项锚到非线性平衡点，G1 才是 1e-12 级
sched.psi_l=xbar(5); sched.psi_r=xbar(7); % linearize at the actual equilibrium
[A, B, f, aux] = tinympc14_fused_ab(p, p.sym7.sw_fused, sched);
[Ad, Bd, fd] = tinympc14_zoh_ab(A, B, f, p.Ts);

    model = struct('A', A, 'B', B, 'f', f, 'aux', aux, 'Ad', Ad, 'Bd', Bd, 'fd', fd, 'xbar', xbar, 'u_ss', u_ss, 'eq', eq, 'lh', lh, 'psi', psi);
    sp0 = tinympc14_sched_pt(p, 0.27, 0, 0);
    [model.vector_xb0, model.vector_uss0] = tinympc14_equilibrium(p, p.sym7.sw_fused, sp0);
    save(fullfile(fileparts(p.STAGING_DIR), 'qr_nominal_model.mat'), 'model', '-v7');
end

Q = diag(p.Q_DIAG);
Rm = diag(p.R_DIAG);
rho = p.rho;
[Kinf, Pinf, ecl] = tinympc14_dare_gain(Ad, Bd, Q, Rm, rho, NX, NU);
qr_build_check(Ad, Bd, Q+rho*eye(NX), Rm+rho*eye(NU), Kinf, Pinf);
QuuInv = inv(Rm + rho*eye(NU) + Bd'*Pinf*Bd);
AmBKt = (Ad - Bd*Kinf)';

q_diag = p.Q_DIAG(:);
r_diag = p.R_DIAG(:);
u_min = p.u_min(:);
u_max = p.u_max(:);
x_min = p.x_min(:);
x_max = p.x_max(:);
du_rate = [1.0e6; 1.0e6; 1.0e6; 1.0e6; p.DF_RATE; p.DF_RATE];

%% ---- G1/G2 定点 ----
g1 = max(abs(A*xbar + B*eq.u_cont + f));
g2 = eq.res_fp;

%% ---- G3/G4/G5 DARE 恒等式 ----
Qbar = Q + rho*eye(NX);
Rbar = Rm + rho*eye(NU);
res_dare = Pinf - (Qbar + Ad'*Pinf*Ad - Ad'*Pinf*Bd*QuuInv*Bd'*Pinf*Ad);
g3_abs = max(abs(res_dare(:)));
g3_rel = norm(res_dare, 'fro') / max(1, norm(Pinf, 'fro'));
g4 = max(abs(QuuInv*(Rbar + Bd'*Pinf*Bd) - eye(NU)));
g4 = max(g4(:));
g5 = max(abs(AmBKt(:) - reshape((Ad - Bd*Kinf)', [], 1)));

%% ---- G6 独立 double 基线（SDA）+ 闭环稳定 ----
[K_sda, P_sda] = dare_sda(Ad, Bd, Qbar, Rbar);
eig_sda = max(abs(eig(Ad - Bd*K_sda)));
g6_diff = abs(ecl - eig_sda);
pinf_rel = max(abs(Pinf(:) - P_sda(:))) / max(1, max(abs(Pinf(:))));

%% ---- G7 正定 ----
g7_P = min(eig((Pinf + Pinf')/2));
g7_Q = min(eig((QuuInv + QuuInv')/2));

%% ---- G8 单精度闭环 ----
Ad_s = single(Ad);
Bd_s = single(Bd);
K_s = single(Kinf);
M_s = Ad_s - Bd_s*K_s;
g8 = max(abs(eig(double(M_s))));
g8_diff = abs(g8 - ecl);
g7_P_s = min(eig(double((single(Pinf) + single(Pinf)')/2)));
g7_Q_s = min(eig(double((single(QuuInv) + single(QuuInv)')/2)));

fprintf('==== Leg4 单 MPC 离线缓存（%d 态 / %d 输入 / NH=%d / Ts=%.3f s）====\n', NX, NU, p.NH, p.Ts);
fprintf('调度点：lL = lR = %.3f m，psi = %.2f rad，theta_leg = %.6f rad，theta_b = %.3f rad\n', ...
        lh, psi, eq.theta_leg, p.sym7.pitch_ref);
fprintf('u_ss = [%s]（前 4 路 N·m，后 2 路 N）\n', num2str(u_ss', '%+.4f '));
fprintf('\n');

fprintf('-- G1~G8 --\n');
fprintf('   G1 连续定点 max|A*xbar+B*u_cont+f| = %.2e（<=1e-12）\n', g1);
fprintf('   G2 离散定点 max|Ad*xbar+Bd*u_ss+fd-xbar| = %.2e（<=1e-12）\n', g2);
fprintf('   G3 DARE 残差 绝对 %.2e / 相对 %.2e（<=1e-9）\n', g3_abs, g3_rel);
fprintf('   G4 max|QuuInv*(R+rho*I+Bd''*Pinf*Bd)-I| = %.2e（<=1e-9）\n', g4);
fprintf('   G5 max|AmBKt-(Ad-Bd*Kinf)''| = %.2e（=0）\n', g5);
fprintf('   G6 闭环 |eig|max = %.9f（<1）；SDA 基线 %.9f，差 %.2e（<=1e-6）\n', ecl, eig_sda, g6_diff);
fprintf('   G7 min eig(Pinf) = %.2e，min eig(QuuInv) = %.2e（>0）\n', g7_P, g7_Q);
fprintf('   G8 单精度闭环 |eig|max = %.9f（<1），与 double 差 %.2e（<=1e-4）\n', g8, g8_diff);
fprintf('   INFO：Pinf(double) vs SDA 相对差 %.2e；单精度 min eig(Pinf) = %.2e，min eig(QuuInv) = %.2e\n', ...
        pinf_rel, g7_P_s, g7_Q_s);
fprintf('   INFO：u_ss 在箱内 %s；力分量 %+.4f / %+.4f N\n', ...
        mat2str(all(u_ss >= u_min & u_ss <= u_max)), u_ss(5), u_ss(6));
fprintf('\n');

%% ---- 写 staging ----
write_tinympc14_data(p.STAGING_DIR, Ad, Bd, fd, Kinf, Pinf, QuuInv, AmBKt, ...
                q_diag, r_diag, u_min, u_max, x_min, x_max, du_rate, p);
fprintf('已写 tinympc14_data.h/.c 到 %s\n', p.STAGING_DIR);

%% ---- 生成物字段检查 ----
hdr = fileread(fullfile(p.STAGING_DIR, 'tinympc14_data.h'));
src = fileread(fullfile(p.STAGING_DIR, 'tinympc14_data.c'));
need_h = {'TINYMPC14_IFACE_VER', 'TINYMPC14_NX', 'TINYMPC14_NU', ...
          'TINYMPC14_NH', 'TINYMPC14_RHO', 'TINYMPC14_TS', ...
          'TINYMPC14_H_REF', 'TINYMPC14_PITCH_REF', ...
          'TINYMPC14_GRAV_FF', 'TINYMPC14_ROLL_TRACK', 'TINYMPC14_ROLL_CLAMP', ...
          'tinympc14_model_t', 'tinympc14_q_diag', 'tinympc14_r_diag', ...
          'tinympc14_u_min', 'tinympc14_u_max', 'tinympc14_x_min', ...
          'tinympc14_x_max', 'tinympc14_du_rate'};
miss = need_h(~cellfun(@(s) contains(hdr, s), need_h));
rhotok = regexp(hdr, '#define\s+TINYMPC14_RHO\s+([^\s/]+)', 'tokens', 'once');
if isempty(rhotok)
    rho_txt = '(missing)';
else
    rho_txt = rhotok{1};
end
rho_ok = ~isempty(regexp(rho_txt, '^\d+\.\d+f?$', 'once'));
if isempty(miss)
    miss_txt = '(none)';
else
    miss_txt = strjoin(miss, ', ');
end
fprintf('字段缺失：%s；RHO 字面量：%s\n', miss_txt, rho_txt);
assert(isempty(miss) && rho_ok, '生成物字段或 RHO 字面量不合格');

assert(g1 <= 1e-12, 'G1 未通过');
assert(g2 <= 1e-12, 'G2 未通过');
assert(g3_rel <= 1e-9, 'G3 未通过');
assert(g4 <= 1e-9, 'G4 未通过');
assert(g5 == 0, 'G5 未通过');
assert(ecl < 1 && g6_diff <= 1e-6, 'G6 未通过');
assert(g7_P > 0 && g7_Q > 0, 'G7 未通过');
assert(g8 < 1 && g8_diff <= 1e-4, 'G8 未通过');
fprintf('S3 判据全部通过\n');

%% ========================================================================
end

function write_tinympc14_data(out_dir, Ad, Bd, fd, Kinf, Pinf, QuuInv, AmBKt, ...
                         q_diag, r_diag, u_min, u_max, x_min, x_max, du_rate, p)
if ~exist(out_dir, 'dir'), mkdir(out_dir); end
NX = p.NX; NU = p.NU; NH = p.NH;
stamp = datestr(now, 'yyyy-mm-dd HH:MM:SS');

rhotxt = sprintf('%.9g', p.rho);
if ~any(rhotxt == '.') && ~any(rhotxt == 'e') && ~any(rhotxt == 'E')
    rhotxt = [rhotxt '.0'];
end

temporary_h=[tempname(out_dir) '.h'];
temporary_c=[tempname(out_dir) '.c'];
fh = fopen(temporary_h, 'w','n','UTF-8');
assert(fh>=0,'Cannot write temporary nominal header');
fprintf(fh, '// Leg4 单 MPC 完整离线缓存（单点 lL=lR=%.2f m，psi=%.2f，theta_b=%.3f）\n', ...
        p.sym7.sched_l, p.sym7.sched_psi, p.sym7.pitch_ref);
fprintf(fh, '// 由 Matlab/mixmpc_pre/deploy/s03_tinympc14_cache.m 生成，勿手改；重新部署请重跑该脚本\n');
fprintf(fh, '#ifndef TINYMPC14_DATA_H\n#define TINYMPC14_DATA_H\n\n');
fprintf(fh, '#define TINYMPC14_IFACE_VER 3  /* physical parameters and corrected objective */\n');
fprintf(fh, '#define TINYMPC14_NX %d\n#define TINYMPC14_NU %d\n#define TINYMPC14_NH %d\n', NX, NU, NH);
fprintf(fh, '#define TINYMPC14_RHO %sf  /* ADMM 罚参数，已烘焙进矩阵，运行时不得偏离 */\n', rhotxt);
fprintf(fh, '#define TINYMPC14_MAX_ITER %d\n', p.max_iter);
fprintf(fh, '#define TINYMPC14_PRI_TOL %.9ef\n', p.pri_tol);
fprintf(fh, '#define TINYMPC14_DUA_TOL %.9ef\n', p.dua_tol);
if isfield(p,'ROLL_TIP_GAIN'), fprintf(fh, '#define TINYMPC14_ROLL_TIP_GAIN %.9ef\n', p.ROLL_TIP_GAIN); end
fprintf(fh, '#define TINYMPC14_TS %.3ff   /* s，预测节点间隔 */\n', p.Ts);
fprintf(fh, '#define TINYMPC14_H_REF %.9ef\n', p.H_REF);
fprintf(fh, '#define TINYMPC14_PITCH_REF %.9ef\n', p.sym7.pitch_ref);
fprintf(fh, '#define TINYMPC14_F_MIN %.2ff /* N，单腿总支持力下限，箱界按 −ffwd 平移后约束总量 */\n', p.F_MIN);
fprintf(fh, '#define TINYMPC14_F_MAX %.2ff /* N，单腿总支持力上限，箱界按 −ffwd 平移后约束总量 */\n', p.F_MAX);
fprintf(fh, '#define TINYMPC14_DF_RATE %.2ff /* N/步，力变化率上限，只走力通道 */\n', p.DF_RATE);
fprintf(fh, '#define TINYMPC14_A_MAX %.2ff /* rad，横滚角软上限 */\n', p.A_MAX);
fprintf(fh, '#define TINYMPC14_GRAV_FF %.9ef /* kg，MgF 插件重力前馈口径 */\n', p.COMP.LEG_GRAV_FF);
fprintf(fh, '#define TINYMPC14_ROLL_TRACK %.9ef /* measured full track, m */\n', p.COMP.ROLL_TRACK);
fprintf(fh, '#define TINYMPC14_G %.9ef\n', p.g);
fprintf(fh, '#define TINYMPC14_WHEEL_R %.9ef\n', p.R_w);
fprintf(fh, '#define TINYMPC14_LEG_MASS %.9ef\n', p.m_l);
fprintf(fh, '#define TINYMPC14_HALF_TRACK %.9ef\n', p.R_TRK);
fprintf(fh, '#define TINYMPC14_M_SUP %.9ef\n', p.M_SUP);
if isfield(p,'M_LOAD')
    fprintf(fh, '#define TINYMPC14_M_LOAD %.9ef /* kg, axial gravity load at nominal height */\n', p.M_LOAD);
else
    fprintf(fh, '#define TINYMPC14_M_LOAD TINYMPC14_M_SUP\n');
end
fprintf(fh, '#define TINYMPC14_H_OFF %.9ef\n', p.H_OFF);
if isfield(p,'CAD_MODEL')
    fprintf(fh,'#define TINYMPC14_CAD_MODEL 1\n');
    fprintf(fh,'#define TINYMPC14_WORK_LEN_MIN %.12ef\n',p.WORK_LEN_MIN);
    fprintf(fh,'#define TINYMPC14_WORK_MASS_MAX %.12ef\n',p.WORK_MASS_MAX);
    fprintf(fh,'#define TINYMPC14_WORK_LOAD_MIN %.12ef\n',p.WORK_LOAD_MIN);
    fprintf(fh,'#define TINYMPC14_WORK_LOAD_MAX %.12ef\n',p.WORK_LOAD_MAX);
    fprintf(fh,'#define TINYMPC14_L_MIN %.12ff\n#define TINYMPC14_L_MAX %.12ff\n',p.LEG_RANGE);
    fprintf(fh,'#define TINYMPC14_M_TOTAL %.12ef\n#define TINYMPC14_M_WHEEL %.12ef\n',tinympc14_m_tot(p),p.m_w);
    fprintf(fh,'#define TINYMPC14_CAD_ROWS %d\n',size(p.HR_DATA,1));
    fprintf(fh,'/* H [m], roll tip slope [N m/rad], whole mass * ground CoM height [kg m]. */\n');
    fprintf(fh,'extern const float tinympc14_cad_table[TINYMPC14_CAD_ROWS][3];\n');
end
fprintf(fh, '#define TINYMPC14_ROLL_CLAMP %.1ff /* N，向心力插件单侧限幅 */\n\n', p.COMP.ROLL_CLAMP);
fprintf(fh, '/* 全部矩阵行优先展平：M[i*cols+j]，状态序 [s ds phi dphi th_ll dth_ll th_lr dth_lr th_b dth_b a da H dH] */\n');
fprintf(fh, '/* 输入序 [T_wl T_wr T_bl T_br F_l F_r]：前 4 路 N·m，后 2 路 N；跨链耦合已常驻 A/B */\n');
fprintf(fh, '/* fd is the physical affine linearization residual, including vertical gravity. */\n');
fprintf(fh, '/* External code only: Tp=TB+MgF; Ftotal=clip(Fmpc+Fc); Fmotor=Ftotal-Fs. */\n');
fprintf(fh, 'typedef struct\n{\n');
fprintf(fh, '    float Ad[TINYMPC14_NX * TINYMPC14_NX];\n');
fprintf(fh, '    float Bd[TINYMPC14_NX * TINYMPC14_NU];\n');
fprintf(fh, '    float fd[TINYMPC14_NX];\n');
fprintf(fh, '    float Kinf[TINYMPC14_NU * TINYMPC14_NX];\n');
fprintf(fh, '    float Pinf[TINYMPC14_NX * TINYMPC14_NX];\n');
fprintf(fh, '    float QuuInv[TINYMPC14_NU * TINYMPC14_NU];\n');
fprintf(fh, '    float AmBKt[TINYMPC14_NX * TINYMPC14_NX];\n');
fprintf(fh, '} tinympc14_model_t;\n\n');
fprintf(fh, 'extern const tinympc14_model_t tinympc14_model;\n');
fprintf(fh, 'extern const float tinympc14_q_diag[TINYMPC14_NX];\n');
fprintf(fh, 'extern const float tinympc14_r_diag[TINYMPC14_NU];\n');
fprintf(fh, 'extern const float tinympc14_u_min[TINYMPC14_NU];\n');
fprintf(fh, 'extern const float tinympc14_u_max[TINYMPC14_NU];\n');
fprintf(fh, 'extern const float tinympc14_x_min[TINYMPC14_NX];\n');
fprintf(fh, 'extern const float tinympc14_x_max[TINYMPC14_NX];\n');
fprintf(fh, 'extern const float tinympc14_du_rate[TINYMPC14_NU]; /* 前 4 路 1e6 = 不设速率；后 2 路 N/步 */\n');
fprintf(fh, '\n#endif\n');
fclose(fh);

fc = fopen(temporary_c, 'w','n','UTF-8');
assert(fc>=0,'Cannot write temporary nominal source');
fprintf(fc, '// Leg4 单 MPC 完整离线缓存（单点，psi=0，无调度）\n');
fprintf(fc, '// 由 Matlab/mixmpc_pre/deploy/s03_tinympc14_cache.m 生成于 %s，勿手改；重新部署请重跑该脚本\n', stamp);
fprintf(fc, '#include "tinympc14_data.h"\n\n');
if isfield(p,'CAD_MODEL')
    fprintf(fc,'const float tinympc14_cad_table[TINYMPC14_CAD_ROWS][3] =\n{\n');
    for k=1:size(p.HR_DATA,1)
        H=p.HR_DATA(k,1); first_moment=tinympc14_table_row(p.TURN_DATA,H);
        fprintf(fc,'    {%.12ef, %.12ef, %.12ef},\n',H,p.HR_DATA(k,7),first_moment);
    end
    fprintf(fc,'};\n\n');
end

fprintf(fc, 'const tinympc14_model_t tinympc14_model =\n{\n');
write_mat(fc, Ad, 'Ad');
write_mat(fc, Bd, 'Bd');
write_prop_vec(fc, fd, 'fd：机身与腿质心偏置、升降重力和横滚静差的离散仿射，14 x 1');
write_mat(fc, Kinf, 'Kinf');
write_mat(fc, Pinf, 'Pinf');
write_mat(fc, QuuInv, 'QuuInv');
write_mat(fc, AmBKt, 'AmBKt');
fprintf(fc, '};\n\n');

write_vec(fc, q_diag, 'tinympc14_q_diag');
write_vec(fc, r_diag, 'tinympc14_r_diag');
write_vec(fc, u_min, 'tinympc14_u_min');
write_vec(fc, u_max, 'tinympc14_u_max');
write_vec(fc, x_min, 'tinympc14_x_min');
write_vec(fc, x_max, 'tinympc14_x_max');
write_vec(fc, du_rate, 'tinympc14_du_rate');
fclose(fc);
replace_file(temporary_h,fullfile(out_dir,'tinympc14_data.h'));
replace_file(temporary_c,fullfile(out_dir,'tinympc14_data.c'));
end

function replace_file(source,destination)
for attempt=1:11
    [ok,msg]=movefile(source,destination,'f');
    if ok, return; end
    if attempt<11, pause(.1); end
end
error('tinympc14:WriteFailed','Cannot replace %s: %s',destination,msg);
end

function write_mat(fid, M, name)
[r, c] = size(M);
fprintf(fid, '    { /* %s：%d x %d，行序 k = i*%d+j */\n', name, r, c, c);
Mv = M.'; Mv = Mv(:).';
for i = 1:numel(Mv)
    fprintf(fid, '    %.9ef%s', Mv(i), pick_sep(i, numel(Mv), c));
end
fprintf(fid, '    },\n');
end

function write_vec(fid, v, name)
fprintf(fid, 'const float %s[%d] =\n{\n    ', name, numel(v));
for i = 1:numel(v)
    fprintf(fid, '%.9ef%s', v(i), pick_sep(i, numel(v), numel(v)));
end
fprintf(fid, '};\n\n');
end

function write_prop_vec(fid, v, label)
fprintf(fid, '    { /* %s */\n', label);
for i = 1:numel(v)
    fprintf(fid, '    %.9ef%s', v(i), pick_sep(i, numel(v), numel(v)));
end
fprintf(fid, '    },\n');
end

function s = pick_sep(i, n, row_len)
if i == n
    s = sprintf('\n');
elseif mod(i, row_len) == 0
    s = sprintf(',\n');
else
    s = ', ';
end
end

function [K, P] = dare_sda(A, B, Q, R)
% 结构保持倍增 DARE（double 独立基线）；K 与 dlqr 同式
n = size(A, 1);
G = B * (R \ B');
H = Q;
Ak = A;
for it = 1:100
    IGH = eye(n) + G * H;
    Ai = IGH \ Ak;
    Hn = H + Ak' * H * Ai;
    Gn = G + Ak * (IGH \ (G * Ak'));
    An = Ak * Ai;
    Hn = (Hn + Hn') / 2;
    if it > 1 && max(abs(Hn(:) - H(:))) < 1e-14 * max(1, max(abs(Hn(:))))
        H = Hn;
        break;
    end
    H = Hn; G = Gn; Ak = An;
end
P = H;
K = (R + B' * P * B) \ (B' * P * A);
end
