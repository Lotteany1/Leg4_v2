function tinympc14_zoh_generate(fast_mode, staging_dir, p, cache)
if nargin < 1, fast_mode = false; end
% s07_tinympc14_zoh_data.m
% Leg4 f_extra 的 ZOH 系数表：单点 lL=lR=0.27 m / psi=0
% 把 zoh 段写入 staging/tinympc14_data.h/.c 的带标记段：4x14 系数 + 自检，并刷新来源清单
% 版本：v0.2  日期：2026-10-20
% 用法：matlab -batch "cd('E:/DevelopLotteany1/0_Code/Leg4_v1_ultimate/Matlab/mixmpc_pre/deploy'); s07_tinympc14_zoh_data"


addpath(fileparts(fileparts(mfilename('fullpath')))); mixmpc_paths();   % mpc 顶层：tinympc14_config
if nargin < 3, p = tinympc14_config(); end
if nargin >= 2, p.STAGING_DIR = staging_dir; end
NX = p.NX;
rows0 = [1 9 11 13];
rowsm = rows0 + 1;

if fast_mode
    if nargin<4, cache = qr_build_cache('read', fileparts(fileparts(mfilename('fullpath')))); end
    fz = cache.zoh.fz; err = cache.zoh.err;
else
sched = tinympc14_sched_pt(p, p.sym7.sched_l, 0, p.sym7.sched_psi);
[A, ~, ~] = tinympc14_fused_ab(p, p.sym7.sw_fused, sched);

Fz = zeros(NX, NX);
for j = 1:NX
    ej = zeros(NX, 1);
    ej(j) = 1;
    [~, ~, Fz(:, j)] = tinympc14_zoh_ab(A, [], ej, p.Ts);
end
Fz4 = Fz(:, rowsm);
fz = Fz4.';

fe = zeros(NX, 1);
fe(rowsm) = [0.3; -0.2; 0.1; -0.05];
[~, ~, fed_ref] = tinympc14_zoh_ab(A, [], fe, p.Ts);
fed_tab = Fz4 * fe(rowsm);
err = max(abs(fed_tab - fed_ref)) / max(1, max(abs(fed_ref)));

end
fprintf('ZOH 4x14 表  NX=%d NF=%d  Ts=%.4f s\n', NX, numel(rows0), p.Ts);
fprintf('自检 max|fed_tab - fed_exact| / max|fed_exact| = %.3e\n', err);
assert(err <= 1e-6, 'ZOH table self-check failed');

build_dir = fileparts(p.STAGING_DIR);
if ~exist(build_dir, 'dir'), mkdir(build_dir); end
save(fullfile(build_dir, 'tinympc14_zoh_cache.mat'), 'fz', 'rows0', '-v7');
if ~fast_mode
    model = struct('fz', fz, 'rows0', rows0, 'err', err);
    save(fullfile(build_dir, 'qr_zoh_model.mat'), 'model', '-v7');
end

stg = p.STAGING_DIR;
hname = fullfile(stg, 'tinympc14_data.h');
cname = fullfile(stg, 'tinympc14_data.c');

htm = [tempname(stg) '.txt'];
fid = fopen(htm, 'w');
fprintf(fid, '// Leg4 f_extra 的 ZOH 系数表（单点 lL=lR=0.27 m，psi=0）\n');
fprintf(fid, '// 由 Matlab/mixmpc_pre/deploy/s07_tinympc14_zoh_data.m 生成，勿手改；重新部署请重跑该脚本\n');
fprintf(fid, '#define TINYMPC14_ZOH_IFACE_VER 1\n');
fprintf(fid, '#define TINYMPC14_ZOH_NX 14\n');
fprintf(fid, '#define TINYMPC14_ZOH_NF 4\n\n');
fprintf(fid, '/* fed[i] = sum_r tinympc14_fz[r*NX+i] * fe[rows[r]]，rows = {1,9,11,13}（C 0-based） */\n');
fprintf(fid, 'extern const float tinympc14_fz[TINYMPC14_ZOH_NF][TINYMPC14_ZOH_NX];\n');
fclose(fid);
hsec = fileread(htm);
delete(htm);

ctm = [tempname(stg) '.txt'];
fid = fopen(ctm, 'w');
fprintf(fid, '// Leg4 f_extra 的 ZOH 系数表（单点 lL=lR=0.27 m，psi=0）\n');
fprintf(fid, '// 由 Matlab/mixmpc_pre/deploy/s07_tinympc14_zoh_data.m 生成于 %s，勿手改\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
fprintf(fid, 'const float tinympc14_fz[TINYMPC14_ZOH_NF][TINYMPC14_ZOH_NX] =\n{\n');
for r = 1:size(fz, 1)
    fprintf(fid, '    { ');
    for i = 1:size(fz, 2)
        if i < size(fz, 2)
            fprintf(fid, '%.9ef, ', fz(r, i));
        else
            fprintf(fid, '%.9ef ', fz(r, i));
        end
    end
    fprintf(fid, '}, /* r=%d fe(%d) */\n', r - 1, rows0(r));
end
fprintf(fid, '};\n');
fclose(fid);
csec = fileread(ctm);
delete(ctm);

tinympc14_section(hname, 'zoh', hsec);
tinympc14_section(cname, 'zoh', csec);
fprintf('写入 %s 与 %s 的 zoh 段\n', hname, cname);
if nargin < 2
    tinympc14_data_manifest('tinympc14_data','deploy/s07_tinympc14_zoh_data.m');
end

end
