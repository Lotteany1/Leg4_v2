function tinympc14_generate_vectors(xb0, uss0, p, out_dir)
% Regenerate current-weight C comparison vectors, without running long closed loops.
here = fileparts(mfilename('fullpath'));
addpath(fileparts(here)); mixmpc_paths();
if nargin < 3, p = tinympc14_config(); end
if nargin < 4, out_dir = fullfile(fileparts(here), 'build', 'host'); end
NX = p.NX; NU = p.NU;
umin = p.u_min(:); umax = p.u_max(:);
%% ---- C 对拍向量 ----
V = struct('x0', {}, 'xref', {}, 'fed', {}, 'ffwd', {}, 'u_prev', {}, ...
           'lL', {}, 'lR', {}, 'u_exp', {}, 'conv', {}, 'dua', {});

V = add_vec(V, p, 0.27, 0.27, xb0, xb0, zeros(NX,1), zeros(NU,1), uss0);
V = add_vec(V, p, p.LEG_RANGE(1), p.LEG_RANGE(1), xb0, xb0, zeros(NX,1), zeros(NU,1), uss0);
V = add_vec(V, p, 0.34, 0.34, xb0, xb0, zeros(NX,1), zeros(NU,1), uss0);
V = add_vec(V, p, p.LEG_RANGE(1), 0.34, xb0, xb0, zeros(NX,1), zeros(NU,1), uss0);
x5 = xb0;  x5(13) = x5(13) + 0.15;
V = add_vec(V, p, 0.27, 0.27, x5, xb0, zeros(NX,1), zeros(NU,1), uss0);
x6ref = xb0;  x6ref(13) = x6ref(13) + 0.05;
up6 = uss0;  up6(5:6) = p.F_MIN;
V = add_vec(V, p, 0.27, 0.27, xb0, x6ref, zeros(NX,1), zeros(NU,1), up6);
x7 = xb0;  x7(11) = 2.0;
V = add_vec(V, p, 0.27, 0.27, x7, xb0, zeros(NX,1), zeros(NU,1), uss0);
fed8 = zeros(NX,1);  fed8([2 10 12 14]) = 1e-3*[0.3; -0.2; 0.1; -0.05];
ff8 = [zeros(NU-2,1); 8; -8];
V = add_vec(V, p, 0.27, 0.27, xb0, xb0, fed8, ff8, uss0);
if isfield(p,'ONLINE_TERMINAL')
    % Previously failing regions of the polynomial model, including a
    % clamped input to exercise the C scheduler thresholds and float bounds.
    for sample = [.1766,.2225,.331,.173; .1733,.308,.176,.337; -.2467,.169,.24,-.34]
        state = xb0;
        state([5,7]) = sample(3)+[.02;-.02];
        V = add_vec(V,p,sample(1),sample(2),state,xb0,fed8,ff8,uss0);
    end
    stream = RandStream('mt19937ar','Seed',20261117);
    for sample = [.16+.18*rand(stream,2,200); -.3+.6*rand(stream,1,200)]
        state = xb0;
        state([5,7]) = sample(3)+[.015;-.015];
        V = add_vec(V,p,sample(1),sample(2),state,xb0,fed8,ff8,uss0);
    end
end

fprintf('\n-- C 固定对拍向量（共 %d 例）--\n',numel(V));
for i = 1:min(numel(V),12)
    fprintf('   #%d lL=%.2f lR=%.2f conv=%d max|u|=%.3f\n', ...
            i, V(i).lL, V(i).lR, V(i).conv, max(abs(V(i).u_exp)));
end

%% ---- 随机批量（模型固定 0.27，热启动延续）----
rng(20261020);
NR = 1000;
sx = [0.02; 0.2; 0.03; 0.3; 0.00; 0.5; 0.00; 0.5; 0.02; 0.2; 0.05; 1.0; 0.02; 0.5];  % 腿角不放扰：模型 psi 与 C 侧同点
Rand = struct('x0', {}, 'xref', {}, 'fed', {}, 'ffwd', {}, 'u_prev', {}, ...
              'u_exp', {}, 'conv', {}, 'iters', {}, 'pri', {}, 'dua', {});
warm = [];
for i = 1:NR
    x0r = double(single(xb0 + sx .* (2*rand(NX,1) - 1)));
    xrr = double(single(xb0 + sx .* (2*rand(NX,1) - 1)));
    fer = zeros(NX,1);
    fer([2 10 12 14]) = single(0.5 * (2*rand(4,1) - 1));
    fedr = double(single(1e-3 * fer));   % 与旧 ZOH 同量级的仿射扰动
    ffr = double(single([zeros(NU-2,1); 10*(2*rand(2,1) - 1)]));
    upr = double(single(min(max(uss0 + [0.5;0.5;0.5;0.5;10;10] .* (2*rand(NU,1) - 1), ...
                            umin), umax)));
    [ur, infr, warm] = tinympc14_solve_reference(p, double(single(0.27)), double(single(0.27)), ...
                                 x0r, xrr, fedr, ffr, upr, warm);
    Rand(i).x0 = x0r;
    Rand(i).xref = xrr;
    Rand(i).fed = fedr;
    Rand(i).ffwd = ffr;
    Rand(i).u_prev = upr;
    Rand(i).u_exp = ur;
    Rand(i).conv = infr.converged;
    Rand(i).iters = infr.iters;
    Rand(i).pri = infr.pri;
    Rand(i).dua = infr.dua;
end
n_conv = sum([Rand.conv]);
fprintf('\n-- 随机批量 %d 例：收敛 %d，回退 %d --\n', NR, n_conv, NR - n_conv);

%% ---- 补偿插件向量（输入先过 single，每段从复位状态起跑）----
% 只对拍序列末步：三笔输出与单笔有效性状态；列序 len_l len_r ang_l ang_r v omega
CVV0 = [0.27 0.27 0.00 0.00 0.5 1.0];
CVVNaN = [0.27 0.27 0.00 0.00 0.5 NaN];
SEQC = {
    [0.27 0.27 0.00 0.00 0.0 0.0];
    [p.LEG_RANGE(1) 0.34 0.10 -0.05 0.8 1.5];
    [0.20 0.33 0.05 0.20 -1.2 0.7];
    [0.34 p.LEG_RANGE(1) -0.30 0.40 2.0 -3.0];
    [0.27 NaN  0.00 0.00 0.5 1.0];
    CVVNaN;
    [NaN  NaN  0.00 0.00 0.5 1.0];
    [0.27 0.27 NaN  0.00 0.5 1.0];
    [CVV0; CVVNaN];
    [CVV0; 0.27 NaN  0.00 0.00 0.5 1.0];
    [CVV0; 0.27 0.27 NaN  0.00 0.5 1.0];
    [CVV0; repmat(CVVNaN, 6, 1)];
    [CVV0; CVVNaN; CVV0];
};
NCV_STEP = max(cellfun(@(s) size(s, 1), SEQC));
Comp = struct('n', {}, 'in', {}, 'MgF', {}, 'Fs', {}, 'Fc', {}, 'ffwd', {}, 'st', {});
for i = 1:numel(SEQC)
    seq = double(single(SEQC{i}));
    stc = [];
    for k = 1:size(seq, 1)
        [cc, stc] = tinympc14_output(p, seq(k,1), seq(k,2), seq(k,3), seq(k,4), seq(k,5), seq(k,6), stc);
    end
    ins = zeros(NCV_STEP, 6);
    ins(1:size(seq,1), :) = seq;
    Comp(i).n = size(seq, 1);
    Comp(i).in = ins;
    Comp(i).MgF = cc.MgF;
    Comp(i).Fs = cc.Fs;
    Comp(i).Fc = cc.Fc;
    Comp(i).ffwd = cc.ffwd;
    Comp(i).st = cc.st;
end
fprintf('\n-- 补偿向量 %d 段（含单侧失效 无关失效 沿用超时 恢复）--\n', numel(Comp));

%% ---- 写向量文件 ----
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

fh = fopen(fullfile(out_dir, 'tinympc14_vectors.h'), 'w');
fprintf(fh, '/* 由 Matlab/mpc/parts/s08_tinympc14_runtime_check.m 生成，勿手改 */\n');
fprintf(fh, '#ifndef TINYMPC14_VECTORS_H\n#define TINYMPC14_VECTORS_H\n\n#include <math.h>\n\n');
fprintf(fh, 'typedef struct\n{\n');
fprintf(fh, '    float x0[14], xref[14], fed[14], ffwd[6], u_prev[6];\n');
fprintf(fh, '    float lL, lR;\n');
fprintf(fh, '    float u_expect[6];\n');
fprintf(fh, '    int   conv_expect;\n');
fprintf(fh, '    float dua_expect;   /* 参考解最后一次迭代的对偶残差 */\n');
fprintf(fh, '} tinympc14_vec_t;\n\n');
fprintf(fh, 'static const tinympc14_vec_t tinympc14_vectors[] =\n{\n');
for i = 1:numel(V)
    fprintf(fh, '    {\n');
    fprintf(fh, '        {%s},\n', cf_list(V(i).x0));
    fprintf(fh, '        {%s},\n', cf_list(V(i).xref));
    fprintf(fh, '        {%s},\n', cf_list(V(i).fed));
    fprintf(fh, '        {%s},\n', cf_list(V(i).ffwd));
    fprintf(fh, '        {%s},\n', cf_list(V(i).u_prev));
    fprintf(fh, '        %.9ef, %.9ef,\n', single(V(i).lL), single(V(i).lR));
    fprintf(fh, '        {%s},\n', cf_list(V(i).u_exp));
    fprintf(fh, '        %d,\n', V(i).conv);
    fprintf(fh, '        %.9ef\n', single(V(i).dua));
    fprintf(fh, '    },\n');
end
fprintf(fh, '};\n\n#endif\n');
fclose(fh);

fh = fopen(fullfile(out_dir, 'tinympc14_rand_vectors.h'), 'w');
fprintf(fh, '/* 由 Matlab/mpc/parts/s08_tinympc14_runtime_check.m 生成，勿手改 */\n');
fprintf(fh, '/* 模型固定 lL=lR=0.27；求解器 Init 一次后逐例 Solve 热启动延续 */\n');
fprintf(fh, '#ifndef TINYMPC14_RAND_VECTORS_H\n#define TINYMPC14_RAND_VECTORS_H\n\n#include <math.h>\n\n');
fprintf(fh, 'typedef struct\n{\n');
fprintf(fh, '    float x0[14], xref[14], fed[14], ffwd[6], u_prev[6];\n');
fprintf(fh, '    float u_expect[6];\n');
fprintf(fh, '    int   conv_expect;\n');
fprintf(fh, '    float dua_expect;   /* 参考解最后一次迭代的对偶残差 */\n');
fprintf(fh, '} tinympc14_rand_vec_t;\n\n');
fprintf(fh, 'static const tinympc14_rand_vec_t tinympc14_rand_vectors[%d] =\n{\n', NR);
for i = 1:NR
    fprintf(fh, '    {\n');
    fprintf(fh, '        {%s},\n', cf_list(Rand(i).x0));
    fprintf(fh, '        {%s},\n', cf_list(Rand(i).xref));
    fprintf(fh, '        {%s},\n', cf_list(Rand(i).fed));
    fprintf(fh, '        {%s},\n', cf_list(Rand(i).ffwd));
    fprintf(fh, '        {%s},\n', cf_list(Rand(i).u_prev));
    fprintf(fh, '        {%s},\n', cf_list(Rand(i).u_exp));
    fprintf(fh, '        %d,\n', Rand(i).conv);
    fprintf(fh, '        %.9ef\n', single(Rand(i).dua));
    fprintf(fh, '    },\n');
end
fprintf(fh, '};\n\n#endif\n');
fclose(fh);

fh = fopen(fullfile(out_dir, 'tinympc14_comp_vectors.h'), 'w');
fprintf(fh, '/* 由 Matlab/mpc/parts/s08_tinympc14_runtime_check.m 生成，勿手改 */\n');
fprintf(fh, '#ifndef TINYMPC14_COMP_VECTORS_H\n#define TINYMPC14_COMP_VECTORS_H\n\n#include <math.h>\n\n');
fprintf(fh, '#define TINYMPC14_COMP_VEC_STEP %d\n\n', NCV_STEP);
fprintf(fh, 'typedef struct\n{\n');
fprintf(fh, '    int   n_step;                     /* 有效步数 序列从复位状态起跑 */\n');
fprintf(fh, '    float in[TINYMPC14_COMP_VEC_STEP][6];  /* len_l len_r ang_l ang_r v omega 未用行补 0 */\n');
fprintf(fh, '    float MgF[2], Fs[2], Fc[2], ffwd[6];\n');
fprintf(fh, '    unsigned char st[3][2];           /* 单笔状态 行序 MgF Fs Fc 0 无可用值 1 有效 2 沿用 */\n');
fprintf(fh, '} tinympc14_comp_vec_t;\n\n');
fprintf(fh, 'static const tinympc14_comp_vec_t tinympc14_comp_vectors[] =\n{\n');
for i = 1:numel(Comp)
    fprintf(fh, '    {\n');
    fprintf(fh, '        %d,\n', Comp(i).n);
    fprintf(fh, '        {\n');
    for k = 1:NCV_STEP
        fprintf(fh, '            {%s},\n', cf_list(Comp(i).in(k,:)));
    end
    fprintf(fh, '        },\n');
    fprintf(fh, '        {%s},\n', cf_list(Comp(i).MgF));
    fprintf(fh, '        {%s},\n', cf_list(Comp(i).Fs));
    fprintf(fh, '        {%s},\n', cf_list(Comp(i).Fc));
    fprintf(fh, '        {%s},\n', cf_list(Comp(i).ffwd));
    fprintf(fh, '        {{%d,%d},{%d,%d},{%d,%d}},\n', Comp(i).st(1,1), Comp(i).st(1,2), ...
            Comp(i).st(2,1), Comp(i).st(2,2), Comp(i).st(3,1), Comp(i).st(3,2));
    fprintf(fh, '    },\n');
end
fprintf(fh, '};\n\n#endif\n');
fclose(fh);

save(fullfile(fileparts(out_dir), 'tinympc14_solver_vectors.mat'), 'V', 'Rand', 'Comp', 'xb0', 'uss0', '-v7');
fprintf('\n已写 solver/test/tinympc14_vectors.h（%d 例）、tinympc14_rand_vectors.h（%d 例）、tinympc14_comp_vectors.h（%d 例）\n', ...
        numel(V), NR, numel(Comp));



end

function V = add_vec(V, p, lL, lR, x0, xref, fed, ffwd, u_prev)
x0 = double(single(x0(:)));  xref = double(single(xref(:)));
fed = double(single(fed(:)));  ffwd = double(single(ffwd(:)));
u_prev = double(single(u_prev(:)));
lL = double(single(lL));  lR = double(single(lR));
psim=double(0.5*single(single(x0(5))+single(x0(7))));
if max(abs([lL-.27,lR-.27]))<=.005 && abs(psim)<=.05
    psim=0;
end
[u, info] = tinympc14_solve_reference(p, lL, lR, x0, xref, fed, ffwd, u_prev, [],psim);
V(end+1).x0 = x0;
V(end).xref = xref;
V(end).fed = fed;
V(end).ffwd = ffwd;
V(end).u_prev = u_prev;
V(end).lL = lL;
V(end).lR = lR;
V(end).u_exp = u;
V(end).conv = info.converged;
V(end).dua = info.dua;
end

function s = cf_list(v)
s = sprintf('%.9ef,', single(v(:)));
s = strrep(s, '-Inff', '-INFINITY');
s = strrep(s, 'Inff', 'INFINITY');
s = strrep(s, 'NaNf', 'NAN');
s = s(1:end-1);
end

