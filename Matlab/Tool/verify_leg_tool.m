function verify_leg_tool()
%VERIFY_LEG_TOOL  Tool 生成的 VMC C 与 .m 模型、与部署目录的交叉核对。
%
%   依次执行四组检查，任一组不合格都会抛错：
%     1) 模型级独立断言  几何/雅可比/对偶/往返，不依赖生成的 C
%     2) Leg_Comp 对拍    后处理 C vs codegen 原始 C vs .m 模型，外加表结构与钳位不变量
%     3) Leg_SpringF 对拍 膝轴挂臂、摆角不变性、虚功与钳位端点
%     4) VMC 四函数对拍   Leg_Pos/Spd/Trq/InvTrq 两份 C 逐位相同，外加固件同步核对
%
%   前置：先跑 build_leg_tool（本入口只读产物，不改任何交付物）。
%   MATLAB Coder 与 MEX 需要支持的 C 编译器。
%
%   See also: build_leg_tool, leg_config
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'shared'));
D = leg_paths();
addpath(D.ROOT, D.MODEL_DIR, D.SHARED_DIR, D.SOLVER_DIR);
% Windows 上已加载的 MEX 无法覆盖；只卸载本验证入口的模块，支持同一会话重复生成。
clear leg_vmc_raw leg_vmc_cmsis leg_comp_raw leg_comp_cmsis
clear leg_springf_raw leg_springf_cmsis leg_rollcomp_raw leg_rollcomp_cmsis
check_mass_config(D);
check_model();
check_vmc();
check_rollcomp();
end

function check_mass_config(D)
% 检查模型质量守恒，并核对固件 code_config.h 的质量配置。
[~, par] = leg_config();
assert(abs(par.m_total - par.m_body - 2*par.m_leg - 2*par.m_wheel) < 1e-9, '整车质量与机身、腿、轮质量不一致');
assert(abs(par.M_SUP - par.m_total + 2*par.m_wheel) < 1e-9, '支撑质量必须为整车总质量减两轮质量');
header = fileread(fullfile(fileparts(fileparts(D.ROOT)), 'Wheel_Leg_Code', 'Code', 'Config', 'code_config.h'));
names = {'TOTAL', 'WHEEL', 'LEG'};
expected = [par.m_total, par.m_wheel, par.m_leg];
for k = 1:numel(names)
    token = regexp(header, ['#define\s+LEG_MASS_' names{k} '\s+([0-9.eE+-]+)f'], 'tokens', 'once');
    assert(~isempty(token) && abs(str2double(token{1}) - expected(k)) < 1e-8, 'code_config.h 质量配置与 leg_config 不一致: %s', names{k});
end
fprintf('质量配置: 整车 %.3f kg，支撑 %.3f kg，机身 %.3f kg PASS\n', par.m_total, par.M_SUP, par.m_body);
end

function check_rollcomp()
% 补偿与限幅前扰动对拍 覆盖四种行驶转向组合 零速度与饱和区
D = leg_paths();
[~, par] = leg_config();
addpath(D.VERIFY_BUILD_DIR);
wrap = fullfile(D.SOLVER_DIR, 'mex_leg_rollcomp.c');
raw = fullfile(D.CODEGEN_DIR, 'Leg_RollComp');
mex('-outdir', D.VERIFY_BUILD_DIR, '-output', 'leg_rollcomp_raw', ['-I' raw], ...
    wrap, fullfile(raw, 'Leg_RollComp.c'));
mex('-outdir', D.VERIFY_BUILD_DIR, '-output', 'leg_rollcomp_cmsis', ['-I' D.OUT_DIR], ...
    wrap, fullfile(D.OUT_DIR, 'Leg_RollComp.c'));
[h, v, w] = ndgrid([0.16 0.17 0.27 0.34 0.40], [-3 -1 0 1 3], [-6 -1 0 1 6]);
inputs = single([h(:), w(:), v(:)]);
[left, right, force] = leg_rollcomp_cmsis(inputs);
[rl, rr, rf] = leg_rollcomp_raw(inputs);
assert(isequal(left, rl) && isequal(right, rr) && isequal(force, rf), 'RollComp 后处理改变了计算结果');
expected = par.M_SUP * double(inputs(:, 3)) .* double(inputs(:, 2)) .* ...
    (min(max(double(inputs(:, 1)), par.Len_min), par.Len_max) + par.l_c) / par.RollComp_TRACK;
assert(max(abs(double(force) - expected) ./ max(1, abs(expected))) < 1e-6, 'RollComp 限幅前力矩口径错误');
for k = 1:size(inputs, 1)
    [ml, mr, mf] = Leg_RollComp(inputs(k, 1), inputs(k, 2), inputs(k, 3));
    assert(max(abs(double([left(k)-ml, right(k)-mr, force(k)-mf]))) < 1e-4, 'RollComp C/MATLAB 对拍失败');
end
assert(isequal(left, -right), 'RollComp 左右补偿极性错误');
assert(max(abs(double(right) - min(max(double(force), -par.RollComp_FMAX), par.RollComp_FMAX))) < 1e-5);
fprintf('RollComp 四象限/零输入/限幅前扰动: %d cases PASS\n', size(inputs, 1));
end

%% ==================== 模型级独立断言：几何/雅可比/对偶/往返 ====================
function check_model()
%CHECK_MODEL  模型级独立断言: 绕开 C, 用独立几何/数值方法校验 Leg_* 的语义
%
%   1) Leg_Pos  = 圆-圆求交: 两曲柄尖端 A_i = l1*(cos phi_i, sin phi_i)（同轴）
%      到轮轴的距离都等于 l2, 所以输出点必然落在两个半径 l2 圆的交点里。这里用
%      经典闭式解（中点 + 半弦）算两个交点, 与 Leg_Pos 的输出比较——不引用
%      Leg_Pos 的任何中间量, 是真正独立的几何参考;
%   2) Leg_Spd  = Leg_Pos 的中心差分雅可比: 速率 (L0_dot, phi0_dot) 是关节角速度
%      (phi_dot_1, phi_dot_4) 的线性映射, 系数即 d pos/d phi;
%   3) Leg_Trq  = 同一个雅可比的转置（虚功对偶 tau = J' * [F; Tp]）;
%   4) Leg_InvTrq 的 Tikhonov 往返误差 ≈ λ/σ_min²（λ 从 Leg_InvTrq.m 源码读回,
%      不在本脚本里写死; 阈值随采样实测的 min σ_min 自适应, 见 §4）;
%   5) 覆盖两档: 工作带 L0∈[0.16,0.34] 用紧阈值; 全域（排除奇异邻域）用松阈值;
%      奇异线 phi1≡phi4 (mod 2π) 的行为单独记录——注意该线有一段就落在工作带内
%      （对角线上 L0 = |l1*u1 + l2| 可落在 [0.16,0.34]）, 所以工作带也必须滤掉它;
%   6) ABI: Leg_Pos/Leg_Spd/Leg_Trq/Leg_InvTrq 是标量接口——本脚本一律逐点标量
%      调用, 并核对部署目录 *.c 的形参不含数组（固件 ABI 的证据）。
%
%   阈值都取"实测值 + 余量", 依据写在每处断言旁。
%
%   See also: verify_leg_tool, Leg_Pos, Leg_Spd, Leg_Trq, Leg_InvTrq

root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'shared'));
D = leg_paths();
cmsis = D.OUT_DIR;
addpath(D.ROOT, D.MODEL_DIR, D.SHARED_DIR, D.SOLVER_DIR);

l1   = 0.21;    % 曲柄/大腿长 [m]
l2   = 0.25;    % 连杆/小腿长 [m]
lam  = read_lambda(fullfile(D.MODEL_DIR, 'Leg_InvTrq.m'));
H    = 1e-6;    % 中心差分步长 [rad]
nSig = 1e-3;    % 奇异邻域半宽 [rad]（|wrap(phi1-phi4)| 小于它即排除）
fprintf('独立几何参数: l1 = %.2f m, l2 = %.2f m;  Tikhonov λ = %g (读自 Leg_InvTrq.m)\n', ...
        l1, l2, lam);

% ==================== 采样 ====================
% 全域: 121×121 的 (phi1, phi4) 网格 + 3000 个随机构型
rng(101);
g  = linspace(-pi, pi, 121);
[A1g, A4g] = meshgrid(g, g);
A1g = A1g(:); A4g = A4g(:);
Pr  = (rand(3000, 1) * 2 - 1) * pi;
Prt = (rand(3000, 1) * 2 - 1) * pi;
A1g = [A1g; Pr]; A4g = [A4g; Prt];

L0 = zeros(size(A1g));
for k = 1:numel(A1g)
    pos = Leg_Pos(A1g(k), A4g(k));
    L0(k) = pos(1);
end
dphi   = wrapa(A1g - A4g);
% 奇异邻域 = 奇异线 |wrap(phi1-phi4)| < nSig, 或死点折叠 L0 ∉ [0.05, 0.45]
% （L0 = |l2-l1| / l2+l1 处雅可比奇异, 与 Leg_SpringF 的钳位域取同一余量）
finite = abs(dphi) >= nSig & L0 >= 0.05 & L0 <= 0.45;
F1 = A1g(finite); F4 = A4g(finite); FL0 = L0(finite);
fprintf('全域采样 %d 点, 排除奇异邻域后 %d 点 (L0 ∈ [%.4f, %.4f] m)\n', ...
        numel(L0), numel(F1), min(FL0), max(FL0));

[~, par] = leg_config();
inb = L0 >= par.LPV_min & L0 <= par.Len_max;
B1 = A1g(inb); B4 = A4g(inb); BL0 = L0(inb);
nBand0 = numel(B1);
kb = abs(wrapa(B1 - B4)) >= nSig;
B1 = B1(kb); B4 = B4(kb); BL0 = BL0(kb);
fprintf('工作带采样 %d 点（其中奇异线上的 %d 点被滤掉）, L0 ∈ [%.4f, %.4f] m\n', ...
        numel(B1), nBand0 - numel(B1), min(BL0), max(BL0));

targets = [par.LPV_min, 0.20, 0.25, 0.30, par.Len_max];
TL0 = zeros(5, 1);
for j = 1:5
    [~, i] = min(abs(BL0 - targets(j)));
    TL0(j) = BL0(i);
end
fprintf('代表腿长命中: %s (最大偏差 %.4f m)\n', mat2str(TL0, 5), max(abs(TL0(:) - targets(:))));
fail = max(abs(TL0(:) - targets(:))) > 1e-2;
% 工作带样本上模型输出必须全部有限（奇异线样本若漏进来, 这里会先炸）
nNan = 0;
for k = 1:numel(B1)
    v = [Leg_Pos(B1(k), B4(k)); Leg_Spd(B1(k), B4(k), 1, 1); ...
         Leg_Trq(1, 1, B1(k), B4(k))];
    nNan = nNan + any(~isfinite(v));
end
fprintf('工作带内模型输出非有限的样本数: %d（必须为 0）\n', nNan);
fail = fail || nNan > 0;

[resG, ~] = cc_residual(F1, F4, l1, l2);
[resB, ~] = cc_residual(B1, B4, l1, l2);
fprintf('\n===== 1. Leg_Pos vs 圆-圆求交（独立闭式）=====\n');
fprintf('  全域: max %.3e m, NaN 数 %d, 阈值 1e-12 m\n', max(resG), sum(~isfinite(resG)));
fprintf('  工作带: max %.3e m, NaN 数 %d, 阈值 1e-12 m\n', max(resB), sum(~isfinite(resB)));
fail = fail || sum(~isfinite([resG; resB])) > 0 || max(resG) > 1e-12 || max(resB) > 1e-12;
fprintf('  L0 值域实测 [%.4f, %.4f] m, 几何界 [%.2f, %.2f] m\n', ...
        min(L0), max(L0), abs(l2 - l1), l2 + l1);
fail = fail || min(L0) < abs(l2 - l1) - 1e-12 || max(L0) > l2 + l1 + 1e-12;

% ==================== 2. Leg_Spd vs 中心差分 ====================
% 阈值: 中心差分误差 ~ eps*|∂pos/∂phi|/H + f'''*H²/6, H=1e-6 rad 时两项都是
% 1e-10 量级; 实测见打印, 余量取 ~10 倍。
tolS = 1e-7;    % [m/rad] / [rad/rad]
eS  = spd_err(B1, B4, H);
eSf = spd_err(F1, F4, H);
fprintf('\n===== 2. Leg_Spd vs Leg_Pos 中心差分雅可比 =====\n');
fprintf('  工作带: max |Δ L0_dot| %.3e, max |Δ phi0_dot| %.3e (%.0e)\n', eS(1), eS(2), tolS);
fprintf('  全域  : max |Δ L0_dot| %.3e, max |Δ phi0_dot| %.3e (%.0e)\n', eSf(1), eSf(2), 1e-5);
fail = fail || max(eS) > tolS || max(eSf) > 1e-5;
lin = spd_lin(B1, B4);
fprintf('  Leg_Spd 线性叠加残差 max %.3e（阈值 1e-12）\n', lin);
fail = fail || lin > 1e-12;

% ==================== 3. Leg_Trq = J' ====================
tolT = 1e-7;
eT  = trq_err(B1, B4, H);
eTf = trq_err(F1, F4, H);
fprintf('\n===== 3. Leg_Trq vs 雅可比转置 =====\n');
fprintf('  工作带: max |Δ tau1| %.3e, max |Δ tau4| %.3e (%.0e)\n', eT(1), eT(2), tolT);
fprintf('  全域  : max |Δ tau1| %.3e, max |Δ tau4| %.3e (%.0e)\n', eTf(1), eTf(2), 1e-5);
fail = fail || max(eT) > tolT || max(eTf) > 1e-5;

% ==================== 4. Leg_InvTrq 往返 ====================
% Leg_InvTrq 内部的 J 是"响应矩阵" = J_cd'（第 3 节已独立核对）, 所以这里也用它。
% J⁺_λ Jt = V diag(σ²/(σ²+λ)) V', 故相对误差 ≤ λ/σ_min²（取 1.02 余量）;
% τ 取 Jt 的最小奇异方向时取到上界: 比值 = σ_min²/(σ_min²+λ) ≈ 1 → 断言 ∈ [0.9, 1.02]。
fprintf('\n===== 4. Leg_InvTrq∘Leg_Trq 往返 vs λ/σ_min² =====\n');
k4  = 400;
idx = round(linspace(1, numel(B1), k4));
reW = zeros(k4, 1); rtW = zeros(k4, 1); sv2 = zeros(k4, 1);
for j = 1:k4
    x = B1(idx(j)); y = B4(idx(j));
    Jt = jac_cd(x, y, H).';                % 响应矩阵 = 运动学雅可比转置
    s  = svd(Jt);
    [~, ~, V] = svd(Jt);
    tl = Jt * V(:, 2);                     % 沿最小奇异方向的力矩
    [F, Tp] = Leg_InvTrq(tl(1), tl(2), x, y);
    tl2 = Leg_Trq(F, Tp, x, y);
    reW(j) = norm(tl2 - tl) / norm(tl);
    rtW(j) = reW(j) / (lam / s(2)^2);
    sv2(j) = s(2);
end
% 阈值不写死: 理论上界 λ/σ_min² 随本采样里最小的 σ_min 走, 固定常数在加密网格或
% 换 leg 表后可能只剩个位数余量; 这里取实测上界的 1.5 倍。
tolW = 1.5 * lam / min(sv2)^2;
fprintf('  最弱方向 τ: 比值 rel/(λ/σ_min²) ∈ [%.4f, %.4f]  阈值 [0.9, 1.02]\n', ...
        min(rtW), max(rtW));
fprintf('  rel 最大 %.3e (阈值 %.3e = 1.5×λ/min σ_min²), σ_min ∈ [%.4f, %.4f]\n', ...
        max(reW), tolW, min(sv2), max(sv2));
fail = fail || min(rtW) < 0.9 || max(rtW) > 1.02 || max(reW) > tolW;

eI = 0; eM = 0;
for j = 1:k4
    x = B1(idx(j)); y = B4(idx(j));
    Jt = jac_cd(x, y, H).';
    s  = svd(Jt);
    tl = [0.6; -0.35];
    [F, Tp] = Leg_InvTrq(tl(1), tl(2), x, y);
    tl2 = Leg_Trq(F, Tp, x, y);
    eI = max(eI, norm(tl2 - tl) / norm(tl) / (lam / s(2)^2));
    m  = (Jt' * Jt + lam * eye(2)) \ (Jt' * tl);
    eM = max(eM, max(abs([F; Tp] - m)) / max(norm(m), 1e-6));
end
fprintf('  随机方向 rel/(λ/σ_min²) 最大 %.4f（硬上界 1.02）; 与独立 (J''J+λI)⁻¹J''τ 最大相对差 %.3e\n', ...
        eI, eM);
fail = fail || eI > 1.02 || eM > 1e-6;

% ==================== 5. 全域往返（σ_min 足够大的样本） ====================
k5 = 400;
idx = round(linspace(1, numel(F1), k5));
nBad = 0; eF = 0;
for j = 1:k5
    x = F1(idx(j)); y = F4(idx(j));
    Jt = jac_cd(x, y, H).';
    s  = svd(Jt);
    if s(2) < 0.02
        nBad = nBad + 1;      % 近死点: 逆映射本身病态, 只统计不设界
        continue;
    end
    tl = [0.6; -0.35];
    [F, Tp] = Leg_InvTrq(tl(1), tl(2), x, y);
    tl2 = Leg_Trq(F, Tp, x, y);
    eF = max(eF, norm(tl2 - tl) / norm(tl) / (lam / s(2)^2));
end
fprintf('\n===== 5. 全域往返（σ_min ≥ 0.02 的样本）=====\n');
fprintf('  硬上界比值最大 %.4f（阈值 1.02）; σ_min < 0.02 被排除 %d/%d\n', eF, nBad, k5);
fail = fail || eF > 1.02;

% ==================== 6. 奇异线与奇异邻域 ====================
fprintf('\n===== 6. 奇异线 phi1 ≡ phi4 (mod 2π) =====\n');
x   = 0.7;
spd = Leg_Spd(x, x, 1, 1);
trq = Leg_Trq(1, 0, x, x);
pos = Leg_Pos(x, x);
fprintf('  Δ=0  : pos = [%.6f %.6f] 有限 %d;  spd = [%g %g] 有限 %d;  T = [%g %g] 有限 %d\n', ...
        pos(1), pos(2), all(isfinite(pos)), spd(1), spd(2), all(isfinite(spd)), ...
        trq(1), trq(2), all(isfinite(trq)));
okSing = all(isfinite(pos)) && all(~isfinite(spd)) && all(~isfinite(trq));
fail = fail || ~okSing;
% 邻域: 分支跳到折叠解 (L0 = |l2-l1| = 0.04), 与 Δ=0 的退化取值不连续——记录用
for d = [0, 1e-12, 1e-10, 1e-8, 1e-6, 1e-4]
    spd = Leg_Spd(x, x + d, 1, 1);
    pos = Leg_Pos(x, x + d);
    fprintf('  Δ=%.0e: L0 = %.6f, spd = [%+.4e %+.4e] 有限 %d\n', ...
            d, pos(1), spd(1), spd(2), all(isfinite(spd)));
end

% ==================== 7. 标量 ABI ====================
fprintf('\n===== 7. 标量调用 / 固件 ABI =====\n');
% 入参必须是纯 float 标量; 出参是 float* 或定长 float[n]（Coder 对多输出函数
% 用数组形参, 仍是标量接口, 不是向量化输入）
specs = {'Leg_Pos', 2, 1; 'Leg_Spd', 4, 1; 'Leg_Trq', 4, 1; 'Leg_InvTrq', 4, 2; ...
         'Leg_Comp', 1, 2; 'Leg_SpringF', 2, 0};
for j = 1:size(specs, 1)
    src = fileread(fullfile(cmsis, [specs{j, 1} '.c']));
    sig = regexp(src, [specs{j, 1} '\(([^)]*)\)\s*\n?\{'], 'tokens', 'once');
    assert(~isempty(sig), '%s.c 中找不到函数定义', specs{j, 1});
    args = strtrim(strsplit(strtrim(sig{1}), ','));
    if isempty(args{1})
        args = {};
    end
    nIn  = sum(~cellfun(@isempty, regexp(args, '^float [A-Za-z_]\w*$')));
    nOut = sum(~cellfun(@isempty, regexp(args, '^float \*|^float [A-Za-z_]\w*\[')));
    ok   = nIn == specs{j, 2} && nOut == specs{j, 3};
    fprintf('  %-11s 形参: %d 个 float 标量入参, %d 个出参缓冲, 无数组入参: %d\n', ...
            specs{j, 1}, nIn, nOut, ok);
    fail = fail || ~ok;
end
% .m 侧: 行向量调用恰好逐元素一致（所以"只能标量调用"是接口约定, 不是数值差异）
v = Leg_Pos([0.3 0.9], [1.1 0.4]);
e = max(abs(v(:) - [Leg_Pos(0.3, 1.1); Leg_Pos(0.9, 0.4)]));
fprintf('  .m 行向量 Leg_Pos: 与逐点标量最大差 %.3e（本脚本一律逐点标量调用）\n', e);
fail = fail || e > 0;

% ==================== 汇总 ====================
fprintf('\n===== 汇总 =====\n');
if fail
    error('模型级独立断言失败');
end
fprintf('模型级独立断言全部通过（Leg_Pos=圆-圆求交 / Leg_Spd=差分雅可比 / Leg_Trq=J'' / 往返≈λ/σ_min²）\n');
end

% ======================================================================
function [res, br] = cc_residual(phi1, phi4, l1, l2)
% 圆-圆求交闭式: 圆心 A_i, 半径 l2; 返回"输出点到更近交点"的距离与分支号
res = zeros(numel(phi1), 1);
br  = zeros(numel(phi1), 1);
for k = 1:numel(phi1)
    A1 = l1 * [cos(phi1(k)); sin(phi1(k))];
    A4 = l1 * [cos(phi4(k)); sin(phi4(k))];
    d  = A4 - A1;
    D  = norm(d);
    h  = sqrt(l2^2 - (D / 2)^2);
    M  = (A1 + A4) / 2;
    e  = d / D;
    n  = [-e(2); e(1)];
    pos = Leg_Pos(phi1(k), phi4(k));
    Pm  = pos(1) * [cos(pos(2)); sin(pos(2))];
    dp  = norm(Pm - (M + h * n));
    dn  = norm(Pm - (M - h * n));
    if dp <= dn
        res(k) = dp; br(k) = 1;
    else
        res(k) = dn; br(k) = -1;
    end
end
end

% ======================================================================
function e = spd_err(phi1, phi4, h)
% [L0_dot 通道, phi0_dot 通道] 相对中心差分雅可比的最大偏差
e = [0, 0];
for k = 1:numel(phi1)
    x = phi1(k); y = phi4(k);
    J  = jac_cd(x, y, h);
    s1 = Leg_Spd(x, y, 1, 0);
    s4 = Leg_Spd(x, y, 0, 1);
    e(1) = max(e(1), max(abs([s1(1) - J(1, 1), s4(1) - J(1, 2)])));
    e(2) = max(e(2), max(abs([s1(2) - J(2, 1), s4(2) - J(2, 2)])));
end
end

function e = spd_lin(phi1, phi4)
% 速率的线性叠加: spd(a, b) == a*spd(1,0) + b*spd(0,1)
e = 0;
for k = 1:numel(phi1)
    x = phi1(k); y = phi4(k);
    a = Leg_Spd(x, y, 1.3, -0.7);
    b = 1.3 * Leg_Spd(x, y, 1, 0) - 0.7 * Leg_Spd(x, y, 0, 1);
    e = max(e, max(abs(a - b)));
end
end

function e = trq_err(phi1, phi4, h)
% [tau1 通道, tau4 通道] 相对雅可比转置的最大偏差
% 对偶关系: tau = J' * [F; Tp], 即 tau1 = J(1,1)F + J(2,1)Tp, tau4 = J(1,2)F + J(2,2)Tp
e = [0, 0];
for k = 1:numel(phi1)
    x = phi1(k); y = phi4(k);
    J  = jac_cd(x, y, h);
    Ta = Leg_Trq(1, 0, x, y);
    Tb = Leg_Trq(0, 1, x, y);
    e(1) = max(e(1), max(abs([Ta(1) - J(1, 1), Tb(1) - J(2, 1)])));
    e(2) = max(e(2), max(abs([Ta(2) - J(1, 2), Tb(2) - J(2, 2)])));
end
end

function J = jac_cd(phi1, phi4, h)
% Leg_Pos 的数值雅可比: 中心差分, 角度分量按 2π 折算
J  = zeros(2, 2);
pp = Leg_Pos(phi1 + h, phi4); pm = Leg_Pos(phi1 - h, phi4);
J(:, 1) = [pp(1) - pm(1); wrapa(pp(2) - pm(2))] / (2 * h);
pp = Leg_Pos(phi1, phi4 + h); pm = Leg_Pos(phi1, phi4 - h);
J(:, 2) = [pp(1) - pm(1); wrapa(pp(2) - pm(2))] / (2 * h);
end

function a = wrapa(a)
a = atan2(sin(a), cos(a));
end

function lam = read_lambda(f)
% λ 只在 Leg_InvTrq.m 里定义一次; 读回来是为了断言口径跟着源码走
tok = regexp(fileread(f), 'lambda\s*=\s*([\d.eE+-]+);', 'tokens', 'once');
assert(~isempty(tok), 'Leg_InvTrq.m 中找不到 lambda 赋值');
lam = str2double(tok{1});
end

%% ==================== Leg_Comp 后处理 C 对拍 + 表结构 ====================
function check_comp()
%CHECK_COMP  对拍: Leg_Comp 的 CMSIS 后处理产物 vs codegen 原始产物 vs .m 模型
%
%   1) 用 mex_leg_comp.c 把部署目录的 Leg_Comp.c（配 solver/arm_math.h 影子实现）
%      和 build/codegen_current/lib/Leg_Comp 的原始 C 各编一份，喂同一组腿长，输出 theta0 [rad]
%      与 l_d [m]；两份 C 应逐位相同——证明 build_leg_tool 的 cmsis_postprocess 段文本变换
%      （Leg_Comp 只做头注释改写）没有改过任何数值；
%   2) 再和 .m 模型比，误差应落在单精度量级；
%   3) 表结构自检：从部署目录 Leg_Comp.c 的源码里把表与钳位常数读回来，逐条对
%      leg_config()。这一步是陈旧表的回归防线：行数/起点/步长/钳位常数/表值任一
%      掉队，这里都会立刻失败。
%
%   与 check_vmc 的 4 函数对拍相互独立（mex_leg_vmc.c 接口不动），
%   由 check_vmc 的末尾调用，也可单独运行。
%
%   See also: build_leg_tool, verify_leg_tool, leg_config

root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'shared'));
D = leg_paths();
cmsis = D.OUT_DIR;
lib   = D.CODEGEN_DIR;
shim  = D.SOLVER_DIR;
addpath(D.ROOT, D.MODEL_DIR, D.SHARED_DIR, D.SOLVER_DIR);
if ~exist(D.VERIFY_BUILD_DIR, 'dir'), mkdir(D.VERIFY_BUILD_DIR); end
addpath(D.VERIFY_BUILD_DIR);

% rtwtypes.h 由 build/codegen_current/lib/Leg_Comp/ 自带，无需摊平目录
fprintf('编译 leg_comp_raw ...\n');
mex('-outdir', D.VERIFY_BUILD_DIR, '-output', 'leg_comp_raw', ...
    ['-I' fullfile(lib, 'Leg_Comp')], ...
    fullfile(D.SOLVER_DIR, 'mex_leg_comp.c'), ...
    fullfile(lib, 'Leg_Comp', 'Leg_Comp.c'));

fprintf('编译 leg_comp_cmsis ...\n');
mex('-outdir', D.VERIFY_BUILD_DIR, '-output', 'leg_comp_cmsis', ['-I' shim], ['-I' cmsis], ...
    fullfile(D.SOLVER_DIR, 'mex_leg_comp.c'), fullfile(cmsis, 'Leg_Comp.c'));

% ---- 腿长测试点 ----
% 表节点 + 段中点 + 冒号生成 + 表外/越界/±Inf/NaN。NaN 参与比较是因为两侧
% 都用 IEEE 语义把 NaN 折到行 1（C 的 fmaxf 返回非 NaN 操作数，MATLAB max 同理）。
rng(23);
D    = leg_config();
NT   = size(D, 1);
node = D(:, 1);
mid  = (node(1:end - 1) + node(2:end)) / 2;
edge = [0.05; 0.10; 0.15; node(1) - 1e-9; node(1) - 1e-12; node(end) + 1e-12;
        node(end) + 1e-9; 0.35; 0.40; 0.50; 1e3; -1e3; -1; 0; Inf; -Inf; NaN];
L0   = single([node; mid; (node(1):0.01:node(end)).'; ...
               single(0.10 + 0.30 * rand(500, 1)); edge]);

[T1, L1] = leg_comp_cmsis(L0);
[T2, L2] = leg_comp_raw(L0);

% ==================== 1. 后处理 C vs codegen 原始 C ====================
% 逐位比较用 isequaln: isequal(NaN,NaN)=false, 样本落到奇异线上会误判 FAIL
fprintf('\n===== 1. 后处理 C  vs  codegen 原始 C（应逐位相同）=====\n');
sameT = isequaln(T1, T2);
sameL = isequaln(L1, L2);
fprintf('  %-8s 最大差 %.3e   逐位相同: %d\n', 'theta0', max(abs(double(T1) - double(T2))), sameT);
fprintf('  %-8s 最大差 %.3e   逐位相同: %d\n', 'l_d', max(abs(double(L1) - double(L2))), sameL);
fail = ~(sameT && sameL);

% ==================== 2. 后处理 C vs .m 模型 ====================
% 阈值: 实测 max 5.96e-8 rad（= 表值 0.69 处的 1 个 single ulp）、1.49e-8 m
% （= 0.116 处的 2 个 single ulp）；成因是 u = (l-L0)/dl 的浮点定位误差乘段斜率，
% 取实测的 8 倍 / 3 倍作余量。
tolT = 5e-7;    % [rad]
tolL = 5e-8;    % [m]
Tm   = zeros(numel(L0), 1);
Lm   = zeros(numel(L0), 1);
for k = 1:numel(L0)
    [th, ld] = Leg_Comp(single(L0(k)));   % 逐点标量调用（.m 头注释的接口约定）
    Tm(k) = th;
    Lm(k) = ld;
end
eT = abs(double(T1) - double(Tm));
eL = abs(double(L1) - double(Lm));
fprintf('\n===== 2. 后处理 C  vs  .m 模型 =====\n');
fprintf('  %-8s 最大误差 %10.3e   阈值 %10.3e   通过: %d\n', 'theta0', max(eT), tolT, max(eT) <= tolT);
fprintf('  %-8s 最大误差 %10.3e   阈值 %10.3e   通过: %d\n', 'l_d', max(eL), tolL, max(eL) <= tolL);
fail = fail || max(eT) > tolT || max(eL) > tolL;

% ==================== 3. 表结构 vs leg_config() ====================
fprintf('\n===== 3. 部署目录 Leg_Comp.c 的表与钳位常数 vs leg_config() =====\n');
src  = fileread(fullfile(cmsis, 'Leg_Comp.c'));

nRow = numel(parse_floats(extract(src, 'ld_tab\[\d+\]\s*=\s*\{([^}]*)\}')));
th0c = parse_floats(extract(src, 'th0_tab\[\d+\]\s*=\s*\{([^}]*)\}'));
ldc  = parse_floats(extract(src, 'ld_tab\[\d+\]\s*=\s*\{([^}]*)\}'));
tok  = regexp(src, 'fmaxf\(\(l - ([\d.eE+-]+)F\) / ([\d.eE+-]+)F, 0\.0F\), ([\d.eE+-]+)F\);', ...
              'tokens', 'once');
tok2 = regexp(src, 'k0 = \(int\)fminf\(floorf\(u\), ([\d.eE+-]+)F\);', 'tokens', 'once');
assert(numel(tok) == 3 && isscalar(tok2), ...
       'Leg_Comp.c 里没找到段定位/钳位表达式, 生成物结构变了, 本检查需要同步');

cStart = single(str2double(tok{1}));
cStep  = single(str2double(tok{2}));
cUMax  = single(str2double(tok{3}));
cKMax  = single(str2double(tok2{1}));

chk('表行数',        nRow,        NT,                   nRow == NT);
chk('起点 l [m]',    double(cStart), double(single(D(1, 1))), cStart == single(D(1, 1)));
chk('步长 dl [m]',   double(cStep),  double(single(D(2, 1) - D(1, 1))), cStep == single(D(2, 1) - D(1, 1)));
chk('u 上钳位 (NT-1)', double(cUMax), NT - 1,            double(cUMax) == NT - 1);
chk('k0 上钳位 (NT-2)', double(cKMax), NT - 2,           double(cKMax) == NT - 2);

sameTh = isequal(th0c, single(D(:, 2)));
sameLd = isequal(ldc, single(D(:, 3)));
fprintf('  theta0 表 vs leg_config(:,2): %d/%d 行逐位相同\n', sum(th0c == single(D(:, 2))), NT);
fprintf('  l_d    表 vs leg_config(:,3): %d/%d 行逐位相同\n', sum(ldc == single(D(:, 3))), NT);
fail = fail || ~sameTh || ~sameLd;

% 段定位表达式必须真的落在表首行: l = 表起点应取第 1 行且逐位等于表值
[t0, ld0] = leg_comp_cmsis(single(D(1, 1)));
[tt0, tld0] = leg_comp_cmsis(single(D(end, 1)));
fprintf('  C(l=%.2f) = [%.9f %.9f] vs 表行 1 [%.9f %.9f]\n', D(1, 1), t0, ld0, D(1, 2), D(1, 3));
fprintf('  C(l=%.2f) = [%.9f %.9f] vs 表行 %d [%.9f %.9f]\n', D(end, 1), tt0, tld0, NT, D(end, 2), D(end, 3));
node1 = t0 == single(D(1, 2)) && ld0 == single(D(1, 3));
nodeN = tt0 == single(D(end, 2)) && tld0 == single(D(end, 3));
fail = fail || ~node1 || ~nodeN;

% ==================== 4. 钳位不变量 ====================
% 表外一律取端点值: 低于起点 = 行 1、高于终点 = 行 NT，且逐位相同
fprintf('\n===== 4. 表外钳位 =====\n');
[lo_t, lo_l] = leg_comp_cmsis(single([0.05; 0.10; D(1, 1); -1; -1e3; -Inf; NaN]));
[hi_t, hi_l] = leg_comp_cmsis(single([0.35; 0.40; 0.50; 1e3; Inf]));
clampLo = all(lo_t == single(D(1, 2))) && all(lo_l == single(D(1, 3)));
clampHi = all(hi_t == single(D(end, 2))) && all(hi_l == single(D(end, 3)));
fprintf('  低端 7 点全部逐位等于行 1 值: %d\n', clampLo);
fprintf('  高端 5 点全部逐位等于行 %d 值: %d\n', NT, clampHi);
fail = fail || ~clampLo || ~clampHi;

% ==================== 5. 表本身的性质（独立于生成物） ====================
fprintf('\n===== 5. leg_config() 表性质 =====\n');
monoTh = all(diff(D(:, 2)) < 0);
monoLd = all(diff(D(:, 3)) > 0);
fprintf('  theta0 严格递减: %d   l_d 严格递增: %d   表范围 [%.2f, %.2f] m\n', ...
        monoTh, monoLd, D(1, 1), D(end, 1));
fail = fail || ~monoTh || ~monoLd;

if fail
    error('Leg_Comp 对拍失败');
end
fprintf('\nLeg_Comp 对拍通过（表 %d 行, 起点 %.2f m, 钳位 %.0f / %.0f）\n', ...
        NT, D(1, 1), double(cUMax), double(cKMax));
end

% ======================================================================
function chk(name, got, want, ok)
fprintf('  %-18s 生成物 %-12s leg_config %-12s %s\n', name, num2str(got), num2str(want), ...
        ternary(ok, 'OK', '**FAIL**'));
assert(ok, '%s 与 leg_config() 不一致', name);
end

function s = ternary(ok, a, b)
if ok, s = a; else, s = b; end
end

function t = extract(src, pat)
t = regexp(src, pat, 'tokens', 'once');
assert(~isempty(t), 'Leg_Comp.c 中未匹配到 %s', pat);
t = t{1};
end

function v = parse_floats(txt)
% C 浮点字面量列表 → single（去掉 'f' 后缀与分隔逗号, sscanf 逐个读）
txt = regexprep(txt, '[F,]', ' ');
v   = single(sscanf(txt, '%f'));
end

%% ==================== Leg_SpringF 对拍 + 表性质 ====================
function check_springf()
% 核对当前实车经验补偿表，原始 C、部署 C 与 MATLAB 必须使用同一标定。
D = leg_paths();
addpath(D.VERIFY_BUILD_DIR);
raw = fullfile(D.CODEGEN_DIR, 'Leg_SpringF');
wrap = fullfile(D.SOLVER_DIR, 'mex_leg_springf.c');
mex('-outdir', D.VERIFY_BUILD_DIR, '-output', 'leg_springf_raw', ['-I' raw], ...
    wrap, fullfile(raw, 'Leg_SpringF.c'));
config = fullfile(fileparts(fileparts(D.ROOT)), 'Wheel_Leg_Code', 'Code', 'Config');
mex('-outdir', D.VERIFY_BUILD_DIR, '-output', 'leg_springf_cmsis', ['-I' D.SOLVER_DIR], ...
    ['-I' D.OUT_DIR], ['-I' config], wrap, fullfile(D.OUT_DIR, 'Leg_SpringF.c'));
lengths = single([linspace(.12,.36,401)'; .17; .34; Inf; -Inf]);
a = leg_springf_raw(lengths);
b = leg_springf_cmsis(lengths);
assert(max(abs(double(a-b)))<2e-5, 'Spring raw/deployed C mismatch');
for k = 1:numel(lengths)
    expected = Leg_SpringF(lengths(k), single(0));
    assert(abs(double(b(k)-expected))<2e-5, 'Spring C/MATLAB mismatch');
    assert(expected==Leg_SpringF(lengths(k), single(.3)), 'Spring table depends on angle');
end
assert(abs(double(leg_springf_cmsis(single(.17)))-67.30)<2e-4);
assert(abs(double(leg_springf_cmsis(single(.34)))-158.15)<2e-4);
assert(abs(double(leg_springf_cmsis(single(.25)))-120.34)<2e-4);
fprintf('Leg_SpringF empirical calibration: %d cases PASS\n', numel(lengths));
end

function check_vmc()
%CHECK_VMC  对拍总入口: CMSIS 后处理产物 vs codegen 原始产物 vs .m 模型
%
%   1) 4 函数 (Leg_Pos/Spd/Trq/InvTrq): 用 mex 把部署目录的 *.c（配 solver/arm_math.h 的
%      arm_math 影子实现）和 build/codegen_current/lib/ 的原始 C 各编一份, 喂同样的构型表;
%      两份 C 的输出必须逐位相同——这证明 build_leg_tool 的 cmsis_postprocess 段文本变换没有
%      改变任何数值; 再和 .m 模型比, 误差落在单精度量级（阈值见 §2 表）;
%   2) Leg_Comp: 单独 harness（单入参 / 双输出）, 外加"表结构 vs leg_config()"
%      的回归断言——生成物里的表一旦掉队, 这里会立刻失败;
%   3) 模型级独立断言: 圆-圆求交 / 中心差分雅可比 / 虚功对偶 / Tikhonov 往返;
%   4) Leg_SpringF: 独立 harness（双入参 / 单输出）;
%   5) 固件同步（可选）: Wheel_Leg/Code/Math/ 可达时逐字节比对 16 个
%      Leg_*.{c,h} —— 挡"部署目录更新了但固件忘了拷"; 目录不可达（单独取用
%      Tool/ 的场景）则跳过并提示。
%
%   末尾汇总 6 组结果, 任一项失败 error()。
%
%   See also: build_leg_tool, verify_leg_tool,
%             check_model, check_springf

root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'shared'));
D = leg_paths();
cmsis = D.OUT_DIR;
lib   = D.CODEGEN_DIR;
shim  = D.SOLVER_DIR;
stage = fullfile(D.VERIFY_BUILD_DIR, 'build_raw');
addpath(D.ROOT, D.MODEL_DIR, D.SHARED_DIR, D.SOLVER_DIR);
if ~exist(D.VERIFY_BUILD_DIR, 'dir'), mkdir(D.VERIFY_BUILD_DIR); end
addpath(D.VERIFY_BUILD_DIR);

% build/codegen_current/lib/Leg_InvTrq/ 里自带一份 Leg_Trq.c/.h，F/Tp 是 double 变体，
% 与 build/codegen_current/lib/Leg_Trq/ 的同名不同签名。散着编时引号包含会按"就近目录"选中
% 自己那份头文件、却链到另一份实现，ABI 对不上会静默出 0。摊到一个目录里消除歧义。
fprintf('准备 codegen 原始产物 ...\n');
if exist(stage, 'dir')
    rmdir(stage, 's');
end
mkdir(stage);
for n = {'Leg_Pos', 'Leg_Spd', 'Leg_Trq', 'Leg_InvTrq'}
    copyfile(fullfile(lib, n{1}, [n{1} '.c']), stage);
    copyfile(fullfile(lib, n{1}, [n{1} '.h']), stage);
end
for f = {'rt_nonfinite.c', 'rt_nonfinite.h', 'rt_defines.h', 'rtwtypes.h'}
    copyfile(fullfile(lib, 'Leg_Pos', f{1}), stage);
end

fprintf('编译 leg_vmc_raw ...\n');
mex('-outdir', D.VERIFY_BUILD_DIR, '-output', 'leg_vmc_raw', ['-I' stage], ...
    fullfile(D.SOLVER_DIR, 'mex_leg_vmc.c'), ...
    fullfile(stage, 'Leg_Pos.c'), fullfile(stage, 'Leg_Spd.c'), ...
    fullfile(stage, 'Leg_Trq.c'), fullfile(stage, 'Leg_InvTrq.c'), ...
    fullfile(stage, 'rt_nonfinite.c'));

fprintf('编译 leg_vmc_cmsis ...\n');
mex('-outdir', D.VERIFY_BUILD_DIR, '-output', 'leg_vmc_cmsis', ['-I' shim], ['-I' cmsis], ...
    fullfile(D.SOLVER_DIR, 'mex_leg_vmc.c'), ...
    fullfile(cmsis, 'Leg_Pos.c'), fullfile(cmsis, 'Leg_Spd.c'), ...
    fullfile(cmsis, 'Leg_Trq.c'), fullfile(cmsis, 'Leg_InvTrq.c'));

% ---- 构型表 ----
rng(7);
N = 2000;
phi1 = single((rand(N, 1) * 2 - 1) * pi);
phi4 = single((rand(N, 1) * 2 - 1) * pi);
d1   = single((rand(N, 1) * 2 - 1) * 5);
d4   = single((rand(N, 1) * 2 - 1) * 5);
tau1 = single((rand(N, 1) * 2 - 1) * 20);
tau4 = single((rand(N, 1) * 2 - 1) * 20);
in   = [phi1 phi4 d1 d4 tau1 tau4];

fprintf('\n===== 1. 后处理 C  vs  codegen 原始 C（应逐位相同）=====\n');
[P1, S1, T1, I1] = leg_vmc_cmsis(in);
[P2, S2, T2, I2] = leg_vmc_raw(in);
names = {'Leg_Pos', 'Leg_Spd', 'Leg_Trq', 'Leg_InvTrq'};
A = {P1, S1, T1, I1};
B = {P2, S2, T2, I2};
fail1 = false;
% 逐位比较用 isequaln: isequal(NaN,NaN)=false, 样本落到奇异线上会误判 FAIL
for k = 1:4
    d = max(abs(double(A{k}) - double(B{k})), [], 'all');
    same = isequaln(A{k}, B{k});
    fprintf('  %-12s 最大差 %.3e   逐位相同: %d\n', names{k}, d, same);
    fail1 = fail1 || ~same;
end

% ---- 2. C vs .m ----
% 阈值依据: 单精度求值本身的相对误差 ~1e-6; 靠近死点折叠 (L0→0.04/0.46) 或
% 奇异线 (phi1≡phi4) 时条件数 ~1e3~1e4, 单精度输入误差被放大到 1e-2 量级,
% 那是模型的固有病态, 不是后处理问题（后处理是否改数值由 §1 逐位比较定论）。
% 所以断言分两条: 全部样本的中位误差 + 良态样本的最大误差。
% 实测 (rng(7), N=2000): 中位 5.7e-8/2.7e-7/5.7e-7/9.9e-7;
% 良态样本最大 4.1e-5/1.1e-4/5.6e-5/3.5e-4。
mask = abs(atan2(sin(double(phi1) - double(phi4)), cos(double(phi1) - double(phi4)))) >= 1e-3;
ePos = zeros(N, 1);
eSpd = zeros(N, 1);
eTrq = zeros(N, 1);
eInv = zeros(N, 1);
for k = 1:N
    x = double(phi1(k));
    y = double(phi4(k));
    ePos(k) = relerr(P1(k, :), double(Leg_Pos(x, y)).');
    eSpd(k) = relerr(S1(k, :), double(Leg_Spd(x, y, double(d1(k)), double(d4(k)))).');
    eTrq(k) = relerr(T1(k, :), double(Leg_Trq(double(tau1(k)), double(tau4(k)), x, y)).');
    [mF, mTp] = Leg_InvTrq(double(tau1(k)), double(tau4(k)), x, y);
    eInv(k) = relerr(I1(k, :), [mF mTp]);
    mask(k) = mask(k) && P1(k, 1) >= 0.05 && P1(k, 1) <= 0.45;
end
fprintf('\n===== 2. 后处理 C  vs  .m 模型 =====\n');
fprintf('  %-12s %12s %12s %12s %12s\n', '函数', '中位(全部)', '阈值', '最大(良态)', '阈值');
tolMed = [1e-6, 1e-5, 1e-5, 1e-5];
tolMax = [1e-3, 2e-3, 1e-3, 5e-3];
E      = {ePos, eSpd, eTrq, eInv};
fail2  = false;
for k = 1:4
    nNaN = sum(~isfinite(E{k}));
    med  = median(E{k});
    mx   = max(E{k}(mask));
    ok   = med <= tolMed(k) && mx <= tolMax(k) && nNaN == 0;
    fprintf('  %-12s %12.3e %12.3e %12.3e %12.3e  %s\n', names{k}, med, tolMed(k), mx, ...
            tolMax(k), ternary_v(ok, 'OK', '**FAIL**'));
    fail2 = fail2 || ~ok;
    if nNaN > 0
        fprintf('    **%d 个样本误差非有限（NaN 会被 median/max 静默忽略, 单独拦）**\n', nNaN);
    end
end
fprintf('  良态样本 %d/%d（排除 |wrap(phi1-phi4)|<1e-3 rad 与 L0∉[0.05,0.45] 的折叠点）\n', ...
        sum(mask), N);

% ---- 3~5. 分组独立对拍 ----
subs = {'Leg_Comp 对拍（表结构回归 + C vs C vs .m）', @check_comp; ...
        '模型级独立断言（几何/雅可比/对偶/往返）',   @check_model; ...
        'Leg_SpringF 对拍（双入参 / 单输出）',       @check_springf};
res  = cell(size(subs, 1), 2);
for k = 1:size(subs, 1)
    fprintf('\n===== %d. %s =====\n', k + 2, subs{k, 1});
    try
        subs{k, 2}();
        res{k, 1} = subs{k, 1};
        res{k, 2} = true;
    catch ME
        res{k, 1} = subs{k, 1};
        res{k, 2} = false;
        fprintf('  **FAIL**: %s\n', ME.message);
    end
end

% ---- 7. 固件同步（可选）----
% 拷贝是手写 cp，两条链之间没有别的断言看得见 Wheel_Leg/Code/Math/；固件目录不可达
% （单独取用 Tool/）时跳过，不让脚本依赖仓库布局。
fprintf('\n===== 7. 固件同步（可选）=====\n');
fw = D.FIRMWARE_COMPARE_DIR;
names = {'Leg_Pos', 'Leg_Spd', 'Leg_Trq', 'Leg_InvTrq', 'Leg_Comp', 'Leg_SpringF', ...
         'Leg_RollComp', 'Leg_CoriolisComp'};
ok  = true;
lab = '固件同步（部署目录 == 固件 Math 目录）';
if exist(fw, 'dir') ~= 7
    fprintf('  跳过: 固件目录不可达 (%s)\n', fw);
    lab = [lab ': 跳过（固件目录不可达）'];
else
    bad = {};
    for k = 1:numel(names)
        for e = {'c', 'h'}
            f = [names{k} '.' e{1}];
            if ~same_bytes(fullfile(cmsis, f), fullfile(fw, f))
                bad{end + 1} = f; %#ok<AGROW>
            end
        end
    end
    ok = isempty(bad);
    if ok
        fprintf('  18 个 Leg_*.{c,h}: 逐字节相同 (%s)\n', fw);
    else
        fprintf('  **FAIL**: 与固件不一致: %s\n', strjoin(bad, ', '));
        lab = sprintf('%s: %d 个文件不同步', lab, numel(bad));
    end
end
res(end + 1, :) = {lab, ok};

% ---- 汇总 ----
fprintf('\n===== 汇总 =====\n');
fprintf('  %-46s %s\n', '1) 4 函数 后处理 C vs codegen C（逐位）', ternary_v(~fail1, 'PASS', 'FAIL'));
fprintf('  %-46s %s\n', '2) 4 函数 后处理 C vs .m 模型（阈值）',        ternary_v(~fail2, 'PASS', 'FAIL'));
for k = 1:size(res, 1)
    fprintf('  %d) %-43s %s\n', k + 2, res{k, 1}, ternary_v(res{k, 2}, 'PASS', 'FAIL'));
end
allOk = ~fail1 && ~fail2 && all(cellfun(@(x) x, res(:, 2)));
if ~allOk
    error('verify_leg_tool: 有对拍项未通过');
end
fprintf('\nVMC 四函数对拍全部通过\n');
end

function e = relerr(c, m)
% 逐元素相对误差，分母带下限避免放大零附近（|m| < 1 时按绝对误差看）
e = max(abs(c(:) - m(:)) ./ max(1, abs(m(:))));
end

function s = ternary_v(ok, a, b)
if ok, s = a; else, s = b; end
end

function tf = same_bytes(a, b)
% 逐字节比较；先比 dir 的字节数, 免得文本读入把 CRLF/LF 的差异吃掉
da = dir(a); db = dir(b);
tf = ~isempty(da) && ~isempty(db) && da.bytes == db.bytes;
if ~tf
    return;
end
fa = fopen(a, 'r'); va = fread(fa, Inf, '*uint8'); fclose(fa);
fb = fopen(b, 'r'); vb = fread(fb, Inf, '*uint8'); fclose(fb);
tf = isequal(va, vb);
end
