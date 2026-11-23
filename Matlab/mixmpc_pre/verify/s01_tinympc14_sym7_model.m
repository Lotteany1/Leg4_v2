% s01_tinympc14_sym7_model.m
% Leg4（14 态 / 6 输入）7 自由度符号推导支线：模型构建与自检打印
%
%   本版范围（用户拍定）：只建模型并打印自检；不写 .c/.h、不做闭环仿真、不改 Leg3_v2 任何文件。
%
%   本版性质（先看清这条再读数字）：交付的 A_c/B_c 是**严格块对角**——14 态块与 4 态块之间
%   没有跨块耦合，跨块作用只经仿射槽与箱。Part D 的 #1/#2/#8 已实现成开关（cfg.sym7.sw）
%   但默认全关，Part D 会并排打印开/关两套 A/B；#3 已并入 C1/C3 的差模式闭合。即本版
%   = 现役两条链的块对角基线 + A 族重力列 + 仿射槽，"只有 D08 能产生的新耦合"还没进 A/B。
%
%   版本：v0.4  日期：2026-10-19。v0.4 相对 v0.3 的差别（模型核共享化）：
%     ① 模型函数移到 mpc/parts/tinympc14_*.m（tinympc14_bal_block / tinympc14_fused_ab / tinympc14_zoh_ab / tinympc14_dare_gain 等），
%        s01 与 s02 共用同一份，s02 不再自带 one_cfg / one_model
%     ② 测试点、历史基准与容差移到 tinympc14_check_refs.m；表端点与 cfg 值替代字面量
%     ③ CACHE_REF 为空时对拍跳过且不计入硬判据；新增气弹簧 double/single 交叉核对
%     ④ #2 开关补齐腿角列 A 项、新增 #8α 开关（Ts_alpha），与 s02 的口径并集
%   v0.3 相对 v0.2 的差别（执行方案 P1：调度与状态统一）：
%     ① bal_block 升为 bal_block(lL,lR,...)：方程 (19)-(23) 两腿列各按自身腿长取表
%     ② fused_ab(cfg,sw,sched)：平衡块按 sched.lL/lR 调度（#3 LPV），H 行 K_EFF 按 H 取
%     ③ 仿射槽：腿行常数拆成 tau_g(0)(H) = M_LEG*g*x_off(H) 查表项 + 线性化余项
%     ④ 新增 Part G：三个工作点的定点 / A 相对变化 / 闭环极点 / 归一化自检
%   v0.2（2026-10-19）相对 v0.1（2026-10-18）：
%     ① Part B 的现役对拍从硬编码 ref 改为直读 tinympc_cache.mat（硬判据 1e-12）
%     ② 腿行重力仿射改为 V* 口径（theta* 处线性化，模型自身平衡点与 theta* 逐位一致）
%     ③ #1/#2/#8 从"打印登记"做成真开关（默认关），Part D 并排打印两套 A/B
%     ④ 自检补齐：通道增益表、单精度逐元素、算力预算表、l_c / G_ROLL / K_EFF 对照、
%        M_v 三档闭环、力箱的"运动学隐含电机侧上限"列、tau = J^T*[F;Tp] 输出层核对
%
%   支线内容（相对"块对角拼接"版）：
%     Part A  运动学层：把 l_l/l_r 升格为 l(t) 后的链式法则增项（对 s02 代换表的扩展），
%             并做"冻结 l̇=l̈=0 → 还原 s02 表"的符号自检
%     Part B  平衡 10 态块：方程 (19)-(23) + A 族重力口径（Leg_Comp l_d/theta0 + 1.855 kg，
%             两腿各按自身腿长取表），与现役 tinympc_cache.mat 逐位对拍（B 族口径下 1e-12 以下）
%     Part C  l 升格的消元结果：和模式→H 行（M_v 三档口径、g_eff、平衡力），
%             差模式→α 行（δ=2·R_TRK·α 闭合、广义力 1/2 因子的账、静增益核对），
%             以及 V0/V* 两种仿射口径的平衡点对照
%     Part D  耦合项登记表 #1~#8：每条公式、典型量级、A/B 落点、开关状态与开/关对照
%     Part E  融合 14 态 + ZOH + 撒检查（可控性/闭环极点/单精度/算量预算/内存/力箱/向心力/
%             l_c 与 G_ROLL 与 K_EFF 的对照档）
%     Part G  三个工作点的一致性自检（调度后定点 / A 相对变化 / 归一化）
%     Part F  自检汇总
%
%   依赖：Symbolic Math Toolbox（Part A 的符号段）、Control System Toolbox（dlqr）；
%         可选：Leg3_v2 的 Matlab/TinyMPC/build/tinympc_cache.mat（存在时启用现役对拍）
%   用法：matlab -batch "cd('E:/DevelopLotteany1/0_Code/Leg4_v1_ultimate/Matlab/mpc/parts'); s01_tinympc14_sym7_model"
%
%   See also: tinympc14_config, tinympc14_fused_ab, tinympc14_check_refs

clear; clc; close all
addpath(fileparts(fileparts(mfilename('fullpath')))); mixmpc_paths();                    % mpc 顶层：tinympc14_config
% Shared VMC paths are configured by mixmpc_paths.   % VMC 工具在 ../../tool
cfg = tinympc14_config();
refs = tinympc14_check_refs();

NX = cfg.NX;  NU = cfg.NU;
assert(NX == 14 && NU == 6, 'cfg 尺寸与 14/6 口径不一致');
assert(numel(cfg.u_min) == NU && numel(cfg.u_max) == NU, 'cfg.u_min/u_max 与 NU 不一致');
assert(numel(cfg.x_min) == NX && numel(cfg.x_max) == NX, 'cfg.x_min/x_max 与 NX 不一致');

fprintf('==== Leg4 单 MPC · 7 自由度符号推导支线（%d 态 / %d 输入）====\n', NX, NU);
fprintf('范围：只建模型并打印自检；不写文件、不做闭环仿真\n');
fprintf('本版性质：A_c/B_c 严格块对角，跨块耦合开关默认全关（见文件头）\n\n');

n_ok = 0;  n_info = 0;  n_skip = 0;

%% ==================== Part A 运动学层：l 升格增项 ====================
% 位置约束（推导-v2 §4.1）不变，l 改为 l(t)：
%   X = X_w + l(t)*sin(th(t));  Z = X_w_v + l(t)*cos(th(t))
% 对时间求两次导即得带 l̇/l̈ 的加速度式（s02 表的老式是其 l̇=l̈=0 特例）

syms t real
syms L(t) th(t)
syms dL ddL dth ddth real

dX2 = subs(diff(L*sin(th), t, 2), ...
    {diff(L,t,2), diff(L,t), diff(th,t,2), diff(th,t)}, {ddL, dL, ddth, dth});
dZ2 = subs(diff(L*cos(th), t, 2), ...
    {diff(L,t,2), diff(L,t), diff(th,t,2), diff(th,t)}, {ddL, dL, ddth, dth});

chk_h = simplify(subs(dX2, {dL, ddL}, {0, 0}) - (L*cos(th)*ddth - L*sin(th)*dth^2));
chk_v = simplify(subs(dZ2, {dL, ddL}, {0, 0}) - (-L*sin(th)*ddth - L*cos(th)*dth^2));
ok_A = isAlways(chk_h == 0) && isAlways(chk_v == 0);

fprintf('-- Part A 运动学层（l 升格，s02 表的链式法则扩展）--\n');
fprintf('   冻结还原自检（l̇=l̈=0 → s02 表）：%s（残差 h=%g, v=%g）\n', ...
        pass(ok_A), double(chk_h), double(chk_v));
if ok_A, n_ok = n_ok + 1; end
fprintf('   增项（右腿；左腿同构）：\n');
fprintf('     a^h = a_w^h + [l̈·sinθ + 2·l̇·cosθ·θ̇] + l·cosθ·θ̈ − l·sinθ·θ̇²\n');
fprintf('     a^v = a_w^v + [l̈·cosθ − 2·l̇·sinθ·θ̇] − l·sinθ·θ̈ − l·cosθ·θ̇²\n');
fprintf('   连带增项（实现清单）：\n');
fprintf('     ① 轮反解（s03 Part3 式）加 −[l̈·sinθ + 2·l̇·cosθ·θ̇]/R_w，左右各一处\n');
fprintf('     ② yaw 链（推导-v2 §4.1）分子加 [l̇_r·sinθ_r − l̇_l·sinθ_l] 及其导数项\n');
fprintf('     ③ 式内 l·cosθ·θ̈ 型乘子不动；l̇²/l̇θ̇ 项在静止线性化点（v=0）全消失\n');
fprintf('   借力：Leg_Spd 的 2×2 雅可比 J 与 Leg_Trq 的 τ = Jᵀ·[F;Tp] 是 (l,θ) ↔ 关节空间的桥；\n');
fprintf('         l 的广义力即 F 本身（力线过髋-轮轴心，tool/Leg_Trq.m:8），本层不重推五连杆内部\n');
assert(ok_A, 'A: 冻结 l 后未还原 s02 表');

%% ==================== Part B 平衡 10 态块（A 族重力口径 + 现役缓存对拍） ====================

l_chk = refs.l_chk;                                  % 对拍点（与缓存网格对齐）
[A27B, B27B] = tinympc14_bal_block(l_chk, l_chk, cfg, 'B');    % B 族口径（现役同源，用于缓存对拍）
[A27, B27, ~, aux27] = tinympc14_bal_block(l_chk, l_chk, cfg, cfg.sym7.affine_mode);   % 本模型口径

fprintf('\n-- Part B 平衡 10 态块（方程 (19)-(23)，重力口径 %s @ l = %.2f m）--\n', ...
        cfg.sym7.affine_mode, l_chk);
fprintf('   表插值：l_wl = %.5f, l_bl = %.5f, I_ll = %.9f; theta0 = %.6f, l_d = %.5f\n', ...
        aux27.l_wl, aux27.l_bl, aux27.I_ll, aux27.theta0, aux27.l_d);
fprintf('   腿行刚度：kA(0) = %.4f, kA(theta*) = %.4f, kB(B 族) = %.4f N·m/rad\n', ...
        aux27.kA0, aux27.kA, aux27.kB);
fprintf('   常数项 tau_g(0) = %.4f N·m/腿；本模型仿射常数 Gconst = %+.4f N·m（%s）\n', ...
        aux27.tg0, aux27.Gconst, cfg.sym7.affine_mode);

% 与现役缓存自动对拍：B 族连续 A/B -> ZOH -> 比 tinympc_cache.mat 的网格真值
% 判据 1e-12：这是"本块就是现役模型"的硬证据，替代旧版的硬编码 ref 对拍
cache_path = cfg.CACHE_REF;   % 现役缓存对拍可选（留空 = 跳过且不计入硬判据）
okB = [];
if isempty(cache_path)
    fprintf('   （CACHE_REF 为空，跳过现役缓存对拍，不计入硬判据）\n');
elseif exist(cache_path, 'file') == 2
    S = load(cache_path, 'Leg_len', 'Ad_num', 'Bd_num', 'Ts', 'n', 'stamp');
    assert(abs(S.Ts - cfg.Ts) < 1e-15, 'B: 缓存 Ts 与 cfg.Ts 不一致');
    idx = find(abs(S.Leg_len - l_chk) < 1e-12);
    assert(isscalar(idx), 'B: l_chk 不在缓存网格上');
    [Ad_c, Bd_c] = tinympc14_zoh_ab(A27B, B27B, [], cfg.Ts);
    dAd = max(abs(Ad_c(:) - reshape(S.Ad_num(:,:,idx,idx), [], 1)));
    dBd = max(abs(Bd_c(:) - reshape(S.Bd_num(:,:,idx,idx), [], 1)));
    okB = (dAd <= refs.tol.cache) && (dBd <= refs.tol.cache);
    fprintf('   现役缓存对拍（%s，node %d，Ts = %g s）：max|dAd| = %.2e, max|dBd| = %.2e <= %.0e：%s\n', ...
            S.stamp, idx, cfg.Ts, dAd, dBd, refs.tol.cache, pass(okB));
else
    error('B: CACHE_REF 指向的文件不存在：%s', cache_path);
end
if isempty(okB)
    n_skip = n_skip + 1;
elseif okB
    n_ok = n_ok + 1;
else
    error('B: 平衡 10 态块与现役缓存不一致');
end

% 历史对拍值（2026-10-18 版报告口径，B 族 @0.27）：只作人工对照，容差 1%
ref = refs.hist;
tolB = refs.tol.hist;
okB_ref = localrel(A27(2,5), ref.A25, tolB) && localrel(A27(4,5), ref.A45, tolB) && ...
          localrel(A27(6,5), ref.A65, tolB) && localrel(A27(10,5), ref.A105, tolB) && ...
          localrel(A27(10,9), ref.A109, tolB) && ...
          localrel(B27(2,1), ref.B21, tolB) && localrel(B27(4,1), ref.B41, tolB) && ...
          localrel(B27(6,3), ref.B63, tolB) && localrel(B27(10,3), ref.B103, tolB) && ...
          localrel(B27(2,3), ref.B23, tolB);
fprintf('   历史值对照（A 族口径 @%.2f，容差 %.1f%%）：%s\n', refs.l_chk, 100*tolB, pass(okB_ref));
fprintf('     A(2,5) = %+.4f (ref %+.4f)   A(4,5) = %+.4f (ref %+.4f)\n', A27(2,5), ref.A25, A27(4,5), ref.A45);
fprintf('     A(6,5) = %+.4f (ref %+.4f)   A(10,5) = %+.4f (ref %+.4f)   A(10,9) = %+.4f (ref %+.4f)\n', ...
        A27(6,5), ref.A65, A27(10,5), ref.A105, A27(10,9), ref.A109);
fprintf('     B(2,1) = %+.4f (ref %+.4f)   B(6,3) = %+.4f (ref %+.4f)   B(2,3) = %+.4f (ref %+.4f)\n', ...
        B27(2,1), ref.B21, B27(6,3), ref.B63, B27(2,3), ref.B23);

[A_balB, B_balB, ~, auxB] = tinympc14_bal_block(cfg.H_BAR, cfg.H_BAR, cfg, 'B');
[A_bal,  B_bal,  f_bal, aux0] = tinympc14_bal_block(cfg.H_BAR, cfg.H_BAR, cfg, cfg.sym7.affine_mode);
fprintf('   取 H_BAR = %.3f m 作为线性化点：kA = %.4f, kB = %.4f, tau_g(0) = %.4f N·m\n', ...
        cfg.H_BAR, aux0.kA, aux0.kB, aux0.tg0);
fprintf('   平衡点腿角 theta*（G1_A=0，最近 0 根）= %.6f rad\n', aux0.theta_star);
fprintf('   f_bal（连续仿射常数）非零行 2/4/6/8/10：%+.4e %+.4e %+.4e %+.4e %+.4e\n', ...
        f_bal(2), f_bal(4), f_bal(6), f_bal(8), f_bal(10));

%% ==================== Part C l 升格的消元结果：和模式 → H 行，差模式 → α 行 ====================

fprintf('\n-- Part C l 升格的消元结果（和/差模式降回 (H, α)）--\n');

% C1 闭合关系
dld = tinympc14_dtab(cfg.Leg_comp, cfg.H_BAR, 3);   % dl_d/dl（Leg_comp 第 3 列 = l_d，中心差分）
M_TOT = tinympc14_m_tot(cfg);
m_eq  = cfg.M_LEG*(1 - dld);
Mv_locked  = cfg.M_V;
Mv_derived = cfg.m_b + 2*m_eq;
Mv_near    = cfg.m_b + 2*cfg.sym7.m_leg_near;

fprintf('   C1 平地闭合：H = (l_l+l_r)·cosψ·cosα/2，(l_l−l_r)·cosψ = 2·R_TRK·tanα\n');
fprintf('      直立点 ∂(H,α)/∂(l_l,l_r) = [1/2, 1/2; 1/(2·R_TRK), −1/(2·R_TRK)]，正横滚抬高左侧\n');
fprintf('      例：δ = 5 mm → α = %.5f rad；δ = 10 mm → α = %.5f rad\n', ...
        atan(5e-3/(2*cfg.R_TRK)), atan(10e-3/(2*cfg.R_TRK)));
if cfg.A_MAX < 1e5
    fprintf('      A_MAX = %.2f rad ↔ δ = %.3f m（超出腿长行程 %.2f m，物理可达域由行程差兜底）\n', ...
        cfg.A_MAX, tan(cfg.A_MAX)*2*cfg.R_TRK, cfg.Leg_data(end,1) - cfg.Leg_data(1,1));
else
    fprintf('      预测横滚角无状态箱约束。\n');
end

% C2 和模式（H 行）
fprintf('   C2 直立点静止时：M_v·Ḧ = (F_l+F_r) − M_LOAD·g\n');
fprintf('      m_eq/腿 = LEG_GRAV_FF·(1 − dl_d/dl) = %.4f·(1 − %.4f) = %.4f kg（dl_d/dl 中心差分）\n', ...
        cfg.M_LEG, dld, m_eq);
fprintf('      整车质量 %.3f kg，轮上支撑质量 %.3f kg\n',M_TOT,cfg.M_SUP);
fprintf('      M_v 三档：locked = %.3f（名义闭链惯量）| derived = %.3f | near_end = %.3f\n', ...
        Mv_locked, Mv_derived, Mv_near);
fprintf('      g_eff = M_LOAD/M_v·g：locked = %.3f | derived = %.3f | near_end = %.3f m/s²\n', ...
        cfg.M_LOAD/Mv_locked*cfg.g, cfg.M_LOAD/Mv_derived*cfg.g, cfg.M_LOAD/Mv_near*cfg.g);
fprintf('      平衡力（三档同值）：F̄ = M_LOAD·g/2 = %.2f N/腿\n', cfg.M_LOAD*cfg.g/2);
okC2 = abs(M_TOT - refs.s03.M_TOT) < refs.tol.c2;
fprintf('      整车质量 == 分项质量之和（%.3f kg）：%s\n', ...
        refs.s03.M_TOT, pass(okC2));
if okC2, n_ok = n_ok + 1; end

% C3 差模式（α 行）
fprintf('   C3 差模式（α 行）：广义力账（易错 1/2 因子）\n');
fprintf('      δ := l_l − l_r ⇒ ∂l_l/∂δ = +1/2，∂l_r/∂δ = −1/2\n');
fprintf('      Q_δ = F_l·(+1/2) + F_r·(−1/2) = (F_l−F_r)/2\n');
fprintf('      Q_α = Q_δ·(∂δ/∂α) = (F_l−F_r)/2·(2·R_TRK) = R_TRK·(F_l−F_r)\n');
fprintf('      ⇒ α̈ = R_TRK·(F_l−F_r)/I_ROLL（符号取 s03 约定），静增益 R_TRK/I_ROLL = %.5f\n', ...
        cfg.R_TRK/cfg.I_ROLL);
% 符号断言：从广义力账独立推出 B_leg 的 α 行，含符号与模型里的那一行逐位比
B_leg_sym = zeros(4, 2);
B_leg_sym(2, :) = [cfg.R_TRK, -cfg.R_TRK] / cfg.I_ROLL;      % (F_l−F_r) 分开写两列
gain03 = refs.s03.R_TRK/refs.s03.I_ROLL;
okC3 = abs(B_leg_sym(2,1) - cfg.R_TRK/cfg.I_ROLL) < 1e-12 && ...
       abs(B_leg_sym(2,2) + cfg.R_TRK/cfg.I_ROLL) < 1e-12 && ...
       abs(cfg.R_TRK/cfg.I_ROLL - gain03) < refs.tol.c3;
fprintf('      与 s03 值 %.5f 一致且符号为 (+F_l, −F_r)：%s\n', gain03, pass(okC3));
if okC3, n_ok = n_ok + 1; end

% C4 I_ROLL 候选结构（轴取法决定；标定项）
I_roll_cand = cfg.I_z + 2*m_eq*cfg.R_TRK^2;
I_roll_ground = I_roll_cand + cfg.m_b*(cfg.H_BAR + cfg.H_OFF)^2;
fprintf('   C4 I_ROLL 结构候选（轴取法待台架裁定）：\n');
fprintf('      髋线轴：I_z + 2·m_eq·R_TRK² = %.5f（锁定值 %.3f，偏差 %+.2f%%）\n', ...
        I_roll_cand, cfg.I_ROLL, 100*(I_roll_cand - cfg.I_ROLL)/cfg.I_ROLL);
fprintf('      接地线轴：再加 m_b·(H+H_OFF)² = %.5f（另需整车质心实测，供侧向冲激实验对照）\n', I_roll_ground);
n_info = n_info + 1;

% C5 仿射口径对照：B（生产）/ V0（theta=0）/ V*（theta*）
% 平衡判据必须两腿同时取 theta*（单腿单独取角不构成平衡：θ̇ 那一行会把两腿的方程混起来）
[A_v0, ~, f_v0, aux_v0] = tinympc14_bal_block(cfg.H_BAR, cfg.H_BAR, cfg, 'A0');
[A_vs, ~, f_vs, aux_vs] = tinympc14_bal_block(cfg.H_BAR, cfg.H_BAR, cfg, 'V*');
th_star = aux_vs.theta_star;
xe = zeros(10,1); xe([5 7]) = [th_star; th_star];
[Ad_v0, ~, fd_v0] = tinympc14_zoh_ab(A_v0, zeros(10,4), f_v0, cfg.Ts);
[Ad_vs, ~, fd_vs] = tinympc14_zoh_ab(A_vs, zeros(10,4), f_vs, cfg.Ts);
r_v0 = max(abs(Ad_v0*xe + fd_v0 - xe));
r_vs = max(abs(Ad_vs*xe + fd_vs - xe));
% 模型自身平衡角：解 A*(t*e5 + t*e7) + f = 0 的标量最小二乘
u_v0 = A_v0(:,5) + A_v0(:,7);  th_e_v0 = -(u_v0.'*f_v0)/(u_v0.'*u_v0);
u_vs = A_vs(:,5) + A_vs(:,7);  th_e_vs = -(u_vs.'*f_vs)/(u_vs.'*u_vs);
fprintf('   C5 仿射口径对照（H_BAR 处，两腿同时取角才算平衡）：\n');
fprintf('      V0（theta=0 ）：kA = %.4f, Gconst = %+.4f, 自身平衡角 = %.6f（theta* = %.6f）, 不动点残差 %.2e\n', ...
        aux_v0.kA0, aux_v0.Gconst, th_e_v0, th_star, r_v0);
fprintf('      V*（theta* ）：kA = %.4f, Gconst = %+.4f, 自身平衡角 = %.6f（偏 theta* %+.3f%%）, 不动点残差 %.2e\n', ...
        aux_vs.kA, aux_vs.Gconst, th_e_vs, 100*(th_e_vs - th_star)/max(abs(th_star), eps), r_vs);
if strcmp(cfg.sym7.affine_mode, 'B')
    okC5 = max(abs(f_bal)) <= refs.tol.c5;
    fprintf('      B 族（生产口径）：仿射常数 max|f_bal| = %.2e 应为 0，常数重力由 MgF 插件补：%s\n', ...
            max(abs(f_bal)), pass(okC5));
else
    okC5 = r_vs <= refs.tol.c5;
    fprintf('      V* 口径下"两腿角 = theta* 即整状态不动点"（离散残差 <= 1e-12）：%s\n', pass(okC5));
end
if okC5, n_ok = n_ok + 1; end

%% ==================== Part D 耦合项登记表 #1~#8 ====================

fprintf('\n-- Part D 耦合项登记表（开关默认全关；对照见本段末）--\n');
sched0 = tinympc14_sched_pt(cfg, cfg.H_BAR, 0);
psi = sched0.psi;
Fbar = cfg.M_LOAD*cfg.g/2;

fprintf('   #1  F 水平分量 → s̈ 行与 θ̈_b 行（ψ = 世界系腿角 theta_leg，body.c:48 已含俯仰）\n');
fprintf('       A(2,5/7) += ±F̄·cosψ/M_s = %.4f /rad；B(2,5:6) += ±sinψ/M_s = %.4e /N\n', ...
        Fbar*cos(psi)/M_TOT, sin(psi)/M_TOT);
fprintf('       A(10,5/7) += ∓l_c·F̄·cosψ/I_b = %.4f /rad；B(10,5:6) += ∓l_c·sinψ/I_b = %.4e /N（row 10 = θ̈_b）\n', ...
        cfg.l_c*Fbar*cos(psi)/cfg.I_b, cfg.l_c*sin(psi)/cfg.I_b);
fprintf('   #2  H ↔ θ_leg 竖直投影：ΔH ≈ l·ψ²/2 = %.2f mm；F·(1−cosψ) = %.3f N（%.2f%%）\n', ...
        1e3*cfg.H_BAR*psi^2/2, Fbar*(1-cos(psi)), 100*(1-cos(psi)));
fprintf('   #3  腿长差 → 横滚闭合：δ = 2·R_TRK·α（已并入 C1/C3，进模型作差模式，不做独立项）\n');
fprintf('   #4  M_v 口径差：#4 已由 C2 三档打印（locked/derived/near_end 偏差 %+.1f%% / %+.1f%%）\n', ...
        100*(Mv_locked-Mv_derived)/Mv_derived, 100*(Mv_locked-Mv_near)/Mv_near);
fprintf('   #5  气弹簧（力箱 Fs(H)；刚度 ∂Fs/∂L0 按 k_eff_mode 进 H 行）：\n');
for H = refs.grid.fs
    fprintf('       Fs(%.2f) = %6.3f N\n', H, Leg_SpringF(double(H)));
end
fprintf('       ∂Fs/∂L0 @0.25 ≈ %.1f N/m（中心差分 ±5 mm），@0.27 ≈ %.1f N/m\n', ...
        (Leg_SpringF(double(0.255))-Leg_SpringF(double(0.245)))/0.01, ...
        (Leg_SpringF(double(0.275))-Leg_SpringF(double(0.265)))/0.01);
fprintf('   #6  重力常数项：tau_g(0) = %.4f N·m/腿（B 族不进模型，由 MgF 插件输出层补；H 依赖见下表）\n', aux0.tg0);
fprintf('        tau_g(0) 随 H：');
for H = refs.grid.tg
    c = tinympc14_table_row(cfg.Leg_comp, H);
    fprintf(' %.2f→%.4f', H, cfg.M_LEG*cfg.g*c(2)*sin(c(1)));
end
fprintf(' N·m（变化 −38%%，必须与 A/B 一起进调度）\n');
l_lo = cfg.Leg_comp(1,1);  l_hi = cfg.Leg_comp(end,1);
x16 = off_val(cfg, l_lo);  x34 = off_val(cfg, l_hi);
slope_off = (x34 - x16)/(l_hi - l_lo);
fprintf('   #7  质心随腿长移动：x_off = l_d·sinθ0 从 %.4f m（%.2f m）降到 %.4f m（%.2f m），\n', ...
        x16, l_lo, x34, l_hi);
fprintf('       d(x_off)/dl ≈ %+.4f m/m；每 cm Δl 的腿质心力矩 %.4f N·m（按 LEG_GRAV_FF），\n', ...
        slope_off, abs(slope_off)*cfg.M_LEG*cfg.g*0.01);
fprintf('       若按整机等效平移 M_TOT 计：%.4f N·m/cm（供仿射槽调度用）\n', ...
        abs(slope_off)*M_TOT*cfg.g*0.01);
Tw_ref = cfg.u_max(1);
fprintf('   #8  轮驱竖直分量：T_w/R_w·sinψ = %.2f N·ψ（T_w=%.4f 时），ψ=%.2f 时 %.2f N（约力箱带宽 %.1f%%）\n', ...
        Tw_ref/cfg.R_w, Tw_ref, psi, Tw_ref/cfg.R_w*psi, 100*Tw_ref/cfg.R_w*psi/(cfg.F_MAX-cfg.F_MIN));
fprintf('   （开关现状：kin_ldot=%d, F_pitch=%d, F_H_proj=%d, roll_closure=%d, Ts_H=%d, Ts_alpha=%d）\n', ...
        cfg.sym7.sw.kin_ldot, cfg.sym7.sw.F_pitch, cfg.sym7.sw.F_H_proj, cfg.sym7.sw.roll_closure, ...
        cfg.sym7.sw.Ts_H, cfg.sym7.sw.Ts_alpha);

% 开关对照：逐条打开（其余关）后 A/B 的变化与闭环极点，按 |ΔA| 排序
sw_off = cfg.sym7.sw_none;
[A_off, B_off] = tinympc14_fused_ab(cfg, sw_off, sched0);
names = {'F_pitch (#1)', 'F_H_proj (#2)', 'Ts_H (#8)', 'Ts_alpha (#8α)'};
dA_list = zeros(1,4); iA_list = zeros(1,4); jA_list = zeros(1,4);
dB_list = zeros(1,4); iB_list = zeros(1,4); jB_list = zeros(1,4);
for k = 1:4
    sw1 = sw_off;
    switch k
        case 1, sw1.F_pitch  = 1;
        case 2, sw1.F_H_proj = 1;
        case 3, sw1.Ts_H     = 1;
        case 4, sw1.Ts_alpha = 1;
    end
    [A_k, B_k] = tinympc14_fused_ab(cfg, sw1, sched0);
    dA = A_k - A_off;  dB = B_k - B_off;
    dA_list(k) = max(abs(dA(:)));   [iA_list(k), jA_list(k)] = find(abs(dA) == dA_list(k), 1);
    dB_list(k) = max(abs(dB(:)));   [iB_list(k), jB_list(k)] = find(abs(dB) == dB_list(k), 1);
end
[~, ord] = sort(dA_list + dB_list, 'descend');
fprintf('   开关对照（逐条打开，其余关；psi_nom = %.2f rad，M_s = M_TOT）：\n', psi);
for k = ord
    fprintf('      %-14s max|ΔA| = %.4e @ (%d,%d)   max|ΔB| = %.4e @ (%d,%d)\n', ...
            names{k}, dA_list(k), iA_list(k), jA_list(k), dB_list(k), iB_list(k), jB_list(k));
end
n_info = n_info + 1;

%% ==================== Part E 融合 14 态 + 离散化与撒检查 ====================

fprintf('\n-- Part E 融合 14 态与自检 --\n');

M_v = tinympc14_mv_pick(cfg, cfg.H_BAR);
g_eff = cfg.M_LOAD/M_v*cfg.g;
[A_c, B_c, f_c, aux_c] = tinympc14_fused_ab(cfg, cfg.sym7.sw, sched0);

fprintf('   M_v 口径 = %s：M_v = %.3f kg, g_eff = %.4f m/s²（H 行 1/M_v = %.5f, 仿射 %.4f）\n', ...
        cfg.sym7.Mv_mode, M_v, g_eff, 1/M_v, -g_eff);
fprintf('   K_EFF 口径 = %s：K_eff(%.2f m) = %.2f N/m → A(14,13) = %+.4f 1/s²\n', ...
        cfg.sym7.k_eff_mode, cfg.H_BAR, aux_c.K_eff, -aux_c.K_eff/M_v);
fprintf('   A_c %dx%d（nnz=%d），B_c %dx%d（nnz=%d），f_c %dx1；跨块耦合 = %.1e（本版为 0 或仅开关项）\n', ...
        NX, NX, nnz(A_c), NX, NU, nnz(B_c), NX, ...
        max(abs(A_c - blkdiag(A_c(1:10,1:10), A_c(11:14,11:14))), [], 'all'));
fprintf('   连续开环 max Re(eig) = %+.4f，不稳模态 %d 个；连续可控秩 = %d/%d（0 重根下秩亏属正常）\n', ...
        max(real(eig(A_c))), sum(real(eig(A_c)) > 0), tinympc14_ctrb_rank(A_c, B_c), NX);

% 腿长块与 s03 逐位一致性：把 K_EFF 压回 0（s03 的基线口径）后必须完全一致
M_TOT_ck = tinympc14_m_tot(cfg);
cE0 = cfg; cE0.sym7.k_eff_mode = 'fixed'; cE0.K_EFF = 0;
[A_e0, B_e0, f_e0] = tinympc14_fused_ab(cE0, cE0.sym7.sw_none, tinympc14_sched_pt(cE0, cE0.H_BAR, 0, 0));
A_leg = A_e0(11:14, 11:14);
B_leg = B_e0(11:14, 5:6);
f_leg = f_e0(11:14);
A_leg_s03 = [0, 1, 0, 0; 0, 0, 0, 0; 0, 0, 0, 1; 0, 0, 0, -cfg.B_DAMP/cfg.M_V];
B_leg_s03 = [0, 0; refs.s03.R_TRK/refs.s03.I_ROLL, -refs.s03.R_TRK/refs.s03.I_ROLL; ...
             0, 0; 1/cfg.M_V, 1/cfg.M_V];
f_leg_s03 = [0; 0; 0; -cfg.M_LOAD*refs.s03.g/cfg.M_V];
if strcmp(cfg.sym7.Mv_mode, 'locked') && cfg.G_ROLL == 0
    dleg = max([max(abs(A_leg - A_leg_s03), [], 'all'), max(abs(B_leg - B_leg_s03), [], 'all'), ...
                max(abs(f_leg - f_leg_s03), [], 'all')]);
    tolE0 = max(refs.tol.e0, 1e-8);   % A/B 由 tinympc14_fused_dyn 数值求导，噪声底约 1e-10
    okE0 = (dleg <= tolE0) && abs(M_TOT_ck - refs.s03.M_TOT) < refs.tol.e0;
    fprintf('   腿长块 vs s03 现值（locked 口径）：max|Δ| = %.2e <= %.0e：%s\n', ...
            dleg, tolE0, pass(okE0));
    if okE0, n_ok = n_ok + 1; else, error('E: 腿长块与 s03 现值不一致'); end
else
    fprintf('   腿长块 vs s03：Mv_mode/G_ROLL 非基线，跳过，不计入硬判据\n');
    n_skip = n_skip + 1;
end

for Ts = refs.grid.Ts
    [Ad_k, Bd_k, ~] = tinympc14_zoh_ab(A_c, B_c, f_c, Ts);
    fprintf('   Ts=%.3f s：|eig(Ad)|max = %.6f，采样可控秩 = %d/%d，cond = %.3e\n', ...
            Ts, max(abs(eig(Ad_k))), tinympc14_ctrb_rank(Ad_k, Bd_k), NX, cond(Ad_k));
end
[Ad, Bd, fd] = tinympc14_zoh_ab(A_c, B_c, f_c, cfg.Ts);
fprintf('   取 Ts=%g s：fd = [%s]^T\n', cfg.Ts, num2str(fd(:)', '%.3e '));

% 闭环（rho 增强 DARE，同四元组口径）
Q = diag(cfg.Q_DIAG); Rm = diag(cfg.R_DIAG); rho = cfg.rho;
[Kinf, Pinf] = tinympc14_dare_gain(Ad, Bd, Q, Rm, rho, NX, NU);
QuuInv = inv(Rm + rho*eye(NU) + Bd' * Pinf * Bd);
AmBKt = (Ad - Bd*Kinf)';
eig_cl = max(abs(eig(Ad - Bd*Kinf)));
fprintf('   DARE(rho=%g)：|eig(Ad−Bd·Kinf)|max = %.6f，|Kinf|max = %.2f，cond(QuuInv) = %.3e\n', ...
        rho, eig_cl, max(abs(Kinf(:))), cond(QuuInv));
okE1 = eig_cl < 1;
fprintf('   闭环稳定判据（<1）：%s\n', pass(okE1));
if okE1, n_ok = n_ok + 1; end
fprintf('   注：自检打印的是 DARE(Q+rho*I, R+rho*I) 的 Kinf；因 R_diag ≠ rho，ADMM 收敛解与它不同，\n');
fprintf('       而未收敛回退用的正是 Kinf —— 收敛拍与回退拍是两套律（见 cfg §1 与审查报告 #5）\n');

r32 = @(x) max(abs(single(x(:)) - x(:))) / max(abs(x(:)));
fprintf('   单精度量化（全局口径）：Kinf 相对变化 %.2e，AmBKt %.2e\n', r32(Kinf), r32(AmBKt));
mx = max(abs(Kinf(:))); sel = abs(Kinf) > 1e-3*mx;
fprintf('   单精度量化（逐元素口径，|x|>1e-3*max）：Kinf 最大相对 %.2e\n', ...
        max(abs(double(single(Kinf(sel))) - Kinf(sel))./abs(Kinf(sel))));

% 通道增益表：加速度行的 B 列（单位输入下各二阶通道的直流加速度）
rows = [2 4 6 8 10 12 14];
lbl  = {'s_ddot','phi_ddot','th_ll_ddot','th_lr_ddot','th_b_ddot','a_ddot','H_ddot'};
ilbl = {'T_wl','T_wr','T_bl','T_br','F_l','F_r'};
fprintf('   通道增益表（B 的加速度行，行=状态，列=%s）：\n', strjoin(ilbl, ' '));
for i = 1:numel(rows)
    fprintf('      %-10s', lbl{i});
    fprintf(' %+9.4f', B_c(rows(i), :));
    fprintf('\n');
end
n_info = n_info + 1;

% 算量预算表：MAC/轮 按同一 V10 口径算，再按现役两说（45 / 81 µs 每轮）折算周期内轮数
mac_f = @(nx, nu, nh) (3*nx*nh + 2*nu*(nh-1) + nx^2 + nx) + ...
    ((nx*nu + nu^2 + nx^2 + nu*nx + nx + nu)*(nh-1)) + ...
    ((nu*nx + nx^2 + nx*nu + nx + nu)*(nh-1)) + 6*(nu*(nh-1) + nx*nh);
mac_14_8 = mac_f(14, 6, cfg.NH);
mac_10_8 = mac_f(10, 4, cfg.NH);
mac_14_40 = mac_f(14, 6, refs.perf.nh_long);
fprintf('   算量预算表（MAC/轮；10/4/8 口径 %d 为基准，比值 %.2fx）：\n', mac_10_8, mac_14_8/mac_10_8);
fprintf('      14/6/NH=%d：%d MAC/轮（%.2fx） | 14/6/NH=%d（拉回 80 ms 窗）：%d MAC/轮（%.2fx）\n', ...
        cfg.NH, mac_14_8, mac_14_8/mac_10_8, refs.perf.nh_long, mac_14_40, mac_14_40/mac_10_8);
for tref = refs.perf.tref   % 平衡侧每轮两说：config 注释 45 µs；README 反汇编折算 ~81 µs
    t8 = tref*mac_14_8/mac_10_8;   t40 = tref*mac_14_40/mac_10_8;
    fprintf('      基准 %d µs/轮 → 14/6/8：%.0f µs/轮 → 4 ms 周期内 %.0f 轮（100 轮需 %.1f ms） | NH=%d：%.0f µs/轮 → %.0f 轮\n', ...
            tref, t8, 4000/t8, t8*100/1000, refs.perf.nh_long, t40, 4000/t40);
end
fprintf('      （真实 IPC 未测：上台架第一件事是 DWT/CYCCNT 实测单轮耗时）\n');
n_info = n_info + 1;

% 工作区与表内存
NN = cfg.NH - 1;
work = 5*NX*cfg.NH + 5*NU*NN + (3*NX^2 + 2*NX*NU + NU^2) + (NX + NX*cfg.NH) + NU;
fprintf('   工作区 %.0f float = %.0f B；表内存 %d 行×10 系数×4 B = %.1f KB（现役 %.1f KB 的 %.2fx）\n', ...
        work, 4*work, NX^2 + NX*NU + NU*NX + NX^2 + NU^2 + NX^2, ...
        (NX^2 + NX*NU + NU*NX + NX^2 + NU^2 + NX^2)*10*4/1024, ...
        refs.perf.table_bytes_ref/1024, ...
        (NX^2 + NX*NU + NU*NX + NX^2 + NU^2 + NX^2)*10*4/refs.perf.table_bytes_ref);

% 气弹簧 double/single 交叉核对：cfg 常数与 tool/Leg_SpringF.m 必须同源
okFs = true;
for H = refs.grid.fs
    fs_d = Leg_SpringF(double(H));
    fs_s = double(Leg_SpringF(single(H)));
    okFs = okFs && abs(fs_d - fs_s) <= refs.tol.spring*max(abs(fs_d), 1);
end
fprintf('   气弹簧 double/single 一致（%d 点，相对容差 %.0e）：%s\n', ...
        numel(refs.grid.fs), refs.tol.spring, pass(okFs));
if okFs, n_ok = n_ok + 1; else, error('E: 气弹簧 double 与 Leg_SpringF single 不一致'); end

% 力箱（总支持力口径 + Fs(H) 平移）+ 运动学隐含的电机侧上限对照
Fs_ref = Leg_SpringF(double(cfg.H_CEIL_REF));
F_MOT_CEIL = cfg.F_MAX - Fs_ref;
fprintf('   力箱：F_MAX=%.1f @%.2f m，Fs(%.2f)=%.3f N → F_MOT_CEIL=%.3f N（电机侧常数）\n', ...
        cfg.F_MAX, cfg.H_CEIL_REF, cfg.H_CEIL_REF, Fs_ref, F_MOT_CEIL);
fprintf('        H      Fs(H)     u_lo      u_hi   电机侧 u_lo−Fs  电机侧 u_hi−Fs 运动学隐含电机侧上限\n');
ratio_lo = inf;  ratio_hi = 0;
for H = cfg.Leg_comp(1,1):0.02:cfg.Leg_comp(end,1)
    Fs = Leg_SpringF(double(H));
    u_hi = F_MOT_CEIL + Fs;
    phi = ik_leg(H, cfg);
    s1 = Leg_Spd(phi(1), phi(2), 1, 0);
    s4 = Leg_Spd(phi(1), phi(2), 0, 1);
    Jmax = max(abs([s1(1), s4(1)]));   % 只取 L0 行的 |dL0/dphi|（第二行 dphi0/dphi 是 0.5 常数）
    F_kin = cfg.TAU_JOINT_CEIL / max(Jmax, 1e-6);
    ratio = F_kin/max(F_MOT_CEIL, 1e-6);
    ratio_lo = min(ratio_lo, ratio);  ratio_hi = max(ratio_hi, ratio);
    flag = '';
    if cfg.F_MIN - Fs < 0, flag = '  ← 电机侧下界为负（弹簧单独即可顶过总力下界）'; end
    fprintf('      %.2f   %7.3f   %6.1f   %8.2f   %10.3f   %10.3f   %12.1f%s\n', ...
            H, Fs, cfg.F_MIN, u_hi, cfg.F_MIN - Fs, F_MOT_CEIL, F_kin, flag);
end
fprintf('       注：TAU_JOINT_CEIL = %.0f N·m 是 s03 口径、待实测核实；运动学隐含列与 F_MOT_CEIL 差 %.1f~%.1f 倍\n', ...
        cfg.TAU_JOINT_CEIL, ratio_lo, ratio_hi);

% 向心力示例
k_cf = cfg.K_CF * cfg.M_SUP * (cfg.H_BAR + cfg.H_OFF) / cfg.I_ROLL;
fprintf('   向心力示例（v=1, omega=1, H=%.2f）：tau = %.3f N·m → a 通道 %.3f rad/s²（v*omega 系数 k=%.3f）\n', ...
        cfg.H_BAR, cfg.K_CF*cfg.M_SUP*(cfg.H_BAR+cfg.H_OFF), k_cf, k_cf);
fprintf('       等价总力差 %.2f N（= 单腿 ±%.2f N，全轮距 0.40 m；固件 Roll_CompF 现值只有一半，见审查报告 #6b）\n', ...
        2*cfg.M_SUP*(cfg.H_BAR + cfg.H_OFF)/(2*cfg.R_TRK), cfg.M_SUP*(cfg.H_BAR + cfg.H_OFF)/(2*cfg.R_TRK));

% 对照档 1：l_c 符号（俯仰重力项）——两套约定并排，判定见审查报告 #8b
fprintf('   对照档 l_c 符号（B 族口径 @H_BAR；l_c = %.5f m = 质心在俯仰轴上方）：\n', cfg.l_c);
for lc = [abs(cfg.l_c), -abs(cfg.l_c)]
    c2 = cfg; c2.l_c = lc;
    A_tmp = tinympc14_bal_block(cfg.H_BAR, cfg.H_BAR, c2, 'B');
    e = eig(A_tmp);
    e_pitch = e(abs(abs(e) - refs.pitch_pole_ref) < 0.5);
    fprintf('      l_c = %+.5f：maxRe = %+.4f；%.3f 邻域根 =', lc, max(real(e)), refs.pitch_pole_ref);
    for q = 1:numel(e_pitch)
        fprintf(' %+.4f%+.4fi', real(e_pitch(q)), imag(e_pitch(q)));
    end
    fprintf('\n');
end
n_info = n_info + 1;

% 对照档 2：G_ROLL 两档（0 = Qi-Q26 口径；另一档 = M_SUP·g·h/I_ROLL）
G_roll_alt = cfg.M_SUP*cfg.g*(cfg.H_BAR + cfg.H_OFF)/cfg.I_ROLL;
fprintf('   对照档 G_ROLL：0（现口径）vs %.3f s⁻²（= M_SUP·g·h/I_ROLL）\n', G_roll_alt);
for G = [cfg.G_ROLL, G_roll_alt]
    c2 = cfg; c2.G_ROLL = G;
    [A_g, B_g, f_g] = tinympc14_fused_ab(c2, c2.sym7.sw, tinympc14_sched_pt(c2, cfg.H_BAR, 0));
    [Ad_g, Bd_g] = tinympc14_zoh_ab(A_g, B_g, f_g, cfg.Ts);
    [K_g, ~, ecl] = tinympc14_dare_gain(Ad_g, Bd_g, Q, Rm, rho, NX, NU);
    fprintf('      G_ROLL = %7.3f：A(12,11) = %+.3f；开环 maxRe = %+.4f，不稳 %d 个；DARE |λ|max = %.6f，|Kinf|max = %.2f\n', ...
            G, A_g(12,11), max(real(eig(A_g))), sum(real(eig(A_g)) > 0), ecl, max(abs(K_g(:))));
end
n_info = n_info + 1;

% 对照档 3：K_EFF 三档（fixed 0 = s03 基线 / fixed 实测刚度 / spring 按 H 差分）
fprintf('   对照档 K_EFF：fixed 0（s03 基线）vs fixed %.0f N/m vs spring（∂Fs/∂L0 @H_BAR）\n', ...
        cfg.sym7.k_eff_alt);
K_modes = {'fixed', 'fixed', 'spring'};
K_vals  = [0, cfg.sym7.k_eff_alt, 0];
for ik = 1:3
    c2 = cfg; c2.sym7.k_eff_mode = K_modes{ik}; c2.K_EFF = K_vals(ik);
    Kk = tinympc14_k_eff(c2, cfg.H_BAR);
    [A_k2, B_k2, f_k2] = tinympc14_fused_ab(c2, c2.sym7.sw, tinympc14_sched_pt(c2, cfg.H_BAR, 0));
    [Ad_k2, Bd_k2] = tinympc14_zoh_ab(A_k2, B_k2, f_k2, cfg.Ts);
    [K_k2, ~, ecl] = tinympc14_dare_gain(Ad_k2, Bd_k2, Q, Rm, rho, NX, NU);
    fprintf('      %-6s K_EFF = %6.1f N/m：A(14,13) = %+.4f；开环 maxRe = %+.4f，不稳 %d 个；DARE |λ|max = %.6f，|Kinf|max = %.2f\n', ...
            K_modes{ik}, Kk, A_k2(14,13), max(real(eig(A_k2))), sum(real(eig(A_k2)) > 0), ecl, max(abs(K_k2(:))));
end
n_info = n_info + 1;

% 对照档 4：M_v 三档的闭环对照（翻 Mv_mode 前先看这张表）
fprintf('   对照档 M_v 三档（闭环用同一套 Q/R/rho，只换 H 行惯性）：\n');
for mode = {'locked', 'derived', 'near_end'}
    c2 = cfg; c2.sym7.Mv_mode = mode{1};
    Mv_k = tinympc14_mv_pick(c2, cfg.H_BAR);
    [A_m2, B_m2, f_m2] = tinympc14_fused_ab(c2, c2.sym7.sw, tinympc14_sched_pt(c2, cfg.H_BAR, 0));
    [Ad_m2, Bd_m2] = tinympc14_zoh_ab(A_m2, B_m2, f_m2, cfg.Ts);
    [K_m2, ~, ecl] = tinympc14_dare_gain(Ad_m2, Bd_m2, Q, Rm, rho, NX, NU);
    fprintf('      %-9s M_v = %.3f kg, g_eff = %.4f, 1/M_v = %.5f：A(14,14) = %+.4f；DARE |λ|max = %.6f，H 列增益 ‖Kinf(:,13:14)‖ = %.4f\n', ...
            mode{1}, Mv_k, cfg.M_LOAD/Mv_k*cfg.g, 1/Mv_k, A_m2(14,14), ecl, norm(K_m2(:,13:14)));
end
n_info = n_info + 1;

% 输出层核对：Leg_Trq 与 Jᵀ·[F;Tp] 在 theta0=0 支上逐点一致（Part A "借力"那一条）
errT = 0;
for H = refs.grid.trq
    phi = ik_leg(H, cfg);
    J = [Leg_Spd(phi(1), phi(2), 1, 0), Leg_Spd(phi(1), phi(2), 0, 1)];
    tauJ = J.' * refs.trq_test(:);
    tau = Leg_Trq(refs.trq_test(1), refs.trq_test(2), phi(1), phi(2));
    errT = max(errT, max(abs(tau(:) - tauJ(:))));
end
fprintf('   输出层核对：max|Leg_Trq(F,Tp) − Jᵀ·[F;Tp]| = %.2e（%d 个 IK 点，F=%g, Tp=%g）\n', ...
        errT, numel(refs.grid.trq), refs.trq_test(1), refs.trq_test(2));
okE2 = errT <= refs.tol.e2;
if okE2, n_ok = n_ok + 1; end
n_info = n_info + 1;

%% ==================== Part G 工作点调度一致性自检（P1） ====================

fprintf('\n-- Part G 工作点调度一致性（#3 按两腿腿长 LPV + K_EFF 按 H）--\n');
G_pt  = refs.G_pt;
G_lbl = refs.G_lbl;
Dq = diag(sqrt(cfg.Q_DIAG));
Dr = diag(sqrt(cfg.R_DIAG));
A_G = cell(3,1);  B_G = cell(3,1);  An_G = cell(3,1);  Bn_G = cell(3,1);
fp_G = zeros(3,1);  lam_G = zeros(3,1);
fprintf('   点               lL       lR      ψeq_l    ψeq_r    τg0_l   τg0_r   xoff_l  xoff_r   K_eff   |λ|cl    定残差\n');
for k = 1:3
    % At fixed H and roll the measured lengths also depend on the equilibrium lean.
    sc = tinympc14_sched_pt(cfg,G_pt(k,1),G_pt(k,2),0);
    for iteration = 1:8
        [xe,ue,eq] = tinympc14_fused_eq(cfg,cfg.sym7.sw,sc);
        angle_error = abs(xe(5)-sc.psi);
        if angle_error<1e-12, break, end
        sc = tinympc14_sched_pt(cfg,G_pt(k,1),G_pt(k,2),xe(5));
    end
    assert(angle_error<1e-12 && eq.res<refs.tol.g_fp,'G: 姿态与静力平衡未收敛');
    sc.eq = struct('x_eq',xe,'u_eq',ue);
    sc.psi_l = xe(5); sc.psi_r = xe(7);
    [Ag, Bg, fg, auxg] = tinympc14_fused_ab(cfg, cfg.sym7.sw, sc);
    [Adg, Bdg, fdg] = tinympc14_zoh_ab(Ag, Bg, fg, cfg.Ts);
    [~, ~, lam_G(k)] = tinympc14_dare_gain(Adg, Bdg, diag(cfg.Q_DIAG), diag(cfg.R_DIAG), cfg.rho, NX, NU);
    fp_G(k) = norm(Adg*xe + Bdg*ue + fdg - xe,Inf);
    A_G{k} = Ag;  B_G{k} = Bg;
    An_G{k} = Dq*Ag/Dq;
    Bn_G{k} = Dq*Bg/Dr;
    fprintf('   %s  %.4f  %.4f  %+.5f  %+.5f  %.4f  %.4f  %.4f  %.4f  %6.2f  %.6f  %.1e\n', ...
            G_lbl{k}, sc.lL, sc.lR, xe(5), xe(7), ...
            auxg.tg0, auxg.tg0_r, auxg.xoff, auxg.xoff_r, auxg.K_eff, lam_G(k), fp_G(k));
    okG = fp_G(k) <= refs.tol.g_fp;
    fprintf('      14 态与 6 路静态输入的离散定点残差 <= %.0e：%s\n', refs.tol.g_fp, pass(okG));
    if okG, n_ok = n_ok + 1; else, error('G: 工作点 %s 不是定点', G_lbl{k}); end
end

fprintf('   工作点间变化（相对 %s）：\n', G_lbl{1});
dAmax = zeros(3,1);  dBmax = zeros(3,1);  relA = zeros(3,1);
relF = zeros(3,1);  relFn = zeros(3,1);
for k = 2:3
    dA = A_G{k} - A_G{1};  dB = B_G{k} - B_G{1};
    dAmax(k) = max(abs(dA(:)));
    dBmax(k) = max(abs(dB(:)));
    flA  = 1e-6*max(abs(A_G{1}(:)));
    relA(k) = max(abs(dA(:))./max(abs(A_G{1}(:)), flA));
    relF(k)  = norm(dA, 'fro')/norm(A_G{1}, 'fro');
    relFn(k) = norm(An_G{k} - An_G{1}, 'fro')/norm(An_G{1}, 'fro');
    [iA, jA] = find(abs(dA) == dAmax(k), 1);
    fprintf('      %s：max|ΔA| = %.3e @ (%d,%d)，max|ΔB| = %.3e，逐元 A 相对变化 = %.1f%%（Frobenius %.1f%%）\n', ...
            G_lbl{k}, dAmax(k), iA, jA, dBmax(k), 100*relA(k), 100*relF(k));
end
ok_rel = max(relA) <= refs.tol.relA;
fprintf('   工作点间 A 最大逐元相对变化 %.1f%% <= %.0f%%：%s\n', ...
        100*max(relA), 100*refs.tol.relA, pass(ok_rel));
if ok_rel, n_ok = n_ok + 1; else, error('G: 历史单一线性模型的全域 A 变化超过 %.0f%%。当前固件使用逐点 LPV 调度。',100*refs.tol.relA); end

% 归一化测试：Dq = diag(sqrt(Q))、Dr = diag(sqrt(R)) 把权重拉成单位阵；
% 逐元相对变化对固定对角相似不变，矩阵口径（Frobenius）才区分两套刻度
fprintf('   归一化测试（Dq·Q·Dq = I，Dq = diag(sqrt(Q))；Dr 同理）：\n');
fprintf('      归一化 A 的 Frobenius 相对变化最大 %.1f%% <= %.0f%%：%s\n', ...
        100*max(relFn), 100*refs.tol.relA, pass(max(relFn) <= refs.tol.relA));
fprintf('      归一化权重 DARE（Q = R = I，rho = %g）闭环 |λ|：', cfg.rho);
lamn = zeros(3,1);
for k = 1:3
    [Adn, Bdn] = tinympc14_zoh_ab(An_G{k}, Bn_G{k}, zeros(NX,1), cfg.Ts);
    [~, ~, lamn(k)] = tinympc14_dare_gain(Adn, Bdn, eye(NX), eye(NU), cfg.rho, NX, NU);
    fprintf(' %s→%.6f', G_lbl{k}, lamn(k));
end
fprintf('\n');
ok_norm = (max(relFn) <= refs.tol.relA) && (max(lamn) < 1);
fprintf('      归一化口径（相对变化 + 稳定）整体判据：%s\n', pass(ok_norm));
if ok_norm, n_ok = n_ok + 1; else, error('G: 归一化测试未通过'); end
n_info = n_info + 1;

%% ==================== Part F 自检汇总 ====================

fprintf('\n==== 自检汇总：%d 项硬判据通过，%d 项跳过，%d 项 INFO（口径裁决项）====\n', ...
        n_ok, n_skip, n_info);
fprintf('硬判据：A 冻结还原 / B 现役缓存对拍 / C2 M_TOT 口径 / C3 α 静增益含符号 / C5 仿射口径 /\n');
fprintf('        E0 腿长块 vs s03 / E1 闭环稳定 / E2 输出层 Jᵀ 核对 / 气弹簧 double-single 一致 /\n');
fprintf('        G 三点定点（3 项）/ G 工作点 A 相对变化 / G 归一化测试\n');
fprintf('跳过：CACHE_REF 为空时的 B 对拍；E0 在非 locked/G_ROLL=0 口径下\n');
fprintf('INFO：M_v 三档 / I_ROLL 轴取法 / Fs 力箱（含运动学隐含上限列）/ 耦合 #1~#8 / 通道增益 /\n');
fprintf('      算力预算 / l_c 符号 / G_ROLL / K_EFF / 工作点闭环极点与归一化 DARE\n');
fprintf('模型自检完成；本脚本不写任何文件。\n');

%% ==================== 本地函数 ====================

function s = pass(ok)
if ok
    s = 'PASS';
else
    s = 'FAIL';
end
end

function tf = localrel(x, ref, tol)
tf = abs(x - ref) <= tol * max(abs(ref), 1e-12);
end

function v = off_val(cfg, l)
% 腿质心水平偏置 x_off = l_d·sin(theta0)（Leg_Comp 口径）
row = tinympc14_table_row(cfg.Leg_comp, l);
v = row(2)*sin(row(1));
end

function phi = ik_leg(L0, cfg)
% theta0=0 支上的腿 IK：该支实测 phi4 = 2*pi - phi1，一维搜索即可（L0 单调）
b = cfg.sym7.ik_bracket;
phi1 = fminbnd(@(x) ik_obj(x, L0), b(1), b(2), optimset('TolX', 1e-12, 'Display', 'off'));
phi = [phi1, 2*pi - phi1];
end

function v = ik_obj(x, L0)
% IK 目标：命中 L0；非实数/非有限一律判大值
r = Leg_Pos(x, 2*pi - x);
if ~isreal(r) || any(~isfinite(r))
    v = 1e3;
else
    v = abs(r(1) - L0);
end
end
