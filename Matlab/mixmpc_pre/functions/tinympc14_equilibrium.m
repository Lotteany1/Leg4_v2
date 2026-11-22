function [xbar, u_ss, info] = tinympc14_equilibrium(cfg, sw, sched)
% 融合模型静止平衡：两腿同角、alpha=0、theta_b 取 pitch_ref，解腿角与六路输入
% 平衡点由非线性核 tinympc14_fused_eq 解出，再作为仿射项锚点；u_ss 即平衡输入
if nargin < 3 || isempty(sched)
    sched = tinympc14_sched_pt(cfg, cfg.sym7.sched_l, 0, cfg.sym7.sched_psi);
end
NX = cfg.NX;
[xe, ue, ei] = tinympc14_fused_eq(cfg, sw, sched);
eq = struct('x_eq', xe, 'u_eq', ue, 'theta_leg', ei.theta_leg, ...
            'res', ei.res, 'iters', ei.iters);
sched.eq = eq;
sched.psi_l=xe(5); sched.psi_r=xe(7);
[A, B, f, aux] = tinympc14_fused_ab(cfg, sw, sched);
acc = 2:2:NX;
E = [A(acc, 5) + A(acc, 7), B(acc, :)];

xbar = eq.x_eq;
u_ss = eq.u_eq;

[Ad, Bd, fd] = tinympc14_zoh_ab(A, B, f, cfg.Ts);

info = struct('theta_leg', eq.theta_leg, 'u_cont', eq.u_eq, ...
              'rank_E', rank(E), 'cond_E', cond(E), ...
              'res_E', eq.res, ...
              'res_fp', max(abs(Ad*xbar + Bd*u_ss + fd - xbar)), ...
              'eq', eq, 'theta_star', aux.theta_star);
end
