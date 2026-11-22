function op = tinympc14_fused_op(cfg, sched)
% 本调度点的平衡块与腿长块参数包：一次算好供非线性核与数值雅可比复用
[op.A_bal, op.B_bal, op.f_bal, op.aux] = tinympc14_bal_block(sched.lL, sched.lR, cfg, cfg.sym7.affine_mode);
op.lL = sched.lL;
op.lR = sched.lR;
op.H  = sched.H;  % sched.H is absolute; only x(13) uses H-H_REF.
op.M_v = tinympc14_mv_pick(cfg, sched.H);
% Old and training configurations use the wheel-excluded support mass.
op.M_load = cfg.M_SUP;
if isfield(cfg,'M_LOAD')
    op.M_load = cfg.M_LOAD; % (dV/dH)/g of the coupled leg mechanism at nominal height
end
op.K_eff = tinympc14_k_eff(cfg, sched.H);
op.psi = sched.psi;
end
