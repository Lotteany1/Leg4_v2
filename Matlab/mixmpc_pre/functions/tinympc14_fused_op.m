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
op.I_roll = cfg.I_ROLL;
if isfield(cfg,'roll_height') && ~isfield(cfg,'URDF_SOURCE')
    h = sched.H;
    if isfield(sched,'roll_height')
        h = sched.roll_height;
    end
    h = min(max(h,cfg.roll_height(1)),cfg.roll_height(end));
    k = 1 + sum(h > cfg.roll_height(2:end-1));
    t = (h-cfg.roll_height(k))/(cfg.roll_height(k+1)-cfg.roll_height(k));
    op.I_roll = cfg.roll_inertia(k) + t*(cfg.roll_inertia(k+1)-cfg.roll_inertia(k));
end
op.psi = sched.psi;
end
