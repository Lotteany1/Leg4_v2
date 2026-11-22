function mdl = tinympc14_model_at(p, lL, lR, psiL, psiR, roll, height)
% Evaluate the controller model at the measured operating point.
if ~isfield(p,'URDF_SOURCE') || isfield(p,'ONLINE_TERMINAL')
    mdl = tinympc14_lpv_eval(p, lL, lR, (psiL+psiR)/2);
    return;
end
sched = struct('lL',lL,'lR',lR,'psi',(psiL+psiR)/2, ...
    'psi_l',psiL,'psi_r',psiR,'alpha',roll,'H',height);
[A, B, f] = tinympc14_fused_ab(p, p.sym7.sw_fused, sched);
mdl.force_roll = roll;
mdl.force_psi = [psiL; psiR];
[mdl.Ad, mdl.Bd, mdl.fd] = tinympc14_zoh_ab(A, B, f, p.Ts);
mdl.q_diag = p.Q_DIAG(:);
mdl.r_diag = p.R_DIAG(:);
mdl.u_min = p.u_min(:);
mdl.u_max = p.u_max(:);
mdl.x_min = p.x_min(:);
mdl.x_max = p.x_max(:);
[mdl.Kinf,mdl.Pinf] = tinympc14_dare_gain(mdl.Ad,mdl.Bd,diag(mdl.q_diag), ...
    diag(mdl.r_diag),p.rho,p.NX,p.NU);
mdl.AmBKt = (mdl.Ad - mdl.Bd*mdl.Kinf)';
mdl.Bd_pinv = pinv(mdl.Bd);
mdl.u_ss = mdl.Bd_pinv * (-mdl.fd);
mdl.rc = tinympc14_riccati_cache(mdl,p);
end
