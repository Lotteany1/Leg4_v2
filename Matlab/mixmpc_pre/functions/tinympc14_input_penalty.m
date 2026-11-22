function penalty = tinympc14_input_penalty(mdl, cfg)
% Algorithmic input metric; original Q/R and terminal cost remain unchanged.
nu=numel(mdl.r_diag);
penalty=cfg.rho*ones(nu,1);
if numel(mdl.q_diag)==14 && nu==6
    % Wheel torque bounds and axial force slew limits can stay active across
    % the horizon. Scale their ADMM consensus steps in their own input units.
    channels = [1,2,5,6];
    weights = mdl.r_diag(:);
    penalty(channels) = max(cfg.rho,weights(channels));
end
end
