function mdl = tinympc14_lpv_eval(p, lL, lR, psi)
% Evaluate the same scheduling backend and float model used by TinyMPC.
if nargin < 4 || isempty(psi)
    psi = 0;
end
lL = double(single(lpv_clamp(lL,p.LEG_RANGE)));
lR = double(single(lpv_clamp(lR,p.LEG_RANGE)));
if ~isfinite(psi), psi = 0; end
if isfield(p,'ONLINE_TERMINAL')
    table = p.ONLINE_TERMINAL;
    lL = clamp_angle(lL,table.length_grid);
    lR = clamp_angle(lR,table.length_grid);
    psi = clamp_angle(psi,table.angle_grid);
    [Ad,Bd,fd,P] = tinympc14_online_model(p,table,lL,lR,psi);
    mdl.Ad = double(single(Ad));
    mdl.Bd = double(single(Bd));
    mdl.fd = double(single(fd));
    mdl.Pinf = double(single(P));
else
    [mdl,psi] = embedded_model(p,lL,lR,psi);
end
mdl.q_diag = p.Q_DIAG(:);
mdl.r_diag = p.R_DIAG(:);
mdl.u_min  = p.u_min(:);
mdl.u_max  = p.u_max(:);
mdl.x_min  = p.x_min(:);
mdl.x_max  = p.x_max(:);
mdl.Bd_pinv = pinv(mdl.Bd);
mdl.u_ss    = mdl.Bd_pinv * (-mdl.fd);
mdl.lL      = lL;
mdl.lR      = lR;
mdl.psi     = psi;
mdl.force_roll = atan((lL-lR)*cos(psi)/(2*p.R_TRK));
mdl.force_psi = psi;
mdl.rc = tinympc14_riccati_cache(mdl,p);
if isfield(p,'ONLINE_TERMINAL')
    reference_rc=tinympc14_riccati_cache(mdl,p,p.rho*ones(numel(mdl.r_diag),1));
    mdl.Kinf = double(single(reference_rc.K(:,:,end)));
    mdl.QuuInv = double(single(reference_rc.Qu(:,:,end)));
    mdl.AmBKt = double(single(reference_rc.At(:,:,end)));
end
end

function [mdl,psi] = embedded_model(p,lL,lR,psi)
persistent cached
if isempty(cached)
    here = fileparts(mfilename('fullpath'));
    cached = load(fullfile(here, '..', 'build', 'tinympc14_lpv_psi_cache.mat'), 'table','weights');
end
assert(isequal(cached.weights.Q,p.Q_DIAG) && isequal(cached.weights.R,p.R_DIAG) ...
    && cached.weights.rho==p.rho,'Terminal weights changed; rebuild before evaluation');
psi = clamp_angle(psi,[-.3,.3]);
[mdl.Ad,mdl.Bd,mdl.fd,mdl.Kinf,mdl.Pinf,mdl.QuuInv,mdl.AmBKt] = ...
    tinympc14_lpv_embedded(p,cached.table,lL,lR,psi);
end

function psi = clamp_angle(psi,grid)
psi = double(single(min(max(psi,grid(1)),grid(end))));
end

function v = lpv_clamp(v,range)
if ~isfinite(v)
    v = 0.27;
    return;
end
v = min(max(v, range(1)), range(2));
end

