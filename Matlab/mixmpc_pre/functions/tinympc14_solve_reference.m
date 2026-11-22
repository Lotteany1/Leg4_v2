function [u_tot, info, warm] = tinympc14_solve_reference(p, lL, lR, x0, xref, fed, ffwd, u_prev, warm,psi)
if nargin<10, psi=0; end
mdl = tinympc14_lpv_eval(p, lL, lR, psi);
NU = p.NU;  NN = p.NH - 1;
umin = p.u_min(:);  umax = p.u_max(:);
rate6 = [Inf(NU-2,1); p.DF_RATE; p.DF_RATE];
x0 = x0(:);  xref = xref(:);  fed = fed(:);  ffwd = ffwd(:);  u_prev = u_prev(:);
u_ss = mdl.Bd_pinv * (xref - mdl.Ad*xref - mdl.fd - fed);
u_ff = u_ss + ffwd;
f_eff = mdl.fd + fed + mdl.Bd*u_ff;
Pf = mdl.Pinf * f_eff;
APf = mdl.AmBKt * Pf;
BPf = mdl.Bd' * Pf;
u_prev_clip = min(max(u_prev, umin), umax);
u_seed = min(max(u_prev_clip - u_ff, umin - u_ff), umax - u_ff);
mdl_s = mdl;
mdl_s.u_min = umin - u_ff;
mdl_s.u_max = umax - u_ff;
opts = struct('fd', f_eff, 'APf', APf, 'BPf', BPf, 'df_rate', rate6, ...
              'u_prev', u_seed, 'fallback', 'uss', 'fb_ffwd', ffwd, 'return_proj', true);
[u_bare, info] = tinympc14_admm(mdl_s, x0, repmat(xref, 1, p.NH), zeros(NU, NN), p, warm, opts);
u_tot = min(max(u_bare + u_ff, max(umin, u_prev_clip - rate6)), ...
                             min(umax, u_prev_clip + rate6));
if nargout > 2 && all(isfinite(info.v(:))) && all(isfinite(info.g(:))) && ...
   all(isfinite(info.z(:))) && all(isfinite(info.y(:)))
    warm = struct('v', info.v, 'g', info.g, 'z', info.z, 'y', info.y);
elseif nargout > 2
    warm = [];
end
end

