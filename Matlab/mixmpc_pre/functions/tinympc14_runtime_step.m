function [rt, y] = tinympc14_runtime_step(rt, tick, x, xref, lL, lR, psiL, psiR, v, omega, ffwd_ext, measured_force, pending_diff_impulse)
% x is the observation delivered by the estimator, including H-H_REF and dH.
% Measurement filtering runs at the estimator/control boundary in firmware.
% measured_force is the optional paired total axial feedback in N. NaN marks an
% unavailable side. Only its error for the first upcoming node enters prediction.
% pending_diff_impulse [N·s] is the co-sampled external actuator observer's
% left-minus-right impulse, with the first-node dynamic correction removed once.
p = rt.p;
if nargin < 11 || isempty(ffwd_ext)
    ffwd_ext = zeros(p.NU, 1);
end
if nargin < 12 || isempty(measured_force), measured_force = [NaN; NaN]; end
if nargin < 13 || isempty(pending_diff_impulse), pending_diff_impulse = 0; end
measured_force = reshape(measured_force,2,1);
NX = p.NX;  NU = p.NU;  NH = p.NH;  NN = NH - 1;
x = x(:);  xref = xref(:);
umin = p.u_min(:);  umax = p.u_max(:);

y = struct('u_tot', zeros(NU,1), 'u_mpc',zeros(NU,1), 'u_bare', zeros(NU,1), 'u_ff', zeros(NU,1), ...
           'u_ss', zeros(NU,1), 'ffwd', zeros(NU,1), 'fed', zeros(NX,1), ...
           'MgF', zeros(2,1), 'Fs', zeros(2,1), 'Fc', zeros(2,1), ...
           'Tp', zeros(2,1), 'F_motor', zeros(2,1), 'T_w', zeros(2,1), ...
           'u_motor', zeros(NU,1), 'u_plant', zeros(NU,1), 'c', [], ...
           'conv', 1, 'iters', 0, 'node', -1, 'stale', false, ...
           'fallback', false, 'nan_input', false, 'lpv_updated', false);

bad = ~(all(isfinite(x)) && all(isfinite(xref)) && all(isfinite(ffwd_ext(:))) && isfinite(lL) && isfinite(lR) && ...
        isfinite(psiL) && isfinite(psiR) && isfinite(v) && isfinite(omega));
c = tinympc14_output(p, lL, lR, psiL, psiR, v, omega);

if bad
    rt.nan = rt.nan + 1;
    rt.seq_valid = false;
    u_tot = min(max(rt.u_prev, umin), umax);
    y.u_tot = u_tot;
    y.u_mpc = u_tot;
    y.u_bare = u_tot;
    y.u_plant = u_tot;
    y.u_motor = u_tot;
    y.nan_input = true;
    y.fallback = true;
    rt.fallback = rt.fallback + 1;
    y.Tp = c.MgF + u_tot(3:4);
    y.F_motor = u_tot(5:6) - c.Fs;
    y.T_w = -u_tot(1:2);
    y.c = c;
    y.MgF = c.MgF;  y.Fs = c.Fs;  y.Fc = c.Fc;
    rt.u_prev = u_tot;
    return;
end

range = p.LEG_RANGE;
psi = repmat(mean(x([5,7])),1,2);
if isfield(p,'URDF_SOURCE')
    range = p.LEG_RANGE;
    if ~isfield(p,'ONLINE_TERMINAL'), psi = x([5,7])'; end
end
len = min(max([lL,lR],range(1)),range(2));
solve_due = tick - rt.solve_tick >= rt.solve_period_ms - 1e-9;
at_upper = all(len >= range(2)) && ~all(rt.last_l >= range(2));
if (solve_due || ~(isfield(p,'URDF_SOURCE') || isfield(p,'ONLINE_TERMINAL'))) && ...
        (at_upper || max(abs(len-rt.last_l)) > 0.005 || max(abs(psi-rt.last_psi)) > 0.05)
    rt.model = tinympc14_model_at(p, len(1), len(2), psi(1), psi(2), x(11), p.H_REF+x(13));
    rt.last_l = len;
    rt.last_psi = psi;
    rt.warm = [];
    rt.seq_valid = false;
    rt.lpv_updates = rt.lpv_updates + 1;
    y.lpv_updated = true;
end
mdl = rt.model;
valid_force = isfinite(measured_force);
span = umax(5:6) - umin(5:6);
force_error = min(max(measured_force-rt.u_prev(5:6),-span),span);
force_error(~valid_force) = 0;
% G = H^-1 Bd' P Bd(:,5:6), H = R + rho I + Bd' P Bd.
force_gain = zeros(NU,2);
if any(valid_force)
    if isfield(mdl, 'QuuInv')
        input_selector = eye(NU);
        force_gain = input_selector(:,5:6) - mdl.QuuInv(:,5:6)*diag(mdl.r_diag(5:6)+p.rho);
    else
        BPB = mdl.Bd'*mdl.Pinf*mdl.Bd;
        force_gain = (diag(mdl.r_diag(:)+p.rho)+BPB) \ BPB(:,5:6);
    end
end
sensed_delta = force_gain * force_error;

ffwd = ffwd_ext(:); % generic solver test offset only; Fc is an output-layer plugin
roll_force = tinympc14_roll_force(p,p.H_REF+x(13),x(11),mdl.force_roll,mdl.force_psi);
fed = mdl.Bd(:,5:6)*roll_force;
u_ss = mdl.Bd_pinv * (xref - mdl.Ad*xref - mdl.fd - fed);
u_ff = u_ss + ffwd;

f_eff = mdl.fd + fed + mdl.Bd*u_ff;
Pf = mdl.Pinf * f_eff;
APf = mdl.AmBKt * Pf;
BPf = mdl.Bd' * Pf;

u_prev_clip = min(max(rt.u_prev, umin), umax);
force_comp = [zeros(4,1); c.Fc];
umin_s = umin - force_comp - u_ff;
umax_s = umax - force_comp - u_ff;
u_seed = min(max(u_prev_clip - force_comp - u_ff, umin_s), umax_s);

if solve_due
    mdl_s = mdl;
    mdl_s.u_min = umin_s;
    mdl_s.u_max = umax_s;
    opts = struct('fd', f_eff, 'APf', APf, 'BPf', BPf, 'df_rate', rt.rate, ...
                  'u_prev', u_seed, 'fallback', 'uss', 'fb_ffwd', ffwd, ...
                  'return_proj', true);
    opts.first_fd = f_eff + mdl.Bd(:,5:6)*force_error;
    [~, info] = tinympc14_admm(mdl_s, x, repmat(xref, 1, NH), zeros(NU, NN), p, rt.warm, opts);
    rt.solve_tick = tick;
    rt.iters(end+1) = info.iters;
    rt.conv(end+1) = info.converged;
    y.iters = info.iters;
    y.conv = info.converged;
    if all(isfinite(info.v(:))) && all(isfinite(info.g(:))) && ...
       all(isfinite(info.z(:))) && all(isfinite(info.y(:)))
        rt.warm = struct('v', info.v, 'g', info.g, 'z', info.z, 'y', info.y);
    else
        rt.warm = [];
    end
    if info.converged
        rt.seq = info.z(:, 1:NN);
        rt.seq_roll_rate = info.x(12,:);
        rt.seq_uff = u_ff;
        rt.seq_force_error = force_error;
        rt.base_tick = tick;
        rt.seq_valid = true;
        rt.node_prev = -1;
        rt.commit = rt.commit + 1;
    else
        rt.seq_valid = false;
    end
end

u_bare = [];
if rt.seq_valid
    idx = floor((tick - rt.base_tick + 1e-9) / rt.node_ticks);
    if idx >= 0 && idx < NN
        if idx <= rt.node_prev
            rt.node_repeat = rt.node_repeat + 1;
        end
        rt.node_prev = idx;
        u_bare = rt.seq(:, idx + 1)+rt.seq_uff-u_ff;
        innovation = force_error;
        if idx == 0 && isfield(rt,'seq_force_error')
            innovation(valid_force) = innovation(valid_force) - rt.seq_force_error(valid_force);
        end
        u_bare = u_bare - force_gain*innovation;
        y.node = idx;
    else
        rt.seq_valid = false;
        rt.stale = rt.stale + 1;
        y.stale = true;
    end
end

if isempty(u_bare)
    u_fb = u_ss + ffwd - mdl.Kinf * (x - xref) - sensed_delta;
    u_bare = u_fb - u_ff;
    rt.fallback = rt.fallback + 1;
    y.fallback = true;
end

tot_lo = max(umin, u_prev_clip - rt.output_rate);
tot_hi = min(umax, u_prev_clip + rt.output_rate);
% Slew and box constraints apply to total support after axial feedforward.
request = u_bare + u_ff + force_comp;
% Grounded roll feedback includes the impulse that the last accepted differential
% must deliver while returning to the static differential at this posture.
diff_gain = mdl.Kinf(5,:)-mdl.Kinf(6,:);
rate_gain = diff_gain(12);
diff_gain(12) = 0;
steady_diff = (u_ff(5)+force_comp(5))-(u_ff(6)+force_comp(6))-diff_gain*(x-xref);
steady_diff = min(max(steady_diff,umin(5)-umax(6)),umax(5)-umin(6));
excess = (rt.u_prev(5)-rt.u_prev(6))-steady_diff;
step = sum(rt.output_rate(5:6));
impulse = 0;
if all(valid_force) && isscalar(pending_diff_impulse) && isfinite(pending_diff_impulse)
    impulse = pending_diff_impulse;
end
if step>0 && abs(excess)>step
    n = floor(abs(excess)/step);
    impulse = impulse + sign(excess)*rt.control_dt*n*(abs(excess)-0.5*(n+1)*step);
end
roll_gain = (mdl.Bd(12,5)-mdl.Bd(12,6))/(2*p.Ts);
rate_error = roll_gain*impulse;
if y.node >= 0 && isfield(rt,'seq_roll_rate')
    % The committed force node uses its own predicted angular rate. Compare the
    % fresh observation to the same instant between the 2 ms prediction nodes.
    k = y.node+1;
    fraction = (tick-rt.base_tick)/rt.node_ticks-y.node;
    predicted = rt.seq_roll_rate(k)+fraction*(rt.seq_roll_rate(k+1)-rt.seq_roll_rate(k));
    rate_error = rate_error+x(12)-predicted;
end
% Keep the nominal differential damping when the scheduled rate gain falls.
extra_damping = max(0,rt.nominal_roll_damping-rate_gain);
request(5:6) = request(5:6)+[-0.5;0.5]*(rate_gain*rate_error+extra_damping*(x(12)-xref(12)));
u_tot = min(max(request, tot_lo), tot_hi);
% Grounded runtime counterpart to balance.c and the physical total-force allocator.
% At 0.08..0.14 rad use the same roll reserve as turn_limit.h (allow..stop).
tilt = max(abs([x(11), x(11) + NN*p.Ts*x(12)]));
lower_share = min(max((0.14-tilt)/(0.14-0.08), 0), 1);
if x(13) > xref(13) && lower_share > 0
    steady = sum(u_ff(5:6) + force_comp(5:6));
    common = steady - sum(sensed_delta(5:6)) - sum(mdl.Kinf(5:6,:),1)*(x-xref);
    original = sum(u_tot(5:6));
    if common < steady && common < original
        cap = max(common,sum(tot_lo(5:6)));
        holding_diff = u_ff(5) + force_comp(5) - u_ff(6) - force_comp(6) - ...
                       (mdl.Kinf(5,11)-mdl.Kinf(6,11))*(x(11)-xref(11));
        bounded_diff = u_tot(5)-u_tot(6);
        diff = request(5)-request(6);
        if bounded_diff*holding_diff > 0 && diff*holding_diff > 0
            reserve = min([abs(holding_diff),abs(bounded_diff),abs(diff)]);
            inner_min = tot_lo(5);
            if holding_diff > 0, inner_min = tot_lo(6); end
            cap = min(original,max(cap,2*inner_min+reserve));
        end
        left_min = max(tot_lo(5), cap-tot_hi(6));
        left_max = min(tot_hi(5), cap-tot_lo(6));
        conflict = max(2*left_min-cap-diff, diff-(2*left_max-cap));
        if conflict > 0 && cap < original
            total = original-lower_share*min(original-cap,conflict);
            u_tot(5) = min(max((total+diff)/2,max(tot_lo(5),total-tot_hi(6))),min(tot_hi(5),total-tot_lo(6)));
            u_tot(6) = total-u_tot(5);
        end
    end
end
u_mpc = u_tot - force_comp;
u_bare = u_mpc - u_ff;
rt.u_prev = u_tot;

y.u_tot = u_tot;
y.u_mpc = u_mpc;
y.u_bare = u_bare;
y.u_ff = u_ff;
y.u_ss = u_ss;
y.ffwd = ffwd;
y.fed = fed;
y.c = c;
y.MgF = c.MgF;  y.Fs = c.Fs;  y.Fc = c.Fc;
y.Tp = c.MgF + u_tot(3:4);
y.F_motor = u_tot(5:6) - c.Fs;
y.T_w = -u_tot(1:2);
y.u_plant = u_tot;
% B-family plant already uses gravity-compensated hip torque T_B.
y.u_motor = [u_tot(1:2); y.Tp; u_tot(5:6) - c.Fs];
end
