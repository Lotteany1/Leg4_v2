function summary = verify_mixmpc_urdf(urdf_file, wheel_radius)
% Exercise the single 14-state / 6-input controller on the reduced URDF model.
old_path = path;
cleanup = onCleanup(@() path(old_path)); %#ok<NASGU>
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
mixmpc_paths();
p = tinympc14_urdf_config(urdf_file, wheel_radius);
fprintf('URDF: body %.3f kg, leg %.3f kg, wheel %.3f kg, half-track %.4f m, leg range [%.4f %.4f] m\n', ...
    p.m_b,p.m_l,p.m_w,p.R_TRK,p.LEG_RANGE);
sw = p.sym7.sw_fused;
sched = tinympc14_sched_pt(p,p.H_REF,0,0);
[xe, ue, eq] = tinympc14_fused_eq(p,sw,sched);
assert(eq.res < 1e-8 && all(ue >= p.u_min(:) & ue <= p.u_max(:)), ...
    'No feasible nominal equilibrium.');
mdl = tinympc14_model_at(p,sched.lL,sched.lR,xe(5),xe(7),0,p.H_REF);
assert(max(abs(eig(mdl.Ad-mdl.Bd*mdl.Kinf))) < 1, 'Unstable nominal feedback.');

% A mean-angle table misses differential leg tilt, even when the mean is zero.
asym = tinympc14_model_at(p,sched.lL,sched.lR,0.16,-0.16,0,p.H_REF);
assert(norm(asym.Bd([12,14],1:2)-mdl.Bd([12,14],1:2),'fro') > 1e-4, ...
    'The predictor lost the differential wheel-to-height/roll coupling.');
assert(norm(asym.Ad(2:2:10,[11,13]),'fro') > 1e-4, ...
    'The predictor lost the height/roll-to-balance coupling.');

x = xe;
rt = tinympc14_runtime_init(p);
xref = xe;
steps = round(5/p.Ts);
for k = 1:steps
    time = (k-1)*p.Ts;
    if time >= 0.25
        xref(1) = xe(1) + 0.04;
        xref(3) = xe(3) + 0.05;
        xref(13) = xe(13) + 0.015;
        xref(11) = xe(11) + 0.025;
    end
    if k == round(0.75/p.Ts)+1
        x([10,12]) = x([10,12]) + [0.25;0.35];
    end
    sched = tinympc14_sched_pt(p,p.H_REF+x(13),x(11),mean(x([5,7])));
    assert(all([sched.lL,sched.lR] >= p.LEG_RANGE(1) & [sched.lL,sched.lR] <= p.LEG_RANGE(2)), ...
        'Closed loop exceeded the URDF knee workspace.');
    prev = rt.u_prev;
    [rt,y] = tinympc14_runtime_step(rt,1000*time,x,xref,sched.lL,sched.lR,x(5),x(7),x(2),x(4));
    u = y.u_plant;
    assert(all(u >= p.u_min(:)-1e-9 & u <= p.u_max(:)+1e-9), 'Input box violated.');
    assert(all(abs(u(5:6)-prev(5:6)) <= p.DF_RATE+1e-9), 'Support force slew violated.');
    op = tinympc14_fused_op(p,sched);
    a = tinympc14_fused_dyn(p,sw,op,x,u);
    b = tinympc14_fused_dyn(p,sw,op,x+p.Ts*a/2,u);
    c = tinympc14_fused_dyn(p,sw,op,x+p.Ts*b/2,u);
    d = tinympc14_fused_dyn(p,sw,op,x+p.Ts*c,u);
    x = x + p.Ts*(a+2*b+2*c+d)/6;
    assert(all(isfinite(x)), 'Nonfinite closed-loop state.');
end
summary = struct('position_error_m',x(1)-xref(1), 'yaw_error_rad',x(3)-xref(3), ...
    'height_error_m',x(13)-xref(13), 'roll_error_rad',x(11)-xref(11), ...
    'pitch_error_rad',x(9)-xref(9), 'solve_success_ratio',mean(rt.conv), ...
    'fallback_ratio',rt.fallback/steps,'model_updates',rt.lpv_updates);
disp(summary);
assert(abs(summary.position_error_m)<0.01 && abs(summary.yaw_error_rad)<0.01 ...
    && abs(summary.height_error_m)<0.005 && abs(summary.roll_error_rad)<0.01 ...
    && abs(summary.pitch_error_rad)<0.02 && summary.solve_success_ratio>0.9 ...
    && summary.model_updates>0,'URDF unified MPC closed-loop checks failed.');
fprintf('URDF unified MPC: PASS\n');
end
