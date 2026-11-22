function tinympc14_online_runtime_check(p)
% Exercise the firmware's 1/2/4 ms periods on the nonlinear rolling model.
control_ms = 1;
dt = control_ms/1000;
sw = p.sym7.sw_fused;
nominal = tinympc14_sched_pt(p,p.H_REF,0,0);
xe = tinympc14_fused_eq(p,sw,nominal);
x = xe;
xr = xe;
rt = tinympc14_runtime_init(p,control_ms);
peak_slew = 0;
for tick = 0:control_ms:4999
    % Drive, then brake while lifting and commanding a small lateral lean.
    if tick >= 500 && tick < 1500
        xr(1) = xe(1)+0.00018*(tick-500);
        xr(2) = 0.18;
    elseif tick >= 1500
        xr(1) = xe(1)+0.18;
        xr(2) = 0;
        xr(11) = xe(11)+0.02;
        xr(13) = xe(13)+0.035;
    end
    if tick == 2200
        x([10,12]) = x([10,12])+[0.35;0.5];
    end
    sched = tinympc14_sched_pt(p,p.H_REF+x(13),x(11),mean(x([5,7])));
    assert(all([sched.lL,sched.lR]>=.16 & [sched.lL,sched.lR]<=.34), ...
        'Closed-loop leg geometry left the supported range.');
    prev = rt.u_prev;
    [rt,y] = tinympc14_runtime_step(rt,tick,x,xr,sched.lL,sched.lR,x(5),x(7),x(2),x(4));
    u = y.u_plant;
    assert(all(u>=p.u_min(:)-1e-9 & u<=p.u_max(:)+1e-9),'Input outside the box.');
    peak_slew = max(peak_slew,max(abs(u(5:6)-prev(5:6))));
    op = tinympc14_fused_op(p,sched);
    a = tinympc14_fused_dyn(p,sw,op,x,u);
    b = tinympc14_fused_dyn(p,sw,op,x+dt*a/2,u);
    c = tinympc14_fused_dyn(p,sw,op,x+dt*b/2,u);
    d = tinympc14_fused_dyn(p,sw,op,x+dt*c,u);
    x = x + dt*(a+2*b+2*c+d)/6;
end
err = x([1,9,11,13])-xr([1,9,11,13]);
fprintf(['ONLINE runtime: position %.4g m, pitch %.4g rad, roll %.4g rad, height %.4g m, ' ...
    'slew %.3f N/ms, convergence %.2f%%, LPV updates %d\n'],err,peak_slew,100*mean(rt.conv),rt.lpv_updates);
assert(all(abs(err)<[.025;.02;.012;.006]) && peak_slew <= p.DF_RATE/2+1e-8 ...
    && mean(rt.conv)>.95 && rt.node_repeat>0 && rt.stale==0,'Online runtime regression failed.');
end
