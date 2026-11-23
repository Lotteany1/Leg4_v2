% Three closed loops against the nonlinear projection kernel (RK4).
% This validates the reduced model, not a complete flexible multibody plant.
clear; clc
here=fileparts(mfilename('fullpath')); addpath(here); addpath(fileparts(here)); mixmpc_paths();
p=tinympc14_config();
[xe,~]=tinympc14_fused_eq(p,p.sym7.sw_fused,tinympc14_sched_pt(p,.27,0,0));
for sc=1:3
    x=xe; rt=tinympc14_runtime_init(p); maxroll=0; lmin=Inf; lmax=-Inf;
    for k=1:round(3/p.Ts)
        time=(k-1)*p.Ts; xr=xe;
        if sc==2 && time>=.5, xr(13)=xr(13)+.05; end
        if sc==3 && abs(time-1)<p.Ts/2, x(12)=x(12)+.5; end
        sp=tinympc14_sched_pt(p,p.H_REF+x(13),x(11),.5*(x(5)+x(7)));
        lmin=min(lmin,min([sp.lL sp.lR])); lmax=max(lmax,max([sp.lL sp.lR]));
        [rt,y]=tinympc14_runtime_step(rt,2*(k-1),x,xr,sp.lL,sp.lR,x(5),x(7),x(2),x(4));
        u=y.u_plant;
        a=dyn(p,x,u); b=dyn(p,x+p.Ts*a/2,u); c=dyn(p,x+p.Ts*b/2,u); d=dyn(p,x+p.Ts*c,u);
        x=x+p.Ts*(a+2*b+2*c+d)/6;
        assert(all(isfinite(x)),'Nonlinear plant diverged'); maxroll=max(maxroll,abs(x(11)));
    end
    fprintf('NONLINEAR case %d: terminal dH %.4e m, roll peak %.4e rad, legs [%.4f,%.4f] m, fallback %.2f%%\n', ...
        sc,x(13)-xr(13),maxroll,lmin,lmax,100*rt.fallback/round(3/p.Ts));
    assert(abs(x(13)-xr(13))<.01 && maxroll<.1 && lmin>=p.LEG_RANGE(1) && lmax<=p.LEG_STROKE_MAX);
end
fprintf('S10 reduced nonlinear closed loops: PASS\n');
function xd=dyn(p,x,u)
s=tinympc14_sched_pt(p,p.H_REF+x(13),x(11),.5*(x(5)+x(7)));
u(5:6)=u(5:6)+tinympc14_roll_force(p,p.H_REF+x(13),x(11),x(11),x([5,7]));
op=tinympc14_fused_op(p,s); xd=tinympc14_fused_dyn(p,p.sym7.sw_fused,op,x,u);
end
