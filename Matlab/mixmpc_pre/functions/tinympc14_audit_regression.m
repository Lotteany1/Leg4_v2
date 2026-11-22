function tinympc14_audit_regression()
% Independent invariants; historical regression scripts may share implementation defects.
p = tinympc14_config();
fail = 0;
s = tinympc14_sched_pt(p, .27, .03, .2);
[xe, ue] = tinympc14_fused_eq(p, p.sym7.sw_fused, s);
s.eq = struct('x_eq', xe, 'u_eq', ue);
[A,B,f,aux] = tinympc14_fused_ab(p,p.sym7.sw_fused,s);
op = tinympc14_fused_op(p,s);
r = norm(A*aux.x_ref+B*aux.u_ref+f-tinympc14_fused_dyn(p,p.sym7.sw_fused,op,aux.x_ref,aux.u_ref),Inf);
fail = check(fail,r < 1e-9,'affine linearization matches dynamics at its expansion point',r);
fail = check(fail,abs(op.H-s.H)<1e-12,'scheduled height is absolute, without a second H_REF',op.H-s.H);
% Compare against both wheel-center projections on a level floor. Each
% reconstructed center must have zero vertical coordinate in this frame.
geom_error = 0;
for roll = [-0.18,0,0.18]
    for psi = [-0.30,0,0.30]
        sp = tinympc14_sched_pt(p,.25,roll,psi);
        z_axles = sp.H + [1,-1]*p.R_TRK*sin(roll) ...
                  - [sp.lL,sp.lR]*cos(roll)*cos(psi);
        geom_error = max(geom_error,max(abs(z_axles)));
    end
end
fail = check(fail,geom_error<1e-12,'leg scheduling matches both projected wheel heights',geom_error);
coupling = norm(A(2:2:10,[11,13]),'fro');
fail = check(fail,coupling > 1e-3,'deployed predictor includes height and roll geometry coupling',coupling);
% Hold the leg lookup fixed to isolate the analytic force projections.
s.psi_l=.17; s.psi_r=-.23;
s.eq.u_eq=[1.1;-0.7;0.2;-0.4;83;70];
direction=sin((1:p.NX)');
direction([11,13])=0;
actuation=cos((1:p.NU)');
step=1e-6;
jac_error=0;
diagnostic=p.sym7.sw_fused;
diagnostic.F_pitch=1;
for model = [p.sym7.sw_none,p.sym7.sw_fused,diagnostic]
    [Aa,Ba,~,pt]=tinympc14_fused_ab(p,model,s);
    forward=tinympc14_fused_dyn(p,model,op,pt.x_ref+step*direction,pt.u_ref+step*actuation);
    backward=tinympc14_fused_dyn(p,model,op,pt.x_ref-step*direction,pt.u_ref-step*actuation);
    jac_error=max(jac_error,norm((forward-backward)/(2*step)-Aa*direction-Ba*actuation,Inf));
end
fail = check(fail,jac_error<1e-7,'analytic projection Jacobian matches nonlinear directional differences',jac_error);
% 独立对两轮平地约束作虚位移。用轮心、腿长的中心差分校核输入作功，
% 分别覆盖同向和差动轴力、非对称腿角与轮矩；不调用被测腿长雅可比。
work_error=0; gradient_error=0; bias_error=0;
for phi=[-.2,0,.16]
    g=tinympc14_sched_pt(p,.30,phi,0);
    g.psi_l=.30; g.psi_r=-.19;
    g.eq=struct('x_eq',zeros(14,1),'u_eq',[.7;-1.3;.5;-.4;82;60]);
    [Awork,Bwork,~,pt]=tinympc14_fused_ab(p,p.sym7.sw_fused,g);
    opwork=tinympc14_fused_op(p,g);
    cos_psi=cos([g.psi_l;g.psi_r]);
    sin_psi=sin([g.psi_l;g.psi_r]);
    for velocity=[1,0,-.31;0,1,.57]
        dh=step*velocity(1); da=step*velocity(2);
        forward=(g.H+dh+[1;-1]*p.R_TRK*sin(phi+da))./(cos(phi+da)*cos_psi);
        backward=(g.H-dh+[1;-1]*p.R_TRK*sin(phi-da))./(cos(phi-da)*cos_psi);
        dL=(forward-backward)/(2*step);
        expected=dL'*(pt.u_ref(5:6)+pt.u_ref(1:2).*sin_psi/p.R_w);
        power=velocity'*opwork.M_hr*Bwork([14,12],:)*pt.u_ref;
        work_error=max(work_error,abs(power-expected));
    end
    dir=sin((1:14)');
    ap=tinympc14_fused_dyn(p,p.sym7.sw_fused,opwork,pt.x_ref+step*dir,pt.u_ref);
    am=tinympc14_fused_dyn(p,p.sym7.sw_fused,opwork,pt.x_ref-step*dir,pt.u_ref);
    da=(ap-am)/(2*step);
    gradient_error=max(gradient_error,norm(da([14,12])-Awork([14,12],:)*dir,Inf));
    support=zeros(6,1);
    [jh,jr]=tinympc14_leg_jacobian(p.R_TRK,g.H,phi,[g.psi_l,g.psi_r]);
    [~,gravity]=tinympc14_hr_params(p,g.H);
    load=gravity(1)+opwork.K_eff*pt.x_ref(13);
    support(5:6)=[jh;jr]\[load;load*g.H*tan(phi)+gravity(2)];
    sustained=tinympc14_fused_dyn(p,p.sym7.sw_fused,opwork,pt.x_ref,support);
    bias_error=max(bias_error,norm(sustained([14,12])-[0;p.G_ROLL*phi],Inf));
end
fail=check(fail,work_error<1e-7,'closed-chain axial and wheel work for heave/roll virtual displacements',work_error);
fail=check(fail,gradient_error<1e-7,'tilted heave/roll dynamics include the full geometric force gradient',gradient_error);
fail=check(fail,bias_error<1e-11,'geometric moment of static support counted once with external tipping',bias_error);
% The projected potential gradient gives axial holding force. The sum of
% external contact normals includes the full wheel and leg weights.
[~, hardware] = leg_config();
upright = zeros(p.NX,1);
upright_op = tinympc14_fused_op(p,tinympc14_sched_pt(p,p.H_REF,0,0));
support = [zeros(4,1); [1,1;p.R_TRK,-p.R_TRK]\upright_op.G_hr];
xd = tinympc14_fused_dyn(p,p.sym7.sw_fused,upright_op,upright,support);
fail = check(fail,abs(xd(14))<1e-10 && abs(tinympc14_m_tot(p)-hardware.m_total)<1e-10, ...
    'static axial force balances projected gravity, preserving weighed vehicle mass',xd(14));
% Adding 1 N to each leg raises the body. Adding +1/-1 N to left/right
% produces positive roll with zero heave in the symmetric upright model.
sum_input = support; sum_input(5:6) = sum_input(5:6)+1;
dif_input = support; dif_input(5:6) = dif_input(5:6)+[1;-1];
sum_xd = tinympc14_fused_dyn(p,p.sym7.sw_fused,upright_op,upright,sum_input);
dif_xd = tinympc14_fused_dyn(p,p.sym7.sw_fused,upright_op,upright,dif_input);
expected=upright_op.M_hr\[2,0;0,2*p.R_TRK];
gain_error = norm([sum_xd([14,12]),dif_xd([14,12])]-expected,Inf);
fail = check(fail,gain_error<1e-10 && p.m_b<p.M_V && p.M_V<p.M_LOAD && p.M_LOAD<p.M_SUP, ...
    'axial force sum/difference and projected heave inertia',gain_error);
sw = p.sym7.sw_none;
x = zeros(14,1); x(5)=.25; x(7)=-.1; u=[zeros(4,1);120;100];
xd=tinympc14_fused_dyn(p,sw,op,x,u);
[mass,gravity]=tinympc14_hr_params(p,p.H_REF);
expected=mass\[sum(u(5:6))-gravity(1);p.R_TRK*(u(5)-u(6))-gravity(2)];
r=norm(xd([14,12])-expected,Inf);
fail=check(fail,r<1e-10,'projection switch applies to both roll and height',r);

% Internal axial support cannot appear as an extra external horizontal force.
% At upright, H/length acceleration terms in horizontal momentum are second order.
s0=tinympc14_sched_pt(p,.27,0,0);
[A0,B0,~,ax0]=tinympc14_fused_ab(p,p.sym7.sw_fused,s0);
c=zeros(1,14);
c(2)=-(p.m_b+2*p.m_l+2*p.m_w+2*p.I_w/p.R_w^2);
c(6)=-(p.m_l*ax0.l_wl+p.m_b*s0.lL/2);
c(8)=-(p.m_l*ax0.l_wr+p.m_b*s0.lR/2);
% Above momentum coefficients are expressed in wheel-center velocity.
% Substitute v_wheel=v_hip-(lL*dpsiL+lR*dpsiR)/2 before evaluating
% a model whose state 2 is the measured mean hip velocity.
c(6)=c(6)-c(2)*s0.lL/2;
c(8)=c(8)-c(2)*s0.lR/2;
momentum=max(norm(c*A0,Inf),norm(c*B0+[1 1 0 0 0 0]/p.R_w,Inf));
fail=check(fail,momentum<1e-6,'upright horizontal momentum after eliminating internal leg forces',momentum);

% Scalar QP, condensed independently: sum Q*(x-r)^2 + R*u^2,
% terminal (Pinf-rho)*(x-r)^2. No input or state constraints active.
c=struct('NH',8,'rho',.2,'max_iter',10000,'pri_tol',1e-10,'dua_tol',1e-10);
a=.95; b=.3; q=2; rr=1;
[k,pp]=dlqr(a,b,q+c.rho,rr+c.rho);
m=struct('Ad',a,'Bd',b,'Kinf',k,'Pinf',pp,'QuuInv',1/(rr+c.rho+b*b*pp), ...
 'AmBKt',a-b*k,'q_diag',q,'r_diag',rr,'u_min',-1e6,'u_max',1e6,'x_min',-1e6,'x_max',1e6);
N=c.NH-1; S=zeros(c.NH,N); t=zeros(c.NH,1); t(1)=.4;
for j=1:N, S(j+1,:)=a*S(j,:); S(j+1,j)=b; t(j+1)=a*t(j); end
W=diag([repmat(q,1,N),pp-c.rho]); ref=ones(c.NH,1)*.8;
U=(S'*W*S+rr*eye(N))\(S'*W*(ref-t));
[got,info]=tinympc14_admm(m,.4,ref',zeros(1,N),c,[],struct('fallback','ref'));
fail=check(fail,info.converged && abs(got-U(1))<1e-7,'ADMM agrees with independent condensed QP for nonzero reference',got-U(1));

% Steady tracking with affine dynamics must require zero correction.
f0=(1-a)*.8;
opts=struct('fd',f0,'APf',(a-b*k)*pp*f0,'BPf',b*pp*f0,'fallback','ref');
[got,info]=tinympc14_admm(m,.8,ref',zeros(1,N),c,[],opts);
fail=check(fail,info.converged && abs(got)<1e-7,'nonzero equilibrium has zero optimized correction',got);
if exist('tinympc14_project_input','file')
 z=tinympc14_project_input([10 -10],-100,100,1,0);
 fail=check(fail,max(abs(z-[.5 -.5]))<1e-8,'rate projection is the Euclidean projection',[z]);
end
c=tinympc14_output(p,.27,.27,0,0,.5,.4);
first_moment=tinympc14_table_row(p.TURN_DATA,.27);
roll_moment=p.R_TRK*(c.Fc(1)-c.Fc(2))+first_moment*.5*.4;
fail=check(fail,abs(roll_moment)<1e-5 && all(c.ffwd==0),'external centripetal couple uses the full measured track',roll_moment);
rt=tinympc14_runtime_init(p);
[xr,~]=tinympc14_fused_eq(p,p.sym7.sw_fused,tinympc14_sched_pt(p,.27,0,0));
[~,yy]=tinympc14_runtime_step(rt,0,xr,xr,.27,.27,0,0,.5,.4);
fail=check(fail,all(yy.ffwd==0) && norm(yy.u_tot(5:6)-yy.u_mpc(5:6)-yy.Fc,Inf)<1e-9, ...
    'centripetal compensation is applied after MPC',yy.u_tot(5:6)-yy.u_mpc(5:6));
fprintf('AUDIT regression failures: %d\n',fail);
assert(fail==0,'Leg4 audit regressions failed');
end
function fail=check(fail,ok,name,val)
fprintf('%s %s: %s\n',string_if(ok),name,mat2str(val,6));
fail=fail+~ok;
end
function s=string_if(ok)
if ok, s='PASS'; else, s='FAIL'; end
end
