% Local dynamics, shared hardware, off-grid LPV and independent condensed QP.
clear; clc
here=fileparts(mfilename('fullpath')); addpath(here); addpath(fileparts(here)); mixmpc_paths();
p=tinympc14_config(); tinympc14_audit_regression();
assert(p.R_w==.060 && abs(p.m_l-1.306200443680559)<1e-12 && p.R_l==.2072 && p.R_TRK==p.R_l);
assert(p.M_LEG==p.m_l && p.COMP.LEG_GRAV_FF==p.m_l && p.COMP.ROLL_TRACK==2*p.R_TRK);
fprintf('PASS approved 777 hardware model parameters\n');
rng(11); err=zeros(1,5); emax=0; min_eig=Inf; bad_terminal=0;
for k=1:200
    lL=.16+.18*rand; lR=.16+.18*rand; psi=-.3+.6*rand;
    % Resolve the two projections independently as the estimator does.
    projections=[lL,lR]*cos(psi);
    al=atan2(projections(1)-projections(2),2*p.R_TRK);
    H=mean(projections)*cos(al);
    s=struct('H',H,'alpha',al,'lL',lL,'lR',lR,'psi',psi);
    [xe,ue,ei]=tinympc14_fused_eq(p,p.sym7.sw_fused,s);
    assert(ei.res<1e-10,'Nonlinear equilibrium did not converge');
    s.eq=struct('x_eq',xe,'u_eq',ue);
    [A,B,f,ax]=tinympc14_fused_ab(p,p.sym7.sw_fused,s);
    op=tinympc14_fused_op(p,s);
    assert(norm(A*ax.x_ref+B*ax.u_ref+f-tinympc14_fused_dyn(p,p.sym7.sw_fused,op,ax.x_ref,ax.u_ref),Inf)<1e-10);
    [Ad,Bd,fd]=tinympc14_zoh_ab(A,B,f,p.Ts);
    [K,P]=tinympc14_dare_gain(Ad,Bd,diag(p.Q_DIAG),diag(p.R_DIAG),p.rho,p.NX,p.NU);
    m=tinympc14_lpv_eval(p,lL,lR,psi);
    truth={Ad,Bd,fd,K,P}; fit={m.Ad,m.Bd,m.fd,m.Kinf,m.Pinf};
    for j=1:5
        err(j)=max(err(j),norm(fit{j}-truth{j},'fro')/max(norm(truth{j},'fro'),1e-8));
    end
    emax=max(emax,max(abs(eig(m.Ad-m.Bd*m.Kinf))));
    terminal_eig=min(eig((m.Pinf+m.Pinf')/2-p.rho*eye(p.NX)));
    min_eig=min(min_eig,terminal_eig);
    bad_terminal=bad_terminal+(terminal_eig<=0);
end
fprintf('OFFGRID 200: rel[Ad Bd fd K P]=%s; closed-loop radius %.9f\n',mat2str(err,5),emax);
fprintf('OFFGRID terminal cost: %d nonpositive samples, min eigenvalue %.3e\n',bad_terminal,min_eig);
assert(bad_terminal==0,'LPV terminal cost is not positive definite off-grid');
assert(err(1)<.005 && err(2)<.02 && err(3)<.005 && err(4)<.03 && err(5)<.03 && emax<1,'LPV approximation gate failed');
fprintf('S9 fused verification: PASS\n');
