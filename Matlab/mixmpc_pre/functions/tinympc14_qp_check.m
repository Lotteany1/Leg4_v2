function tinympc14_qp_check()
% Independent condensed QP, including slew constraints and nonzero references.
p=tinympc14_config(); rng(31);
assert(exist('quadprog','file')~=0,'QP check requires Optimization Toolbox');
opt=optimoptions('quadprog','Display','off','OptimalityTolerance',1e-10,'ConstraintTolerance',1e-10);
% Stress the projection separately from the MPC recursion.
N=p.NH-1; D=diff(eye(N)); C=[D;-D]; worst=0;
for k=1:100
    w=300*randn(1,N); prev=20+130*rand; rate=6;
    lo=20*ones(N,1); hi=150*ones(N,1); lo(1)=max(lo(1),prev-rate); hi(1)=min(hi(1),prev+rate);
    z=tinympc14_project_input(w,20,150,rate,prev);
    [q,~,flag]=quadprog(eye(N),-w',C,rate*ones(2*(N-1),1),[],[],lo,hi,[],opt);
    assert(flag>0); worst=max(worst,max(abs(z(:)-q)));
end
assert(worst<1e-6); fprintf('PASS projection vs quadprog, 100 stress cases, max error %.3e\n',worst);
% Actual 14/6 LPV model. Condense the dynamics independently.
m=tinympc14_lpv_eval(p,.27,.27,0); n=p.NX; nu=p.NU;
S=zeros(n*p.NH,nu*N); T=zeros(n*p.NH,n); T(1:n,:)=eye(n);
for k=1:N
    r=k*n+(1:n); old=(k-1)*n+(1:n);
    T(r,:)=m.Ad*T(old,:); S(r,:)=m.Ad*S(old,:); S(r,(k-1)*nu+(1:nu))=m.Bd;
end
Pt=(m.Pinf+m.Pinf')/2-p.rho*eye(n);
Q=blkdiag(kron(eye(N),diag(p.Q_DIAG)),Pt); R=kron(eye(N),diag(p.R_DIAG));
H=S'*Q*S+R; H=(H+H')/2;
[xe,ue]=tinympc14_fused_eq(p,p.sym7.sw_fused,tinympc14_sched_pt(p,.27,0,0));
base=m.fd+m.Bd*ue; offset=zeros(n*p.NH,1);
for k=1:N, r=k*n+(1:n); offset(r)=m.Ad*offset((k-1)*n+(1:n))+base; end
boundsL=p.u_min(:)-ue; boundsH=p.u_max(:)-ue;
m.u_min=boundsL; m.u_max=boundsH;
rate=[1e6*ones(4,1);6;6]; Du=kron(eye(N)-diag(ones(N-1,1),-1),eye(nu));
Aineq=[Du;-Du]; dr=repmat(rate,2*N,1); cfg=p;
cfg.max_iter=10000; cfg.pri_tol=1e-7; cfg.dua_tol=1e-7;
worst=0;
for j=1:6
    x=xe; x(13)=x(13)+.02*(j-3); x(12)=.1*(j-3);
    xr=repmat(xe,1,p.NH); xr(13,:)=xr(13,:)+.01;
    fq=S'*Q*(T*x+offset-xr(:)); seed=zeros(nu,1);
    lb=repmat(boundsL,N,1); ub=repmat(boundsH,N,1);
    opts=struct('fd',base,'df_rate',rate,'u_prev',seed);
    [~,info]=tinympc14_admm(m,x,xr,zeros(nu,N),cfg,[],opts);
    scale=max(1,norm(H,Inf));
    [uq,~,flag]=quadprog(H/scale,fq/scale,Aineq,dr,[],[],lb,ub,[],opt);
    assert(flag>0 && info.converged,'MPC or independent QP did not converge');
    er=norm(info.z(:)-uq,Inf); worst=max(worst,er);
    if j==1
        % This upward height step also applies a roll-rate perturbation. Check
        % the rate-limited first axial command against the same independent
        % optimum within the deployed iteration budget and residual thresholds.
        [~,live]=tinympc14_admm(m,x,xr,zeros(nu,N),p,[],opts);
        assert(any(abs(abs(uq(5:6))-rate(5:6))<2e-4),'Expected an active axial slew limit');
        assert(live.converged && norm(live.z(:,1)-uq(1:nu),Inf)<2e-4, ...
            'Rate-limited axial MPC failed to reach the QP optimum within the deployment budget');
        fprintf('PASS active axial slew, %d/%d iterations, primal %.3e, dual %.3e\n', ...
            live.iters,p.max_iter,live.pri,live.dua);
    end
end
assert(worst<2e-4); fprintf('PASS 14/6 MPC vs independent constrained QP, 6 cases, max error %.3e\n',worst);
end
