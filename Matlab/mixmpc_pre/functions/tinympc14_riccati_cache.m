function rc = tinympc14_riccati_cache(mdl, cfg, input_penalty)
% Finite-horizon augmented Riccati factors: fitted tables do not solve DARE exactly.
n=numel(mdl.q_diag); nu=numel(mdl.r_diag); N=cfg.NH-1;
if nargin<3, input_penalty=tinympc14_input_penalty(mdl,cfg); end
A=mdl.Ad; B=mdl.Bd; P=(mdl.Pinf+mdl.Pinf')/2;
Q=diag(mdl.q_diag)+cfg.rho*eye(n); R=diag(mdl.r_diag)+diag(input_penalty);
rc.K=zeros(nu,n,N); rc.Qu=zeros(nu,nu,N); rc.At=zeros(n,n,N); rc.Pnext=zeros(n,n,N);
for k=N:-1:1
    rc.Pnext(:,:,k)=P;
    H=R+B'*P*B;
    rc.Qu(:,:,k)=H\eye(nu);
    K=H\(B'*P*A); rc.K(:,:,k)=K;
    rc.At(:,:,k)=(A-B*K)';
    P=Q+K'*R*K+(A-B*K)'*P*(A-B*K);
    P=(P+P')/2;
end
end
