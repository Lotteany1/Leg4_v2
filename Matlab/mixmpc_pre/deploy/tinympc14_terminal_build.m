function [table,model,report] = tinympc14_terminal_build(p,model)
% Cache only Q/R-independent dynamics; every build solves DARE afresh.
length_grid=linspace(.16,.34,19);
angle_scale=.0001;
angle_axis=linspace(-asinh(.32/angle_scale),asinh(.32/angle_scale),33);
[left,right,angle]=ndgrid(length_grid,length_grid,angle_scale*sinh(angle_axis));
count=numel(left);
phase_clock=tic;
if nargin<2
    model=struct('length_grid',length_grid,'angle_axis',angle_axis, ...
        'angle_scale',angle_scale,'Ad',zeros(14,14,count),'Bd',zeros(14,6,count));
    centers=zeros(numel(length_grid));
    for i=1:numel(length_grid)
        for j=1:numel(length_grid)
            [~,~,~,~,centers(i,j)]=tinympc14_scheduled_ab(p,length_grid(i),length_grid(j),0);
        end
    end
    assert(max(abs(centers),[],'all')<.02,'Equilibrium lies outside terminal table coverage');
    angle=angle+repmat(centers,1,1,numel(angle_axis));
    for k=1:count
        [A,B,f,residual]=tinympc14_scheduled_ab(p,left(k),right(k),angle(k));
        assert(residual<1e-10,'Terminal grid equilibrium failed');
        [model.Ad(:,:,k),model.Bd(:,:,k)]=tinympc14_zoh_ab(A,B,f,p.Ts);
    end
else
    assert(isequal(model.length_grid,length_grid) && isequal(model.angle_axis,angle_axis) ...
        && model.angle_scale==angle_scale,'Terminal grid changed; run full build');
end
fprintf('Terminal dynamics: %d cached grid points\n',count);
fprintf('ROTATION_PHASE grid_dynamics %.9f reused=%d\n',toc(phase_clock),nargin>=2);
lower=tril(true(14)); diagonal=eye(14)>0;
factors=zeros(count,nnz(lower)+14*6);
phase_clock=tic;[gains,costs]=batch_dare(model.Ad,model.Bd,p);
fprintf('ROTATION_PHASE batch_dare %.9f\n',toc(phase_clock));phase_clock=tic;
for k=1:count
    K=gains(:,:,k); P=costs(:,:,k);
    L=chol((P+P')/2-p.rho*eye(14),'lower');
    L(diagonal)=log(L(diagonal)); factors(k,:)=[L(lower)',K(:)'];
end
offset=mean(factors,1); scale=max(max(abs(factors-offset),[],1),1e-12);
[U,S,V]=svd((factors-offset)./scale,'econ','vector');
fprintf('ROTATION_PHASE terminal_compress %.9f\n',toc(phase_clock));phase_clock=tic;
% Compression must satisfy the same 3% accuracy and stability gates, including
% after float rounding. Increase representation rank, never the tolerance.
reference=verification_reference(p);
fprintf('ROTATION_PHASE terminal_independent_reference %.9f\n',toc(phase_clock));phase_clock=tic;
for rank=[24,32,48,64,96,189]
    coords=U(:,1:rank).*S(1:rank)';
    step=max(max(abs(coords),[],1)/32767,1e-20);
    table=struct('length_grid',length_grid,'angle_axis',angle_axis,'angle_scale',angle_scale, ...
        'coords',int16(round(coords./step)), ...
        'basis',single((V(:,1:rank)'.*scale).*step'),'offset',single(offset));
    report=verify_table(p,table,reference);
    fprintf('Terminal rank %d: P error %.6g, K error %.6g, min eig %.6g, radius %.9f; worst K at %s\n', ...
        rank,report.max_terminal_error,report.max_gain_error,report.min_terminal_eig,report.max_radius,mat2str(report.worst_gain_point,6));
    if report.max_terminal_error<.03 && report.max_gain_error<.03 ...
            && report.min_terminal_eig>0 && report.max_radius<1
        report.rank=rank;
        report.table_bytes=2*numel(table.coords)+4*(numel(table.basis)+numel(table.offset));
        fprintf('ROTATION_PHASE terminal_fit_verify %.9f\n',toc(phase_clock));
        return;
    end
end
error('Terminal interpolation failed the unchanged accuracy/stability gates');
end

function reference=verification_reference(p)
stream=RandStream('mt19937ar','Seed',11);
samples=[.16+.18*rand(stream,400,2),-.3+.6*rand(stream,400,1)];
[a,b,c]=ndgrid(.16+.18*[0,.03,.5,.97,1],.16+.18*[0,.03,.5,.97,1],[-.3,-.18,0,.18,.3]);
samples=[samples;a(:),b(:),c(:)];
reference=struct('samples',samples,'Ad',[],'Bd',[],'P',[],'K',[],'zoh',zeros(1,3));
for k=1:size(samples,1)
    x=samples(k,:);
    [A,B,f]=tinympc14_scheduled_ab(p,x(1),x(2),x(3));
    [Ad,Bd,fd]=tinympc14_zoh_ab(A,B,f,p.Ts);
    [K,P]=tinympc14_dare_gain(Ad,Bd,diag(p.Q_DIAG),diag(p.R_DIAG),p.rho,14,6);
    [Ae,Be,fe]=tinympc14_scheduled_zoh(p,x(1),x(2),x(3));
    reference.zoh=max(reference.zoh,[norm(Ae-Ad,Inf),norm(Be-Bd,Inf),norm(fe-fd,Inf)]);
    reference.Ad(:,:,k)=Ad; reference.Bd(:,:,k)=Bd;
    reference.P(:,:,k)=P; reference.K(:,:,k)=K;
end
end

function report=verify_table(p,table,reference)
samples=reference.samples;
report=struct('samples',size(samples,1),'max_terminal_error',0,'max_gain_error',0, ...
    'min_terminal_eig',Inf,'max_radius',0,'max_zoh_error',zeros(1,3));
report.max_zoh_error=reference.zoh;
for k=1:size(samples,1)
    x=samples(k,:); Ad=reference.Ad(:,:,k); Bd=reference.Bd(:,:,k);
    P=reference.P(:,:,k); K=reference.K(:,:,k);
    [~,~,~,Ke,Pt]=tinympc14_lpv_embedded(p,table,x(1),x(2),x(3));
    report.max_terminal_error=max(report.max_terminal_error,norm(Pt-P,'fro')/norm(P,'fro'));
    gain_error=norm(Ke-K,'fro')/norm(K,'fro');
    if gain_error>=report.max_gain_error, report.worst_gain_point=x; end
    report.max_gain_error=max(report.max_gain_error,gain_error);
    report.min_terminal_eig=min(report.min_terminal_eig,min(eig((Pt+Pt')/2-p.rho*eye(14))));
    report.max_radius=max(report.max_radius,max(abs(eig(Ad-Bd*Ke))));
end
assert(max(report.max_zoh_error)<1e-8,'Embedded ZOH accuracy gate failed');
end

function [K,P]=batch_dare(A,B,p)
% Structure-preserving doubling of independent DAREs using page operations.
% Avoid thousands of toolbox dispatches; the defining residual is checked on
% every grid point. Off-grid validation still uses independent dlqr references.
n=p.NX; m=p.NU; count=size(A,3);
I=repmat(eye(n),1,1,count);
Q=diag(p.Q_DIAG)+p.rho*eye(n); R=diag(p.R_DIAG)+p.rho*eye(m);
G=pagemtimes(B,pagemldivide(R,pagetranspose(B)));
H=repmat(Q,1,1,count); Ak=A;
for iteration=1:100
    W=I+pagemtimes(G,H);
    Ai=pagemldivide(W,Ak);
    Hn=H+pagemtimes(pagemtimes(pagetranspose(Ak),H),Ai);
    Gn=G+pagemtimes(Ak,pagemldivide(W,pagemtimes(G,pagetranspose(Ak))));
    An=pagemtimes(Ak,Ai);
    Hn=(Hn+pagetranspose(Hn))/2;
    relative=max(abs(Hn-H),[],[1 2])./max(1,max(abs(Hn),[],[1 2]));
    H=Hn; G=(Gn+pagetranspose(Gn))/2; Ak=An;
    if max(relative,[],'all')<1e-13, break; end
end
assert(iteration<100,'Batch DARE failed to converge');
P=H;
BtP=pagemtimes(pagetranspose(B),P);
K=pagemldivide(R+pagemtimes(BtP,B),pagemtimes(BtP,A));
res=P-(Q+pagemtimes(pagemtimes(pagetranspose(A),P),A-pagemtimes(B,K)));
relative=max(abs(res),[],[1 2])./max(1,max(abs(P),[],[1 2]));
assert(all(isfinite(P),'all') && max(relative,[],'all')<1e-9,'Batch DARE residual failed');
fprintf('Batch DARE: %d points, %d doubling steps, max relative residual %.3g\n', ...
    count,iteration,max(relative,[],'all'));
end
