function [report, table] = verify_mixmpc_terminal_grid(p)
% Verify online model assembly and a compressed positive-definite terminal table.
old_path = path;
cleanup = onCleanup(@() path(old_path)); %#ok<NASGU>
addpath(fileparts(fileparts(mfilename('fullpath'))));
mixmpc_paths();
if nargin < 1, p = tinympc14_config(); end
limits = [.16,.34];
if isfield(p,'LEG_RANGE'), limits = p.LEG_RANGE; end
length_grid = linspace(limits(1),limits(2),19);
angle_grid = -0.30:0.025:0.30;
rank = 24;
n = p.NX;
lower = tril(true(n));
diagonal = eye(n)>0;
[left,right,angle] = ndgrid(length_grid,length_grid,angle_grid);
factors = zeros(numel(left),nnz(lower));
for i = 1:numel(left)
    [~,~,~,~,P] = exact_model(p,left(i),right(i),angle(i));
    L = chol((P+P')/2-p.rho*eye(n),'lower');
    L(diagonal) = log(L(diagonal));
    factors(i,:) = L(lower)';
end

% Equalize the components before truncation, then store signed 16-bit
% coordinates and a single-precision reconstruction basis and offset.
offset = mean(factors,1);
scale = max(max(abs(factors-offset),[],1),1e-12);
[U,S,V] = svd((factors-offset)./scale,'econ','vector');
coords = U(:,1:rank).*S(1:rank)';
step = max(abs(coords),[],1)/32767;
table = struct('length_grid',length_grid,'angle_grid',angle_grid, ...
    'coords',int16(round(coords./step)), ...
    'basis',single((V(:,1:rank)'.*scale).*step'), ...
    'offset',single(offset));

% Off-grid random samples, plus lengths at and between the grid boundaries.
stream = RandStream('mt19937ar','Seed',11);
samples = [limits(1)+diff(limits)*rand(stream,400,2), -0.30+0.60*rand(stream,400,1)];
probe = limits(1)+diff(limits)*[0,.03,.5,.97,1];
[a,b,c] = ndgrid(probe,probe,[-.3,-.18,0,.18,.3]);
samples = [samples; a(:),b(:),c(:)];
R = diag(p.R_DIAG)+p.rho*eye(p.NU);
report = struct('table_bytes',2*numel(table.coords)+4*(numel(table.basis)+numel(table.offset)), ...
    'samples',size(samples,1),'min_terminal_eig',Inf,'max_radius',0, ...
    'max_terminal_error',0,'max_fallback_gain_error',0,'max_zoh_error',zeros(1,3));
for k = 1:size(samples,1)
    sample = samples(k,:);
    [Ad,Bd,fd,K,P] = exact_model(p,sample(1),sample(2),sample(3));
    [Ae,Be,fe] = tinympc14_scheduled_zoh(p,sample(1),sample(2),sample(3));
    Pt = double(single(tinympc14_terminal_eval(table,p.rho,sample(1),sample(2),sample(3))));
    Ke = (R+Be'*Pt*Be)\(Be'*Pt*Ae);
    report.min_terminal_eig = min(report.min_terminal_eig,min(eig(Pt-p.rho*eye(n))));
    report.max_radius = max(report.max_radius,max(abs(eig(Ad-Bd*Ke))));
    report.max_terminal_error = max(report.max_terminal_error,norm(Pt-P,'fro')/norm(P,'fro'));
    report.max_fallback_gain_error = max(report.max_fallback_gain_error,norm(Ke-K,'fro')/norm(K,'fro'));
    report.max_zoh_error = max(report.max_zoh_error, ...
        [norm(Ae-Ad,Inf),norm(Be-Bd,Inf),norm(fe-fd,Inf)]);
end
disp(report);
assert(report.min_terminal_eig>0 && report.max_radius<1, ...
    'Terminal reconstruction or fallback stability failed.');
assert(report.max_terminal_error<.03 && report.max_fallback_gain_error<.03 ...
    && max(report.max_zoh_error)<1e-8, 'Embedded model approximation exceeded its error gates.');
fprintf('Compressed terminal grid and embedded ZOH: PASS\n');
end

function [Ad,Bd,fd,K,P] = exact_model(p,left,right,psi)
[A,B,f,residual] = tinympc14_scheduled_ab(p,left,right,psi);
assert(residual < 1e-10,'Nominal balance calculation drifted.');
[Ad,Bd,fd] = tinympc14_zoh_ab(A,B,f,p.Ts);
[K,P] = tinympc14_dare_gain(Ad,Bd,diag(p.Q_DIAG),diag(p.R_DIAG),p.rho,p.NX,p.NU);
end
