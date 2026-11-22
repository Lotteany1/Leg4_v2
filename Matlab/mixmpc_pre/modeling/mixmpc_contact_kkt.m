function [result, lambda, info] = mixmpc_contact_kkt(M, free, J, target, mu)
% Solve J*result = target with result = free + inv(M)*J'*lambda.
% For dual support at a redundant pose, search the one-dimensional force
% nullspace for a nonnegative reaction satisfying both Coulomb cones.
if isempty(J)
    result = free;
    lambda = zeros(0,1);
    info = struct('rank',0,'residual',0,'consistent',true,'admissible',true);
    return
end
R = chol(M);
[U,D,~] = svd(J/R,'econ');
s = diag(D);
r = sum(s>1e-9*s(1));
span = U(:,1:r);
lambda = span*((span'*(target-J*free))./s(1:r).^2);
tol = 1e-8*max(1,norm(lambda,Inf));
admissible = in_cones(lambda,mu,tol);
if ~admissible && size(J,1)-r==1
    z = U(:,end);
    candidates = [];
    for k = 1:numel(mu)
        ids = 3*(k-1)+(1:3);
        p = lambda(ids);
        d = z(ids);
        if abs(d(3))>1e-13
            candidates(end+1) = -p(3)/d(3); %#ok<AGROW>
        end
        % Squared cone boundary on the affine force line. Also test the
        % original cone because squaring alone would accept tensile forces.
        P = diag([1 1 -mu(k)^2]);
        t = roots([d'*P*d 2*p'*P*d p'*P*p]);
        candidates = [candidates real(t(abs(imag(t))<1e-9))']; %#ok<AGROW>
    end
    [~,order] = sort(abs(candidates));
    for u = candidates(order)
        test = lambda+u*z;
        if in_cones(test,mu,tol)
            lambda = test;
            admissible = true;
            break
        end
    end
end
result = free + R\(R'\(J'*lambda));
residual = norm(J*result-target,Inf);
info = struct('rank',r,'residual',residual, ...
    'consistent',residual<=1e-8*max(1,norm(target,Inf)), ...
    'admissible',admissible);
end

function inside = in_cones(lambda,mu,tol)
f = reshape(lambda,3,[]);
inside = all(f(3,:)>=-tol & vecnorm(f(1:2,:))<=mu'.*f(3,:)+tol);
end
