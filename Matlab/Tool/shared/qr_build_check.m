function qr_build_check(A, B, Q, R, K, P)
%QR_BUILD_CHECK Weight validity and scale-normalized discrete Riccati checks.
assert(isequal(size(Q), [size(A,1), size(A,1)]) && ...
    isequal(size(R), [size(B,2), size(B,2)]), 'Q/R dimensions do not match model');
assert(all(isfinite([Q(:); R(:)])), 'Q/R must be finite');
assert(norm(Q-Q','fro') <= 1e-12*max(1,norm(Q,'fro')) && ...
    norm(R-R','fro') <= 1e-12*max(1,norm(R,'fro')), 'Q/R must be symmetric');
assert(min(eig((Q+Q')/2)) >= 0 && min(eig((R+R')/2)) > 0, ...
    'Q must be positive semidefinite and R positive definite');
if nargin < 5, return; end
assert(all(isfinite([A(:); B(:); K(:); P(:)])), 'Nonfinite model/gain');
assert(min(eig((P+P')/2)) > 0, 'Riccati P is not positive definite');
H = R+B'*P*B;
assert(min(eig((H+H')/2)) > 0, 'Riccati input Hessian is not positive definite');
residual = P-(Q+A'*P*A-A'*P*B*(H\(B'*P*A)));
assert(norm(residual,'fro')/max(1,norm(P,'fro')) <= 1e-7, 'DARE residual exceeds 1e-7');
assert(max(abs(eig(A-B*K))) < 1, 'Discrete closed loop is unstable');
end
