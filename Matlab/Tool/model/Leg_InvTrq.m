function [F,Tp] = Leg_InvTrq(tau1,tau4,phi1,phi4)
%#codegen
% 同一 001 闭环雅可比的正则逆，lambda=1e-6 保持既有近奇异抑制口径。
[~,J] = leg_chain(phi1,phi4);
A = J'; lambda = cast(1e-6,'like',phi1);
a = sum(A(:,1).^2)+lambda; b = A(:,1)'*A(:,2); d = sum(A(:,2).^2)+lambda;
rhs = A'*[tau1;tau4]; determinant = a*d-b*b;
F = (d*rhs(1)-b*rhs(2))/determinant;
Tp = (a*rhs(2)-b*rhs(1))/determinant;
end
