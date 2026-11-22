function T = Leg_Trq(F,Tp,phi1,phi4)
%#codegen
% 001 闭环虚功映射：轴力 [N]、极角力矩 [N m] 到电机力矩 [N m]。
[~,J] = leg_chain(phi1,phi4);
T = J'*[F;Tp];
end
