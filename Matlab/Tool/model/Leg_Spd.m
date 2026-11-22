function spd = Leg_Spd(phi1,phi4,phi_dot_1,phi_dot_4)
%#codegen
% 与 001 闭环正解相同的解析雅可比，输出 [腿长速度;极角速度]。
[~,J] = leg_chain(phi1,phi4);
spd = J*[phi_dot_1;phi_dot_4];
end
