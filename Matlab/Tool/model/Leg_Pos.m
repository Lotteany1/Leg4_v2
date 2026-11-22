function pos = Leg_Pos(phi1,phi4)
%#codegen
% 001 实际传动闭环正解。输入电机角 [rad]；输出腿长 [m] 和极角 [rad]。
[pos,~] = leg_chain(phi1,phi4);
end
