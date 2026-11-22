function [Kinf, Pinf, eig_cl] = tinympc14_dare_gain(Ad, Bd, Q, Rm, rho, NX, NU)
% rho 增强 DARE 四元组的口径：dlqr(Ad, Bd, Q + rho*I, R + rho*I)
try
    [Kinf, Pinf] = dlqr(Ad, Bd, Q + rho*eye(NX), Rm + rho*eye(NU));
catch ME
    error('需要 Control System Toolbox（dlqr）：%s', ME.message);
end
eig_cl = max(abs(eig(Ad - Bd*Kinf)));
end
