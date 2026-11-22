function [Ad,Bd,fd,P] = tinympc14_online_model(p, table, left, right, psi)
%#codegen
% Calibrated model and terminal weight at one firmware scheduling point.
[Ad,Bd,fd] = tinympc14_scheduled_zoh(p,left,right,psi);
P = tinympc14_terminal_eval(table,p.rho,left,right,psi);
end
