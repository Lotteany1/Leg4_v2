function [Ad,Bd,fd,P] = tinympc14_online_model(p, table, left, right, psi, roll_height)
%#codegen
% Calibrated model and terminal weight at one firmware scheduling point.
if nargin < 6, roll_height = NaN; end
[Ad,Bd,fd] = tinympc14_scheduled_zoh(p,left,right,psi,roll_height);
P = tinympc14_terminal_eval(table,p.rho,left,right,psi);
end
