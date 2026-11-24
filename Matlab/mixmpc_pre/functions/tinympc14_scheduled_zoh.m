function [Ad,Bd,fd,terminal_center] = tinympc14_scheduled_zoh(p, left, right, psi, roll_height)
%#codegen
% Runtime model entry with constant calibrated parameters and three signals.
coder.inline('never');
if nargin < 5, roll_height = NaN; end
[A,B,f,~,terminal_center] = tinympc14_scheduled_ab(p,left,right,psi,roll_height);
[Ad,Bd,fd] = tinympc14_zoh_embedded(A,B,f,p.Ts);
end
