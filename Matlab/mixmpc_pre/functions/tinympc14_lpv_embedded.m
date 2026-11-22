function [Ad,Bd,fd,K,P,Qu,At] = tinympc14_lpv_embedded(p,table,left,right,psi,rdiag)
%#codegen
% Shared MATLAB/C scheduler. Round the model before deriving its auxiliaries,
% matching the float matrices consumed by the deployed Riccati recursion.
if nargin<6,rdiag=p.R_DIAG;end
[Ad,Bd,fd,center] = tinympc14_scheduled_zoh(p,left,right,psi);
Ad=double(single(Ad)); Bd=double(single(Bd)); fd=double(single(fd));
[P,K]=tinympc14_terminal_cubic_eval(table,p.rho,left,right,psi-center);
P=double(single(P)); K=double(single(K));
H=diag(rdiag)+p.rho*eye(p.NU)+Bd'*P*Bd;
Qu=double(single(H\eye(p.NU)));
At=double(single((Ad-Bd*K)'));
end
