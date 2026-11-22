function [Ad,Bd,fd,K,P,Qu,At] = tinympc14_lpv_kernel(p,table,rdiag,left,right,psi)
%#codegen
% Keep weights and terminal coefficients as read-only data, outside arithmetic.
[Ad,Bd,fd,K,P,Qu,At]=tinympc14_lpv_embedded(p,table,left,right,psi,rdiag);
end
