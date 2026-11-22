function [Ad, Bd, fd] = tinympc14_zoh_embedded(A, B, f, Ts)
% Fixed-order ZOH candidate for the 2 ms firmware scheduler.
% The top rows of the augmented series satisfy a left multiplication by A.
coder.inline('never');
nx = size(A,1);
Astep = Ts*A;
term = Ts*[A,B,f];
delta = term;
for k = 2:5
    term = (Astep*term)/k;
    delta = delta + term;
end
Ad = eye(nx) + delta(:,1:nx);
Bd = delta(:,nx+1:end-1);
fd = delta(:,end);
end
