function [P,K] = tinympc14_terminal_cubic_eval(table, rho, left, right, psi)
%#codegen
% Interpolate log-Cholesky coordinates, never the entries of the cost matrix.
% psi is relative to the local equilibrium lean. The asinh axis resolves the
% narrow gain transition there without changing the physical model or costs.
coder.inline('never');
[il,wl] = axis_weights(table.length_grid,left);
[ir,wr] = axis_weights(table.length_grid,right);
[ip,wp] = axis_weights(table.angle_axis,asinh(psi/table.angle_scale));
nL = numel(table.length_grid);
coords = zeros(1,size(table.basis,1));
for k=1:4
    for j=1:4
        for i=1:4
            node = il+i-1 + nL*(ir+j-2) + nL*nL*(ip+k-2);
            weight = wl(i)*wr(j)*wp(k);
            if isfield(table,'coords')
                node_coords = table.coords(node,:);
            else
                % Deployment reads just this node from the generated cost unit.
                % Coder specializes this branch for a structure without coords.
                node_coords = coder.nullcopy(zeros(1,numel(coords),'int16'));
                coder.ceval('tinympc14_terminal_decode_node',uint32(node-1),coder.wref(node_coords));
            end
            for c=1:numel(coords)
                coords(c) = coords(c)+weight*double(node_coords(c));
            end
        end
    end
end
factor = coords*double(table.basis)+double(table.offset);
L = zeros(14); element = 1;
for j=1:14
    for i=j:14
        L(i,j)=factor(element); element=element+1;
    end
    L(j,j)=exp(L(j,j));
end
P=L*L'+rho*eye(14);
% The fallback gain is interpolated directly. Recovering it from a rounded,
% approximate P amplifies small cost errors in weakly weighted directions.
K=reshape(factor(106:end),6,14);
end

function [base,w] = axis_weights(grid,value)
position = min(max((value-grid(1))/(grid(2)-grid(1)),0),numel(grid)-1);
base = min(max(floor(position),1),numel(grid)-3);
z = position-(base-1);
w = [-(z-1)*(z-2)*(z-3)/6, z*(z-2)*(z-3)/2, ...
     -z*(z-1)*(z-3)/2, z*(z-1)*(z-2)/6];
end
