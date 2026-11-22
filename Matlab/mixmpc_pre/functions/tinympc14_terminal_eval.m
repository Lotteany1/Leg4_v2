function P = tinympc14_terminal_eval(table, rho, left, right, psi)
%#codegen
% Trilinear interpolation of quantized Cholesky coordinates on a uniform grid.
coder.inline('never'); % Do not reserve the factor scratch during dynamics assembly.
nL = numel(table.length_grid);
[il, wl] = grid_axis(table.length_grid, left);
[ir, wr] = grid_axis(table.length_grid, right);
[ip, wp] = grid_axis(table.angle_grid, psi);

coords = zeros(1,size(table.basis,1));
for k = 0:1
    for j = 0:1
        for i = 0:1
            node = il+i + nL*(ir+j-1) + nL*nL*(ip+k-1);
            weight = wl(i+1)*wr(j+1)*wp(k+1);
            for component = 1:numel(coords)
                coords(component) = coords(component) + weight*double(table.coords(node,component));
            end
        end
    end
end
factor = coords*double(table.basis)+double(table.offset);
n = 14;
L = zeros(n);
element = 1;
for j = 1:n
    for i = j:n
        L(i,j) = factor(element);
        element = element+1;
    end
    L(j,j) = exp(L(j,j));
end
P = L*L' + rho*eye(n);
end

function [index, weight] = grid_axis(grid, signal)
position = (signal-grid(1))/(grid(2)-grid(1));
position = min(max(position,0),numel(grid)-1);
cell_index = min(floor(position),numel(grid)-2);
fraction = position-cell_index;
index = cell_index+1;
weight = [1-fraction,fraction];
end
