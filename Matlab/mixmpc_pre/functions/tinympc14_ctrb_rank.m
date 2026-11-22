function r = tinympc14_ctrb_rank(A, B)
% 可控性矩阵秩（手写，避免依赖 ctrb）
n = size(A,1);
W = B;
V = B;
for k = 2:n
    V = A*V;
    W = [W, V];
end
r = rank(W);
end
