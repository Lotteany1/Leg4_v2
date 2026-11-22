function [Ad, Bd, fd] = tinympc14_zoh_ab(A, B, f, Ts)
% ZOH 零阶保持离散化；当前生成链固定调用本函数，没有 Euler/ZOH 切换分支。
% 连续：dx/dt=A*x+B*u+f；离散：x(k+1)=Ad*x(k)+Bd*u(k)+fd。
% Ts [s] 为离散采样周期，来自 cfg.Ts；一个采样区间内 A/B/f 和 u 均冻结。
% 增广矩阵指数同时计算 Ad=expm(A*Ts)、Bd=积分 expm(A*t)*B dt、
% fd=积分 expm(A*t)*f dt（积分区间 0..Ts），不需要 A 可逆，亦不是前向 Euler 近似。
% B 可为空（只计算状态/仿射离散增益）；f 为空时按连续零仿射项处理。
n = size(A,1); m = size(B,2);
if isempty(f), f = zeros(n,1); end
E = expm([A, B, f; zeros(m+1, n + m + 1)] * Ts);
Ad = E(1:n, 1:n);
Bd = E(1:n, n+1:n+m);
fd = E(1:n, end);
end
