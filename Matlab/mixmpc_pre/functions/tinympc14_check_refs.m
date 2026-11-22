function refs = tinympc14_check_refs()
% 自检用的测试点、历史基准与容差；不属于模型参数
refs.l_chk = 0.27;                 % m 缓存对拍点（与缓存网格对齐）

refs.hist = struct('A25', -42.4445, 'A45', -6.5404, 'A65', 338.224, ...
                   'A105', -4.3087, 'A109', 7.9809, ...
                   'B21', 7.8965,  'B41', -6.0223, 'B63', 11.51, ...
                   'B103', -2.2733, 'B23', -1.4447);

p=tinympc14_config();
refs.s03 = struct('M_TOT',p.m_b+2*p.m_l+2*p.m_w,'R_TRK',p.R_l,'I_ROLL',.578,'g',p.g);

refs.tol = struct('cache', 1e-12, 'hist', 1e-2, 'c2', 1e-9, 'c3', 1e-3, 'c5', 1e-12, ...
                  'e0', 1e-12, 'e2', 1e-10, 'g_fp', 1e-12, 'relA', 0.30, ...
                  'spring', 1e-5);

refs.grid.fs  = [0.16, 0.25, 0.27, 0.34];              % m 弹簧力打印点
refs.grid.tg  = [0.16, 0.20, 0.25, 0.30, 0.34];        % m 重力常数打印点
refs.grid.trq = [0.16, 0.20, 0.25, 0.27, 0.30, 0.34];  % m 输出层核对点
refs.grid.Ts  = [0.002, 0.005, 0.010];                 % s 离散化扫描

refs.G_pt = [0.25, 0.00; 0.30, 0.00; 0.25, 0.05];      % [m rad] 工作点
refs.G_lbl = {'(0.25, 0   )', '(0.30, 0   )', '(0.25, 0.05)'};

refs.perf.tref = [45, 81];          % µs/轮 平衡侧两说
refs.perf.nh_long = 40;             % 拉回 80 ms 窗的节点数
refs.perf.table_bytes_ref = 15840;  % B 现役表内存
refs.pitch_pole_ref = 2.825;        % rad/s 俯仰极点参考
refs.trq_test = [100, 5];           % [N N·m] 输出层核对输入
end
