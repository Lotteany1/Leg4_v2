function rt = tinympc14_runtime_init(p, control_ms)
% 运行时参考状态机初始化：LPV 调度点模型 总力种子 异步序列与计数
rt.p = p;
rt.last_l = [p.H_REF p.H_REF];
rt.last_psi = [0 0];
rt.model = tinympc14_model_at(p, p.H_REF, p.H_REF, 0, 0, 0, p.H_REF);
rt.u_prev = min(max(rt.model.u_ss, p.u_min(:)), p.u_max(:));
rt.rate = [Inf(p.NU-2, 1); p.DF_RATE; p.DF_RATE];

rt.warm = [];
rt.seq = zeros(p.NU, p.NH-1);
rt.seq_valid = false;
rt.seq_uff = zeros(p.NU,1);
rt.seq_force_error = zeros(2,1); % N，发布节点 0 已计入的两腿力偏差
rt.base_tick = -Inf;
rt.node_prev = -1;
rt.solve_tick = -Inf;
rt.node_ticks = 1000*p.Ts;
rt.solve_period_ms = 2*rt.node_ticks;
if nargin < 2, control_ms = rt.node_ticks; end
validateattributes(control_ms,{'numeric'},{'scalar','real','finite','positive'});
rt.output_rate = rt.rate*min(control_ms/rt.node_ticks,1);
rt.control_dt = 0.001*control_ms;

rt.stale = 0;
rt.nan = 0;
rt.fallback = 0;
rt.commit = 0;
rt.lpv_updates = 0;
rt.node_repeat = 0;
rt.iters = zeros(1, 0);
rt.conv = zeros(1, 0);
end
