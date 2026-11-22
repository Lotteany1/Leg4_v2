function S = tinympc14_scenarios(p)
% S6/S8 共用闭环工况表：七个场景连同 plant 包
T_END = 3.0;
t = (0:round(T_END/p.Ts)-1)' * p.Ts;

S = struct('name', {}, 'sp1', {}, 'sp2', {}, 't_sw', {}, 'psi1', {}, 'psi2', {}, ...
           'H_step', {}, 't_h0', {}, 't_h1', {}, 'roll_imp', {}, 't_roll', {}, ...
           'b_force', {}, 't_dist', {}, 'ffwd', {}, 'xref_mode', {});

S(1).name = 'H 阶跃 5 cm';
S(1).sp1 = tinympc14_sched_pt(p, 0.27, 0, 0);  S(1).sp2 = S(1).sp1;
S(1).t_sw = Inf;  S(1).psi1 = 0;  S(1).psi2 = 0;
S(1).H_step = 0.05;  S(1).t_h0 = 0.5;  S(1).t_h1 = 2.0;
S(1).roll_imp = 0;  S(1).t_roll = Inf;
S(1).b_force = zeros(2,1);  S(1).t_dist = Inf;
S(1).ffwd = zeros(2,1);  S(1).xref_mode = 'true';

S(2).name = 'psi 阶跃 0.10';
S(2).sp1 = tinympc14_sched_pt(p, 0.27, 0, 0);
S(2).sp2 = tinympc14_sched_pt(p, 0.27, 0, 0.10);
S(2).t_sw = 1.0;  S(2).psi1 = 0;  S(2).psi2 = 0.10;
S(2).H_step = 0;  S(2).t_h0 = Inf;  S(2).t_h1 = Inf;
S(2).roll_imp = 0;  S(2).t_roll = Inf;
S(2).b_force = zeros(2,1);  S(2).t_dist = Inf;
S(2).ffwd = zeros(2,1);  S(2).xref_mode = 'true';

S(3).name = '横滚角速度冲激';
S(3).sp1 = tinympc14_sched_pt(p, 0.27, 0, 0.10);  S(3).sp2 = S(3).sp1;
S(3).t_sw = Inf;  S(3).psi1 = 0.10;  S(3).psi2 = 0.10;
S(3).H_step = 0;  S(3).t_h0 = Inf;  S(3).t_h1 = Inf;
S(3).roll_imp = 0.5;  S(3).t_roll = 1.2;
S(3).b_force = zeros(2,1);  S(3).t_dist = Inf;
S(3).ffwd = zeros(2,1);  S(3).xref_mode = 'true';

S(4).name = '持续力扰动';
S(4).sp1 = tinympc14_sched_pt(p, 0.27, 0, 0);  S(4).sp2 = S(4).sp1;
S(4).t_sw = Inf;  S(4).psi1 = 0;  S(4).psi2 = 0;
S(4).H_step = 0;  S(4).t_h0 = Inf;  S(4).t_h1 = Inf;
S(4).roll_imp = 0;  S(4).t_roll = Inf;
S(4).b_force = [-18; -12];  S(4).t_dist = 1.8;
S(4).ffwd = zeros(2,1);  S(4).xref_mode = 'true';

S(5).name = '非对称腿长';
S(5).sp1 = tinympc14_sched_pt(p, 0.27, 0, 0);
S(5).sp1.lL = 0.25;  S(5).sp1.lR = 0.29;
S(5).sp1.alpha=atan((S(5).sp1.lL-S(5).sp1.lR)/(2*p.R_TRK));
S(5).sp1.H=.5*(S(5).sp1.lL+S(5).sp1.lR)*cos(S(5).sp1.alpha);
S(5).sp2 = S(5).sp1;
S(5).t_sw = Inf;  S(5).psi1 = 0;  S(5).psi2 = 0;
S(5).H_step = 0;  S(5).t_h0 = Inf;  S(5).t_h1 = Inf;
S(5).roll_imp = 0;  S(5).t_roll = Inf;
S(5).b_force = zeros(2,1);  S(5).t_dist = Inf;
S(5).ffwd = zeros(2,1);  S(5).xref_mode = 'true';

S(6).name = '常值前馈（箱平移）';
S(6).sp1 = tinympc14_sched_pt(p, 0.27, 0, 0);  S(6).sp2 = S(6).sp1;
S(6).t_sw = Inf;  S(6).psi1 = 0;  S(6).psi2 = 0;
S(6).H_step = 0.05;  S(6).t_h0 = 0.5;  S(6).t_h1 = 2.0;
S(6).roll_imp = 0;  S(6).t_roll = Inf;
S(6).b_force = zeros(2,1);  S(6).t_dist = Inf;
S(6).ffwd = [8; -8];  S(6).xref_mode = 'true';

S(7).name = 'alpha 箱激活';
S(7).sp1 = tinympc14_sched_pt(p, 0.27, 0, 0.10);  S(7).sp2 = S(7).sp1;
S(7).t_sw = Inf;  S(7).psi1 = 0.10;  S(7).psi2 = 0.10;
S(7).H_step = 0;  S(7).t_h0 = Inf;  S(7).t_h1 = Inf;
S(7).roll_imp = 3.0;  S(7).t_roll = 0.2;
S(7).b_force = zeros(2,1);  S(7).t_dist = Inf;
S(7).ffwd = zeros(2,1);  S(7).xref_mode = 'true';
S(7).x0_alpha = 0.55;

for s = 1:numel(S)
    S(s).pl1 = tinympc14_plant_init(p, S(s).sp1);
    if isfinite(S(s).t_sw)
        S(s).pl2 = tinympc14_plant_init(p, S(s).sp2);
    else
        S(s).pl2 = S(s).pl1;
    end
end
end
