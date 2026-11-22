function [A_c, B_c, f_c, aux] = tinympc14_fused_ab(cfg, sw, sched)
% 融合 14 态线性化：力投影解析求导，腿长表用割线斜率，仿射项由动力学核反推。
%
%   参考位姿 x_ref = [0 0 0 0 psi_l 0 psi_r 0 pitch_ref 0 alpha 0 H-H_REF 0]，
%   默认参考输入 u_ref = [0 0 0 0 Fbar Fbar]，Fbar = op.M_load*g/2。
%   f = dyn(x_ref,u_ref) - A*x_ref - B*u_ref，保证展开点处动力学一致。
%   展开点不一定是平衡点；不得通过改 f 强行制造一个跨调度点的定点。
%
%   腿轴力与腿长共轭。双轮平地约束下，H/alpha 两行使用同一腿长雅可比。
%   轮矩按轮心水平位移作功。psi 取本调度点的腿角。
%   Production F_pitch=0: direct horizontal force addition double-counts
%   internal support after the balance equations have eliminated internal forces.
%
%   sw 沿用登记表开关：#1 sw.F_pitch、#2 sw.F_H_proj、#8 sw.Ts_H、#8alpha sw.Ts_alpha
%   sched 可带 psi_l/psi_r（缺省都取 sched.psi）
%
%   版本：v0.6  日期：2026-10-22
NX = cfg.NX;  NU = cfg.NU;
op = tinympc14_fused_op(cfg, sched);
psil = sched_get(sched, 'psi_l');
psir = sched_get(sched, 'psi_r');

x0 = zeros(NX, 1);
x0(5)  = psil;
x0(7)  = psir;
x0(9)  = cfg.sym7.pitch_ref;
x0(11) = sched.alpha;
x0(13) = sched.H - cfg.H_REF;
u0 = zeros(NU, 1);
u0(5:6) = op.M_load*cfg.g/2;
if isfield(sched,'eq') && ~isempty(sched.eq)
    u0 = sched.eq.u_eq; % input expansion point only; never replace the affine residual
end

A_c = zeros(NX);
B_c = zeros(NX,NU);
A_c(1:10,1:10) = op.A_bal;
B_c(1:10,1:4) = op.B_bal;
A_c(11,12) = 1;
A_c(13,14) = 1;
A_c(12,11) = cfg.G_ROLL;
A_c(14,13:14) = -[op.K_eff,cfg.B_DAMP]/op.M_v;

psi = x0([5,7])';
sin_psi = sin(psi);
cos_psi = cos(psi);
leg_force = u0(5:6)';
wheel_torque = u0(1:2)';
[height_gain,roll_gain,droll_dphi] = tinympc14_leg_jacobian(cfg.R_TRK,sched.H,sched.alpha,psi);
if sw.F_H_proj
    h_arm = height_gain;
    r_arm = roll_gain;
else
    h_arm = [1,1];
    r_arm = [cfg.R_TRK,-cfg.R_TRK];
end
B_c(14,5:6) = h_arm/op.M_v;
B_c(12,5:6) = r_arm/cfg.I_ROLL;
h_force = leg_force;
r_force = leg_force;
if sw.Ts_H
    h_force = h_force+wheel_torque.*sin_psi/cfg.R_w;
    B_c(14,1:2) = h_arm.*sin_psi/(cfg.R_w*op.M_v);
    A_c(14,[5,7]) = h_arm.*wheel_torque.*cos_psi/(cfg.R_w*op.M_v);
end
if isfield(sw,'Ts_alpha') && sw.Ts_alpha
    r_force = r_force+wheel_torque.*sin_psi/cfg.R_w;
    B_c(12,1:2) = r_arm.*sin_psi/(cfg.R_w*cfg.I_ROLL);
    A_c(12,[5,7]) = r_arm.*wheel_torque.*cos_psi/(cfg.R_w*cfg.I_ROLL);
end
if sw.F_H_proj
    tan_phi = tan(sched.alpha);
    sec2_phi = 1+tan_phi^2;
    load = op.M_load*cfg.g+op.K_eff*x0(13);
    A_c(14,[5,7]) = A_c(14,[5,7])+h_arm.*h_force.*tan(psi)/op.M_v;
    A_c(14,11) = sum(h_arm.*h_force)*tan_phi/op.M_v;
    A_c(12,[5,7]) = A_c(12,[5,7])+r_arm.*r_force.*tan(psi)/cfg.I_ROLL;
    A_c(12,11) = A_c(12,11)+(sum(droll_dphi.*r_force)-load*sched.H*sec2_phi)/cfg.I_ROLL;
    A_c(12,13) = (sum(h_arm.*r_force)-load-op.K_eff*sched.H)*tan_phi/cfg.I_ROLL;
end
if sw.F_pitch
    horizontal_gain = [1/tinympc14_m_tot(cfg); -cfg.l_c/cfg.I_b];
    B_c([2,10],5:6) = horizontal_gain*sin_psi;
    A_c([2,10],[5,7]) = A_c([2,10],[5,7]) + horizontal_gain*(leg_force.*cos_psi);
end
% Vary height and roll at fixed measured sagittal leg angles. Level-ground
% closure gives L_i = (H +/- half_track*sin(roll))/(cos(roll)*cos(psi_i)).
length_gain = balance_length_gain(cfg, sched, x0(1:10), u0(1:4));
A_c(1:10,[11,13]) = A_c(1:10,[11,13]) + length_gain * ...
    [roll_gain(:), height_gain(:)];
% The projected CAD kinetic energy includes H/roll cross inertia. Convert
% the existing generalized-force derivatives using that same mass matrix.
rows = [14,12];
change = op.M_hr\diag([op.M_v,cfg.I_ROLL]);
A_c(rows,:) = change*A_c(rows,:);
B_c(rows,:) = change*B_c(rows,:);
% H changes the CAD mass and gravity tables as well as the geometric arms.
xp=x0; xm=x0; delta=1e-5;
xp(13)=xp(13)+delta; xm(13)=xm(13)-delta;
fp=tinympc14_fused_dyn(cfg,sw,op,xp,u0);
fm=tinympc14_fused_dyn(cfg,sw,op,xm,u0);
A_c(rows,13)=(fp(rows)-fm(rows))/(2*delta);
f_c = tinympc14_fused_dyn(cfg, sw, op, x0, u0) - A_c*x0 - B_c*u0;

aux = op.aux;                                   % 平衡块辅助量（tg0/xoff/theta_star 等）原样带出
aux.M_v = op.M_v;
aux.K_eff = op.K_eff;
aux.g_eff = op.M_load/op.M_v*cfg.g;
aux.Fbar = op.M_load*cfg.g/2;
aux.psi_l = psil;
aux.psi_r = psir;
aux.sched = sched;
aux.x_ref = x0;
aux.u_ref = u0;
end

function gain = balance_length_gain(cfg, sched, x, u)
range = cfg.Leg_data([1,end],1);
step = 0.005; % m, half of the original hardware table spacing
len = min(max([sched.lL,sched.lR],range(1)),range(2));
gain = zeros(10,2);
for side = 1:2
    lo = len; hi = len;
    lo(side) = max(range(1),len(side)-step);
    hi(side) = min(range(2),len(side)+step);
    [A0,B0,f0] = tinympc14_bal_block(lo(1),lo(2),cfg,cfg.sym7.affine_mode);
    [A1,B1,f1] = tinympc14_bal_block(hi(1),hi(2),cfg,cfg.sym7.affine_mode);
    gain(:,side) = ((A1-A0)*x + (B1-B0)*u + f1-f0)/(hi(side)-lo(side));
end
end

function v = sched_get(sched, name)
if isfield(sched, name) && isfinite(sched.(name))
    v = sched.(name);
else
    v = sched.psi;
end
end
