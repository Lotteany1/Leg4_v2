function xd = tinympc14_fused_dyn(cfg, sw, op, x, u)
% 14 态降阶动力学。前十态沿用冻结腿长的平衡块，高度和横滚使用同一闭链虚功。
% op 由 tinympc14_fused_op 提供（lL/lR 与本点平衡块）；psi 取 x(5)/x(7)
M_TOT = tinympc14_m_tot(cfg);
K_eff = op.K_eff;
psil = x(5);  psir = x(7);
sl = sin(psil); sr = sin(psir);

xd = zeros(cfg.NX, 1);
xd(1:2:13) = x(2:2:14);
x10 = x(1:10);
xd(2:2:10) = op.A_bal(2:2:10,:)*x10 + op.B_bal(2:2:10,:)*u(1:4) + op.f_bal(2:2:10);

H = cfg.H_REF+x(13);
[M_hr,G_hr] = tinympc14_hr_params(cfg,H);
load = G_hr(1)+K_eff*x(13);
if sw.F_H_proj
    [height_arm,roll_arm] = tinympc14_leg_jacobian(cfg.R_TRK,H,x(11),[psil,psir]);
    % 名义竖直承重 load、两腿等分时的几何倾覆矩。其合成效果已由
    % tinympc14_roll_force 给定，内部仅保留偏离该预载引起的力矩变化。
    preload_moment = load*H*tan(x(11));
else
    height_arm = [1,1];
    roll_arm = [cfg.R_TRK,-cfg.R_TRK];
    preload_moment = 0;
end
heave_force = height_arm*u(5:6)-load-cfg.B_DAMP*x(14);
roll_moment = roll_arm*u(5:6)-preload_moment-G_hr(2);
% 固定髋部前向位置和腿角，轮心水平位移为 -sin(psi_i)*dL_i。
% 模型正轮矩与电机轮矩反号，故等价轴力为 +Tw_i*sin(psi_i)/R_w。
wheel_force = u(1:2).*[sl;sr]/cfg.R_w;
if sw.Ts_H
    heave_force = heave_force+height_arm*wheel_force;
end
if isfield(sw, 'Ts_alpha') && sw.Ts_alpha
    roll_moment = roll_moment+roll_arm*wheel_force;
end
acceleration = M_hr\[heave_force;roll_moment];
xd(14) = acceleration(1);
xd(12) = cfg.G_ROLL*x(11)+acceleration(2);

if sw.F_pitch
    % 水平分量：力平衡进 s 行，髋部力臂 l_c 进 theta_b 行
    xd(2)  = xd(2)  + (u(5)*sl + u(6)*sr)/M_TOT;
    xd(10) = xd(10) - cfg.l_c*(u(5)*sl + u(6)*sr)/cfg.I_b;
end
end
