function [x_eq, u_eq, info] = tinympc14_fused_eq(cfg, sw, sched)
% 非线性平衡：两腿同角、alpha=0、theta_b 取 pitch_ref，解腿角与六路输入使 14 态加速度为零
% 牛顿迭代 + 数值雅可比；返回 x_eq/u_eq 供仿射项锚定
NX = cfg.NX;  NU = cfg.NU;
op = tinympc14_fused_op(cfg, sched);
if strcmp(cfg.sym7.affine_mode,'B') && ~sw.F_pitch
    % Zero longitudinal and yaw acceleration imply zero wheel torque.
    % The two leg gravity moments and the body pitch moment then fix the
    % common leg lean and hip torques. Vertical and roll balance fix forces.
    gravity_stiffness = [op.aux.kB;op.aux.kB_r];
    body_pitch_moment = cfg.m_b*cfg.g*cfg.l_c*cfg.sym7.pitch_ref;
    theta = -body_pitch_moment/sum(gravity_stiffness);
    load = op.M_load*cfg.g + op.K_eff*(sched.H-cfg.H_REF);
    moment = -op.I_roll*cfg.G_ROLL*sched.alpha;
    if sw.F_H_proj
        % 两腿分担升降广义载荷。在该分担点的几何倾覆矩已归入外扰。
        heave_scale = cos(sched.alpha)*cos(theta);
        diff_scale = heave_scale*cos(sched.alpha)/cfg.R_TRK;
        leg_force = (load*heave_scale+[1;-1]*moment*diff_scale)/2;
    else
        leg_force = (load+[1;-1]*moment/cfg.R_TRK)/2;
    end
    x_eq = zeros(NX,1);
    x_eq([5,7]) = theta;
    x_eq(9) = cfg.sym7.pitch_ref;
    x_eq(11) = sched.alpha;
    x_eq(13) = sched.H-cfg.H_REF;
    u_eq = [0;0;-gravity_stiffness*theta;leg_force];
    residual = norm(tinympc14_fused_dyn(cfg,sw,op,x_eq,u_eq),Inf);
    info = struct('theta_leg',theta,'u_eq',u_eq,'res',residual,'iters',0);
    return;
end
u0 = zeros(NU, 1);
u0(5) = op.M_load*cfg.g/2;
u0(6) = u0(5);
z = [cfg.sym7.theta_star_guess; u0];
r = resid(cfg, sw, op, sched, z);
for k = 1:50
    if max(abs(r)) < 1e-12
        break;
    end
    J = zeros(7);
    h = 1e-6;
    for i = 1:7
        zp = z;  zm = z;
        zp(i) = zp(i) + h;
        zm(i) = zm(i) - h;
        J(:, i) = (resid(cfg, sw, op, sched, zp) - resid(cfg, sw, op, sched, zm)) / (2*h);
    end
    dz = -pinv(J) * r;
    t = 1.0;
    for kk = 1:20
        zn = z + t*dz;
        rn = resid(cfg, sw, op, sched, zn);
        if norm(rn) < norm(r)
            break;
        end
        t = 0.5*t;
    end
    z = z + t*dz;
    r = resid(cfg, sw, op, sched, z);
end
u_eq = z(2:7);
x_eq = zeros(NX, 1);
x_eq(5) = z(1);
x_eq(7) = z(1);
x_eq(9) = cfg.sym7.pitch_ref;
x_eq(11) = sched.alpha;
x_eq(13) = sched.H - cfg.H_REF;
info = struct('theta_leg', z(1), 'u_eq', u_eq, 'res', max(abs(r)), 'iters', k);
end

function r = resid(cfg, sw, op, sched, z)
x = zeros(cfg.NX, 1);
x(5) = z(1);
x(7) = z(1);
x(9) = cfg.sym7.pitch_ref;
x(11) = sched.alpha;
x(13) = sched.H - cfg.H_REF;
xd = tinympc14_fused_dyn(cfg, sw, op, x, z(2:7));
r = xd(2:2:14);
end
