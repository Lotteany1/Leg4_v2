function [xd, mechanical] = tinympc14_vlwip_sagittal(cfg, x, u)
% Symmetric, no-slip rolling template for checking the 14-state predictor.
% x = [wheel travel; forward lean; axle-to-CoM length; their velocities].
% u = [sum of model wheel torques (N*m); sum of axial leg forces (N)].
% The supported mass is lumped at the moving CoM, with an effective pitch
% inertia. Hip torques are internal to this lumped body.
slope = sin(x(2));
vertical = cos(x(2));
length = x(3);
speed = x(4);
pitch_rate = x(5);
length_rate = x(6);
m = cfg.M_SUP;
rolling_mass = m + 2*cfg.m_w + 2*cfg.I_w/cfg.R_w^2;

M = [rolling_mass, m*length*vertical, m*slope;
     m*length*vertical, cfg.I_b+m*length^2, 0;
     m*slope, 0, m];
bias = [m*(2*length_rate*pitch_rate*vertical-length*pitch_rate^2*slope);
        2*m*length*length_rate*pitch_rate-m*cfg.g*length*slope;
        m*(cfg.g*vertical-length*pitch_rate^2)];
force = [u(1)/cfg.R_w; -u(1); u(2)];
velocity = [speed;pitch_rate;length_rate];
xd = [velocity; M\(force-bias)];

if nargout > 1
    mechanical = struct('rolling_momentum',M(1,:)*velocity, ...
        'energy',0.5*velocity'*M*velocity + m*cfg.g*length*vertical, ...
        'input_power',force'*velocity);
end
end
