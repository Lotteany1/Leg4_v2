function [A, B, f, aux] = tinympc14_bal_block(lL, lR, cfg, grav_mode)
% 方程 (19)-(23) 单点线性化：两腿各按自身腿长取表，10 态 A/B + 仿射 f
% grav_mode: 'B'  = B 族线性口径（对拍用）
%            'A0' = A 族在 theta=0 线性化（常数 +C2*sin(theta0)）
%            'V*' = A 族在真实平衡点 theta* 线性化（常数 tau_g(0)+余项）
R_w = cfg.R_w; R_l = cfg.R_l; l_c = cfg.l_c; g = cfg.g;
m_w = cfg.m_w; m_l = cfg.m_l; m_b = cfg.m_b;
I_w = cfg.I_w; I_b = cfg.I_b; I_z = cfg.I_z;

rL = tinympc14_table_row(cfg.Leg_data, lL);
rR = tinympc14_table_row(cfg.Leg_data, lR);
l_wl = rL(1); l_bl = rL(2); I_ll = rL(3);
l_wr = rR(1); l_br = rR(2); I_lr = rR(3);
cL = tinympc14_table_row(cfg.Leg_comp, lL);
cR = tinympc14_table_row(cfg.Leg_comp, lR);
theta0 = cL(1); l_d = cL(2);           % 左腿（打印口径沿用左腿）
theta0_r = cR(1); l_d_r = cR(2);

C1g   = (m_b/2 + cfg.M_LEG)*g*lL;      % A 族整机项系数 = (m_b/2 + m_leg)*g*l
C1g_r = (m_b/2 + cfg.M_LEG)*g*lR;
C2g   = cfg.M_LEG*g*l_d;               % A 族腿自重项系数 = m_leg*g*l_d
C2g_r = cfg.M_LEG*g*l_d_r;
kB    = (m_l*l_wl + m_b*lL/2)*g;       % B 族腿行重力刚度
kB_r  = (m_l*l_wr + m_b*lR/2)*g;
tg0   = C2g*sin(theta0);               % tau_g(0) = M_LEG*g*x_off(H)
tg0_r = C2g_r*sin(theta0_r);
theta_star = 0; theta_star_r = 0;

if strcmp(grav_mode, 'B')
    kA = kB;  kA_r = kB_r;  Gconst = 0;  Gconst_r = 0;
else
    try
        theta_star = fzero(@(x) C1g*sin(x) - C2g*sin(x + theta0), cfg.sym7.theta_star_guess);
        theta_star_r = fzero(@(x) C1g_r*sin(x) - C2g_r*sin(x + theta0_r), cfg.sym7.theta_star_guess);
    catch
        theta_star = NaN;  theta_star_r = NaN;
    end
    switch grav_mode
        case 'A0'
            kA   = C1g - C2g*cos(theta0);
            kA_r = C1g_r - C2g_r*cos(theta0_r);
            Gconst = tg0;  Gconst_r = tg0_r;
        case 'V*'
            kA   = C1g*cos(theta_star) - C2g*cos(theta_star + theta0);
            kA_r = C1g_r*cos(theta_star_r) - C2g_r*cos(theta_star_r + theta0_r);
            % 仿射槽：tau_g(0)(H) 查表项 + 线性化余项（和 = kA*theta*，定点仍精确）
            Gconst   = tg0 + (kA*theta_star - tg0);
            Gconst_r = tg0_r + (kA_r*theta_star_r - tg0_r);
        otherwise
            error('grav_mode must be B, A0, or V*.');
    end
end

M = zeros(5);
M(1,1) = I_w*lL/R_w + m_w*R_w*lL + m_l*R_w*l_bl;
M(1,3) = m_l*l_wl*l_bl - I_ll;
M(2,2) = I_w*lR/R_w + m_w*R_w*lR + m_l*R_w*l_br;
M(2,4) = m_l*l_wr*l_br - I_lr;
M(3,1:2) = -(m_w*R_w^2 + I_w + m_l*R_w^2 + m_b*R_w^2/2);
M(3,3) = -(m_l*R_w*l_wl + m_b*R_w*lL/2);
M(3,4) = -(m_l*R_w*l_wr + m_b*R_w*lR/2);
M(4,1:2) = m_w*R_w*l_c + I_w*l_c/R_w + m_l*R_w*l_c;
M(4,3) = m_l*l_wl*l_c;
M(4,4) = m_l*l_wr*l_c;
M(4,5) = -I_b;
M(5,1) = I_z*R_w/(2*R_l) + I_w*R_l/R_w;
M(5,2) = -M(5,1);
M(5,3) = I_z*lL/(2*R_l);
M(5,4) = -I_z*lR/(2*R_l);

Nx = zeros(5,3);           % N_s = -(方程里除 ddq 外的其余项)，故整体取负
Nx(1,1) = -kA;
Nx(2,2) = -kA_r;
Nx(4,3) = -m_b*g*l_c;

Nu = zeros(5,4);           % 同上：Nu = -∂(其余项)/∂T
Nu(1,:) = [ (1 + lL/R_w), 0, -1, 0];
Nu(2,:) = [0, (1 + lR/R_w), 0, -1];
Nu(3,:) = [-1, -1, 0, 0];
Nu(4,:) = [ l_c/R_w, l_c/R_w, 1, 1];
Nu(5,:) = [ R_l/R_w, -R_l/R_w, 0, 0];

Nd = M \ Nx;
Nuc = M \ Nu;
fcst = M \ [Gconst; Gconst_r; 0; 0; 0];

% s/ds use the mean hip position/velocity consumed from body.vel.
% At fixed measured lengths: s_hip=s_wheel+(lL*psiL+lR*psiR)/2.
T = [ R_w/2,        R_w/2,        lL/2,          lR/2,         0;
     -R_w/(2*R_l),  R_w/(2*R_l), -lL/(2*R_l),    lR/(2*R_l),   0;
      0,            0,            1,             0,            0;
      0,            0,            0,             1,            0;
      0,            0,            0,             0,            1];

A = zeros(10);
A(1:2:9, 2:2:10) = eye(5);
A(2:2:10, 5:2:9) = T * Nd;
B = zeros(10,4);
B(2:2:10, :) = T * Nuc;
f = zeros(10,1);
f(2:2:10) = T * fcst;

aux = struct('l_wl', l_wl, 'l_bl', l_bl, 'I_ll', I_ll, 'theta0', theta0, 'l_d', l_d, ...
             'l_wr', l_wr, 'l_br', l_br, 'I_lr', I_lr, 'theta0_r', theta0_r, 'l_d_r', l_d_r, ...
             'kA0', C1g - C2g*cos(theta0), 'kA0_r', C1g_r - C2g_r*cos(theta0_r), ...
             'kA', kA, 'kA_r', kA_r, 'kB', kB, 'kB_r', kB_r, ...
             'tg0', tg0, 'tg0_r', tg0_r, ...
             'xoff', l_d*sin(theta0), 'xoff_r', l_d_r*sin(theta0_r), ...
             'Gconst', Gconst, 'Gconst_r', Gconst_r, ...
             'theta_star', theta_star, 'theta_star_l', theta_star, 'theta_star_r', theta_star_r);
end
