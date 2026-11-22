function [A_c, B_c, f_c, aux] = tinympc14_fused_ab_legacy(cfg, sw, sched)
% 融合 14 态：平衡块按 sched.lL/lR 调度 + 腿长块（M_v 三档、K_EFF 按 H）+ 仿射槽；
% sw 的耦合开关打开时在对应行列加项（psi 取 sched.psi，M_s = M_TOT）
M_TOT = tinympc14_m_tot(cfg);
M_v = tinympc14_mv_pick(cfg, sched.H);
g_eff = M_TOT/M_v*cfg.g;
K_eff = tinympc14_k_eff(cfg, sched.H);
[A_bal, B_bal, f_bal, aux] = tinympc14_bal_block(sched.lL, sched.lR, cfg, cfg.sym7.affine_mode);
A_leg = [0, 1, 0, 0; cfg.G_ROLL, 0, 0, 0; 0, 0, 0, 1; 0, 0, -K_eff/M_v, -cfg.B_DAMP/M_v];
B_leg = [0, 0;
         cfg.R_TRK/cfg.I_ROLL, -cfg.R_TRK/cfg.I_ROLL;
         0, 0;
         1/M_v, 1/M_v];
f_leg = [0; 0; 0; -g_eff];
A_c = blkdiag(A_bal, A_leg);
B_c = [B_bal, zeros(10, 2); zeros(4, 4), B_leg];
f_c = [f_bal; f_leg];

Fbar = M_TOT*cfg.g/2;
sg = cfg.sym7.psi_sign;
if isfield(sched, 'psi')
    ps = sched.psi;
else
    ps = cfg.sym7.psi_nom;
end
cps = cos(ps);  sps = sin(ps);

if sw.F_pitch
    A_c(2,5) = A_c(2,5) + sg*Fbar*cps/M_TOT;
    A_c(2,7) = A_c(2,7) + sg*Fbar*cps/M_TOT;
    if cfg.sym7.psi_inc_pitch
        A_c(2,9) = A_c(2,9) + 2*sg*Fbar*cps/M_TOT;
    end
    B_c(2,5) = B_c(2,5) + sg*sps/M_TOT;
    B_c(2,6) = B_c(2,6) + sg*sps/M_TOT;
    % 水平力作用在髋部、相对机体质心有 l_c 竖直臂 → 进俯仰行（row 10 = θ̈_b；符号按右手系推导）
    A_c(10,5) = A_c(10,5) - cfg.l_c*Fbar*cps/cfg.I_b;
    A_c(10,7) = A_c(10,7) - cfg.l_c*Fbar*cps/cfg.I_b;
    B_c(10,5) = B_c(10,5) - cfg.l_c*sg*sps/cfg.I_b;
    B_c(10,6) = B_c(10,6) - cfg.l_c*sg*sps/cfg.I_b;
end
if sw.F_H_proj
    % #2 竖直投影：腿角列（一阶）+ F 列（二阶）
    A_c(14,5) = A_c(14,5) - Fbar*sps/M_v;
    A_c(14,7) = A_c(14,7) - Fbar*sps/M_v;
    B_c(14,5) = B_c(14,5) + (cps - 1)/M_v;
    B_c(14,6) = B_c(14,6) + (cps - 1)/M_v;
end
if sw.Ts_H
    B_c(14,1) = B_c(14,1) + sg*sps/(cfg.R_w*M_v);
    B_c(14,2) = B_c(14,2) + sg*sps/(cfg.R_w*M_v);
end
if isfield(sw, 'Ts_alpha') && sw.Ts_alpha
    B_c(12,1) = B_c(12,1) + sg*cfg.R_TRK*sps/(cfg.R_w*cfg.I_ROLL);
    B_c(12,2) = B_c(12,2) - sg*cfg.R_TRK*sps/(cfg.R_w*cfg.I_ROLL);
end
aux.M_v = M_v;
aux.g_eff = g_eff;
aux.K_eff = K_eff;
aux.sched = sched;
end
