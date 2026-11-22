function fe = tinympc14_f_extra(p, F_app, T_w, psiL, psiR, psi_bar)
% ψ-A 前馈的仿射增项：四行非零的 14 态向量
M_TOT = tinympc14_m_tot(p);
Fbar = M_TOT * p.g / 2;
sL = sin(psiL); sR = sin(psiR);
cL = cos(psiL); cR = cos(psiR);
f2  = (F_app(1)*sL + F_app(2)*sR - Fbar*(sin(psi_bar) + sin(psi_bar))) / M_TOT;
f10 = -p.l_c * (F_app(1)*sL + F_app(2)*sR) / p.I_b;
f14 = ((T_w(1)/p.R_w)*sL + (T_w(2)/p.R_w)*sR ...
       + F_app(1)*(cL-1) + F_app(2)*(cR-1)) / p.M_V;
f12 = p.R_TRK * ((T_w(1)/p.R_w)*sL - (T_w(2)/p.R_w)*sR) / p.I_ROLL;
fe = zeros(p.NX, 1);
fe(2) = f2; fe(10) = f10; fe(12) = f12; fe(14) = f14;
end
