function M = tinympc14_m_tot(cfg)
% 整机站立质量：m_b + 2*M_LEG + 2*m_w
M = cfg.m_b + 2*cfg.M_LEG + 2*cfg.m_w;
end
