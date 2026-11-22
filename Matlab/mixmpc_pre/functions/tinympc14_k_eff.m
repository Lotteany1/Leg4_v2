function K = tinympc14_k_eff(cfg, H)
% 气弹簧等效刚度：'spring' 按 H 中心差分 dFs/dL0；'fixed' 取 cfg.K_EFF
if strcmp(cfg.sym7.k_eff_mode, 'spring')
    h = cfg.sym7.k_eff_df_h;
    K = (Leg_SpringF(double(H + h)) - Leg_SpringF(double(H - h))) / (2*h);
else
    K = cfg.K_EFF;
end
end
