function M_v = tinympc14_mv_pick(cfg, H)
% H 行等效竖直惯量的三档口径（见 cfg §3）
switch cfg.sym7.Mv_mode
    case 'derived'
        dld = tinympc14_dtab(cfg.Leg_comp, H, 3);
        M_v = cfg.m_b + 2*cfg.M_LEG*(1 - dld);
    case 'near_end'
        M_v = cfg.m_b + 2*cfg.sym7.m_leg_near;
    otherwise
        M_v = cfg.M_V;
end
end
