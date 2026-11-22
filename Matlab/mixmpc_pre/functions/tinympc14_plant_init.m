function pl = tinympc14_plant_init(p, sp)
% 场景 plant 包：真模型 ZOH 与平衡点
[A, B, f, aux] = tinympc14_fused_ab(p, p.sym7.sw_fused, sp);
pl.force_roll = aux.x_ref(11);
pl.force_psi = aux.x_ref([5,7]);
[pl.Adp, pl.Bdp, pl.fdp] = tinympc14_zoh_ab(A, B, f, p.Ts);
[pl.xb, pl.uss, ~] = tinympc14_equilibrium(p, p.sym7.sw_fused, sp);
end
