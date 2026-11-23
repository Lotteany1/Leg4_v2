function u = tinympc14_limit_leg_length(p, mdl, x, u, lo, hi)
% 在实际腿长边界前制动；不裁剪状态、目标或模拟腿长。
% 用现有高度反馈刚度得到自然频率，不增加控制增益或修改 Q/R。
% 边界处要求 d''+2*w*d'+w^2*d>=0（d 为剩余高度），避免带速撞边界。
projection = cos(x(11))*cos(x([5,7]));
if any(projection <= 0), return; end
offset = p.R_TRK*sin(x(11))*[1;-1];
lower = max(p.LEG_RANGE(1)*projection-offset);
upper = min(p.LEG_RANGE(2)*projection-offset);
if lower >= upper, return; end
arm = 1./projection';
mass = tinympc14_mv_pick(p,p.H_REF+x(13));
omega2 = arm*mdl.Kinf(5:6,13)/mass;
if omega2 <= 0, return; end
omega = sqrt(omega2);
height = p.H_REF+x(13);
acc_lo = omega2*(lower-height)-2*omega*x(14);
acc_hi = omega2*(upper-height)-2*omega*x(14);
load = p.M_LOAD*p.g+tinympc14_k_eff(p,height)*x(13)+p.B_DAMP*x(14);
wheel = arm*(u(1:2).*sin(x([5,7]))/p.R_w);
acc = (arm*u(5:6)+wheel-load)/mass;
shift = mass*(min(max(acc,acc_lo),acc_hi)-acc)/sum(arm);
% 两腿同增减，保留原差动力；实际可达范围仍由力箱和变化率决定。
shift = min(max(shift,max(lo(5:6)-u(5:6))),min(hi(5:6)-u(5:6)));
u(5:6) = u(5:6)+shift;
end
