function [dF_lft,dF_rgt,Fc_raw] = Leg_RollComp(H,omega,v)
%#codegen
% 平地准静态载荷转移：CAD 全车质量乘离地质心高度，包含轮子与轮半径。
% H 是髋部相对轮心高度 [m]；v [m/s]、omega [rad/s]，前进左转右侧加载。
[~,p,~,~,turn] = leg_urdf_data();
H=min(max(H,cast(p.Len_min,'like',H)),cast(p.Len_max,'like',H));
first_moment=leg_lookup(turn,H);
Fc_raw=first_moment*v*omega/cast(2*p.half_track,'like',H);
limit=cast(150,'like',H);
dF_rgt=min(max(Fc_raw,-limit),limit);
dF_lft=-dF_rgt;
end
