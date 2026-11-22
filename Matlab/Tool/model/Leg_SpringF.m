%#codegen
function Fs = Leg_SpringF(L0, ang)
% 腿长 L0 [m]，返回被动沿腿力 [N]，伸腿为正。
% 两销随连杆转动，摆角 ang [rad] 保留用于兼容调用。
% Fs = F_spring(s) * ds/dL，控制器取反后加到轴向力 F。
[~, par] = leg_config();
l1 = cast(par.Spr_l1, 'like', L0);
l2 = cast(par.Spr_l2, 'like', L0);
L = min(max(L0, cast(par.Spr_len_min, 'like', L0)), cast(par.Spr_len_max, 'like', L0));
phi = cast(par.Spr_phi, 'like', L0);
s1 = cast(par.Spr_s1, 'like', L0);
s2 = cast(par.Spr_s2, 'like', L0);
cosTheta = (l1*l1 + l2*l2 - L*L)/(2*l1*l2);
theta = acos(min(max(cosTheta, -1), 1));
delta = theta - phi;
s = sqrt(max(s1*s1 + s2*s2 - 2*s1*s2*cos(delta), 0));
dsDL = s1*s2*sin(delta)*L/max(l1*l2*sin(theta)*s, 1e-6);
sMin = cast(par.Spr_pin_min, 'like', L0);
sMax = cast(par.Spr_pin_max, 'like', L0);
assert(sMin > 0 && sMax > sMin && isfinite(sMax), 'Invalid spring pin-centre distances');
fraction = min(max((sMax - s)/(sMax - sMin), 0), 1);
Fs = cast(par.SprF, 'like', L0)*fraction*dsDL;
end
