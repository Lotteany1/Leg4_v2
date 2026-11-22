function [c, st] = tinympc14_output(cfg, len_l, len_r, ang_l, ang_r, v, omega, st)
% 输出层装配：三笔补偿的物理量 + 逐笔沿用状态（对应 tinympc14.c 的 output 段）
%   c = tinympc14_output(cfg, ...)          无历史入口：失效笔回 0 且状态 0
%   [c, st] = tinympc14_output(cfg, ..., st) 带历史入口：失效笔在 5 拍内沿用上次有效值
% 物理量自己算：MgF 用 Tool 的 Leg_Comp 查表 + cfg 常量，Fs 用 Tool 的 Leg_SpringF，
% Fc 用 cfg.M_SUP / H_OFF / COMP.ROLL_TRACK；单精度口径与 C 侧一致
if nargin < 8 || isempty(st)
    st = output_reset();
end

len = [len_l, len_r];
ang = [ang_l, ang_r];
ok_len = isfinite(len);
ok_ang = isfinite(ang);

MgF = zeros(2,1);
Fs  = zeros(2,1);
Fc  = zeros(2,1);
urdf = isfield(cfg,'URDF_SOURCE');
for k = 1:2
    if ok_len(k) && ok_ang(k)
        if urdf
            comp = single(tinympc14_table_row(cfg.Leg_comp,len(k)));
            theta0 = comp(1);
            l_d = comp(2);
        else
            [theta0, l_d] = Leg_Comp(single(len(k)));
        end
        MgF(k) = double(single(cfg.COMP.LEG_GRAV_FF) * single(cfg.g) ...
                        * sin(single(ang(k)) + theta0) * l_d);
    end
    if ok_len(k) && (~urdf || isfield(cfg,'URDF_USE_SPRING'))
        Fs(k) = double(Leg_SpringF(single(len(k))));
    end
end

ok_fc = all(ok_len) && isfinite(v) && isfinite(omega);
if ok_fc
    L0 = min(max(0.5*(single(len_l) + single(len_r)), single(cfg.COMP.ROLL_LEN_CLAMP(1))), ...
             single(cfg.COMP.ROLL_LEN_CLAMP(2)));
    fc = single(cfg.M_SUP) * single(v) * single(omega) * (L0 + single(cfg.H_OFF)) ...
         / single(cfg.COMP.ROLL_TRACK);
    fc = min(max(fc, -single(cfg.COMP.ROLL_CLAMP)), single(cfg.COMP.ROLL_CLAMP));
    Fc(1) = -double(fc);
    Fc(2) =  double(fc);
end

ok = zeros(3,2);
ok(1,:) = ok_len & ok_ang;
ok(2,:) = ok_len;
ok(3,:) = ok_fc;

c = struct('MgF', zeros(2,1), 'Fs', zeros(2,1), 'Fc', zeros(2,1), ...
           'ffwd', zeros(6,1), 'st', zeros(3,2));
for k = 1:2
    [c.MgF(k), c.st(1,k), st] = output_put(st, 1, k, ok(1,k), MgF(k));
    [c.Fs(k),  c.st(2,k), st] = output_put(st, 2, k, ok(2,k), Fs(k));
    [c.Fc(k),  c.st(3,k), st] = output_put(st, 3, k, ok(3,k), Fc(k));
end
end

function st = output_reset()
st = struct('v', zeros(3,2), 'ok', false(3,2), 'age', zeros(3,2));
end

function n = output_hold_n()
n = 5;
end

function [out, status, st] = output_put(st, slot, side, ok, val)
if ok
    st.v(slot, side) = val;
    st.ok(slot, side) = true;
    st.age(slot, side) = 0;
    out = st.v(slot, side);
    status = 1;
    return;
end

st.age(slot, side) = min(st.age(slot, side) + 1, 255);
if st.ok(slot, side) && st.age(slot, side) <= output_hold_n()
    out = st.v(slot, side);
    status = 2;
    return;
end

out = 0;
status = 0;
end
