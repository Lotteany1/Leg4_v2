function [M,G,tip] = tinympc14_hr_params(p,H)
% 双轮无侧滑的 CAD 升降/横滚切向惯量与重力，坐标顺序 [H,roll]。
if isfield(p,'HR_DATA')
    row = tinympc14_table_row(p.HR_DATA,H);
    M = [row(1),row(2);row(2),row(3)];
    G = row(4:5)';
    tip = row(6);
else
    M = diag([p.M_V,p.I_ROLL]);
    G = [p.M_LOAD*p.g;0];
    tip = p.M_SUP*p.g*(H+p.H_OFF);
end
end
