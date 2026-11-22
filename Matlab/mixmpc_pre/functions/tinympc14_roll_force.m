function force = tinympc14_roll_force(p, height, roll, model_roll, model_psi)
% 用观测 H、roll 求倾覆矩，用接收该扰动的动力学工作点还原两腿等效轴力。
% 两腿分别满足 J_H*F=0、J_roll*F=M_tip。psi 可为共同角或左右两角。
if nargin<4, model_roll=roll; end
if nargin<5, model_psi=0; end
force = zeros(2,1);
if isfield(p,'ROLL_TIP_GAIN')
    [~,~,slope] = tinympc14_hr_params(p,height);
    moment = p.ROLL_TIP_GAIN*slope*roll;
    force = [1;-1]*(moment/(2*p.R_TRK));
    if isfield(p,'sym7') && isfield(p.sym7,'sw_fused') && p.sym7.sw_fused.F_H_proj
        force = force.*cos(model_psi(:))*cos(model_roll)^2;
    end
end
end
