function [height_arm,roll_arm,droll_dphi] = tinympc14_leg_jacobian(b,H,phi,psi)
% 双轮平地闭链 L_i=(H±b sin(phi))/(cos(phi)cos(psi_i))。
% 固定两腿矢状角的虚位移 dL_i=height_arm_i*dH+roll_arm_i*dphi。
% 轴力与 +L 共轭，乘上述两臂分别得到虚升降力和虚横滚力矩。
sec_phi = 1/cos(phi);
tan_phi = tan(phi);
side_arm = [b,-b]*sec_phi;
height_arm = [sec_phi,sec_phi]./cos(psi(:)');
roll_arm = height_arm.*(H*tan_phi+side_arm);
droll_dphi = height_arm.*(H*(tan_phi^2+sec_phi^2)+2*side_arm*tan_phi);
end
