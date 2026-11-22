function q = mixmpc_vertical_pose(model,height)
% 同一 CAD 装配支路上求两侧轮轴在髋轴正下方的位姿，height 可为标量或两侧长度。
robot = model.robot;
hip = arrayfun(@(s)s.frames(1),robot.side);
wheel = arrayfun(@(s)s.frames(11),robot.side);
q = zeros(12,1);
for i = 1:40
    kin = mixmpc_full_kinematics(model,q,zeros(12,1));
    offset = kin.pose.p([1 3],wheel)-kin.pose.p([1 3],hip);
    offset(2,:) = offset(2,:)+height;
    if norm(offset(:),Inf)<1e-12, return; end
    J = kin.Jv([1 3],7:10,wheel)-kin.Jv([1 3],7:10,hip);
    J = reshape(permute(J,[1 3 2]),4,4);
    step = J\offset(:);
    q(7:10) = q(7:10)-step/max(1,norm(step,Inf)/0.2);
end
error('001 vertical pose failed at %s m.',mat2str(height));
end
