function pose = mixmpc_robot_fk(robot, q, p_base, R_base)
% Forward kinematics of the URDF tree, with radians on its revolute axes.
if nargin < 3
    p_base = zeros(3,1);
    R_base = eye(3);
end
n = numel(robot.links);
pose.p = nan(3,n);
pose.R = nan(3,3,n);
pose.p(:,robot.base) = p_base;
pose.R(:,:,robot.base) = R_base;
for joint = robot.joints
    parent_R = pose.R(:,:,joint.parent);
    rotation = joint.rotation;
    if joint.q
        a = joint.axis;
        v = q(joint.q);
        cross_a = [0 -a(3) a(2); a(3) 0 -a(1); -a(2) a(1) 0];
        rotation = rotation * (eye(3) + sin(v)*cross_a + (1-cos(v))*(cross_a*cross_a));
    end
    pose.p(:,joint.child) = pose.p(:,joint.parent) + parent_R*joint.translation;
    pose.R(:,:,joint.child) = parent_R*rotation;
end
end
